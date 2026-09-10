#!/usr/bin/env python3

from __future__ import annotations
import argparse
import contextlib
import fcntl
import hashlib
import http.client
import json
import os
import re
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
import urllib.error
import urllib.request
from pathlib import Path, PurePosixPath
from typing import BinaryIO, Iterator, Mapping, Sequence
from urllib.parse import quote, unquote, urlsplit


DEFAULT_REPOSITORY = "jpyaaa3/elesim"
DEFAULT_REF = "main"
CACHE_SCHEMA_VERSION = 1
BOOTSTRAP_CONTRACT_SCHEMA_VERSION = 1
BOOTSTRAP_API_VERSION = 1
REQUIRED_SETUP_COMMANDS = ("wizard", "gui", "install", "update", "status")
VERIFY_BOOTSTRAP_SOURCE_ENV = "ELESIM_VERIFY_BOOTSTRAP_SOURCE"
_FULL_COMMIT_RE = re.compile(r"[0-9a-fA-F]{40}")
_REVISION_RE = re.compile(r"(?:git-[0-9a-f]{40}|sha256-[0-9a-f]{64})\Z")
_BOOTSTRAP_ROLES = ("pilot", "sim", "ui", "robot")
_BOOTSTRAP_ROLE_RUNTIMES = {
    "pilot": PurePosixPath("payload/runtime/docker/pilot"),
    "sim": PurePosixPath("payload/runtime/docker/sim"),
    "ui": PurePosixPath("payload/runtime/docker/ui"),
    "robot": PurePosixPath("payload/runtime/native/robot"),
}
_BOOTSTRAP_ROLE_APPLICATIONS = {
    role: runtime / "app"
    for role, runtime in _BOOTSTRAP_ROLE_RUNTIMES.items()
}
_BOOTSTRAP_SOURCE_FILES = frozenset(
    {
        PurePosixPath("installer/bootstrap/bootstrap.py"),
        PurePosixPath("installer/bootstrap/install.sh"),
        PurePosixPath("installer/bootstrap/bootstrap-contract.json"),
        PurePosixPath("payload/runtime/docker/setup/app/pyproject.toml"),
        PurePosixPath("payload/runtime/docker/setup/app/requirements.lock"),
        PurePosixPath("payload/runtime/common/protocol/pyproject.toml"),
        PurePosixPath("payload/runtime/common/elesim_interfaces/CMakeLists.txt"),
        PurePosixPath("payload/runtime/common/elesim_interfaces/package.xml"),
        PurePosixPath("payload/runtime/docker/shared/Dockerfile.app"),
        PurePosixPath("payload/runtime/docker/setup/Dockerfile"),
        PurePosixPath("payload/runtime/docker/setup/tools-entrypoint"),
        PurePosixPath("payload/runtime/docker/shared/robotpkg.asc"),
        PurePosixPath("payload/runtime/docker/dev/Dockerfile"),
        PurePosixPath("payload/runtime/docker/dev/requirements.lock"),
        PurePosixPath("payload/runtime/docker/dev/entrypoint.sh"),
        PurePosixPath("payload/runtime/docker/dev/dev-env.sh"),
        *(
            _BOOTSTRAP_ROLE_APPLICATIONS[role] / "pyproject.toml"
            for role in _BOOTSTRAP_ROLES
        ),
        *(
            _BOOTSTRAP_ROLE_RUNTIMES[role] / "requirements.lock"
            for role in _BOOTSTRAP_ROLES
        ),
        *(
            PurePosixPath("payload/runtime/docker") / role / "entrypoint"
            for role in ("pilot", "sim", "ui")
        ),
    }
)
_BOOTSTRAP_SOURCE_TREES = (
    PurePosixPath("payload/runtime/docker/setup/app"),
    PurePosixPath("payload/runtime/common/protocol"),
    PurePosixPath("payload/runtime/common/elesim_interfaces/msg"),
    PurePosixPath("payload/runtime/common/elesim_interfaces/srv"),
    PurePosixPath("payload/runtime/common/elesim_interfaces/action"),
    *(
        _BOOTSTRAP_ROLE_APPLICATIONS[role] / f"elesim_{role}"
        for role in _BOOTSTRAP_ROLES
    ),
    *(
        PurePosixPath("payload/config") / role
        for role in _BOOTSTRAP_ROLES
    ),
    PurePosixPath("payload/data/models/assemblies/zed-mini"),
    PurePosixPath("payload/data/models/assemblies/d435"),
    PurePosixPath("payload/data/models/perception"),
    PurePosixPath("payload/data/models/arm"),
    PurePosixPath("payload/data/models/objects"),
    PurePosixPath("payload/data/policies"),
    PurePosixPath("payload/data/calibration"),
)
_BOOTSTRAP_SETUP_PYTHON_FILES = frozenset(
    PurePosixPath("payload/runtime/docker/setup/app/elesim_setup") / f"{name}.py"
    for name in (
        "__init__",
        "_security_storage",
        "capabilities",
        "cli",
        "configuration",
        "connection_gui",
        "connection_manager",
        "connections",
        "container_installer",
        "credentials",
        "developer",
        "doctor",
        "gui",
        "host_helper",
        "host_proxy",
        "installer",
        "instance_compose",
        "instance_identity",
        "instance_preparation",
        "instance_remove",
        "instance_runtime",
        "instance_security",
        "instances",
        "manager_lifecycle",
        "manager_ownership",
        "network",
        "operation_lock",
        "ownership",
        "profiles",
        "releases",
        "release_publication",
        "request",
        "runtime_status",
        "secure_deployment",
        "security_authority",
        "security_policy",
        "security_provisioning",
        "security_views",
        "service",
        "shell",
        "state",
        "uninstall",
        "updater",
    )
)
_BOOTSTRAP_PROTOCOL_PYTHON_FILES = frozenset(
    PurePosixPath("payload/runtime/common/protocol/elesim_protocol") / f"{name}.py"
    for name in (
        "__init__",
        "authority",
        "contracts",
        "dds_transport",
        "encoded_rgbd",
        "messages",
        "operator",
        "payloads",
        "peer",
        "rgbd",
        "serde",
        "tracing",
        "transport",
    )
)
_BOOTSTRAP_ROLE_ENTRYPOINT_FILES = frozenset(
    {
        PurePosixPath("payload/runtime/docker/pilot/app/elesim_pilot/main.py"),
        PurePosixPath("payload/runtime/docker/sim/app/elesim_sim/main.py"),
        PurePosixPath("payload/runtime/docker/ui/app/elesim_ui/main.py"),
        PurePosixPath("payload/runtime/native/robot/app/elesim_robot/main.py"),
        PurePosixPath(
            "payload/runtime/native/robot/app/elesim_robot/go2/unitree_bridge_daemon.py"
        ),
    }
)
_BOOTSTRAP_ROLE_CONFIG_FILES = frozenset(
    PurePosixPath("payload/config") / role / relative
    for role, relatives in {
        "pilot": (
            "config.yaml",
            "perception/detector.real_green_hsv.json",
            "perception/detector.sim_hsv.json",
            "perception/detector.yolo.example.json",
            "runtime.yaml",
        ),
        "sim": (
            "config.yaml",
            "runtime.yaml",
        ),
        "ui": (
            "default.yaml",
            "perception/detector.real_green_hsv.json",
            "perception/detector.sim_hsv.json",
            "perception/detector.yolo.example.json",
        ),
        "robot": ("default.yaml",),
    }.items()
    for relative in relatives
)
_BOOTSTRAP_REQUIRED_TREE_FILES = frozenset(
    {
        *_BOOTSTRAP_SETUP_PYTHON_FILES,
        *_BOOTSTRAP_PROTOCOL_PYTHON_FILES,
        *_BOOTSTRAP_ROLE_ENTRYPOINT_FILES,
        *_BOOTSTRAP_ROLE_CONFIG_FILES,
        PurePosixPath("payload/runtime/common/elesim_interfaces/msg/RgbdFrame.msg"),
        PurePosixPath(
            "payload/runtime/common/elesim_interfaces/srv/OpenSimulationSession.srv"
        ),
        PurePosixPath(
            "payload/runtime/common/elesim_interfaces/action/RunOperatorWorkflow.action"
        ),
        PurePosixPath("payload/data/models/assemblies/zed-mini/bundle.json"),
        PurePosixPath("payload/data/models/assemblies/d435/bundle.json"),
        PurePosixPath("payload/data/models/perception/yolov8n-seg.pt"),
        PurePosixPath("payload/data/models/arm/default.json"),
        PurePosixPath("payload/data/models/objects/demo_box.obj"),
        PurePosixPath("payload/data/calibration/cameras/zed_mini.hand_eye.json"),
        PurePosixPath("payload/data/calibration/cameras/d435.hand_eye.json"),
        PurePosixPath("payload/data/calibration/arm/no_sag.json"),
        PurePosixPath("payload/data/calibration/arm/sag_model.json"),
        *(
            _BOOTSTRAP_ROLE_APPLICATIONS[role] / f"elesim_{role}" / "__init__.py"
            for role in _BOOTSTRAP_ROLES
        ),
    }
)
_BOOTSTRAP_EXCLUDED_CONFIG_FILES = frozenset(
    {
        PurePosixPath("payload/config/pilot/runtime.public.example.yaml"),
        PurePosixPath("payload/config/sim/runtime.public.example.yaml"),
        PurePosixPath("payload/config/ui/public.example.yaml"),
        PurePosixPath("payload/config/robot/public.example.yaml"),
    }
)
_BOOTSTRAP_SOURCE_ONLY_COMPONENTS = frozenset(
    (
        "tests",
        "fixtures",
        "__pycache__",
        ".pytest_cache",
        "docs",
        "workbench",
        "coturn",
    )
)
_BOOTSTRAP_SOURCE_ONLY_PATHS = (PurePosixPath("payload/runtime/docker/sim/app/elesim_sim/rl"),)


class BootstrapError(RuntimeError):
    pass


def _bootstrap_source_only(relative: PurePosixPath) -> bool:
    return bool(_BOOTSTRAP_SOURCE_ONLY_COMPONENTS.intersection(relative.parts)) or any(
        excluded == relative or excluded in relative.parents
        for excluded in _BOOTSTRAP_SOURCE_ONLY_PATHS
    )


_ROSIDL_SOURCE_RE = re.compile(
    r'"((?:msg/[A-Za-z][A-Za-z0-9_]*\.msg|'
    r'srv/[A-Za-z][A-Za-z0-9_]*\.srv|'
    r'action/[A-Za-z][A-Za-z0-9_]*\.action))"'
)


def _bootstrap_source_path_allowed(relative: PurePosixPath) -> bool:
    if (
        relative in _BOOTSTRAP_EXCLUDED_CONFIG_FILES
        or relative.name.endswith(".pyc")
        or _bootstrap_source_only(relative)
        or any(part.endswith(".egg-info") for part in relative.parts)
        or any(part in {".pytest_cache", "__pycache__", "build"} for part in relative.parts)
    ):
        return False
    if relative in _BOOTSTRAP_SOURCE_FILES:
        return True
    return any(
        tree == relative or tree in relative.parents
        for tree in _BOOTSTRAP_SOURCE_TREES
    )


def _bootstrap_source_directory_allowed(relative: PurePosixPath) -> bool:
    if (
        _bootstrap_source_only(relative)
        or any(part.endswith(".egg-info") for part in relative.parts)
        or any(part in {".pytest_cache", "__pycache__", "build"} for part in relative.parts)
    ):
        return False
    return any(relative in path.parents for path in _BOOTSTRAP_SOURCE_FILES) or any(
        tree == relative
        or tree in relative.parents
        or relative in tree.parents
        for tree in _BOOTSTRAP_SOURCE_TREES
    )


def archive_url(repository: str, ref: str) -> str:
    repo = str(repository).strip().strip("/")
    revision = str(ref).strip()
    if repo.count("/") != 1 or not revision:
        raise BootstrapError("Please fill out repository or reference")
    return f"https://codeload.github.com/{repo}/tar.gz/{quote(revision, safe='')}"


def safe_extract_archive(archive: Path, destination: Path) -> Path:
    destination.mkdir(parents=True, exist_ok=True)
    roots: set[str] = set()
    with tarfile.open(archive, mode="r:*") as bundle:
        members = bundle.getmembers()
        for member in members:
            path = PurePosixPath(member.name)
            if path.is_absolute() or ".." in path.parts or not path.parts:
                raise BootstrapError(f"unsafe archive member: {member.name!r}")
            if member.issym() or member.islnk() or member.isdev():
                relative = PurePosixPath(*path.parts[1:])
                if (
                    _bootstrap_source_path_allowed(relative)
                    or _bootstrap_source_directory_allowed(relative)
                ):
                    raise BootstrapError(
                        f"Unsupported archive link/device: {member.name!r}"
                    )
                continue
            roots.add(path.parts[0])
        if len(roots) != 1:
            raise BootstrapError("Source archive must contain exactly one top-level directory")

        for member in members:
            relative = PurePosixPath(member.name)
            target = destination.joinpath(*relative.parts)
            resolved = target.resolve()
            try:
                resolved.relative_to(destination.resolve())
            except ValueError as exc:
                raise BootstrapError(f"archive escaped destination: {member.name!r}") from exc
            if member.isdir():
                continue
            if not member.isfile():
                continue
            source_relative = PurePosixPath(*relative.parts[1:])
            if not _bootstrap_source_path_allowed(source_relative):
                continue
            resolved.parent.mkdir(parents=True, exist_ok=True)
            source = bundle.extractfile(member)
            if source is None:
                raise BootstrapError(f"cannot read archive member: {member.name!r}")
            with source, resolved.open("wb") as handle:
                shutil.copyfileobj(source, handle)
            resolved.chmod(member.mode & 0o777)

    root = destination / next(iter(roots))
    if not (root / "payload/runtime/docker/setup/app/pyproject.toml").is_file():
        raise BootstrapError("Downloaded archive does not contain the EleSim setup package")
    return root


def _url_fingerprint(url: str) -> str:
    return hashlib.sha256(url.encode("utf-8")).hexdigest()


def _is_github_codeload_url(url: str) -> bool:
    parsed = urlsplit(url)
    return parsed.scheme == "https" and parsed.hostname == "codeload.github.com"


def _immutable_commit(ref: str | None, url: str) -> str | None:
    if not _is_github_codeload_url(url):
        return None
    if ref is not None:
        candidate = str(ref).strip()
    else:
        candidate = unquote(urlsplit(url).path.rstrip("/").rsplit("/", 1)[-1])
    if _FULL_COMMIT_RE.fullmatch(candidate) is None:
        return None
    return candidate.lower()


def _log_value(value: str) -> str:
    printable = "".join(character if character.isprintable() else "?" for character in value)
    return printable[:120]


def _log_source(*, ref: str | None, revision: str, status: str) -> None:
    reference = _log_value(ref) if ref is not None else "custom-archive"
    print(f"[bootstrap] source ref={reference} revision={revision} status={status}")


@contextlib.contextmanager
def _locked_url_cache(cache: Path) -> Iterator[None]:
    cache.mkdir(parents=True, exist_ok=True)
    lock_path = cache / ".lock"
    with lock_path.open("a+b") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lock.fileno(), fcntl.LOCK_UN)


def _read_index(cache: Path) -> dict[str, object] | None:
    try:
        value = json.loads((cache / "current.json").read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return None
    if not isinstance(value, dict) or value.get("schema_version") != CACHE_SCHEMA_VERSION:
        return None
    return value


def _index_text(index: Mapping[str, object], key: str) -> str | None:
    value = index.get(key)
    return value if isinstance(value, str) and value else None


def _validated_source_revision(cache_root: Path, url: str, source_root: Path) -> str:
    """Return the revision authenticated by the completed download cache.

    The setup process must receive the revision selected by bootstrap, never a
    caller-supplied environment value.  Re-read the cache index and verify it
    still points at the exact validated snapshot returned by ``download_source``.
    """

    cache = Path(cache_root) / "sources-v2" / _url_fingerprint(url)
    index = _read_index(cache)
    revision = _index_text(index or {}, "revision") if index is not None else None
    if revision is None or _REVISION_RE.fullmatch(revision) is None:
        raise BootstrapError("Downloaded source has no canonical revision")
    cached_root = _snapshot_root(cache, index)
    try:
        if cached_root is None or cached_root.resolve() != Path(source_root).resolve():
            raise BootstrapError("Downloaded source cache changed during bootstrap")
    except OSError as exc:
        raise BootstrapError("Downloaded source cache cannot be verified") from exc
    return revision


def _write_source_revision_handoff(revision: str) -> None:
    """Publish bootstrap's authenticated revision for an outer update shell.

    ``install.sh`` starts bootstrap as a child process, so the authenticated
    value cannot flow back through the child's environment.  The shell opts
    into this handoff only for its ``update`` command and supplies a fixed
    ``<invocation>/maintenance/.bootstrap-source-revision`` path.  Refuse
    arbitrary paths and write atomically with private permissions so a stale
    or partially-written value cannot be consumed by the publisher.
    """

    raw_path = os.environ.get("ELESIM_SOURCE_REVISION_FILE", "").strip()
    if not raw_path:
        return
    if _REVISION_RE.fullmatch(revision) is None:
        raise BootstrapError("Cannot write a non-canonical source revision handoff")
    path = Path(raw_path).expanduser()
    invocation_raw = os.environ.get("ELESIM_INVOCATION_DIR", "").strip()
    if (
        not path.is_absolute()
        or path.name != ".bootstrap-source-revision"
        or path.parent.name != "maintenance"
        or not invocation_raw
    ):
        raise BootstrapError("Invalid source revision handoff path")
    invocation = Path(invocation_raw).expanduser()
    try:
        if path.parent.parent.resolve() != invocation.resolve():
            raise BootstrapError("Source revision handoff is outside the invocation directory")
    except OSError as exc:
        raise BootstrapError("Cannot validate source revision handoff path") from exc

    parent = path.parent
    temporary: Path | None = None
    try:
        parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        parent_stat = os.lstat(parent)
        if stat.S_ISLNK(parent_stat.st_mode) or not stat.S_ISDIR(parent_stat.st_mode):
            raise BootstrapError("Source revision handoff directory is unsafe")
        fd, temporary_name = tempfile.mkstemp(
            prefix=".bootstrap-source-revision.", dir=str(parent)
        )
        temporary = Path(temporary_name)
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, "wb") as handle:
            handle.write((revision + "\n").encode("ascii"))
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = None
    except BootstrapError:
        raise
    except (OSError, UnicodeError) as exc:
        raise BootstrapError(f"Cannot write source revision handoff: {exc}") from exc
    finally:
        if temporary is not None:
            try:
                temporary.unlink()
            except OSError:
                pass


def _validate_source_snapshot(root: Path) -> None:
    required_files = tuple(
        root.joinpath(*relative.parts)
        for relative in sorted(
            _BOOTSTRAP_SOURCE_FILES | _BOOTSTRAP_REQUIRED_TREE_FILES,
            key=lambda value: value.as_posix(),
        )
    )
    missing = [str(path.relative_to(root)) for path in required_files if not path.is_file()]
    if missing:
        raise BootstrapError(
            "Downloaded archive is missing required setup files: "
            + ", ".join(missing)
        )
    setup_root = root / "payload/runtime/docker/setup/app/elesim_setup"
    actual_setup_python = frozenset(
        PurePosixPath(path.relative_to(root).as_posix())
        for path in setup_root.rglob("*.py")
    )
    if actual_setup_python != _BOOTSTRAP_SETUP_PYTHON_FILES:
        raise BootstrapError(
            "unexpected setup Python module manifest: "
            f"missing={sorted(_BOOTSTRAP_SETUP_PYTHON_FILES - actual_setup_python)!r}; "
            f"unexpected={sorted(actual_setup_python - _BOOTSTRAP_SETUP_PYTHON_FILES)!r}"
        )
    protocol_root = root / "payload/runtime/common/protocol/elesim_protocol"
    actual_protocol_python = frozenset(
        PurePosixPath(path.relative_to(root).as_posix())
        for path in protocol_root.rglob("*.py")
    )
    if actual_protocol_python != _BOOTSTRAP_PROTOCOL_PYTHON_FILES:
        raise BootstrapError(
            "unexpected protocol Python module manifest: "
            f"missing={sorted(_BOOTSTRAP_PROTOCOL_PYTHON_FILES - actual_protocol_python)!r}; "
            f"unexpected={sorted(actual_protocol_python - _BOOTSTRAP_PROTOCOL_PYTHON_FILES)!r}"
        )
    actual_role_configs = frozenset(
        PurePosixPath(path.relative_to(root).as_posix())
        for role in _BOOTSTRAP_ROLES
        for path in (root / "payload/config" / role).rglob("*")
        if path.is_file()
        and PurePosixPath(path.relative_to(root).as_posix())
        not in _BOOTSTRAP_EXCLUDED_CONFIG_FILES
    )
    if actual_role_configs != _BOOTSTRAP_ROLE_CONFIG_FILES:
        raise BootstrapError(
            "Unexpected role config manifest: "
            f"missing={sorted(_BOOTSTRAP_ROLE_CONFIG_FILES - actual_role_configs)!r}; "
            f"unexpected={sorted(actual_role_configs - _BOOTSTRAP_ROLE_CONFIG_FILES)!r}"
        )
    _validate_rosidl_source_manifest(root / "payload/runtime/common/elesim_interfaces")
    unexpected: list[str] = []
    for path in root.rglob("*"):
        relative = PurePosixPath(path.relative_to(root).as_posix())
        if path.is_symlink():
            unexpected.append(relative.as_posix())
        elif path.is_dir():
            if not _bootstrap_source_directory_allowed(relative):
                unexpected.append(relative.as_posix())
        elif (
            not path.is_file()
            or (
                relative not in _BOOTSTRAP_EXCLUDED_CONFIG_FILES
                and not _bootstrap_source_path_allowed(relative)
            )
        ):
            unexpected.append(relative.as_posix())
        if len(unexpected) >= 5:
            break
    if unexpected:
        raise BootstrapError(
            "Downloaded archive contains files outside the install source boundary: "
            + ", ".join(unexpected)
        )
    validate_bootstrap_contract(root)


def _validate_rosidl_source_manifest(interface_root: Path) -> None:
    cmake = interface_root / "CMakeLists.txt"
    try:
        declared_values = _ROSIDL_SOURCE_RE.findall(cmake.read_text(encoding="utf-8"))
    except (OSError, UnicodeError) as exc:
        raise BootstrapError(f"cannot read ROSIDL manifest: {cmake}") from exc
    declared = frozenset(declared_values)
    if not declared or len(declared) != len(declared_values):
        raise BootstrapError(
            f"ROSIDL CMake manifest is empty or contains duplicates: {cmake}"
        )
    actual: set[str] = set()
    for directory, suffix in (("msg", ".msg"), ("srv", ".srv"), ("action", ".action")):
        source_dir = interface_root / directory
        try:
            sources = tuple(source_dir.iterdir())
        except OSError as exc:
            raise BootstrapError(f"missing ROSIDL source directory: {source_dir}") from exc
        for source in sources:
            if not source.is_file() or source.suffix != suffix:
                raise BootstrapError(f"unexpected ROSIDL source member: {source}")
            actual.add(source.relative_to(interface_root).as_posix())
    if actual != declared:
        raise BootstrapError(
            "ROSIDL source manifest mismatch: "
            f"missing={sorted(declared - actual)!r}; "
            f"unexpected={sorted(actual - declared)!r}"
        )


def _snapshots_directory(cache: Path) -> Path:
    snapshots = cache / "snapshots"
    try:
        if snapshots.is_symlink():
            raise BootstrapError("Cache directory must not be a symlink")
        resolved_cache = cache.resolve()
        resolved_snapshots = snapshots.resolve()
        resolved_snapshots.relative_to(resolved_cache)
    except (OSError, ValueError) as exc:
        raise BootstrapError("Cache directory escapes its URL cache") from exc
    return snapshots


def _snapshot_root(cache: Path, index: Mapping[str, object] | None) -> Path | None:
    if index is None:
        return None
    revision = _index_text(index, "revision")
    root_name = _index_text(index, "root_name")
    digest = _index_text(index, "archive_sha256")
    if (
        revision is None
        or _REVISION_RE.fullmatch(revision) is None
        or root_name is None
        or root_name in {".", ".."}
        or PurePosixPath(root_name).is_absolute()
        or PurePosixPath(root_name).parts != (root_name,)
        or digest is None
        or re.fullmatch(r"[0-9a-f]{64}", digest) is None
        or (revision.startswith("sha256-") and revision != f"sha256-{digest}")
    ):
        return None
    try:
        snapshots = _snapshots_directory(cache)
    except BootstrapError:
        return None
    snapshot = snapshots / revision
    try:
        if snapshot.is_symlink():
            return None
        resolved_snapshots = snapshots.resolve()
        resolved_snapshot = snapshot.resolve()
        resolved_snapshot.relative_to(resolved_snapshots)
    except (OSError, ValueError):
        return None
    marker = snapshot / ".elesim-source-complete"
    try:
        if marker.read_text(encoding="utf-8").strip() != root_name:
            return None
    except (OSError, UnicodeError):
        return None
    try:
        root_path = snapshot / root_name
        if root_path.is_symlink():
            return None
        root = root_path.resolve()
        root.relative_to(resolved_snapshot)
    except (OSError, ValueError):
        return None
    try:
        _validate_source_snapshot(root)
    except BootstrapError:
        return None
    return root


def _atomic_write_json(path: Path, value: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(value, handle, ensure_ascii=False, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        directory_descriptor = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
    finally:
        temporary.unlink(missing_ok=True)


def _response_header(headers: object, name: str) -> str | None:
    getter = getattr(headers, "get", None)
    if getter is None:
        return None
    value = getter(name)
    return str(value).strip() if value is not None and str(value).strip() else None


def _safe_download_error(exc: BaseException, url: str) -> str:
    if isinstance(exc, urllib.error.HTTPError):
        detail = f"HTTP {exc.code} {exc.reason}"
    else:
        reason = getattr(exc, "reason", None)
        detail = str(reason if reason is not None else exc)
    parsed = urlsplit(url)
    redactions = (url, parsed.query)
    for secret in redactions:
        if secret:
            detail = detail.replace(secret, "<redacted>")
    return detail


def _download_archive(
    url: str,
    archive: Path,
    *,
    validators: Mapping[str, str],
) -> tuple[int, str | None, str | None, str | None]:
    request_headers: dict[str, str] = {}
    etag = validators.get("etag")
    last_modified = validators.get("last_modified")
    if etag:
        request_headers["If-None-Match"] = etag
    if last_modified:
        request_headers["If-Modified-Since"] = last_modified
    request = urllib.request.Request(url, headers=request_headers)
    try:
        response = urllib.request.urlopen(request, timeout=60)
    except urllib.error.HTTPError as exc:
        try:
            if exc.code == 304:
                return (
                    304,
                    _response_header(exc.headers, "ETag") or etag,
                    _response_header(exc.headers, "Last-Modified") or last_modified,
                    None,
                )
            raise BootstrapError(
                f"Source archive download failed: {_safe_download_error(exc, url)}"
            ) from exc
        finally:
            exc.close()
    except (OSError, http.client.HTTPException, urllib.error.URLError) as exc:
        raise BootstrapError(
            f"Source archive download failed: {_safe_download_error(exc, url)}"
        ) from exc

    try:
        with response:
            status_value = getattr(response, "status", None)
            if status_value is None:
                status_value = response.getcode()
            status = 200 if status_value is None else int(status_value)
            response_etag = _response_header(response.headers, "ETag")
            response_last_modified = _response_header(response.headers, "Last-Modified")
            if status == 304:
                return (
                    status,
                    response_etag or etag,
                    response_last_modified or last_modified,
                    None,
                )
            if status != 200:
                raise BootstrapError(f"Source archive download failed: HTTP {status}")
            digest = hashlib.sha256()
            with archive.open("wb") as handle:
                while True:
                    chunk = response.read(1024 * 1024)
                    if not chunk:
                        break
                    digest.update(chunk)
                    handle.write(chunk)
            return status, response_etag, response_last_modified, digest.hexdigest()
    except BootstrapError:
        raise
    except (OSError, http.client.HTTPException, urllib.error.URLError) as exc:
        raise BootstrapError(
            f"source archive download failed: {_safe_download_error(exc, url)}"
        ) from exc


def _archive_revision(archive: Path, digest: str, *, trust_git_comment: bool) -> str:
    if trust_git_comment:
        with tarfile.open(archive, mode="r:*") as bundle:
            bundle.getmembers()
            comment = str(bundle.pax_headers.get("comment", "")).strip()
        if _FULL_COMMIT_RE.fullmatch(comment):
            return f"git-{comment.lower()}"
    return f"sha256-{digest}"


def _publish_snapshot(
    *,
    cache: Path,
    staging: Path,
    source_root: Path,
    revision: str,
    archive_sha256: str,
    replace_existing: bool,
) -> Path:
    root_name = source_root.name
    marker = staging / ".elesim-source-complete"
    marker.write_text(root_name + "\n", encoding="utf-8")
    snapshots = _snapshots_directory(cache)
    snapshots.mkdir(parents=True, exist_ok=True)
    destination = snapshots / revision
    if destination.exists() or destination.is_symlink():
        existing = None
        if not replace_existing:
            existing = _snapshot_root(
                cache,
                {
                    "schema_version": CACHE_SCHEMA_VERSION,
                    "revision": revision,
                    "root_name": root_name,
                    "archive_sha256": archive_sha256,
                },
            )
        if existing is not None:
            return existing
        quarantine = Path(tempfile.mkdtemp(prefix=f".invalid-{revision}-", dir=snapshots))
        quarantine.rmdir()
        os.replace(destination, quarantine)
        try:
            os.replace(staging, destination)
        except BaseException:
            os.replace(quarantine, destination)
            raise
        if quarantine.is_symlink() or quarantine.is_file():
            quarantine.unlink()
        else:
            shutil.rmtree(quarantine)
    else:
        os.replace(staging, destination)
    return destination / root_name


def download_source(
    url: str,
    cache_root: Path,
    *,
    refresh: bool = False,
    ref: str | None = None,
) -> Path:
    cache_root = cache_root.expanduser().resolve()
    cache = cache_root / "sources-v2" / _url_fingerprint(url)
    immutable_commit = _immutable_commit(ref, url)
    with _locked_url_cache(cache):
        index = _read_index(cache)
        cached_root = _snapshot_root(cache, index)
        if immutable_commit is not None and cached_root is not None and not refresh:
            if _index_text(index or {}, "revision") != f"git-{immutable_commit}":
                raise BootstrapError(
                    "Cached source revision does not match the requested immutable commit"
                )
            revision = _index_text(index or {}, "revision")
            assert revision is not None
            _log_source(ref=ref, revision=revision, status="immutable-cache")
            return cached_root

        validators: dict[str, str] = {}
        if not refresh and index is not None:
            etag = _index_text(index, "etag")
            last_modified = _index_text(index, "last_modified")
            if etag is not None:
                validators["etag"] = etag
            if last_modified is not None:
                validators["last_modified"] = last_modified

        with tempfile.TemporaryDirectory(prefix=".download-", dir=cache) as td:
            temporary = Path(td)
            archive = temporary / "source.tar.gz"
            status, etag, last_modified, digest = _download_archive(
                url,
                archive,
                validators=validators,
            )
            if status == 304:
                if not validators:
                    raise BootstrapError(
                        "Source server returned 304 without a conditional request"
                    )
                if cached_root is not None:
                    revision = _index_text(index or {}, "revision")
                    assert revision is not None
                    updated_index = dict(index or {})
                    if etag is not None:
                        updated_index["etag"] = etag
                    if last_modified is not None:
                        updated_index["last_modified"] = last_modified
                    if updated_index != index:
                        _atomic_write_json(cache / "current.json", updated_index)
                    _log_source(ref=ref, revision=revision, status="validated")
                    return cached_root
                status, etag, last_modified, digest = _download_archive(
                    url,
                    archive,
                    validators={},
                )
                if status == 304:
                    raise BootstrapError(
                        "Source server returned 304 after an unconditional retry"
                    )
            if status != 200 or digest is None:
                raise BootstrapError("Source archive did not return content")

            staging = temporary / "snapshot"
            source_root = safe_extract_archive(archive, staging)
            _validate_source_snapshot(source_root)
            revision = _archive_revision(
                archive,
                digest,
                trust_git_comment=_is_github_codeload_url(url),
            )
            if (
                immutable_commit is not None
                and revision != f"git-{immutable_commit}"
            ):
                raise BootstrapError(
                    "Downloaded archive revision does not match the requested "
                    "immutable commit"
                )
            published_root = _publish_snapshot(
                cache=cache,
                staging=staging,
                source_root=source_root,
                revision=revision,
                archive_sha256=digest,
                replace_existing=refresh,
            )
            new_index: dict[str, object] = {
                "schema_version": CACHE_SCHEMA_VERSION,
                "revision": revision,
                "root_name": published_root.name,
                "archive_sha256": digest,
            }
            if etag is not None:
                new_index["etag"] = etag
            if last_modified is not None:
                new_index["last_modified"] = last_modified
            _atomic_write_json(cache / "current.json", new_index)
            _log_source(ref=ref, revision=revision, status="downloaded")
            return published_root


def validate_bootstrap_contract(source_root: Path) -> dict[str, object]:
    path = source_root / "installer/bootstrap/bootstrap-contract.json"
    try:
        contract = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise BootstrapError("Downloaded archive is missing bootstrap-contract.json") from exc
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise BootstrapError(f"Cannot read bootstrap contract: {exc}") from exc
    if not isinstance(contract, dict):
        raise BootstrapError("Bootstrap contract must be a JSON object")
    if contract.get("schema_version") != BOOTSTRAP_CONTRACT_SCHEMA_VERSION:
        raise BootstrapError("Unsupported bootstrap contract schema")
    if contract.get("bootstrap_api") != BOOTSTRAP_API_VERSION:
        raise BootstrapError("Bootstrap generation mismatch: Incompatible bootstrap API")
    commands = contract.get("required_commands")
    if commands != list(REQUIRED_SETUP_COMMANDS):
        raise BootstrapError("Bootstrap generation mismatch: Required setup commands differ")
    return contract


def validate_bootstrap_generation(
    source_root: Path,
    *,
    executing_file: Path | None = None,
) -> None:
    if executing_file is None:
        if os.environ.get(VERIFY_BOOTSTRAP_SOURCE_ENV) != "1":
            return
        candidate = globals().get("__file__")
        if not isinstance(candidate, str) or candidate.startswith("<"):
            return
        executing_file = Path(candidate)
    if not executing_file.is_file():
        return
    archived = source_root / "installer/bootstrap/bootstrap.py"
    if not archived.is_file():
        raise BootstrapError("Downloaded archive is missing installer/bootstrap/bootstrap.py")
    try:
        executing_digest = hashlib.sha256(executing_file.read_bytes()).digest()
        archived_digest = hashlib.sha256(archived.read_bytes()).digest()
    except OSError as exc:
        raise BootstrapError(f"Cannot compare bootstrap generations: {exc}") from exc
    if executing_digest != archived_digest:
        raise BootstrapError(
            "Bootstrap generation mismatch: the branch moved during bootstrap; please retry."
        )


def preflight_setup(executable: Path) -> None:
    completed = subprocess.run(
        (str(executable), "gui", "--help"),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        raise BootstrapError(
            "Installed setup does not provide the required 'gui' command"
        )


def _setup_project_version(source_root: Path) -> str:
    try:
        project = (source_root / "payload/runtime/docker/setup/app/pyproject.toml").read_text(
            encoding="utf-8"
        )
    except (OSError, UnicodeError):
        return "unknown"
    match = re.search(r'(?m)^version\s*=\s*"([^"]+)"\s*$', project)
    return match.group(1) if match is not None else "unknown"


def _ensure_bootstrap_pip(python: Path) -> None:
    probe = subprocess.run(
        (str(python), "-m", "pip", "--version"),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if probe.returncode == 0:
        return

    repair = subprocess.run(
        (str(python), "-m", "ensurepip", "--upgrade"),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if repair.returncode != 0:
        detail = (repair.stderr or repair.stdout or probe.stderr or "").strip()
        if len(detail) > 600:
            detail = detail[-600:]
        suffix = f" ({detail})" if detail else ""
        raise BootstrapError(
            "Pip does not exist in the bootstrap venv. "
            "Please install a proper virtual environment "
            f"and then retry; {suffix}"
        )

    verify = subprocess.run(
        (str(python), "-m", "pip", "--version"),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if verify.returncode != 0:
        detail = (verify.stderr or "").strip()
        suffix = f" ({detail[-600:]})" if detail else ""
        raise BootstrapError(
            "Tried ensurepip recovery in the bootstrap venv, yet cannot run pip; sorry."
            f"{suffix}"
        )


def prepare_bootstrap_venv(source_root: Path, cache_root: Path) -> Path:
    fingerprint = hashlib.sha256(str(source_root).encode("utf-8")).hexdigest()[:16]
    venv = cache_root.expanduser().resolve() / f"venv-{fingerprint}"
    python = venv / "bin/python"
    if not python.is_file():
        print(f"[bootstrap] create venv {venv}")
        subprocess.run((sys.executable, "-m", "venv", str(venv)), check=True)
    _ensure_bootstrap_pip(python)
    commands = (
        (
            str(python),
            "-m",
            "pip",
            "--disable-pip-version-check",
            "install",
            "--upgrade",
            "pip",
            "setuptools>=68,<80",
            "packaging>=24.2,<26",
            "wheel",
        ),
        (str(python), "-m", "pip", "--disable-pip-version-check", "install", "-r", str(source_root / "payload/runtime/docker/setup/app/requirements.lock")),
        (str(python), "-m", "pip", "--disable-pip-version-check", "install", "--force-reinstall", "--no-deps", str(source_root / "payload/runtime/common/protocol")),
        (str(python), "-m", "pip", "--disable-pip-version-check", "install", "--force-reinstall", "--no-deps", str(source_root / "payload/runtime/docker/setup/app")),
        (str(python), "-m", "pip", "--disable-pip-version-check", "check"),
    )
    for command in commands:
        subprocess.run(command, check=True)
    return venv / "bin/elesim-setup"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default=os.environ.get("ELESIM_REPOSITORY", DEFAULT_REPOSITORY))
    parser.add_argument("--ref", default=os.environ.get("ELESIM_REF", DEFAULT_REF))
    parser.add_argument("--url", default=os.environ.get("ELESIM_ARCHIVE_URL", ""))
    parser.add_argument("--cache", default=os.environ.get("ELESIM_CACHE_DIR", "~/.cache/elesim/setup"))
    parser.add_argument("--refresh", action="store_true")
    return parser


def needs_controlling_terminal(arguments: Sequence[str]) -> bool:
    return not any(value in {"gui", "install", "update", "status"} for value in arguments)


def setup_arguments(
    arguments: Sequence[str],
    *,
    repository: str,
    ref: str,
) -> list[str]:
    forwarded = list(arguments) if arguments else ["gui"]
    command: str | None = None
    index = 0
    while index < len(forwarded):
        argument = forwarded[index]
        if argument in {"--source-root", "--state"}:
            index += 2
            continue
        if argument.startswith(("--source-root=", "--state=")):
            index += 1
            continue
        if argument in REQUIRED_SETUP_COMMANDS:
            command = argument
            break
        index += 1
    if command == "gui":
        forwarded.extend(("--repository", repository, "--ref", ref))
    return forwarded


def main(argv: Sequence[str] | None = None) -> int:
    if sys.version_info < (3, 10):
        print("Python 3.10 or higher is required", file=sys.stderr)
        return 2
    parser = _parser()
    args, wizard_args = parser.parse_known_args(argv)
    cache_root = Path(args.cache).expanduser().resolve()
    url = args.url or archive_url(args.repo, args.ref)
    try:
        source_root = download_source(
            url,
            cache_root,
            refresh=bool(args.refresh),
            ref=None if args.url else args.ref,
        )
        validate_bootstrap_contract(source_root)
        validate_bootstrap_generation(source_root)
        executable = prepare_bootstrap_venv(source_root, cache_root)
        preflight_setup(executable)
        print(f"[bootstrap] setup version={_setup_project_version(source_root)}")
        wizard_args = setup_arguments(
            wizard_args,
            repository=args.repo,
            ref=args.ref,
        )
        source_revision = _validated_source_revision(cache_root, url, source_root)
        _write_source_revision_handoff(source_revision)
        command = (str(executable), "--source-root", str(source_root), *wizard_args)
        setup_environment = os.environ.copy()
        # Bootstrap owns this value.  Do not forward a potentially stale or
        # forged ELESIM_SOURCE_REVISION from the outer shell/environment.
        setup_environment["ELESIM_SOURCE_REVISION"] = source_revision
        tty: BinaryIO | None = None
        run_stdin: BinaryIO | int | None = None
        try:
            if not sys.stdin.isatty():
                if needs_controlling_terminal(wizard_args):
                    try:
                        tty = open("/dev/tty", "rb", buffering=0)
                        run_stdin = tty
                    except OSError as exc:
                        raise BootstrapError(
                            "Interactive installation requires a controlling terminal. "
                            "Use the install subcommand for automated installations."
                        ) from exc
                else:
                    run_stdin = subprocess.DEVNULL
            completed = subprocess.run(
                command,
                stdin=run_stdin,
                check=False,
                env=setup_environment,
            )
        finally:
            if tty is not None:
                tty.close()
        return int(completed.returncode)
    except (BootstrapError, OSError, subprocess.CalledProcessError, tarfile.TarError) as exc:
        print(f"Bootstrap error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
