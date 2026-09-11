"""Fail-closed, host-only EleSim uninstaller.

Only Python's standard library is used here.  The command must remain usable
while it removes the generated tools image/venv that originally supplied it.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import secrets
import stat
import subprocess
import sys
import tempfile
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Mapping, Sequence

from .ownership import (
    DOCKER_INSTALL_UUID_LABEL,
    DockerOwnership,
    OwnedPath,
    OwnershipError,
    OwnershipManifest,
    default_manifest_path,
    sha256_file,
)
from .shell import inspect_bash_path, unregister_bash_path


CommandRunner = Callable[[Sequence[str]], subprocess.CompletedProcess[str]]
VIEWER_CLEANUP_WRAPPER = "elesim-viewer-cleanup"
VIEWER_STATE_RELATIVE_PATHS = (
    Path("cache/viewer-xhost"),
    Path(".runtime-cache/viewer-xhost"),
)
SIM_CONTAINER = "elesim-sim"
TAILSCALE_SIDECAR_CONTAINER = "elesim-tailscale"
TAILSCALE_STATE_DESTINATION = "/var/lib/tailscale"
_LEGACY_PINNED_TAILSCALE_IMAGE = re.compile(
    r"^tailscale/tailscale:v[0-9]+\.[0-9]+\.[0-9]+@sha256:[0-9a-f]{64}$"
)
_ROLLING_TAILSCALE_IMAGE = "tailscale/tailscale:stable"
_OFFICIAL_TAILSCALE_REPO_DIGEST = re.compile(
    r"^tailscale/tailscale@sha256:[0-9a-f]{64}$"
)
_DOCKER_IMAGE_ID = re.compile(r"^sha256:[0-9a-f]{64}$")
_RELEASE_KEY = re.compile(r"^[0-9a-f]{64}$")
_RELEASE_SOURCE_REVISION = re.compile(r"^(?:git-[0-9a-f]{40}|sha256-[0-9a-f]{64})$")
_RELEASE_ROLES = frozenset(("pilot", "sim", "ui"))
_MAX_RELEASE_MANIFEST_BYTES = 256 * 1024
_INSTANCE_SYSTEM_ID = re.compile(r"[a-z][a-z0-9_]{0,62}\Z")
_INSTANCE_ENDPOINT_ID = re.compile(r"[a-z][a-z0-9_-]{0,62}\Z")
_INSTANCE_ROLES = frozenset(("pilot", "sim", "ui"))
_INSTANCE_INSTALL_LABELS = (
    "io.elesim.system_id",
    "io.elesim.endpoint_id",
    "io.elesim.role",
)


def _scoped_container_name(install_uuid: str, service: str) -> str:
    """Derive an install-scoped name without importing setup modules.

    The uninstaller is copied as a stdlib-only bundle, so this intentionally
    mirrors the small pure naming rule instead of importing instance_identity.
    """

    scope = uuid.UUID(install_uuid).hex
    digest = hashlib.sha256(service.encode("utf-8")).hexdigest()
    prefix = f"elesim-{scope}-"
    readable_budget = 128 - len(prefix) - len(digest) - 1
    return f"{prefix}{service[:readable_budget]}-{digest}"


def _instance_service_key(system_id: str, endpoint_id: str) -> str:
    """Mirror the bounded instance Compose service identity rule."""

    if _INSTANCE_SYSTEM_ID.fullmatch(system_id) is None:
        raise UninstallSafetyError("scoped instance system_id label is invalid")
    if _INSTANCE_ENDPOINT_ID.fullmatch(endpoint_id) is None:
        raise UninstallSafetyError("scoped instance endpoint_id label is invalid")
    digest = hashlib.sha256(f"{system_id}\0{endpoint_id}".encode("utf-8")).hexdigest()
    return f"svc-{system_id[:24]}-{endpoint_id[:24]}-{digest}"[:128]


def _validate_scoped_instance_labels(
    *, name: str, labels: Mapping[str, object], install_uuid: str
) -> None:
    """Validate the identity labels used by ``compose.instances.yaml``.

    The instance Compose file is a separate aggregate and is not recorded in
    ``DockerOwnership``.  Its labels therefore provide the second, exact
    boundary needed before an uninstaller may remove one of its containers.
    """

    values = tuple(labels.get(key) for key in _INSTANCE_INSTALL_LABELS)
    if any(not isinstance(value, str) or not value for value in values):
        raise UninstallSafetyError(
            f"scoped instance container identity labels are missing: {name}"
        )
    system_id, endpoint_id, role = values
    if role not in _INSTANCE_ROLES:
        raise UninstallSafetyError(
            f"scoped instance container role label is unknown: {name}: {role!r}"
        )
    expected = _scoped_container_name(
        install_uuid, _instance_service_key(system_id, endpoint_id)
    )
    if name != expected:
        raise UninstallSafetyError(
            f"scoped instance container identity does not match its name: {name}"
        )


def _sim_container_name(ownership: DockerOwnership) -> str:
    return (
        SIM_CONTAINER
        if ownership.project == "elesim-runtime"
        else _scoped_container_name(ownership.install_uuid, "sim")
    )


def _tailscale_container_name(ownership: DockerOwnership) -> str:
    return (
        TAILSCALE_SIDECAR_CONTAINER
        if ownership.project == "elesim-runtime"
        else _scoped_container_name(ownership.install_uuid, "tailscale")
    )


class UninstallSafetyError(RuntimeError):
    """Raised before mutation when ownership cannot be proven."""


@dataclass(frozen=True)
class DockerObject:
    name: str
    object_id: str


@dataclass(frozen=True)
class TailscaleStateCleanup:
    """One exact sidecar bind whose root-created children need host ownership."""

    container: DockerObject
    image_id: str
    source: Path
    was_running: bool
    source_device: int
    source_inode: int


@dataclass(frozen=True)
class UninstallPlan:
    manifest: OwnershipManifest
    manifest_sha256: str
    purge_logs: bool
    purge_authority: bool
    remove_paths: tuple[OwnedPath, ...]
    remove_roots: tuple[Path, ...]
    preserve_paths: tuple[Path, ...]
    containers: tuple[DockerObject, ...]
    images: tuple[DockerObject, ...]
    viewer_cleanup: Path | None
    tailscale_state_cleanup: TailscaleStateCleanup | None
    remove_shell_path: bool
    warnings: tuple[str, ...]
    tombstone: Path


def _owned_release_image_ids(manifest: OwnershipManifest) -> tuple[str, ...]:
    """Read release image IDs as additional, install-owned Docker evidence.

    Scoped releases deliberately use content-addressed image IDs.  Their
    tags can disappear after a rebuild, so the install ownership manifest's
    ``local_images`` list is not sufficient for a complete host uninstall.
    This parser is kept stdlib-only because the copied host uninstaller cannot
    import the setup package's release module.
    """

    root = manifest.prefix_path / "releases"
    _ensure_no_symlink_ancestors(root, boundary=manifest.prefix_path)
    if not _lexists(root):
        return ()
    if root.is_symlink() or not root.is_dir():
        raise UninstallSafetyError(f"release registry is not a real directory: {root}")

    found: set[str] = set()
    for release_dir in sorted(root.iterdir(), key=lambda path: path.name):
        if release_dir.name == ".publish.lock":
            if release_dir.is_symlink() or not release_dir.is_file():
                raise UninstallSafetyError("release publication lock is unsafe")
            continue
        if _RELEASE_KEY.fullmatch(release_dir.name) is None:
            raise UninstallSafetyError(f"invalid release registry entry: {release_dir}")
        _ensure_no_symlink_ancestors(release_dir, boundary=manifest.prefix_path)
        if release_dir.is_symlink() or not release_dir.is_dir():
            raise UninstallSafetyError(f"invalid release directory: {release_dir}")
        source = release_dir / "manifest.json"
        _ensure_no_symlink_ancestors(source, boundary=manifest.prefix_path)
        try:
            info = source.lstat()
        except OSError as exc:
            raise UninstallSafetyError(f"release manifest is unavailable: {source}") from exc
        if (
            source.is_symlink()
            or not stat.S_ISREG(info.st_mode)
            or info.st_nlink != 1
            or info.st_size > _MAX_RELEASE_MANIFEST_BYTES
        ):
            raise UninstallSafetyError(f"release manifest is unsafe: {source}")
        try:
            raw = json.loads(source.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise UninstallSafetyError(f"release manifest is malformed: {source}") from exc
        if not isinstance(raw, Mapping):
            raise UninstallSafetyError(f"release manifest is not an object: {source}")
        fields = (
            "schema_version",
            "install_uuid",
            "source_revision",
            "platform",
            "role_images",
            "image_ids",
            "build_fingerprints",
            "runtime_data_digest",
        )
        if set(raw) != {*fields, "release_key"} or raw.get("schema_version") != 1:
            raise UninstallSafetyError(f"release manifest fields are invalid: {source}")
        if raw.get("install_uuid") != manifest.install_uuid:
            raise UninstallSafetyError(f"release belongs to another installation: {source}")
        if (
            not isinstance(raw.get("source_revision"), str)
            or _RELEASE_SOURCE_REVISION.fullmatch(raw["source_revision"]) is None
            or raw.get("platform") not in {"linux/amd64", "linux/arm64"}
            or not isinstance(raw.get("runtime_data_digest"), str)
            or _RELEASE_KEY.fullmatch(raw["runtime_data_digest"]) is None
        ):
            raise UninstallSafetyError(f"release manifest provenance is invalid: {source}")
        role_images = raw.get("role_images")
        image_ids = raw.get("image_ids")
        fingerprints = raw.get("build_fingerprints")
        if not all(isinstance(value, Mapping) for value in (role_images, image_ids, fingerprints)):
            raise UninstallSafetyError(f"release image fields are invalid: {source}")
        roles = set(role_images)
        if not roles or roles != set(image_ids) or roles != set(fingerprints) or not roles <= _RELEASE_ROLES:
            raise UninstallSafetyError(f"release image roles are invalid: {source}")
        install_hex = manifest.install_uuid.replace("-", "")
        for role in sorted(roles):
            fingerprint = fingerprints[role]
            image = role_images[role]
            image_id = image_ids[role]
            if (
                not isinstance(fingerprint, str)
                or _RELEASE_KEY.fullmatch(fingerprint) is None
                or image != f"elesim/{role}:{install_hex}-{fingerprint}"
                or not isinstance(image_id, str)
                or _DOCKER_IMAGE_ID.fullmatch(image_id) is None
            ):
                raise UninstallSafetyError(f"release image provenance is invalid: {source}")
            found.add(image_id)
        canonical_fields = {key: raw[key] for key in fields}
        expected_key = hashlib.sha256(
            (json.dumps(canonical_fields, sort_keys=True, separators=(",", ":"), ensure_ascii=True) + "\n").encode()
        ).hexdigest()
        if raw.get("release_key") != expected_key or release_dir.name != expected_key:
            raise UninstallSafetyError(f"release content key does not match: {source}")
    return tuple(sorted(found))


def plan_uninstall(
    manifest_path: Path | None = None,
    *,
    purge_logs: bool = True,
    purge_authority: bool = True,
    runner: CommandRunner | None = None,
) -> UninstallPlan:
    """Validate every deletion boundary and return an immutable plan."""

    source = default_manifest_path() if manifest_path is None else _canonical(manifest_path)
    try:
        manifest = OwnershipManifest.load(source)
    except OwnershipError as exc:
        raise UninstallSafetyError(str(exc)) from exc
    manifest_digest = sha256_file(source)
    _validate_install_roots(manifest)
    _validate_owned_paths(manifest)
    _validate_wrappers(manifest)
    _validate_systemd(manifest, runner=runner)
    viewer_cleanup = _owned_viewer_cleanup(manifest)

    containers: tuple[DockerObject, ...] = ()
    images: tuple[DockerObject, ...] = ()
    tailscale_state_cleanup: TailscaleStateCleanup | None = None
    if manifest.docker is not None:
        release_image_ids = _owned_release_image_ids(manifest)
        tailscale_state = _owned_tailscale_state_path(manifest)
        containers, images, tailscale_state_cleanup = _validate_docker(
            manifest.docker,
            runner=runner,
            tailscale_state=tailscale_state,
            require_tailscale_state=tailscale_state is not None,
            release_image_ids=release_image_ids,
        )
        if (
            tailscale_state_cleanup is not None
            and not tailscale_state_cleanup.was_running
        ):
            _require_host_removable_tree(
                tailscale_state_cleanup.source,
                reason=(
                    "Cannot safely pin the stopped Tailscale sidecar's existing bind "
                    "with the ownership helper. Start the sidecar and retry, or "
                    "restore exact state permissions on the host"
                ),
            )
            tailscale_state_cleanup = None
        if tailscale_state is not None and tailscale_state_cleanup is None:
            _require_host_removable_tree(
                tailscale_state,
                reason=(
                    "No exact EleSim Tailscale sidecar that wrote the state remains; "
                    "Docker-assisted ownership repair is unavailable"
                ),
            )

    warnings: list[str] = []
    remove_shell_path = False
    if manifest.shell is not None:
        shell_status = inspect_bash_path(
            Path(manifest.shell.bin_dir),
            bashrc=Path(manifest.shell.bashrc),
        )
        remove_shell_path = shell_status == "exact"
        if shell_status == "foreign":
            warnings.append(
                f"Preserving modified or foreign-owned PATH block: {manifest.shell.bashrc}"
            )

    preserve = [Path(value) for value in manifest.external_paths]
    if not purge_logs:
        preserve.extend(Path(value) for value in manifest.log_roots)
    if not purge_authority:
        preserve.extend(Path(value) for value in manifest.authority_roots)
    preserve_paths = _minimal_roots(preserve)

    remove_root_values = [Path(value) for value in manifest.managed_roots]
    if purge_logs:
        remove_root_values.extend(Path(value) for value in manifest.log_roots)
    if purge_authority:
        remove_root_values.extend(Path(value) for value in manifest.authority_roots)
    remove_roots = tuple(
        root
        for root in _minimal_roots(remove_root_values)
        if not _is_protected(root, preserve_paths)
    )
    remove_paths = tuple(
        entry
        for entry in manifest.owned_paths
        if not _is_protected(Path(entry.path), preserve_paths)
    )
    _validate_no_nested_mounts(
        manifest,
        remove_roots=remove_roots,
        remove_paths=remove_paths,
    )

    tombstone = _uninstall_state_root() / f"{manifest.install_uuid}.json"
    if _lexists(tombstone):
        raise UninstallSafetyError(f"uninstall tombstone already exists: {tombstone}")
    return UninstallPlan(
        manifest=manifest,
        manifest_sha256=manifest_digest,
        purge_logs=bool(purge_logs),
        purge_authority=bool(purge_authority),
        remove_paths=remove_paths,
        remove_roots=remove_roots,
        preserve_paths=preserve_paths,
        containers=containers,
        images=images,
        viewer_cleanup=viewer_cleanup,
        tailscale_state_cleanup=tailscale_state_cleanup,
        remove_shell_path=remove_shell_path,
        warnings=tuple(warnings),
        tombstone=tombstone,
    )


def execute_uninstall(
    plan: UninstallPlan,
    *,
    confirm_prefix: str | None = None,
    runner: CommandRunner | None = None,
) -> Path:
    """Execute a prevalidated ownership plan.

    ``confirm_prefix`` remains an internal compatibility guard for callers
    that already supply it.  The host CLI deliberately needs no memorized
    confirmation: locating and validating the exact manifest is the safety
    boundary.
    """

    if confirm_prefix is not None and confirm_prefix != plan.manifest.prefix:
        raise UninstallSafetyError(
            "--confirm-prefix does not match the exact prefix in the ownership manifest: "
            f"expected={plan.manifest.prefix}"
        )

    # Fail closed if the manifest or any ownership fact changed after preflight.
    current = plan_uninstall(
        plan.manifest.path,
        purge_logs=plan.purge_logs,
        purge_authority=plan.purge_authority,
        runner=runner,
    )
    if current.manifest_sha256 != plan.manifest_sha256:
        raise UninstallSafetyError("ownership manifest changed after preflight validation")
    if (
        current.remove_paths != plan.remove_paths
        or current.remove_roots != plan.remove_roots
        or current.containers != plan.containers
        or current.images != plan.images
        or current.viewer_cleanup != plan.viewer_cleanup
        or current.tailscale_state_cleanup != plan.tailscale_state_cleanup
        or current.remove_shell_path != plan.remove_shell_path
    ):
        raise UninstallSafetyError("installation ownership state changed after preflight validation")

    command_runner = _command_runner(runner)
    docker_ownership = current.manifest.docker
    if current.viewer_cleanup is not None:
        sim_container = next(
            (
                container
                for container in current.containers
                if (
                    docker_ownership is not None
                    and container.name == _sim_container_name(docker_ownership)
                )
            ),
            None,
        )
        if sim_container is not None:
            if docker_ownership is None:
                raise UninstallSafetyError(
                    "Docker ownership disappeared after validation"
                )
            stopped = command_runner(
                _docker_command(
                    docker_ownership,
                    ("docker", "container", "stop", sim_container.object_id),
                )
            )
            _require_command(stopped, action="stop Sim Viewer container")
        result = command_runner((str(current.viewer_cleanup),))
        _require_command(result, action="revoke X11 Viewer ACL")
    if current.remove_shell_path and current.manifest.shell is not None:
        result = unregister_bash_path(
            Path(current.manifest.shell.bin_dir),
            bashrc=Path(current.manifest.shell.bashrc),
        )
        if not result.changed:
            raise UninstallSafetyError("PATH block changed after validation; refusing removal")

    tailscale_cleanup = current.tailscale_state_cleanup
    for container in current.containers:
        if (
            tailscale_cleanup is not None
            and container.object_id == tailscale_cleanup.container.object_id
        ):
            continue
        if docker_ownership is None:
            raise UninstallSafetyError("Docker ownership disappeared after validation")
        result = command_runner(
            _docker_command(
                docker_ownership,
                ("docker", "container", "rm", "--force", container.object_id),
            )
        )
        _require_command(result, action=f"remove container {container.name}")
    if tailscale_cleanup is not None:
        if docker_ownership is None:
            raise UninstallSafetyError("Docker ownership disappeared after validation")
        _normalize_tailscale_state_ownership(
            tailscale_cleanup,
            ownership=docker_ownership,
            runner=command_runner,
        )
        # This exact sidecar removal is the ownership-repair commit point. It
        # is deliberately the last container mutation: before it succeeds PID
        # 1 can still be resumed; after it succeeds a rerun continues from the
        # preserved manifest and now host-removable state tree.
        removed = command_runner(
            _docker_command(
                docker_ownership,
                (
                    "docker",
                    "container",
                    "rm",
                    "--force",
                    tailscale_cleanup.container.object_id,
                ),
            )
        )
        if removed.returncode != 0:
            resumed = _resume_tailscale_sidecar(
                tailscale_cleanup,
                ownership=docker_ownership,
                runner=command_runner,
            )
            recovery = (
                ""
                if resumed.returncode == 0
                else f"; sidecar resume/start also failed: {resumed.stderr.strip()}"
            )
            raise UninstallSafetyError(
                "Tailscale sidecar removal failed: "
                f"{removed.stderr.strip()}{recovery}"
            )
    for image in current.images:
        if docker_ownership is None:
            raise UninstallSafetyError("Docker ownership disappeared after validation")
        result = command_runner(
            _docker_command(
                docker_ownership,
                ("docker", "image", "rm", image.name),
            )
        )
        _require_command(result, action=f"remove local image {image.name}")

    filesystem_protection = (*current.preserve_paths, current.manifest.path)
    for root in sorted(current.remove_roots, key=lambda path: len(path.parts), reverse=True):
        _remove_tree(root, protected=filesystem_protection)

    files = [entry for entry in current.remove_paths if entry.kind != "directory"]
    directories = [entry for entry in current.remove_paths if entry.kind == "directory"]
    for entry in sorted(files, key=lambda value: len(Path(value.path).parts), reverse=True):
        path = Path(entry.path)
        if _lexists(path):
            path.unlink()
    for wrapper in current.manifest.wrappers:
        path = Path(wrapper.path)
        if _lexists(path) and not _is_protected(path, current.preserve_paths):
            path.unlink()
    for entry in sorted(
        directories,
        key=lambda value: len(Path(value.path).parts),
        reverse=True,
    ):
        _rmdir_if_empty(Path(entry.path))

    for root in sorted(
        (Path(value) for value in current.manifest.created_roots),
        key=lambda path: len(path.parts),
        reverse=True,
    ):
        if root != current.manifest.prefix_path:
            _rmdir_if_empty(root)

    # Prepare the tombstone first, then publish it only after unlinking the
    # manifest.  These are intentionally the last two ownership mutations.
    tombstone_payload = {
        "schema_version": 1,
        "install_uuid": current.manifest.install_uuid,
        "edition": current.manifest.edition,
        "prefix": current.manifest.prefix,
        "completed_at": datetime.now(timezone.utc).isoformat(),
        "purged_logs": current.purge_logs,
        "purged_authority": current.purge_authority,
        "preserved_paths": [str(path) for path in current.preserve_paths],
    }
    temporary = _write_tombstone_temporary(
        current.tombstone,
        tombstone_payload,
    )
    current.manifest.path.unlink()
    os.replace(temporary, current.tombstone)
    return current.tombstone


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="elesim-uninstall",
        description="ownership manifest 기반의 안전한 EleSim 제거",
    )
    parser.add_argument(
        "--manifest",
        default=str(default_manifest_path()),
        help="install-ownership.json 경로",
    )
    parser.add_argument(
        "--keep-logs",
        action="store_true",
        help="기본 삭제되는 runtime text logs를 보존",
    )
    parser.add_argument(
        "--keep-authority",
        action="store_true",
        help="기본 삭제되는 operator SROS2 Authority를 보존",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        plan = plan_uninstall(
            Path(args.manifest),
            purge_logs=not bool(args.keep_logs),
            purge_authority=not bool(args.keep_authority),
        )
        tombstone = execute_uninstall(plan)
        print(f"EleSim removal complete. Tombstone: {tombstone}")
        return 0
    except (OSError, UninstallSafetyError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2


def _validate_install_roots(manifest: OwnershipManifest) -> None:
    prefix = manifest.prefix_path
    if prefix.is_symlink() or not prefix.is_dir():
        raise UninstallSafetyError(f"prefix is a symlink or not a directory: {prefix}")
    if str(prefix.resolve(strict=True)) != manifest.prefix_realpath:
        raise UninstallSafetyError("prefix realpath differs from the installation-time value")
    bin_dir = manifest.bin_path
    if _lexists(bin_dir):
        if bin_dir.is_symlink() or not bin_dir.is_dir():
            raise UninstallSafetyError(f"bin_dir is a symlink or not a directory: {bin_dir}")
        if str(bin_dir.resolve(strict=True)) != manifest.bin_dir_realpath:
            raise UninstallSafetyError("bin_dir realpath differs from the installation-time value")
    _ensure_no_symlink_ancestors(manifest.path, boundary=prefix)


def _validate_owned_paths(manifest: OwnershipManifest) -> None:
    prefix = manifest.prefix_path
    bin_dir = manifest.bin_path
    for entry in manifest.owned_paths:
        path = Path(entry.path)
        boundary = prefix if _within_or_equal(path, prefix) else bin_dir
        _ensure_no_symlink_ancestors(path, boundary=boundary)
        if not _lexists(path):
            continue
        mode = path.lstat().st_mode
        actual = (
            "file"
            if stat.S_ISREG(mode)
            else "directory"
            if stat.S_ISDIR(mode)
            else "symlink"
            if stat.S_ISLNK(mode)
            else "other"
        )
        if actual != entry.kind:
            raise UninstallSafetyError(
                f"owned path type changed: {path}: expected={entry.kind} actual={actual}"
            )
    for value in (
        *manifest.managed_roots,
        *manifest.log_roots,
        *manifest.authority_roots,
    ):
        path = Path(value)
        _ensure_no_symlink_ancestors(path, boundary=prefix)
        if _lexists(path) and (path.is_symlink() or not path.is_dir()):
            raise UninstallSafetyError(f"managed/preserved root is not a safe directory: {path}")


def _validate_wrappers(manifest: OwnershipManifest) -> None:
    for wrapper in manifest.wrappers:
        path = Path(wrapper.path)
        _ensure_no_symlink_ancestors(path, boundary=manifest.bin_path)
        if not _lexists(path):
            continue
        mode = path.lstat().st_mode
        if not stat.S_ISREG(mode) or stat.S_ISLNK(mode):
            raise UninstallSafetyError(f"wrapper is not a regular file: {path}")
        if sha256_file(path) != wrapper.sha256:
            raise UninstallSafetyError(
                f"wrapper changed after installation; refusing removal: {path}"
            )


def _owned_viewer_cleanup(manifest: OwnershipManifest) -> Path | None:
    """Resolve only the generated, manifest-owned Viewer ACL cleanup command."""

    expected = manifest.bin_path / VIEWER_CLEANUP_WRAPPER
    ownership = next(
        (
            wrapper
            for wrapper in manifest.wrappers
            if Path(wrapper.path) == expected
        ),
        None,
    )
    cache_roots = tuple(
        manifest.prefix_path / relative.parent
        for relative in VIEWER_STATE_RELATIVE_PATHS
    )
    states = tuple(
        manifest.prefix_path / relative
        for relative in VIEWER_STATE_RELATIVE_PATHS
    )
    for state in states:
        _ensure_no_symlink_ancestors(state, boundary=manifest.prefix_path)
        if _lexists(state):
            mode = state.lstat().st_mode
            if stat.S_ISLNK(mode) or not stat.S_ISREG(mode):
                raise UninstallSafetyError(
                    f"EleSim xhost state is not a regular file: {state}"
                )
    if ownership is None:
        if any(_lexists(state) for state in states):
            raise UninstallSafetyError(
                "EleSim-owned X11 Viewer ACL state remains, but the exact cleanup "
                "wrapper is not in the ownership manifest. First use elesim-update "
                "to restore the wrapper, or use elesim-down from the same "
                "installation to revoke it"
            )
        return None
    missing_roots = tuple(
        root for root in cache_roots if str(root) not in manifest.managed_roots
    )
    if missing_roots:
        raise UninstallSafetyError(
            "exact managed root for X11 Viewer cleanup state is not in the ownership "
            "manifest: "
            + ", ".join(str(root) for root in missing_roots)
        )
    if not _lexists(expected):
        raise UninstallSafetyError(
            "ownership manifest lacks the X11 Viewer cleanup wrapper. "
            "use elesim-update to restore the exact wrapper, then retry: "
            f"{expected}"
        )
    mode = expected.lstat().st_mode
    if not stat.S_ISREG(mode) or stat.S_ISLNK(mode) or not os.access(expected, os.X_OK):
        raise UninstallSafetyError(
            f"X11 Viewer cleanup wrapper is not an executable regular file: {expected}"
        )
    return expected


def _validate_systemd(
    manifest: OwnershipManifest,
    *,
    runner: CommandRunner | None,
) -> None:
    if not manifest.systemd_units:
        return
    command_runner = _command_runner(runner)
    for unit in manifest.systemd_units:
        destination = Path(unit.destination)
        result = command_runner(
            (
                "systemctl",
                "show",
                unit.name,
                "--property=LoadState",
                "--property=ActiveState",
                "--property=FragmentPath",
                "--no-pager",
            )
        )
        if result.returncode != 0:
            raise UninstallSafetyError(
                f"Cannot determine systemd status: {unit.name}: {result.stderr.strip()}"
            )
        values = _key_values(result.stdout)
        load_state = values.get("LoadState", "not-found")
        fragment_text = values.get("FragmentPath", "").strip()
        fragment = None if not fragment_text else _canonical(Path(fragment_text))
        installed = _lexists(destination) or load_state != "not-found" or fragment is not None
        active = values.get("ActiveState", "inactive") not in {"inactive", "failed", "dead"}
        if installed or active:
            exact_copy = (
                _lexists(destination)
                and not destination.is_symlink()
                and destination.is_file()
                and sha256_file(destination) == unit.sha256
                and (fragment is None or fragment == destination)
            )
            if not exact_copy:
                raise UninstallSafetyError(
                    f"{unit.name} has a foreign or modified systemd unit with the same name. "
                    "EleSim will not remove this file. Inspect FragmentPath and the unit contents to "
                    f"resolve the conflict: fragment={fragment_text or '-'} "
                    f"expected={unit.destination}"
                )
            raise UninstallSafetyError(
                f"{unit.name} is installed or running in systemd. Run the following exact command first:\n"
                f"  sudo systemctl disable --now {unit.name}\n"
                f"  sudo rm -- {unit.destination}\n"
                "  sudo systemctl daemon-reload"
            )


def _validate_no_nested_mounts(
    manifest: OwnershipManifest,
    *,
    remove_roots: Sequence[Path],
    remove_paths: Sequence[OwnedPath],
) -> None:
    mounts = _mount_points()
    prefix = manifest.prefix_path
    bin_dir = manifest.bin_path
    for mount in mounts:
        for root in remove_roots:
            if _within_or_equal(mount, root):
                raise UninstallSafetyError(
                    f"recursive removal boundary contains a mount/bind mount: root={root} "
                    f"mount={mount}. unmount it first."
                )
        if mount in {prefix, bin_dir}:
            continue
        if not (
            _within_or_equal(mount, prefix)
            or _within_or_equal(mount, bin_dir)
        ):
            continue
        for entry in remove_paths:
            if _within_or_equal(Path(entry.path), mount):
                raise UninstallSafetyError(
                    "exact removal path is inside a nested mount: "
                    f"path={entry.path} mount={mount}. unmount it first."
                )
        for wrapper in manifest.wrappers:
            if _within_or_equal(Path(wrapper.path), mount):
                raise UninstallSafetyError(
                    "wrapper is inside a nested mount: "
                    f"path={wrapper.path} mount={mount}. unmount it first."
                )


def _mount_points() -> tuple[Path, ...]:
    source = Path("/proc/self/mountinfo")
    try:
        lines = source.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise UninstallSafetyError(
            f"Cannot determine mount boundary: {source}: {exc}"
        ) from exc
    mounts: set[Path] = set()
    for line in lines:
        fields = line.split()
        if len(fields) < 6 or "-" not in fields:
            raise UninstallSafetyError("/proc/self/mountinfo has an invalid format")
        value = fields[4]
        for escaped, literal in (
            (r"\040", " "),
            (r"\011", "\t"),
            (r"\012", "\n"),
            (r"\134", "\\"),
        ):
            value = value.replace(escaped, literal)
        mounts.add(_canonical(Path(value)))
    return tuple(sorted(mounts, key=str))


def _owned_tailscale_state_path(manifest: OwnershipManifest) -> Path | None:
    ownership = manifest.docker
    if ownership is None or _tailscale_container_name(ownership) not in ownership.containers:
        return None
    secrets_root = manifest.prefix_path / "secrets"
    if str(secrets_root) not in manifest.managed_roots:
        raise UninstallSafetyError(
            "The exact managed root for Tailscale sidecar state is not in the ownership manifest"
        )
    state = secrets_root / "tailscale"
    protected = tuple(
        Path(value)
        for value in (
            *manifest.external_paths,
            *manifest.log_roots,
            *manifest.authority_roots,
        )
    )
    if any(
        _within_or_equal(state, path) or _within_or_equal(path, state)
        for path in protected
    ):
        raise UninstallSafetyError(
            "Tailscale sidecar state overlaps a preserved/external boundary; ownership cannot be restored"
        )
    _ensure_no_symlink_ancestors(state, boundary=manifest.prefix_path)
    if not _lexists(state):
        return None
    mode = state.lstat().st_mode
    if stat.S_ISLNK(mode) or not stat.S_ISDIR(mode):
        raise UninstallSafetyError(
            f"Tailscale sidecar state is not a real directory: {state}"
        )
    return state


def _validate_tailscale_state_container(
    payload: Mapping[str, object],
    *,
    container: DockerObject,
    state_path: Path,
    runner: CommandRunner,
) -> TailscaleStateCleanup:
    config = payload.get("Config", {})
    if not isinstance(config, Mapping):
        raise UninstallSafetyError("Tailscale sidecar config is invalid")
    if _labels(payload).get("com.docker.compose.service") != "tailscale":
        raise UninstallSafetyError(
            "Compose service for the elesim-tailscale container is not tailscale"
        )
    image_ref = str(config.get("Image", ""))
    image_id = str(payload.get("Image", ""))
    pinned_image = _LEGACY_PINNED_TAILSCALE_IMAGE.fullmatch(image_ref) is not None
    if not pinned_image and image_ref != _ROLLING_TAILSCALE_IMAGE:
        raise UninstallSafetyError(
            "Tailscale sidecar does not use a supported official image"
        )
    if not _DOCKER_IMAGE_ID.fullmatch(image_id):
        raise UninstallSafetyError("Tailscale sidecar image ID is invalid")
    image_result = runner(("docker", "image", "inspect", image_id))
    if image_result.returncode != 0:
        raise UninstallSafetyError(
            "cannot inspect the Tailscale sidecar immutable image: "
            + image_result.stderr.strip()
        )
    image_payload = _inspect_object(
        image_result.stdout,
        kind="image",
        name=image_id,
    )
    repo_digests = image_payload.get("RepoDigests", [])
    expected_digest = image_ref.rsplit("@", 1)[-1] if pinned_image else ""
    official_digest = isinstance(repo_digests, list) and any(
        _OFFICIAL_TAILSCALE_REPO_DIGEST.fullmatch(str(value))
        for value in repo_digests
    )
    digest_matches = (
        f"tailscale/tailscale@{expected_digest}" in repo_digests
        if pinned_image and isinstance(repo_digests, list)
        else official_digest
    )
    if str(image_payload.get("Id", "")) != image_id or not digest_matches:
        raise UninstallSafetyError(
            "Tailscale sidecar image ID or official repository digest differs"
        )
    if config.get("Entrypoint") != ["tailscaled"] or config.get("User", "") != "":
        raise UninstallSafetyError("Tailscale sidecar runtime identity differs from generated configuration")
    if config.get("Cmd") != [
        "--statedir=/var/lib/tailscale",
        "--socket=/tmp/tailscaled.sock",
        "--tun=tailscale0",
    ]:
        raise UninstallSafetyError("Tailscale sidecar daemon arguments differ from generated configuration")

    mounts = payload.get("Mounts", [])
    if not isinstance(mounts, list) or len(mounts) != 1 or not isinstance(
        mounts[0], Mapping
    ):
        raise UninstallSafetyError(
            "Tailscale sidecar must have exactly one state bind mount"
        )
    mount = mounts[0]
    source = str(mount.get("Source", ""))
    try:
        source_path = _canonical(Path(source))
    except (OSError, ValueError) as exc:
        raise UninstallSafetyError("Tailscale state bind source is invalid") from exc
    if (
        str(mount.get("Type", "")) != "bind"
        or source_path != state_path
        or str(mount.get("Destination", "")) != TAILSCALE_STATE_DESTINATION
        or mount.get("RW") is not True
    ):
        raise UninstallSafetyError(
            "Tailscale sidecar state bind differs from the install-owned exact boundary: "
            f"source={source!r} destination={mount.get('Destination')!r}"
        )
    state = payload.get("State", {})
    was_running = isinstance(state, Mapping) and state.get("Running") is True
    source_stat = state_path.lstat()
    return TailscaleStateCleanup(
        container=container,
        image_id=image_id,
        source=state_path,
        was_running=was_running,
        source_device=int(source_stat.st_dev),
        source_inode=int(source_stat.st_ino),
    )


def _require_host_removable_tree(root: Path, *, reason: str) -> None:
    """Prove a tree can be traversed/deleted without following symlinks."""

    if not _lexists(root):
        return
    mode = root.lstat().st_mode
    if stat.S_ISLNK(mode) or not stat.S_ISDIR(mode):
        raise UninstallSafetyError(f"{reason}: Unsupported path type: {root}")
    if not os.access(root, os.R_OK | os.W_OK | os.X_OK):
        raise UninstallSafetyError(f"{reason}: {root}")
    try:
        entries = tuple(os.scandir(root))
    except OSError as exc:
        raise UninstallSafetyError(f"{reason}: {root}: {exc}") from exc
    for entry in entries:
        if entry.is_dir(follow_symlinks=False):
            _require_host_removable_tree(Path(entry.path), reason=reason)


def _normalize_tailscale_state_ownership(
    cleanup: TailscaleStateCleanup,
    *,
    ownership: DockerOwnership,
    runner: CommandRunner,
) -> None:
    """Quiesce an already-mounted sidecar bind and make it host-removable.

    A new ``--mount``/``--volumes-from`` helper would resolve the host source
    path again after validation, permitting a symlink-swap escape.  ``exec``
    instead reuses the running container's existing mount namespace.  PID 1 is
    stopped before walking the tree and remains stopped until Docker removes
    the sidecar, so tailscaled cannot create new root-owned children between
    normalization and removal.
    """

    if not cleanup.was_running:
        raise UninstallSafetyError(
            "Tailscale ownership helper requires an already-running owned sidecar"
        )

    directory_flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
    try:
        source_fd = os.open(cleanup.source, directory_flags)
    except OSError as exc:
        raise UninstallSafetyError(
            f"cannot open Tailscale state root without following symlinks: {cleanup.source}"
        ) from exc
    sentinel_name = f".elesim-uninstall-{secrets.token_hex(16)}"
    sentinel_value = secrets.token_hex(32)
    sentinel_created = False
    try:
        source_stat = os.fstat(source_fd)
        if (
            not stat.S_ISDIR(source_stat.st_mode)
            or int(source_stat.st_dev) != cleanup.source_device
            or int(source_stat.st_ino) != cleanup.source_inode
        ):
            raise UninstallSafetyError("Tailscale state inode changed after validation")
        try:
            sentinel_fd = os.open(
                sentinel_name,
                os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
                0o600,
                dir_fd=source_fd,
            )
        except OSError as exc:
            raise UninstallSafetyError(
                "cannot create Tailscale state mount identity token"
            ) from exc
        try:
            os.write(sentinel_fd, sentinel_value.encode("ascii"))
            os.fsync(sentinel_fd)
        finally:
            os.close(sentinel_fd)
        sentinel_created = True

        script = (
            "state=/var/lib/tailscale; "
            "resume=1; "
            "trap 'test \"$resume\" = 0 || kill -CONT 1 >/dev/null 2>&1 || true' EXIT; "
            'if ! test -f "$state/$1" || test -L "$state/$1" || '
            'test "$(cat "$state/$1")" != "$2"; then '
            'echo "Tailscale state mount identity mismatch" >&2; exit 70; fi; '
            "kill -STOP 1; "
            'if ! test -d "$state" || test -L "$state"; then '
            'echo "refusing invalid Tailscale state root" >&2; exit 70; fi; '
            'if find "$state" -xdev ! -type d ! -type f -print -quit | grep -q .; then '
            'echo "refusing non-regular Tailscale state" >&2; exit 70; fi; '
            'if find "$state" -xdev -type f -links +1 -print -quit | grep -q .; then '
            'echo "refusing hard-linked Tailscale state" >&2; exit 70; fi; '
            'find "$state" -xdev -type d -exec chown "$3:$4" {} +; '
            'find "$state" -xdev -type f -exec chown "$3:$4" {} +; '
            'find "$state" -xdev -type d -exec chmod u+rwx {} +; '
            'find "$state" -xdev -type f -exec chmod u+rw {} +; '
            "resume=0"
        )
        normalized = runner(
            _docker_command(
                ownership,
                (
                    "docker",
                    "container",
                    "exec",
                    "--user",
                    "0:0",
                    cleanup.container.object_id,
                    "/bin/sh",
                    "-ec",
                    script,
                    "elesim-state-cleanup",
                    sentinel_name,
                    sentinel_value,
                    str(os.getuid()),
                    str(os.getgid()),
                ),
            )
        )
        if normalized.returncode != 0:
            raise UninstallSafetyError(
                "Tailscale state ownership restoration failed: "
                + normalized.stderr.strip()
            )
        os.unlink(sentinel_name, dir_fd=source_fd)
        sentinel_created = False
        source_stat = os.fstat(source_fd)
        if (
            not stat.S_ISDIR(source_stat.st_mode)
            or int(source_stat.st_dev) != cleanup.source_device
            or int(source_stat.st_ino) != cleanup.source_inode
        ):
            raise UninstallSafetyError(
                "Tailscale state inode changed during ownership restoration"
            )
        try:
            current_path_stat = cleanup.source.lstat()
        except OSError as exc:
            raise UninstallSafetyError(
                "Tailscale state path disappeared during ownership restoration"
            ) from exc
        if (
            stat.S_ISLNK(current_path_stat.st_mode)
            or not stat.S_ISDIR(current_path_stat.st_mode)
            or int(current_path_stat.st_dev) != cleanup.source_device
            or int(current_path_stat.st_ino) != cleanup.source_inode
        ):
            raise UninstallSafetyError(
                "Tailscale state path/inode changed during ownership restoration"
            )
        _require_host_removable_tree(
            cleanup.source,
            reason=(
                "Docker ownership restoration still cannot safely "
                "remove Tailscale state"
            ),
        )
    except BaseException as exc:
        resumed = _resume_tailscale_sidecar(cleanup, ownership=ownership, runner=runner)
        if resumed.returncode != 0:
            raise UninstallSafetyError(
                f"{exc}; sidecar resume/start also failed: {resumed.stderr.strip()}"
            ) from exc
        raise
    finally:
        if sentinel_created:
            try:
                os.unlink(sentinel_name, dir_fd=source_fd)
            except FileNotFoundError:
                pass
        os.close(source_fd)


def _resume_tailscale_sidecar(
    cleanup: TailscaleStateCleanup,
    *,
    ownership: DockerOwnership,
    runner: CommandRunner,
) -> subprocess.CompletedProcess[str]:
    """Resume PID 1 after a failed in-container ownership transaction."""

    resumed = runner(
        _docker_command(
            ownership,
            (
                "docker",
                "container",
                "kill",
                "--signal",
                "CONT",
                cleanup.container.object_id,
            ),
        )
    )
    if resumed.returncode == 0:
        return resumed
    return runner(
        _docker_command(
            ownership,
            (
                "docker",
                "container",
                "start",
                cleanup.container.object_id,
            ),
        )
    )


def _validate_docker(
    ownership: DockerOwnership,
    *,
    runner: CommandRunner | None,
    tailscale_state: Path | None = None,
    require_tailscale_state: bool = False,
    release_image_ids: Sequence[str] = (),
) -> tuple[
    tuple[DockerObject, ...],
    tuple[DockerObject, ...],
    TailscaleStateCleanup | None,
]:
    raw_runner = _command_runner(runner)

    def command_runner(argv: Sequence[str]) -> subprocess.CompletedProcess[str]:
        return raw_runner(_docker_command(ownership, argv))

    info = command_runner(("docker", "info", "--format", "{{.ServerVersion}}"))
    if info.returncode != 0:
        raise UninstallSafetyError(
            "cannot connect to Docker daemon to validate container/image ownership: "
            + info.stderr.strip()
        )
    if ownership.engine_id:
        identity = command_runner(("docker", "info", "--format", "{{.ID}}"))
        if identity.returncode != 0 or identity.stdout.strip() != ownership.engine_id:
            observed = identity.stdout.strip() or "unavailable"
            raise UninstallSafetyError(
                "the Docker Engine pinned at installation differs from the current daemon: "
                f"expected={ownership.engine_id!r} actual={observed!r}"
            )
    listed_containers = command_runner(
        ("docker", "container", "ls", "--all", "--format", "{{.Names}}")
    )
    if listed_containers.returncode != 0:
        raise UninstallSafetyError(
            "Cannot determine the Docker container list: "
            + listed_containers.stderr.strip()
        )
    container_names = {
        value.strip() for value in listed_containers.stdout.splitlines() if value.strip()
    }
    labeled_containers = command_runner(
        (
            "docker",
            "container",
            "ls",
            "--all",
            "--filter",
            f"label=com.docker.compose.project={ownership.project}",
            "--filter",
            f"label={DOCKER_INSTALL_UUID_LABEL}={ownership.install_uuid}",
            "--format",
            "{{.Names}}",
        )
    )
    if labeled_containers.returncode != 0:
        raise UninstallSafetyError(
            "Cannot determine the EleSim ownership-label container list: "
            + labeled_containers.stderr.strip()
        )
    labeled_names = {
        value.strip() for value in labeled_containers.stdout.splitlines() if value.strip()
    }
    unlisted = sorted(labeled_names - set(ownership.containers))
    if unlisted:
        raise UninstallSafetyError(
            "Containers from this installation that are not in the manifest are "
            "running or remain. Stop them first: "
            + ", ".join(unlisted)
        )

    containers: list[DockerObject] = []
    tailscale_cleanup: TailscaleStateCleanup | None = None
    expected_compose = str(Path(ownership.compose_file).resolve(strict=False))
    alternate_compose = str(
        Path(ownership.compose_file)
        .with_name("compose.instances.yaml")
        .resolve(strict=False)
    )
    expected_configs = {expected_compose, alternate_compose}
    for name in ownership.containers:
        if name not in container_names:
            continue
        result = command_runner(("docker", "container", "inspect", name))
        if result.returncode != 0:
            raise UninstallSafetyError(
                f"cannot inspect listed Docker container: {name}: "
                + result.stderr.strip()
            )
        payload = _inspect_object(result.stdout, kind="container", name=name)
        labels = _labels(payload)
        project = labels.get("com.docker.compose.project", "")
        install_uuid = labels.get(DOCKER_INSTALL_UUID_LABEL, "")
        config_files = labels.get("com.docker.compose.project.config_files", "")
        configs = {
            str(Path(value.strip()).resolve(strict=False))
            for value in config_files.split(",")
            if value.strip()
        }
        if expected_compose not in configs and alternate_compose not in configs:
            raise UninstallSafetyError(
                f"fixed container name belongs to another installation: {name}: "
                f"project={project!r} install_uuid={install_uuid!r} "
                f"compose={config_files!r}"
            )
        if not configs.issubset(expected_configs):
            raise UninstallSafetyError(
                f"fixed container uses an unauthorized Compose file: "
                f"{name}: compose={config_files!r}"
            )
        if alternate_compose in configs:
            expected_scoped_project = (
                f"elesim-runtime-{uuid.UUID(ownership.install_uuid).hex}"
            )
            if ownership.project != expected_scoped_project:
                raise UninstallSafetyError(
                    "Scoped instance Compose cannot be used with a legacy Docker project: "
                    f"{name}"
                )
            _validate_scoped_instance_labels(
                name=name, labels=labels, install_uuid=ownership.install_uuid
            )
        elif any(key in labels for key in _INSTANCE_INSTALL_LABELS):
            raise UninstallSafetyError(
                "scoped instance identity labels require compose.instances.yaml: "
                f"{name}"
            )
        if (
            project != ownership.project
            or install_uuid != ownership.install_uuid
        ):
            raise UninstallSafetyError(
                f"fixed container name belongs to another installation: {name}: "
                f"project={project!r} install_uuid={install_uuid!r} "
                f"compose={config_files!r}"
            )
        object_id = str(payload.get("Id", ""))
        if not object_id:
            raise UninstallSafetyError(f"Docker container ID is empty: {name}")
        container = DockerObject(name=name, object_id=object_id)
        containers.append(container)
        if (
            require_tailscale_state
            and name == _tailscale_container_name(ownership)
        ):
            if tailscale_state is None:
                raise UninstallSafetyError(
                    "Tailscale sidecar is present in the manifest but has no install-owned state boundary"
                )
            tailscale_cleanup = _validate_tailscale_state_container(
                payload,
                container=container,
                state_path=tailscale_state,
                runner=command_runner,
            )

    listed_images = command_runner(
        ("docker", "image", "ls", "--all", "--format", "{{.Repository}}:{{.Tag}}")
    )
    if listed_images.returncode != 0:
        raise UninstallSafetyError(
            "cannot determine Docker image list: " + listed_images.stderr.strip()
        )
    image_names = {
        value.strip() for value in listed_images.stdout.splitlines() if value.strip()
    }
    images: list[DockerObject] = []
    for name in ownership.local_images:
        if name not in image_names:
            continue
        result = command_runner(("docker", "image", "inspect", name))
        if result.returncode != 0:
            raise UninstallSafetyError(
                f"cannot inspect listed Docker image: {name}: "
                + result.stderr.strip()
            )
        payload = _inspect_object(result.stdout, kind="image", name=name)
        labels = _labels(payload)
        project = labels.get("com.docker.compose.project", "")
        install_uuid = labels.get(DOCKER_INSTALL_UUID_LABEL, "")
        if project != ownership.project or install_uuid != ownership.install_uuid:
            raise UninstallSafetyError(
                f"local image tag belongs to another installation: {name}: "
                f"project={project!r} install_uuid={install_uuid!r}"
            )
        object_id = str(payload.get("Id", ""))
        if not object_id:
            raise UninstallSafetyError(f"Docker image ID is empty: {name}")
        images.append(DockerObject(name=name, object_id=object_id))

    # Release manifests retain image IDs even after a rebuild has removed the
    # corresponding immutable tag.  Inspect only those exact IDs; never list
    # or prune unrelated images.  Missing historical IDs are already gone and
    # therefore need no mutation, matching the tag-based path above.
    for image_id in sorted(set(release_image_ids)):
        if not _DOCKER_IMAGE_ID.fullmatch(image_id):
            raise UninstallSafetyError(f"release image ID is invalid: {image_id}")
        result = command_runner(("docker", "image", "inspect", image_id))
        if result.returncode != 0:
            continue
        payload = _inspect_object(result.stdout, kind="image", name=image_id)
        if str(payload.get("Id", "")) != image_id:
            raise UninstallSafetyError(
                f"release image ID differs from inspect result: {image_id}"
            )
        labels = _labels(payload)
        if (
            labels.get("com.docker.compose.project") != ownership.project
            or labels.get(DOCKER_INSTALL_UUID_LABEL) != ownership.install_uuid
        ):
            raise UninstallSafetyError(
                f"release image belongs to another installation or upstream: {image_id}"
            )
        if not any(image.name == image_id for image in images):
            images.append(DockerObject(name=image_id, object_id=image_id))
    return tuple(containers), tuple(images), tailscale_cleanup


def validate_docker_ownership(
    ownership: DockerOwnership,
    *,
    runner: CommandRunner | None = None,
) -> tuple[tuple[DockerObject, ...], tuple[DockerObject, ...]]:
    """Prove exact Docker labels/Compose boundaries without mutating objects."""

    containers, images, _cleanup = _validate_docker(
        ownership.validate(),
        runner=runner,
        require_tailscale_state=False,
    )
    return containers, images


def _docker_command(
    ownership: DockerOwnership,
    argv: Sequence[str],
) -> tuple[str, ...]:
    values = tuple(str(value) for value in argv)
    if not values or values[0] != "docker":
        raise UninstallSafetyError("internal Docker command is malformed")
    if not ownership.context:
        return values
    return ("docker", "--context", ownership.context, *values[1:])


def _inspect_object(stdout: str, *, kind: str, name: str) -> Mapping[str, object]:
    try:
        value = json.loads(stdout)
        if not isinstance(value, list) or len(value) != 1 or not isinstance(value[0], Mapping):
            raise ValueError("expected one object")
        return value[0]
    except (json.JSONDecodeError, ValueError) as exc:
        raise UninstallSafetyError(f"Docker {kind} inspect response is invalid: {name}") from exc


def _labels(payload: Mapping[str, object]) -> Mapping[str, str]:
    config = payload.get("Config", {})
    if not isinstance(config, Mapping):
        return {}
    labels = config.get("Labels", {})
    if not isinstance(labels, Mapping):
        return {}
    return {str(key): str(value) for key, value in labels.items() if value is not None}


def _remove_tree(root: Path, *, protected: tuple[Path, ...]) -> None:
    if not _lexists(root) or _is_protected(root, protected):
        return
    mode = root.lstat().st_mode
    if stat.S_ISLNK(mode) or stat.S_ISREG(mode):
        root.unlink()
        return
    if not stat.S_ISDIR(mode):
        raise UninstallSafetyError(f"Unsupported managed path type: {root}")
    for entry in os.scandir(root):
        path = Path(entry.path)
        if _is_protected(path, protected):
            continue
        if entry.is_dir(follow_symlinks=False):
            _remove_tree(path, protected=protected)
        else:
            path.unlink()
    _rmdir_if_empty(root)


def _write_tombstone_temporary(
    destination: Path,
    payload: Mapping[str, object],
) -> Path:
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        dir=destination.parent,
        prefix=f".{destination.name}.",
        delete=False,
    ) as handle:
        handle.write(json.dumps(payload, ensure_ascii=False, indent=2) + "\n")
        temporary = Path(handle.name)
    temporary.chmod(0o600)
    return temporary


def _uninstall_state_root() -> Path:
    configured = os.environ.get("XDG_STATE_HOME", "").strip()
    base = Path(configured).expanduser() if configured else Path.home() / ".local/state"
    return _canonical(base / "elesim/uninstall")


def _rmdir_if_empty(path: Path) -> None:
    try:
        path.rmdir()
    except FileNotFoundError:
        return
    except OSError:
        # A non-empty directory contains data not covered by the manifest and
        # is deliberately preserved.
        return


def _ensure_no_symlink_ancestors(path: Path, *, boundary: Path) -> None:
    if not _within_or_equal(path, boundary):
        raise UninstallSafetyError(f"path is outside the validation boundary: {path}")
    current = path.parent
    while _within_or_equal(current, boundary):
        if _lexists(current) and current.is_symlink():
            raise UninstallSafetyError(f"path ancestor is a symlink: {current}")
        if current == boundary:
            break
        current = current.parent


def _minimal_roots(paths: Sequence[Path]) -> tuple[Path, ...]:
    ordered = sorted({_canonical(path) for path in paths}, key=lambda path: len(path.parts))
    result: list[Path] = []
    for path in ordered:
        if not any(_within_or_equal(path, parent) for parent in result):
            result.append(path)
    return tuple(result)


def _is_protected(path: Path, roots: Sequence[Path]) -> bool:
    return any(_within_or_equal(path, root) for root in roots)


def _within_or_equal(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def _canonical(path: Path) -> Path:
    return Path(os.path.abspath(os.fspath(path.expanduser())))


def _lexists(path: Path) -> bool:
    return os.path.lexists(path)


def _key_values(text: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in text.splitlines():
        key, separator, value = line.partition("=")
        if separator:
            result[key] = value
    return result


def _command_runner(runner: CommandRunner | None) -> CommandRunner:
    if runner is not None:
        return runner

    def run(command: Sequence[str]) -> subprocess.CompletedProcess[str]:
        try:
            return subprocess.run(
                tuple(command),
                check=False,
                capture_output=True,
                text=True,
            )
        except FileNotFoundError as exc:
            return subprocess.CompletedProcess(
                tuple(command),
                127,
                stdout="",
                stderr=str(exc),
            )

    return run


def _require_command(
    result: subprocess.CompletedProcess[str],
    *,
    action: str,
) -> None:
    if result.returncode != 0:
        raise UninstallSafetyError(f"{action} failed: {result.stderr.strip()}")


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "DockerObject",
    "UninstallPlan",
    "UninstallSafetyError",
    "execute_uninstall",
    "main",
    "plan_uninstall",
    "validate_docker_ownership",
]
