"""Install ownership records used by the host-only safe uninstaller.

The manifest is deliberately independent from :mod:`elesim_setup.state`.
Runtime state is mutable and describes how to run EleSim; this file records
which host resources one exact installation is allowed to remove.
"""

from __future__ import annotations

import hashlib
import fcntl
import json
import os
import re
import shlex
import shutil
import stat
import tempfile
import uuid
import threading
from contextlib import contextmanager
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Iterator, Mapping, Sequence



OWNERSHIP_SCHEMA_VERSION = 1
OWNERSHIP_MANIFEST_NAME = "install-ownership.json"
DOCKER_INSTALL_UUID_LABEL = "io.elesim.install_uuid"
DOCKER_BUILD_FINGERPRINT_LABEL = "io.elesim.build_fingerprint"
_INSTALL_EDITIONS = frozenset({"general", "developer"})
_PATH_KINDS = frozenset({"file", "directory", "symlink"})
_DOCKER_NAME = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9_.-]{0,127}$")
_LOCAL_IMAGE = re.compile(r"^elesim/[a-z0-9][a-z0-9_.-]{0,127}:local$")
_INSTALL_IMAGE = re.compile(
    r"^elesim/[a-z0-9][a-z0-9_.-]{0,127}:([0-9a-f]{32})-([0-9a-f]{64})$"
)

_OWNERSHIP_LOCKS: dict[str, threading.Lock] = {}
_OWNERSHIP_LOCKS_GUARD = threading.Lock()


def _scoped_project_name(install_uuid: str) -> str:
    """Derive the scoped project without importing the setup package.

    ``ownership.py`` is copied into the stdlib-only host uninstaller bundle,
    where sibling setup modules are intentionally absent.
    """

    return f"elesim-runtime-{uuid.UUID(install_uuid).hex}"


def _image_belongs_to_install(image: str, install_uuid: str) -> bool:
    """Accept legacy local tags and only matching immutable install tags."""

    if _LOCAL_IMAGE.fullmatch(image):
        return True
    match = _INSTALL_IMAGE.fullmatch(image)
    return match is not None and match.group(1) == install_uuid.replace("-", "")


class OwnershipError(ValueError):
    """Raised when a manifest would create an unsafe deletion boundary."""


@dataclass(frozen=True)
class OwnedPath:
    path: str
    kind: str

    @classmethod
    def from_path(cls, path: Path) -> "OwnedPath":
        destination = _canonical(path)
        mode = destination.lstat().st_mode
        if stat.S_ISREG(mode):
            kind = "file"
        elif stat.S_ISDIR(mode):
            kind = "directory"
        elif stat.S_ISLNK(mode):
            kind = "symlink"
        else:
            raise OwnershipError(f"Unsupported installation artifact type: {destination}")
        return cls(str(destination), kind)


@dataclass(frozen=True)
class WrapperOwnership:
    path: str
    sha256: str

    @classmethod
    def from_path(cls, path: Path) -> "WrapperOwnership":
        destination = _canonical(path)
        mode = destination.lstat().st_mode
        if not stat.S_ISREG(mode) or stat.S_ISLNK(mode):
            raise OwnershipError(f"wrapper must be a regular file, not a symlink: {destination}")
        return cls(str(destination), sha256_file(destination))


@dataclass(frozen=True)
class ShellOwnership:
    bashrc: str
    bin_dir: str


@dataclass(frozen=True)
class OwnershipRefresh:
    """Proof that the previous manifest was validated before an update."""

    manifest_path: str
    manifest_sha256: str
    install_uuid: str
    edition: str
    prefix: str
    bin_dir: str
    created_at: str
    owned_paths: tuple[OwnedPath, ...]
    managed_roots: tuple[str, ...]
    created_roots: tuple[str, ...]
    wrappers: tuple[WrapperOwnership, ...]
    log_roots: tuple[str, ...]
    authority_roots: tuple[str, ...]
    external_paths: tuple[str, ...]
    shell: ShellOwnership | None
    docker: DockerOwnership | None
    systemd_units: tuple[SystemdUnitOwnership, ...]


@dataclass(frozen=True)
class HostUninstallerBundle:
    root: Path
    wrapper: Path
    files: tuple[Path, ...]


@dataclass(frozen=True)
class DockerOwnership:
    install_uuid: str
    compose_file: str
    project: str
    containers: tuple[str, ...]
    local_images: tuple[str, ...]
    # Empty values preserve schema-v1 manifests created before daemon pinning.
    # New general installs record both so uninstall/update cannot silently
    # operate on a different Docker Desktop/native Engine boundary.
    context: str = ""
    engine_id: str = ""

    def validate(self) -> "DockerOwnership":
        _validate_uuid(self.install_uuid, name="Docker install UUID")
        _require_absolute(self.compose_file, name="Docker compose file")
        if not _DOCKER_NAME.fullmatch(self.project):
            raise OwnershipError(f"Unsafe Compose project name: {self.project!r}")
        expected_scoped_project = _scoped_project_name(self.install_uuid)
        if self.project not in {"elesim-runtime", expected_scoped_project}:
            raise OwnershipError(
                "Docker project is neither an EleSim legacy project nor the current install namespace"
            )
        if len(set(self.containers)) != len(self.containers):
            raise OwnershipError("Duplicate Docker container name")
        if any(not _DOCKER_NAME.fullmatch(value) for value in self.containers):
            raise OwnershipError("Docker container name must be a fixed literal")
        if len(set(self.local_images)) != len(self.local_images):
            raise OwnershipError("Duplicate Docker image name")
        if any(
            (
                self.project == expected_scoped_project
                and _LOCAL_IMAGE.fullmatch(value) is not None
            )
            or not _image_belongs_to_install(value, self.install_uuid)
            for value in self.local_images
        ):
            raise OwnershipError(
                "Deletable images must use the legacy :local tag or the current installation immutable tag"
            )
        if self.context and not _DOCKER_NAME.fullmatch(self.context):
            raise OwnershipError("Invalid Docker context name")
        if len(self.engine_id) > 256 or "\x00" in self.engine_id or "\n" in self.engine_id:
            raise OwnershipError("Invalid Docker Engine ID")
        if bool(self.context) != bool(self.engine_id):
            raise OwnershipError(
                "Docker context and Engine ID must both be specified or both omitted"
            )
        return self


@dataclass(frozen=True)
class SystemdUnitOwnership:
    name: str
    destination: str
    sha256: str

    def validate(self) -> "SystemdUnitOwnership":
        if not self.name.endswith(".service") or not _DOCKER_NAME.fullmatch(self.name):
            raise OwnershipError(f"Unsafe systemd unit name: {self.name!r}")
        _require_absolute(self.destination, name="systemd unit destination")
        if not re.fullmatch(r"[0-9a-f]{64}", self.sha256):
            raise OwnershipError(f"systemd unit SHA-256 is invalid: {self.name}")
        return self


@dataclass(frozen=True)
class OwnershipManifest:
    schema_version: int
    install_uuid: str
    edition: str
    created_at: str
    prefix: str
    prefix_realpath: str
    bin_dir: str
    bin_dir_realpath: str
    manifest_path: str
    owned_paths: tuple[OwnedPath, ...]
    managed_roots: tuple[str, ...]
    created_roots: tuple[str, ...]
    wrappers: tuple[WrapperOwnership, ...]
    log_roots: tuple[str, ...]
    authority_roots: tuple[str, ...]
    external_paths: tuple[str, ...]
    shell: ShellOwnership | None = None
    docker: DockerOwnership | None = None
    systemd_units: tuple[SystemdUnitOwnership, ...] = ()

    @property
    def path(self) -> Path:
        return Path(self.manifest_path)

    @property
    def prefix_path(self) -> Path:
        return Path(self.prefix)

    @property
    def bin_path(self) -> Path:
        return Path(self.bin_dir)

    def validate(self) -> "OwnershipManifest":
        if self.schema_version != OWNERSHIP_SCHEMA_VERSION:
            raise OwnershipError(
                f"Unsupported ownership schema: {self.schema_version!r}"
            )
        try:
            parsed_uuid = uuid.UUID(self.install_uuid)
        except (AttributeError, TypeError, ValueError) as exc:
            raise OwnershipError("install_uuid is not a valid UUID") from exc
        if str(parsed_uuid) != self.install_uuid:
            raise OwnershipError("install_uuid must be a canonical UUID string")
        if self.edition not in _INSTALL_EDITIONS:
            raise OwnershipError(f"Unsupported installation edition: {self.edition!r}")

        prefix = _require_absolute(self.prefix, name="prefix")
        prefix_realpath = _require_absolute(self.prefix_realpath, name="prefix_realpath")
        bin_dir = _require_absolute(self.bin_dir, name="bin_dir")
        _require_absolute(self.bin_dir_realpath, name="bin_dir_realpath")
        manifest_path = _require_absolute(self.manifest_path, name="manifest_path")
        if prefix == Path("/") or bin_dir == Path("/"):
            raise OwnershipError("filesystem root cannot be an install/bin boundary")
        if not _is_descendant(manifest_path, prefix):
            raise OwnershipError("ownership manifest must be inside the installation prefix")
        if prefix_realpath == Path("/"):
            raise OwnershipError("resolved prefix cannot be the filesystem root")

        seen_owned: set[str] = set()
        for entry in self.owned_paths:
            path = _require_absolute(entry.path, name="owned path")
            if entry.kind not in _PATH_KINDS:
                raise OwnershipError(f"Unsupported owned path kind: {entry.kind!r}")
            if entry.path in seen_owned:
                raise OwnershipError(f"Duplicate owned path: {entry.path}")
            seen_owned.add(entry.path)
            if path == manifest_path:
                raise OwnershipError("The manifest itself cannot be in owned_paths")
            if not (_is_descendant(path, prefix) or _is_descendant(path, bin_dir)):
                raise OwnershipError(f"owned path is outside the installation boundary: {path}")

        managed = _validated_unique_paths(self.managed_roots, name="managed root")
        for path in managed:
            if path == prefix or not _is_descendant(path, prefix):
                raise OwnershipError(
                    f"managed root must be a child path, not the prefix itself: {path}"
                )
        created = _validated_unique_paths(self.created_roots, name="created root")
        for path in created:
            if not (
                path == prefix
                or _is_descendant(path, prefix)
                or path == bin_dir
                or _is_descendant(path, bin_dir)
            ):
                raise OwnershipError(f"created root is outside the installation boundary: {path}")

        wrappers: set[str] = set()
        for wrapper in self.wrappers:
            path = _require_absolute(wrapper.path, name="wrapper")
            if not _is_descendant(path, bin_dir):
                raise OwnershipError(f"wrapper is outside bin_dir: {path}")
            if wrapper.path in wrappers:
                raise OwnershipError(f"Duplicate wrapper: {path}")
            wrappers.add(wrapper.path)
            if not re.fullmatch(r"[0-9a-f]{64}", wrapper.sha256):
                raise OwnershipError(f"wrapper SHA-256 is invalid: {path}")

        protected = (
            *_validated_unique_paths(self.log_roots, name="log root"),
            *_validated_unique_paths(self.authority_roots, name="authority root"),
        )
        for path in protected:
            if path == prefix or not _is_descendant(path, prefix):
                raise OwnershipError(f"preserved root is not below the prefix: {path}")
        _validated_unique_paths(self.external_paths, name="external path")

        if self.shell is not None:
            _require_absolute(self.shell.bashrc, name="shell bashrc")
            if _require_absolute(self.shell.bin_dir, name="shell bin_dir") != bin_dir:
                raise OwnershipError("shell PATH bin_dir differs from manifest bin_dir")
        if self.docker is not None:
            self.docker.validate()
            if self.docker.install_uuid != self.install_uuid:
                raise OwnershipError("Docker ownership UUID differs from install UUID")
            compose = Path(self.docker.compose_file)
            if not _is_descendant(compose, prefix):
                raise OwnershipError("Compose file is outside the installation prefix")
        for unit in self.systemd_units:
            unit.validate()
        return self

    def to_dict(self) -> dict[str, object]:
        return asdict(self)

    @classmethod
    def from_dict(cls, raw: Mapping[str, object]) -> "OwnershipManifest":
        if not isinstance(raw, Mapping):
            raise OwnershipError("ownership manifest must be a JSON object")
        shell_raw = raw.get("shell")
        docker_raw = raw.get("docker")
        try:
            shell = (
                None
                if shell_raw is None
                else ShellOwnership(**_mapping(shell_raw, name="shell"))
            )
            docker = None
            if docker_raw is not None:
                values = _mapping(docker_raw, name="docker")
                docker = DockerOwnership(
                    install_uuid=str(values["install_uuid"]),
                    compose_file=str(values["compose_file"]),
                    project=str(values["project"]),
                    containers=tuple(str(value) for value in values["containers"]),
                    local_images=tuple(str(value) for value in values["local_images"]),
                    context=str(values.get("context", "")),
                    engine_id=str(values.get("engine_id", "")),
                )
            manifest = cls(
                schema_version=int(raw["schema_version"]),
                install_uuid=str(raw["install_uuid"]),
                edition=str(raw["edition"]),
                created_at=str(raw["created_at"]),
                prefix=str(raw["prefix"]),
                prefix_realpath=str(raw["prefix_realpath"]),
                bin_dir=str(raw["bin_dir"]),
                bin_dir_realpath=str(raw["bin_dir_realpath"]),
                manifest_path=str(raw["manifest_path"]),
                owned_paths=tuple(
                    OwnedPath(**_mapping(value, name="owned path"))
                    for value in _sequence(raw["owned_paths"], name="owned_paths")
                ),
                managed_roots=tuple(
                    str(value) for value in _sequence(raw["managed_roots"], name="managed_roots")
                ),
                created_roots=tuple(
                    str(value) for value in _sequence(raw["created_roots"], name="created_roots")
                ),
                wrappers=tuple(
                    WrapperOwnership(**_mapping(value, name="wrapper"))
                    for value in _sequence(raw["wrappers"], name="wrappers")
                ),
                log_roots=tuple(
                    str(value) for value in _sequence(raw["log_roots"], name="log_roots")
                ),
                authority_roots=tuple(
                    str(value)
                    for value in _sequence(raw["authority_roots"], name="authority_roots")
                ),
                external_paths=tuple(
                    str(value)
                    for value in _sequence(raw["external_paths"], name="external_paths")
                ),
                shell=shell,
                docker=docker,
                systemd_units=tuple(
                    SystemdUnitOwnership(**_mapping(value, name="systemd unit"))
                    for value in _sequence(raw["systemd_units"], name="systemd_units")
                ),
            )
        except (KeyError, TypeError, ValueError) as exc:
            if isinstance(exc, OwnershipError):
                raise
            raise OwnershipError(f"Invalid ownership manifest field: {exc}") from exc
        return manifest.validate()

    @classmethod
    def load(cls, path: Path) -> "OwnershipManifest":
        source = _canonical(path)
        if source.is_symlink() or not source.is_file():
            raise OwnershipError(f"ownership manifest must be a regular file: {source}")
        try:
            raw = json.loads(source.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise OwnershipError(f"cannot read ownership manifest: {source}: {exc}") from exc
        manifest = cls.from_dict(_mapping(raw, name="manifest"))
        if source != Path(manifest.manifest_path):
            raise OwnershipError(
                f"manifest location differs from recorded location: actual={source} expected={manifest.manifest_path}"
            )
        return manifest


def default_manifest_path(prefix: Path | None = None) -> Path:
    """Return the host manifest path, honoring the wrapper-only override."""

    override = os.environ.get("ELESIM_OWNERSHIP_MANIFEST", "").strip()
    if override:
        return _canonical(Path(override).expanduser())
    root = (
        Path("~/.local/share/elesim").expanduser()
        if prefix is None
        else prefix.expanduser()
    )
    return _canonical(root) / OWNERSHIP_MANIFEST_NAME


def inventory_paths(
    roots: Iterable[Path],
    *,
    exclude: Iterable[Path] = (),
) -> tuple[OwnedPath, ...]:
    """Inventory exact existing paths without following directory symlinks."""

    exclusions = tuple(_canonical(path) for path in exclude)
    collected: dict[str, OwnedPath] = {}
    for root_value in roots:
        root = _canonical(root_value)
        if not _lexists(root) or _protected(root, exclusions):
            continue
        entry = OwnedPath.from_path(root)
        collected[entry.path] = entry
        if entry.kind != "directory":
            continue
        for directory, names, files in os.walk(root, topdown=True, followlinks=False):
            current = Path(directory)
            names[:] = [
                name
                for name in names
                if not _protected(current / name, exclusions)
            ]
            for name in (*names, *files):
                path = current / name
                if _protected(path, exclusions):
                    continue
                child = OwnedPath.from_path(path)
                collected[child.path] = child
    return tuple(collected[key] for key in sorted(collected))


def _write_ownership_manifest_unlocked(
    *,
    prefix: Path,
    bin_dir: Path,
    edition: str,
    inventory_roots: Iterable[Path],
    managed_roots: Iterable[Path],
    created_roots: Iterable[Path],
    wrapper_paths: Iterable[Path],
    log_roots: Iterable[Path] = (),
    authority_roots: Iterable[Path] = (),
    external_paths: Iterable[Path] = (),
    shell_bashrc: Path | None = None,
    docker: DockerOwnership | None = None,
    systemd_units: Iterable[SystemdUnitOwnership] = (),
    manifest_path: Path | None = None,
    install_uuid: str | None = None,
    refresh: OwnershipRefresh | None = None,
) -> OwnershipManifest:
    """Atomically record one completed installation's deletion boundary.

    Installers call this only after every generated file and wrapper has been
    published.  ``created_roots`` must be captured before installation so the
    uninstaller can use ``rmdir`` only for directories this install created.
    """

    prefix_path = _canonical(prefix)
    bin_path = _canonical(bin_dir)
    destination = (
        default_manifest_path(prefix_path)
        if manifest_path is None
        else _canonical(manifest_path)
    )
    if refresh is None:
        if _lexists(destination):
            raise OwnershipError(f"refusing to overwrite existing ownership manifest: {destination}")
    else:
        _validate_refresh_token(
            refresh,
            destination=destination,
            prefix=prefix_path,
            bin_dir=bin_path,
            edition=edition,
        )
        if install_uuid is not None and str(install_uuid) != refresh.install_uuid:
            raise OwnershipError("cannot change install UUID during refresh")
        install_uuid = refresh.install_uuid
    if not prefix_path.is_dir() or prefix_path.is_symlink():
        raise OwnershipError(f"prefix must be an existing directory, not a symlink: {prefix_path}")
    if not bin_path.is_dir() or bin_path.is_symlink():
        raise OwnershipError(f"bin_dir must be an existing directory, not a symlink: {bin_path}")

    logs = _merged_paths(
        () if refresh is None else refresh.log_roots,
        log_roots,
    )
    authorities = _merged_paths(
        () if refresh is None else refresh.authority_roots,
        authority_roots,
    )
    external = _merged_paths(
        () if refresh is None else refresh.external_paths,
        external_paths,
    )
    protected = (*logs, *authorities, *external, destination)
    owned_by_path = {
        entry.path: entry
        for entry in inventory_paths(inventory_roots, exclude=protected)
    }
    if refresh is not None:
        for old_entry in refresh.owned_paths:
            old_path = Path(old_entry.path)
            if (
                _lexists(old_path)
                and not _protected(old_path, protected)
                and old_path != destination
            ):
                current_entry = OwnedPath.from_path(old_path)
                owned_by_path.setdefault(current_entry.path, current_entry)
    owned = tuple(owned_by_path[key] for key in sorted(owned_by_path))
    wrapper_candidates = {_canonical(path) for path in wrapper_paths}
    if refresh is not None:
        wrapper_candidates.update(
            Path(wrapper.path)
            for wrapper in refresh.wrappers
            if _lexists(Path(wrapper.path))
        )
    wrappers = tuple(
        WrapperOwnership.from_path(path)
        for path in sorted(wrapper_candidates, key=str)
    )
    shell = _merged_shell(refresh, shell_bashrc, bin_path)
    docker = _merged_docker(None if refresh is None else refresh.docker, docker)
    units = _merged_systemd(
        () if refresh is None else refresh.systemd_units,
        systemd_units,
    )
    manifest = OwnershipManifest(
        schema_version=OWNERSHIP_SCHEMA_VERSION,
        install_uuid=str(uuid.uuid4()) if install_uuid is None else str(install_uuid),
        edition=edition,
        created_at=(
            refresh.created_at
            if refresh is not None
            else datetime.now(timezone.utc).isoformat()
        ),
        prefix=str(prefix_path),
        prefix_realpath=str(prefix_path.resolve(strict=True)),
        bin_dir=str(bin_path),
        bin_dir_realpath=str(bin_path.resolve(strict=True)),
        manifest_path=str(destination),
        owned_paths=owned,
        managed_roots=tuple(
            sorted(
                {
                    *((refresh.managed_roots) if refresh is not None else ()),
                    *(str(_canonical(path)) for path in managed_roots),
                }
            )
        ),
        created_roots=tuple(
            sorted(
                {
                    *(refresh.created_roots if refresh is not None else ()),
                    *(str(_canonical(path)) for path in created_roots),
                }
            )
        ),
        wrappers=wrappers,
        log_roots=tuple(str(path) for path in logs),
        authority_roots=tuple(str(path) for path in authorities),
        external_paths=tuple(str(path) for path in external),
        shell=shell,
        docker=docker,
        systemd_units=units,
    ).validate()
    destination.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(manifest.to_dict(), ensure_ascii=False, indent=2) + "\n"
    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        dir=destination.parent,
        prefix=f".{destination.name}.",
        delete=False,
    ) as handle:
        handle.write(payload)
        temporary = Path(handle.name)
    temporary.chmod(0o600)
    os.replace(temporary, destination)
    return manifest


@contextmanager
def _ownership_manifest_lock(destination: Path) -> Iterable[None]:
    """Serialize manifest writers without trusting a replaceable lock path."""

    destination = _canonical(destination)
    prefix = destination.parent
    _ensure_no_symlink_ancestors(destination, boundary=prefix)
    if prefix.is_symlink() or not prefix.is_dir():
        raise OwnershipError(f"ownership manifest parent is not a safe directory: {prefix}")
    # Keep the persistent coordination file inside the exact maintenance
    # subtree.  Installers inventory that subtree before publishing the
    # manifest, and scoped installs additionally own it as a managed root, so
    # a clean uninstall cannot leave an unowned dotfile at the prefix root.
    lock_root = prefix / "maintenance"
    _ensure_no_symlink_ancestors(lock_root, boundary=prefix)
    if lock_root.is_symlink() or (lock_root.exists() and not lock_root.is_dir()):
        raise OwnershipError(
            f"ownership manifest lock root is not a safe directory: {lock_root}"
        )
    lock_root.mkdir(mode=0o700, parents=True, exist_ok=True)
    if lock_root.is_symlink() or not lock_root.is_dir():
        raise OwnershipError(
            f"ownership manifest lock root is not a safe directory: {lock_root}"
        )
    lock_path = lock_root / ".ownership-manifest.lock"
    if lock_path.is_symlink():
        raise OwnershipError(f"ownership manifest lock is a symlink: {lock_path}")
    key = str(lock_path)
    with _OWNERSHIP_LOCKS_GUARD:
        thread_lock = _OWNERSHIP_LOCKS.setdefault(key, threading.Lock())
    thread_lock.acquire()
    fd: int | None = None
    try:
        fd = os.open(
            lock_path,
            os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC,
            0o600,
        )
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise OwnershipError(f"ownership manifest lock must be a single regular file: {lock_path}")
        stream = os.fdopen(fd, "a+b")
        fd = None
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
            yield
        finally:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
            stream.close()
    except OSError as exc:
        raise OwnershipError(f"cannot open ownership manifest lock: {lock_path}") from exc
    finally:
        if fd is not None:
            os.close(fd)
        thread_lock.release()


def write_ownership_manifest(
    *,
    prefix: Path,
    bin_dir: Path,
    edition: str,
    inventory_roots: Iterable[Path],
    managed_roots: Iterable[Path],
    created_roots: Iterable[Path],
    wrapper_paths: Iterable[Path],
    log_roots: Iterable[Path] = (),
    authority_roots: Iterable[Path] = (),
    external_paths: Iterable[Path] = (),
    shell_bashrc: Path | None = None,
    docker: DockerOwnership | None = None,
    systemd_units: Iterable[SystemdUnitOwnership] = (),
    manifest_path: Path | None = None,
    install_uuid: str | None = None,
    refresh: OwnershipRefresh | None = None,
) -> OwnershipManifest:
    """Write an ownership manifest while serializing concurrent appenders."""

    destination = (
        default_manifest_path(prefix)
        if manifest_path is None
        else Path(manifest_path)
    )
    with _ownership_manifest_lock(destination):
        return _write_ownership_manifest_unlocked(
            prefix=prefix,
            bin_dir=bin_dir,
            edition=edition,
            inventory_roots=inventory_roots,
            managed_roots=managed_roots,
            created_roots=created_roots,
            wrapper_paths=wrapper_paths,
            log_roots=log_roots,
            authority_roots=authority_roots,
            external_paths=external_paths,
            shell_bashrc=shell_bashrc,
            docker=docker,
            systemd_units=systemd_units,
            manifest_path=manifest_path,
            install_uuid=install_uuid,
            refresh=refresh,
        )


def append_instance_docker_ownership(
    *,
    manifest_path: Path,
    install_uuid: str,
    project: str,
    docker_context: str,
    docker_engine_id: str,
    instance: object,
) -> OwnershipManifest:
    """Append one instance's exact containers to a scoped install manifest.

    Instance registration publishes files and Compose only after this function
    succeeds.  Entries are intentionally append-only: removing an instance
    leaves its exact names in the manifest so an interrupted registration can
    still be cleaned by the host uninstaller.  No Docker API is consulted.
    """

    from .instance_identity import container_name, service_key
    from .instances import InstanceState

    if not isinstance(instance, InstanceState):
        raise OwnershipError("instance ownership requires a validated InstanceState")
    instance.validate()
    try:
        parsed = uuid.UUID(install_uuid)
    except (AttributeError, TypeError, ValueError) as exc:
        raise OwnershipError("install UUID is invalid") from exc
    if str(parsed) != install_uuid:
        raise OwnershipError("install UUID must be a canonical UUID string")
    expected_project = _scoped_project_name(install_uuid)
    if project != expected_project:
        raise OwnershipError("do not add an instance to a legacy or foreign Docker project")
    if not isinstance(docker_context, str) or not _DOCKER_NAME.fullmatch(docker_context):
        raise OwnershipError("scoped instance ownership requires a valid Docker context")
    if not isinstance(docker_engine_id, str) or not docker_engine_id or "\x00" in docker_engine_id or "\n" in docker_engine_id:
        raise OwnershipError("scoped instance ownership requires a valid Docker Engine ID")

    destination = _canonical(manifest_path)
    with _ownership_manifest_lock(destination):
        manifest = OwnershipManifest.load(destination)
        prefix = manifest.prefix_path
        if destination != prefix / OWNERSHIP_MANIFEST_NAME:
            raise OwnershipError("ownership manifest is not at the canonical installation-prefix location")
        if prefix.is_symlink() or not prefix.is_dir():
            raise OwnershipError("ownership manifest prefix is not a safe directory")
        if str(prefix.resolve(strict=True)) != manifest.prefix_realpath:
            raise OwnershipError("ownership manifest prefix realpath changed")
        docker = manifest.docker
        if docker is None:
            raise OwnershipError("do not add an instance without Docker ownership")
        docker.validate()
        if (
            manifest.install_uuid != install_uuid
            or docker.install_uuid != install_uuid
            or docker.project != expected_project
            or docker.context != docker_context
            or docker.engine_id != docker_engine_id
        ):
            raise OwnershipError("foreign or legacy Docker ownership boundary")

        names = tuple(
            container_name(install_uuid, service_key(instance.system_id, endpoint.endpoint_id))
            for endpoint in instance.endpoints
        )
        if instance.turn.mode == "managed":
            names += (
                container_name(install_uuid, service_key(instance.system_id, "coturn")),
            )
        updated_docker = DockerOwnership(
            install_uuid=docker.install_uuid,
            compose_file=docker.compose_file,
            project=docker.project,
            containers=tuple(sorted({*docker.containers, *names})),
            local_images=docker.local_images,
            context=docker.context,
            engine_id=docker.engine_id,
        ).validate()
        updated = OwnershipManifest(
            schema_version=manifest.schema_version,
            install_uuid=manifest.install_uuid,
            edition=manifest.edition,
            created_at=manifest.created_at,
            prefix=manifest.prefix,
            prefix_realpath=manifest.prefix_realpath,
            bin_dir=manifest.bin_dir,
            bin_dir_realpath=manifest.bin_dir_realpath,
            manifest_path=manifest.manifest_path,
            owned_paths=manifest.owned_paths,
            managed_roots=manifest.managed_roots,
            created_roots=manifest.created_roots,
            wrappers=manifest.wrappers,
            log_roots=manifest.log_roots,
            authority_roots=manifest.authority_roots,
            external_paths=manifest.external_paths,
            shell=manifest.shell,
            docker=updated_docker,
            systemd_units=manifest.systemd_units,
        ).validate()
        payload = json.dumps(updated.to_dict(), ensure_ascii=False, indent=2) + "\n"
        with tempfile.NamedTemporaryFile(
            "w", encoding="utf-8", dir=destination.parent,
            prefix=f".{destination.name}.", delete=False,
        ) as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
            temporary = Path(handle.name)
        temporary.chmod(0o600)
        os.replace(temporary, destination)
        try:
            directory_fd = os.open(destination.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        except OSError as exc:
            raise OwnershipError("cannot synchronize ownership manifest parent") from exc
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        return updated


def append_manager_docker_ownership(
    *,
    manifest_path: Path,
    install_uuid: str,
    project: str,
    docker_context: str,
    docker_engine_id: str,
    system_id: str,
) -> OwnershipManifest:
    """Append the exact scoped manager name before its container is created.

    This host-side operation intentionally performs no Docker calls.  The
    generated scoped connection-manager wrapper invokes it before
    ``compose run`` so a crashed one-shot manager remains in the exact
    ownership set that the host-only uninstaller is allowed to remove.
    """

    from .instance_identity import manager_container_name

    name = manager_container_name(install_uuid, system_id)
    try:
        parsed = uuid.UUID(install_uuid)
    except (AttributeError, TypeError, ValueError) as exc:
        raise OwnershipError("install UUID is invalid") from exc
    if str(parsed) != install_uuid:
        raise OwnershipError("install UUID must be a canonical UUID string")
    expected_project = _scoped_project_name(install_uuid)
    if project != expected_project:
        raise OwnershipError("do not add a manager to a legacy or foreign Docker project")
    if not isinstance(docker_context, str) or not _DOCKER_NAME.fullmatch(docker_context):
        raise OwnershipError("scoped manager ownership requires a valid Docker context")
    if (
        not isinstance(docker_engine_id, str)
        or not docker_engine_id
        or "\x00" in docker_engine_id
        or "\n" in docker_engine_id
    ):
        raise OwnershipError("scoped manager ownership requires a valid Docker Engine ID")

    destination = _canonical(manifest_path)
    with _ownership_manifest_lock(destination):
        manifest = OwnershipManifest.load(destination)
        prefix = manifest.prefix_path
        if destination != prefix / OWNERSHIP_MANIFEST_NAME:
            raise OwnershipError("ownership manifest is not at the canonical installation-prefix location")
        if prefix.is_symlink() or not prefix.is_dir():
            raise OwnershipError("ownership manifest prefix is not a safe directory")
        if str(prefix.resolve(strict=True)) != manifest.prefix_realpath:
            raise OwnershipError("ownership manifest prefix realpath changed")
        docker = manifest.docker
        if docker is None:
            raise OwnershipError("do not add a manager without Docker ownership")
        docker.validate()
        if (
            manifest.install_uuid != install_uuid
            or docker.install_uuid != install_uuid
            or docker.project != expected_project
            or docker.context != docker_context
            or docker.engine_id != docker_engine_id
        ):
            raise OwnershipError("foreign or legacy Docker ownership boundary")
        updated_docker = DockerOwnership(
            install_uuid=docker.install_uuid,
            compose_file=docker.compose_file,
            project=docker.project,
            containers=tuple(sorted({*docker.containers, name})),
            local_images=docker.local_images,
            context=docker.context,
            engine_id=docker.engine_id,
        ).validate()
        updated = OwnershipManifest(
            schema_version=manifest.schema_version,
            install_uuid=manifest.install_uuid,
            edition=manifest.edition,
            created_at=manifest.created_at,
            prefix=manifest.prefix,
            prefix_realpath=manifest.prefix_realpath,
            bin_dir=manifest.bin_dir,
            bin_dir_realpath=manifest.bin_dir_realpath,
            manifest_path=manifest.manifest_path,
            owned_paths=manifest.owned_paths,
            managed_roots=manifest.managed_roots,
            created_roots=manifest.created_roots,
            wrappers=manifest.wrappers,
            log_roots=manifest.log_roots,
            authority_roots=manifest.authority_roots,
            external_paths=manifest.external_paths,
            shell=manifest.shell,
            docker=updated_docker,
            systemd_units=manifest.systemd_units,
        ).validate()
        payload = json.dumps(updated.to_dict(), ensure_ascii=False, indent=2) + "\n"
        with tempfile.NamedTemporaryFile(
            "w", encoding="utf-8", dir=destination.parent,
            prefix=f".{destination.name}.", delete=False,
        ) as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
            temporary = Path(handle.name)
        temporary.chmod(0o600)
        os.replace(temporary, destination)
        try:
            directory_fd = os.open(
                destination.parent,
                os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW,
            )
        except OSError as exc:
            raise OwnershipError("cannot synchronize ownership manifest parent") from exc
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        return updated


def append_docker_image_ownership(
    *,
    manifest_path: Path,
    install_uuid: str,
    project: str,
    docker_context: str,
    docker_engine_id: str,
    image: str,
) -> OwnershipManifest:
    """Record one newly published immutable image in the install manifest.

    Scoped ``elesim-update`` intentionally creates a new content-addressed
    image tag on every changed build.  The initial install manifest cannot
    predict that tag, so the host update wrapper appends it immediately after
    the build and before release publication.  This is a file-only operation;
    Docker labels and the release publisher remain the authority that the
    image actually belongs to this install.
    """

    try:
        parsed = uuid.UUID(install_uuid)
    except (AttributeError, TypeError, ValueError) as exc:
        raise OwnershipError("install UUID is invalid") from exc
    if str(parsed) != install_uuid:
        raise OwnershipError("install UUID must be a canonical UUID string")
    expected_project = _scoped_project_name(install_uuid)
    if project != expected_project:
        raise OwnershipError("do not add an image to a legacy or foreign Docker project")
    if not isinstance(docker_context, str) or not _DOCKER_NAME.fullmatch(docker_context):
        raise OwnershipError("scoped image ownership requires a valid Docker context")
    if (
        not isinstance(docker_engine_id, str)
        or not docker_engine_id
        or "\x00" in docker_engine_id
        or "\n" in docker_engine_id
    ):
        raise OwnershipError("scoped image ownership requires a valid Docker Engine ID")
    if not isinstance(image, str) or not _INSTALL_IMAGE.fullmatch(image):
        raise OwnershipError("image ownership accepts only immutable install images")
    if not image.startswith(
        ("elesim/pilot:", "elesim/sim:", "elesim/ui:", "elesim/tools:")
    ) or not image.split(":", 1)[1].startswith(parsed.hex + "-"):
        raise OwnershipError("image does not belong to the current scoped installation")

    destination = _canonical(manifest_path)
    with _ownership_manifest_lock(destination):
        manifest = OwnershipManifest.load(destination)
        prefix = manifest.prefix_path
        if destination != prefix / OWNERSHIP_MANIFEST_NAME:
            raise OwnershipError("ownership manifest is not at the canonical installation-prefix location")
        if prefix.is_symlink() or not prefix.is_dir():
            raise OwnershipError("ownership manifest prefix is not a safe directory")
        if str(prefix.resolve(strict=True)) != manifest.prefix_realpath:
            raise OwnershipError("ownership manifest prefix realpath changed")
        docker = manifest.docker
        if docker is None:
            raise OwnershipError("Do not add an image without Docker ownership")
        docker.validate()
        if (
            manifest.install_uuid != install_uuid
            or docker.install_uuid != install_uuid
            or docker.project != expected_project
            or docker.context != docker_context
            or docker.engine_id != docker_engine_id
        ):
            raise OwnershipError("foreign or legacy Docker ownership boundary")
        updated_docker = DockerOwnership(
            install_uuid=docker.install_uuid,
            compose_file=docker.compose_file,
            project=docker.project,
            containers=docker.containers,
            local_images=tuple(sorted({*docker.local_images, image})),
            context=docker.context,
            engine_id=docker.engine_id,
        ).validate()
        updated = OwnershipManifest(
            schema_version=manifest.schema_version,
            install_uuid=manifest.install_uuid,
            edition=manifest.edition,
            created_at=manifest.created_at,
            prefix=manifest.prefix,
            prefix_realpath=manifest.prefix_realpath,
            bin_dir=manifest.bin_dir,
            bin_dir_realpath=manifest.bin_dir_realpath,
            manifest_path=manifest.manifest_path,
            owned_paths=manifest.owned_paths,
            managed_roots=manifest.managed_roots,
            created_roots=manifest.created_roots,
            wrappers=manifest.wrappers,
            log_roots=manifest.log_roots,
            authority_roots=manifest.authority_roots,
            external_paths=manifest.external_paths,
            shell=manifest.shell,
            docker=updated_docker,
            systemd_units=manifest.systemd_units,
        ).validate()
        payload = json.dumps(updated.to_dict(), ensure_ascii=False, indent=2) + "\n"
        with tempfile.NamedTemporaryFile(
            "w", encoding="utf-8", dir=destination.parent,
            prefix=f".{destination.name}.", delete=False,
        ) as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
            temporary = Path(handle.name)
        temporary.chmod(0o600)
        try:
            os.replace(temporary, destination)
        finally:
            if temporary.exists():
                temporary.unlink()
        try:
            directory_fd = os.open(destination.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        except OSError as exc:
            raise OwnershipError("cannot synchronize ownership manifest parent") from exc
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        return updated


def _refuse_nested_install(candidate: Path, *, destination: Path) -> None:
    """Reject overlaps with an ancestor install's actual removal targets.

    Prefix/bin are placement bases, not recursively owned trees. Uninstall
    recursively removes managed/log/authority roots and unlinks owned files
    and wrappers. Inventory directories are also reserved because refresh may
    inventory their descendants again. Merely created directories are only
    removed when empty and do not reserve their descendants.
    """

    path = _canonical(candidate)
    current = path
    while True:
        marker = _canonical(current / OWNERSHIP_MANIFEST_NAME)
        if marker != destination and _lexists(marker):
            try:
                owner = OwnershipManifest.load(marker)
            except OwnershipError as exc:
                raise OwnershipError(
                    f"cannot validate parent EleSim ownership manifest: {marker}"
                ) from exc
            targets = (
                *owner.managed_roots, *owner.log_roots, *owner.authority_roots,
                owner.manifest_path,
                *(entry.path for entry in owner.owned_paths),
                *(wrapper.path for wrapper in owner.wrappers),
            )
            for value in targets:
                target = _canonical(Path(value))
                if _within_or_equal(path, target) or _within_or_equal(target, path):
                    raise OwnershipError(
                        "New installation prefix/bin cannot overlap or be nested "
                        "inside another EleSim owned path: "
                        f"candidate={path} owned={target} owner={marker}"
                    )
        if current == current.parent:
            break
        current = current.parent


def prepare_ownership_refresh(
    *,
    prefix: Path,
    bin_dir: Path,
    edition: str,
    manifest_path: Path | None = None,
    claimed_paths: Iterable[Path] = (),
) -> OwnershipRefresh | None:
    """Validate a prior same-install manifest before regenerating artifacts.

    Call this before any installer mutation.  A missing manifest means a new
    installation and returns ``None``.  Any foreign, relocated or locally
    modified ownership record fails closed.
    """

    prefix_path = _canonical(prefix)
    bin_path = _canonical(bin_dir)
    destination = (
        default_manifest_path(prefix_path)
        if manifest_path is None
        else _canonical(manifest_path)
    )
    for candidate in (prefix_path, bin_path):
        _refuse_nested_install(candidate, destination=destination)
    claims = tuple(_canonical(path) for path in claimed_paths)
    for path in claims:
        if not (
            _within_or_equal(path, prefix_path)
            or _within_or_equal(path, bin_path)
        ):
            raise OwnershipError(f"claimed install path is outside prefix/bin: {path}")
    if not _lexists(destination):
        existing = tuple(path for path in claims if _lexists(path))
        if existing:
            rendered = "\n".join(f"  - {path}" for path in existing)
            raise OwnershipError(
                "Do not automatically adopt existing EleSim candidate paths without "
                "an ownership manifest. Back up/remove them explicitly or run the "
                f"existing installer's clean uninstall first:\n{rendered}"
            )
        return None
    manifest = OwnershipManifest.load(destination)
    if (
        manifest.prefix != str(prefix_path)
        or manifest.bin_dir != str(bin_path)
        or manifest.edition != edition
    ):
        raise OwnershipError(
            "existing ownership manifest belongs to another prefix/bin/edition installation"
        )
    if prefix_path.is_symlink() or not prefix_path.is_dir():
        raise OwnershipError(f"existing prefix is not a safe directory: {prefix_path}")
    if bin_path.is_symlink() or not bin_path.is_dir():
        raise OwnershipError(f"existing bin_dir is not a safe directory: {bin_path}")
    if str(prefix_path.resolve(strict=True)) != manifest.prefix_realpath:
        raise OwnershipError("existing prefix realpath differs from the manifest")
    if str(bin_path.resolve(strict=True)) != manifest.bin_dir_realpath:
        raise OwnershipError("existing bin_dir realpath differs from the manifest")
    _validate_refresh_paths(manifest)
    for wrapper in manifest.wrappers:
        path = Path(wrapper.path)
        if not _lexists(path):
            continue
        mode = path.lstat().st_mode
        if not stat.S_ISREG(mode) or stat.S_ISLNK(mode):
            raise OwnershipError(f"existing wrapper is not a regular file: {path}")
        if sha256_file(path) != wrapper.sha256:
            raise OwnershipError(
                f"existing wrapper changed after installation: {path}"
            )
    return OwnershipRefresh(
        manifest_path=str(destination),
        manifest_sha256=sha256_file(destination),
        install_uuid=manifest.install_uuid,
        edition=manifest.edition,
        prefix=manifest.prefix,
        bin_dir=manifest.bin_dir,
        created_at=manifest.created_at,
        owned_paths=manifest.owned_paths,
        managed_roots=manifest.managed_roots,
        created_roots=manifest.created_roots,
        wrappers=manifest.wrappers,
        log_roots=manifest.log_roots,
        authority_roots=manifest.authority_roots,
        external_paths=manifest.external_paths,
        shell=manifest.shell,
        docker=manifest.docker,
        systemd_units=manifest.systemd_units,
    )


def ownership_install_uuid(refresh: OwnershipRefresh | None) -> str:
    """Choose the stable install UUID before generating Docker artifacts."""

    return str(uuid.uuid4()) if refresh is None else refresh.install_uuid


def _merged_paths(previous: Iterable[str], current: Iterable[Path]) -> tuple[Path, ...]:
    return tuple(
        Path(value)
        for value in sorted(
            {
                *(str(_canonical(Path(value))) for value in previous),
                *(str(_canonical(path)) for path in current),
            }
        )
    )


def _merged_shell(
    refresh: OwnershipRefresh | None,
    shell_bashrc: Path | None,
    bin_dir: Path,
) -> ShellOwnership | None:
    current = (
        None
        if shell_bashrc is None
        else ShellOwnership(
            bashrc=str(_canonical(shell_bashrc)),
            bin_dir=str(bin_dir),
        )
    )
    previous = None if refresh is None else refresh.shell
    if previous is not None and current is not None and previous != current:
        raise OwnershipError("cannot change existing PATH registration owner during refresh")
    return previous if current is None else current


def _merged_docker(
    previous: DockerOwnership | None,
    current: DockerOwnership | None,
) -> DockerOwnership | None:
    if previous is None:
        return current
    if current is None:
        return previous
    if (
        previous.install_uuid != current.install_uuid
        or previous.compose_file != current.compose_file
        or previous.project != current.project
        or (
            previous.context
            and current.context
            and previous.context != current.context
        )
        or (
            previous.engine_id
            and current.engine_id
            and previous.engine_id != current.engine_id
        )
    ):
        raise OwnershipError("cannot change existing Docker ownership boundary during refresh")
    return DockerOwnership(
        install_uuid=current.install_uuid,
        compose_file=current.compose_file,
        project=current.project,
        containers=tuple(sorted({*previous.containers, *current.containers})),
        local_images=tuple(sorted({*previous.local_images, *current.local_images})),
        context=current.context or previous.context,
        engine_id=current.engine_id or previous.engine_id,
    )


def _merged_systemd(
    previous: Iterable[SystemdUnitOwnership],
    current: Iterable[SystemdUnitOwnership],
) -> tuple[SystemdUnitOwnership, ...]:
    units: dict[str, SystemdUnitOwnership] = {}
    for unit in (*tuple(previous), *tuple(current)):
        existing = units.get(unit.name)
        if existing is not None and existing.destination != unit.destination:
            raise OwnershipError(
                f"cannot change systemd unit destination during refresh: {unit.name}"
            )
        units[unit.name] = unit
    return tuple(units[name] for name in sorted(units))


def _validate_refresh_paths(manifest: OwnershipManifest) -> None:
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
            raise OwnershipError(
                f"existing owned path type changed: {path}: "
                f"expected={entry.kind} actual={actual}"
            )
    for value in (
        *manifest.managed_roots,
        *manifest.log_roots,
        *manifest.authority_roots,
    ):
        path = Path(value)
        _ensure_no_symlink_ancestors(path, boundary=prefix)
        if _lexists(path) and (path.is_symlink() or not path.is_dir()):
            raise OwnershipError(
                f"existing managed/preserved root is not a safe directory: {path}"
            )


def install_host_uninstaller_bundle(
    *,
    prefix: Path,
    bin_dir: Path,
    manifest_path: Path | None = None,
    source_package: Path | None = None,
    bundle_root: Path | None = None,
) -> HostUninstallerBundle:
    """Install the stdlib-only maintenance package and host launcher.

    The launcher never enters a tools/development container, so it can remove
    those containers and their images.  It reports a direct error when the
    host has no ``python3`` rather than falling back to a container.
    """

    prefix_path = _canonical(prefix)
    bin_path = _canonical(bin_dir)
    manifest = (
        default_manifest_path(prefix_path)
        if manifest_path is None
        else _canonical(manifest_path)
    )
    source = Path(__file__).resolve().parent if source_package is None else source_package.resolve()
    bundle_path = (
        prefix_path / "maintenance"
        if bundle_root is None
        else _canonical(bundle_root)
    )
    if not _is_descendant(bundle_path, prefix_path):
        raise OwnershipError("host maintenance bundle must be below the prefix")
    if prefix_path.is_symlink() or not prefix_path.is_dir():
        raise OwnershipError(f"prefix is not a safe directory: {prefix_path}")
    _ensure_no_symlink_ancestors(bundle_path, boundary=prefix_path)
    if _lexists(bundle_path) and (bundle_path.is_symlink() or not bundle_path.is_dir()):
        raise OwnershipError(
            f"host maintenance bundle boundary is not a safe directory: {bundle_path}"
        )
    if _within_or_equal(source, bundle_path) or _within_or_equal(bundle_path, source):
        raise OwnershipError("host maintenance bundle and source package cannot overlap")
    package_root = bundle_path / "elesim_setup"
    if _lexists(package_root) and (package_root.is_symlink() or not package_root.is_dir()):
        raise OwnershipError(
            f"host maintenance package boundary is not a safe directory: {package_root}"
        )
    package_root.mkdir(parents=True, exist_ok=True)
    files: list[Path] = []
    init = package_root / "__init__.py"
    _atomic_text(init, '"""EleSim host-only uninstall maintenance bundle."""\n', mode=0o644)
    files.append(init)
    for name in (
        "ownership.py",
        "build_progress.py",
        "releases.py",
        "shell.py",
        "uninstall.py",
        "host_helper.py",
        "operation_lock.py",
        "instance_identity.py",
        "instance_remove.py",
        "manager_ownership.py",
    ):
        source_file = source / name
        if not source_file.is_file():
            raise OwnershipError(f"host uninstaller source is missing: {source_file}")
        destination = package_root / name
        _atomic_copy(source_file, destination, mode=0o644)
        files.append(destination)

    wrapper = bin_path / "elesim-uninstall"
    script = (
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "if ! command -v python3 >/dev/null 2>&1; then\n"
        "  printf 'host python3 is required to run the EleSim safe uninstaller.\\n' >&2\n"
        "  exit 127\n"
        "fi\n"
        f"export PYTHONPATH={shlex.quote(str(bundle_path))}\n"
        "export PYTHONNOUSERSITE=1\n"
        "export PYTHONDONTWRITEBYTECODE=1\n"
        f"cd -- {shlex.quote(str(bundle_path))}\n"
        "exec python3 -B -S -m elesim_setup.uninstall --manifest "
        + shlex.quote(str(manifest))
        + ' "$@"\n'
    )
    _atomic_text(wrapper, script, mode=0o755)
    return HostUninstallerBundle(bundle_path, wrapper, tuple(files))


def _validate_refresh_token(
    refresh: OwnershipRefresh,
    *,
    destination: Path,
    prefix: Path,
    bin_dir: Path,
    edition: str,
) -> None:
    if (
        refresh.manifest_path != str(destination)
        or refresh.prefix != str(prefix)
        or refresh.bin_dir != str(bin_dir)
        or refresh.edition != edition
    ):
        raise OwnershipError("ownership refresh token differs from the current installation boundary")
    if not _lexists(destination) or destination.is_symlink() or not destination.is_file():
        raise OwnershipError("target ownership manifest disappeared or changed during refresh")
    if sha256_file(destination) != refresh.manifest_sha256:
        raise OwnershipError("ownership manifest changed during installation")
    current = OwnershipManifest.load(destination)
    if current.install_uuid != refresh.install_uuid:
        raise OwnershipError("ownership UUID changed during installation")


def _atomic_text(destination: Path, content: str, *, mode: int) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        dir=destination.parent,
        prefix=f".{destination.name}.",
        delete=False,
    ) as handle:
        handle.write(content)
        temporary = Path(handle.name)
    temporary.chmod(mode)
    os.replace(temporary, destination)


def _atomic_copy(source: Path, destination: Path, *, mode: int) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "wb",
        dir=destination.parent,
        prefix=f".{destination.name}.",
        delete=False,
    ) as handle:
        with source.open("rb") as source_handle:
            shutil.copyfileobj(source_handle, handle)
        temporary = Path(handle.name)
    temporary.chmod(mode)
    os.replace(temporary, destination)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _mapping(value: object, *, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise OwnershipError(f"{name} is not a JSON object")
    return value


def _sequence(value: object, *, name: str) -> Sequence[object]:
    if not isinstance(value, (list, tuple)):
        raise OwnershipError(f"{name} is not a JSON array")
    return value


def _canonical(path: Path) -> Path:
    value = os.path.abspath(os.fspath(path.expanduser()))
    if "\x00" in value or "\n" in value or "\r" in value:
        raise OwnershipError("paths cannot contain NUL or newline characters")
    return Path(value)


def _require_absolute(value: str, *, name: str) -> Path:
    if not isinstance(value, str) or not value or "\x00" in value:
        raise OwnershipError(f"{name} is not a valid path")
    path = Path(value)
    if not path.is_absolute() or str(_canonical(path)) != value:
        raise OwnershipError(f"{name} must be a normalized absolute path: {value!r}")
    return path


def _validate_uuid(value: str, *, name: str) -> str:
    try:
        parsed = uuid.UUID(value)
    except (AttributeError, TypeError, ValueError) as exc:
        raise OwnershipError(f"{name} is not a valid UUID") from exc
    if str(parsed) != value:
        raise OwnershipError(f"{name} must be a canonical UUID string")
    return value


def _validated_unique_paths(values: Iterable[str], *, name: str) -> tuple[Path, ...]:
    result = tuple(_require_absolute(value, name=name) for value in values)
    if len({str(path) for path in result}) != len(result):
        raise OwnershipError(f"{name} contains duplicate paths")
    return result


def _ensure_no_symlink_ancestors(path: Path, *, boundary: Path) -> None:
    if not _within_or_equal(path, boundary):
        raise OwnershipError(f"path is outside the ownership boundary: {path}")
    current = path.parent
    while _within_or_equal(current, boundary):
        if _lexists(current) and current.is_symlink():
            raise OwnershipError(f"path ancestor is a symlink: {current}")
        if current == boundary:
            break
        current = current.parent


def _within_or_equal(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def _is_descendant(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return path != root
    except ValueError:
        return False


def _protected(path: Path, roots: Iterable[Path]) -> bool:
    return any(path == root or _is_descendant(path, root) for root in roots)


def _lexists(path: Path) -> bool:
    return os.path.lexists(path)


__all__ = [
    "DockerOwnership",
    "DOCKER_INSTALL_UUID_LABEL",
    "DOCKER_BUILD_FINGERPRINT_LABEL",
    "HostUninstallerBundle",
    "OWNERSHIP_MANIFEST_NAME",
    "OWNERSHIP_SCHEMA_VERSION",
    "OwnedPath",
    "OwnershipError",
    "OwnershipManifest",
    "OwnershipRefresh",
    "ShellOwnership",
    "SystemdUnitOwnership",
    "WrapperOwnership",
    "default_manifest_path",
    "append_docker_image_ownership",
    "append_instance_docker_ownership",
    "append_manager_docker_ownership",
    "inventory_paths",
    "install_host_uninstaller_bundle",
    "ownership_install_uuid",
    "prepare_ownership_refresh",
    "sha256_file",
    "write_ownership_manifest",
]
