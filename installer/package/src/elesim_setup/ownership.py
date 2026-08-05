"""Install ownership records used by the host-only safe uninstaller.

The manifest is deliberately independent from :mod:`elesim_setup.state`.
Runtime state is mutable and describes how to run Elesim; this file records
which host resources one exact installation is allowed to remove.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import shlex
import shutil
import stat
import tempfile
import uuid
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Mapping, Sequence


OWNERSHIP_SCHEMA_VERSION = 1
OWNERSHIP_MANIFEST_NAME = "install-ownership.json"
DOCKER_INSTALL_UUID_LABEL = "io.elesim.install_uuid"
_INSTALL_EDITIONS = frozenset({"general", "developer"})
_PATH_KINDS = frozenset({"file", "directory", "symlink"})
_DOCKER_NAME = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9_.-]{0,127}$")
_LOCAL_IMAGE = re.compile(r"^elesim/[a-z0-9][a-z0-9_.-]{0,127}:local$")


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
            raise OwnershipError(f"지원하지 않는 설치 산출물 유형입니다: {destination}")
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
            raise OwnershipError(f"wrapper는 symlink가 아닌 일반 파일이어야 합니다: {destination}")
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

    def validate(self) -> "DockerOwnership":
        _validate_uuid(self.install_uuid, name="Docker install UUID")
        _require_absolute(self.compose_file, name="Docker compose file")
        if not _DOCKER_NAME.fullmatch(self.project):
            raise OwnershipError(f"안전하지 않은 Compose project 이름: {self.project!r}")
        if len(set(self.containers)) != len(self.containers):
            raise OwnershipError("Docker container 이름이 중복됩니다")
        if any(not _DOCKER_NAME.fullmatch(value) for value in self.containers):
            raise OwnershipError("Docker container 이름은 고정된 literal이어야 합니다")
        if len(set(self.local_images)) != len(self.local_images):
            raise OwnershipError("Docker image 이름이 중복됩니다")
        if any(not _LOCAL_IMAGE.fullmatch(value) for value in self.local_images):
            raise OwnershipError(
                "삭제 가능한 image는 exact elesim/<name>:local 태그뿐입니다"
            )
        return self


@dataclass(frozen=True)
class SystemdUnitOwnership:
    name: str
    destination: str
    sha256: str

    def validate(self) -> "SystemdUnitOwnership":
        if not self.name.endswith(".service") or not _DOCKER_NAME.fullmatch(self.name):
            raise OwnershipError(f"안전하지 않은 systemd unit 이름: {self.name!r}")
        _require_absolute(self.destination, name="systemd unit destination")
        if not re.fullmatch(r"[0-9a-f]{64}", self.sha256):
            raise OwnershipError(f"systemd unit SHA-256이 유효하지 않습니다: {self.name}")
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
                f"ownership schema {self.schema_version!r}는 지원되지 않습니다"
            )
        try:
            parsed_uuid = uuid.UUID(self.install_uuid)
        except (AttributeError, TypeError, ValueError) as exc:
            raise OwnershipError("install_uuid가 유효한 UUID가 아닙니다") from exc
        if str(parsed_uuid) != self.install_uuid:
            raise OwnershipError("install_uuid는 canonical UUID 문자열이어야 합니다")
        if self.edition not in _INSTALL_EDITIONS:
            raise OwnershipError(f"지원하지 않는 설치 edition: {self.edition!r}")

        prefix = _require_absolute(self.prefix, name="prefix")
        prefix_realpath = _require_absolute(self.prefix_realpath, name="prefix_realpath")
        bin_dir = _require_absolute(self.bin_dir, name="bin_dir")
        _require_absolute(self.bin_dir_realpath, name="bin_dir_realpath")
        manifest_path = _require_absolute(self.manifest_path, name="manifest_path")
        if prefix == Path("/") or bin_dir == Path("/"):
            raise OwnershipError("filesystem root는 install/bin 경계가 될 수 없습니다")
        if not _is_descendant(manifest_path, prefix):
            raise OwnershipError("ownership manifest는 설치 prefix 안에 있어야 합니다")
        if prefix_realpath == Path("/"):
            raise OwnershipError("resolved prefix가 filesystem root일 수 없습니다")

        seen_owned: set[str] = set()
        for entry in self.owned_paths:
            path = _require_absolute(entry.path, name="owned path")
            if entry.kind not in _PATH_KINDS:
                raise OwnershipError(f"지원하지 않는 owned path kind: {entry.kind!r}")
            if entry.path in seen_owned:
                raise OwnershipError(f"중복 owned path: {entry.path}")
            seen_owned.add(entry.path)
            if path == manifest_path:
                raise OwnershipError("manifest 자체는 owned_paths에 넣을 수 없습니다")
            if not (_is_descendant(path, prefix) or _is_descendant(path, bin_dir)):
                raise OwnershipError(f"owned path가 설치 경계 밖입니다: {path}")

        managed = _validated_unique_paths(self.managed_roots, name="managed root")
        for path in managed:
            if path == prefix or not _is_descendant(path, prefix):
                raise OwnershipError(
                    f"managed root는 prefix 자체가 아닌 하위 경로여야 합니다: {path}"
                )
        created = _validated_unique_paths(self.created_roots, name="created root")
        for path in created:
            if not (
                path == prefix
                or _is_descendant(path, prefix)
                or path == bin_dir
                or _is_descendant(path, bin_dir)
            ):
                raise OwnershipError(f"created root가 설치 경계 밖입니다: {path}")

        wrappers: set[str] = set()
        for wrapper in self.wrappers:
            path = _require_absolute(wrapper.path, name="wrapper")
            if not _is_descendant(path, bin_dir):
                raise OwnershipError(f"wrapper가 bin_dir 밖입니다: {path}")
            if wrapper.path in wrappers:
                raise OwnershipError(f"중복 wrapper: {path}")
            wrappers.add(wrapper.path)
            if not re.fullmatch(r"[0-9a-f]{64}", wrapper.sha256):
                raise OwnershipError(f"wrapper SHA-256이 유효하지 않습니다: {path}")

        protected = (
            *_validated_unique_paths(self.log_roots, name="log root"),
            *_validated_unique_paths(self.authority_roots, name="authority root"),
        )
        for path in protected:
            if path == prefix or not _is_descendant(path, prefix):
                raise OwnershipError(f"보존 root가 prefix 하위가 아닙니다: {path}")
        _validated_unique_paths(self.external_paths, name="external path")

        if self.shell is not None:
            _require_absolute(self.shell.bashrc, name="shell bashrc")
            if _require_absolute(self.shell.bin_dir, name="shell bin_dir") != bin_dir:
                raise OwnershipError("shell PATH bin_dir가 manifest bin_dir와 다릅니다")
        if self.docker is not None:
            self.docker.validate()
            if self.docker.install_uuid != self.install_uuid:
                raise OwnershipError("Docker ownership UUID가 install UUID와 다릅니다")
            compose = Path(self.docker.compose_file)
            if not _is_descendant(compose, prefix):
                raise OwnershipError("Compose file이 설치 prefix 밖입니다")
        for unit in self.systemd_units:
            unit.validate()
        return self

    def to_dict(self) -> dict[str, object]:
        return asdict(self)

    @classmethod
    def from_dict(cls, raw: Mapping[str, object]) -> "OwnershipManifest":
        if not isinstance(raw, Mapping):
            raise OwnershipError("ownership manifest가 JSON object가 아닙니다")
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
            raise OwnershipError(f"ownership manifest 필드가 유효하지 않습니다: {exc}") from exc
        return manifest.validate()

    @classmethod
    def load(cls, path: Path) -> "OwnershipManifest":
        source = _canonical(path)
        if source.is_symlink() or not source.is_file():
            raise OwnershipError(f"ownership manifest는 일반 파일이어야 합니다: {source}")
        try:
            raw = json.loads(source.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise OwnershipError(f"ownership manifest를 읽을 수 없습니다: {source}: {exc}") from exc
        manifest = cls.from_dict(_mapping(raw, name="manifest"))
        if source != Path(manifest.manifest_path):
            raise OwnershipError(
                f"manifest 위치가 기록과 다릅니다: actual={source} expected={manifest.manifest_path}"
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
            raise OwnershipError(f"기존 ownership manifest를 덮어쓰지 않습니다: {destination}")
    else:
        _validate_refresh_token(
            refresh,
            destination=destination,
            prefix=prefix_path,
            bin_dir=bin_path,
            edition=edition,
        )
        if install_uuid is not None and str(install_uuid) != refresh.install_uuid:
            raise OwnershipError("refresh 시 install UUID를 변경할 수 없습니다")
        install_uuid = refresh.install_uuid
    if not prefix_path.is_dir() or prefix_path.is_symlink():
        raise OwnershipError(f"prefix는 symlink가 아닌 기존 directory여야 합니다: {prefix_path}")
    if not bin_path.is_dir() or bin_path.is_symlink():
        raise OwnershipError(f"bin_dir는 symlink가 아닌 기존 directory여야 합니다: {bin_path}")

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
    claims = tuple(_canonical(path) for path in claimed_paths)
    for path in claims:
        if not (
            _within_or_equal(path, prefix_path)
            or _within_or_equal(path, bin_path)
        ):
            raise OwnershipError(f"claimed install path가 prefix/bin 밖입니다: {path}")
    if not _lexists(destination):
        existing = tuple(path for path in claims if _lexists(path))
        if existing:
            rendered = "\n".join(f"  - {path}" for path in existing)
            raise OwnershipError(
                "ownership manifest 없는 기존 Elesim 후보 경로를 자동 인수하지 "
                "않습니다. 파일을 정확히 백업·정리하거나 기존 설치기의 clean "
                f"uninstall을 먼저 실행하십시오:\n{rendered}"
            )
        return None
    manifest = OwnershipManifest.load(destination)
    if (
        manifest.prefix != str(prefix_path)
        or manifest.bin_dir != str(bin_path)
        or manifest.edition != edition
    ):
        raise OwnershipError(
            "기존 ownership manifest는 다른 prefix/bin/edition 설치의 소유입니다"
        )
    if prefix_path.is_symlink() or not prefix_path.is_dir():
        raise OwnershipError(f"기존 prefix가 안전한 directory가 아닙니다: {prefix_path}")
    if bin_path.is_symlink() or not bin_path.is_dir():
        raise OwnershipError(f"기존 bin_dir가 안전한 directory가 아닙니다: {bin_path}")
    if str(prefix_path.resolve(strict=True)) != manifest.prefix_realpath:
        raise OwnershipError("기존 prefix realpath가 manifest와 다릅니다")
    if str(bin_path.resolve(strict=True)) != manifest.bin_dir_realpath:
        raise OwnershipError("기존 bin_dir realpath가 manifest와 다릅니다")
    _validate_refresh_paths(manifest)
    for wrapper in manifest.wrappers:
        path = Path(wrapper.path)
        if not _lexists(path):
            continue
        mode = path.lstat().st_mode
        if not stat.S_ISREG(mode) or stat.S_ISLNK(mode):
            raise OwnershipError(f"기존 wrapper가 일반 파일이 아닙니다: {path}")
        if sha256_file(path) != wrapper.sha256:
            raise OwnershipError(
                f"기존 wrapper가 manifest 이후 변경되었습니다: {path}"
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
        raise OwnershipError("refresh에서 기존 PATH registration 소유자를 바꿀 수 없습니다")
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
    ):
        raise OwnershipError("refresh에서 기존 Docker ownership 경계를 바꿀 수 없습니다")
    return DockerOwnership(
        install_uuid=current.install_uuid,
        compose_file=current.compose_file,
        project=current.project,
        containers=tuple(sorted({*previous.containers, *current.containers})),
        local_images=tuple(sorted({*previous.local_images, *current.local_images})),
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
                f"refresh에서 systemd unit 목적지를 바꿀 수 없습니다: {unit.name}"
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
                f"기존 owned path 유형이 변경되었습니다: {path}: "
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
                f"기존 managed/preserved root가 안전한 directory가 아닙니다: {path}"
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
        raise OwnershipError("host maintenance bundle은 prefix 하위여야 합니다")
    if prefix_path.is_symlink() or not prefix_path.is_dir():
        raise OwnershipError(f"prefix가 안전한 directory가 아닙니다: {prefix_path}")
    _ensure_no_symlink_ancestors(bundle_path, boundary=prefix_path)
    if _lexists(bundle_path) and (bundle_path.is_symlink() or not bundle_path.is_dir()):
        raise OwnershipError(
            f"host maintenance bundle 경계가 안전한 directory가 아닙니다: {bundle_path}"
        )
    if _within_or_equal(source, bundle_path) or _within_or_equal(bundle_path, source):
        raise OwnershipError("host maintenance bundle과 source package가 겹칠 수 없습니다")
    package_root = bundle_path / "elesim_setup"
    if _lexists(package_root) and (package_root.is_symlink() or not package_root.is_dir()):
        raise OwnershipError(
            f"host maintenance package 경계가 안전한 directory가 아닙니다: {package_root}"
        )
    package_root.mkdir(parents=True, exist_ok=True)
    files: list[Path] = []
    init = package_root / "__init__.py"
    _atomic_text(init, '"""Elesim host-only uninstall maintenance bundle."""\n', mode=0o644)
    files.append(init)
    for name in ("ownership.py", "shell.py", "uninstall.py", "host_helper.py"):
        source_file = source / name
        if not source_file.is_file():
            raise OwnershipError(f"host uninstaller source가 없습니다: {source_file}")
        destination = package_root / name
        _atomic_copy(source_file, destination, mode=0o644)
        files.append(destination)

    wrapper = bin_path / "elesim-uninstall"
    script = (
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "if ! command -v python3 >/dev/null 2>&1; then\n"
        "  printf 'host python3가 없어 Elesim 안전 제거기를 실행할 수 없습니다.\\n' >&2\n"
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
        raise OwnershipError("ownership refresh token이 현재 설치 경계와 다릅니다")
    if not _lexists(destination) or destination.is_symlink() or not destination.is_file():
        raise OwnershipError("refresh 대상 ownership manifest가 사라졌거나 바뀌었습니다")
    if sha256_file(destination) != refresh.manifest_sha256:
        raise OwnershipError("설치 도중 ownership manifest가 변경되었습니다")
    current = OwnershipManifest.load(destination)
    if current.install_uuid != refresh.install_uuid:
        raise OwnershipError("설치 도중 ownership UUID가 변경되었습니다")


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
        raise OwnershipError(f"{name}이 JSON object가 아닙니다")
    return value


def _sequence(value: object, *, name: str) -> Sequence[object]:
    if not isinstance(value, (list, tuple)):
        raise OwnershipError(f"{name}이 JSON array가 아닙니다")
    return value


def _canonical(path: Path) -> Path:
    value = os.path.abspath(os.fspath(path.expanduser()))
    if "\x00" in value or "\n" in value or "\r" in value:
        raise OwnershipError("경로에 NUL/개행을 사용할 수 없습니다")
    return Path(value)


def _require_absolute(value: str, *, name: str) -> Path:
    if not isinstance(value, str) or not value or "\x00" in value:
        raise OwnershipError(f"{name}가 유효한 경로가 아닙니다")
    path = Path(value)
    if not path.is_absolute() or str(_canonical(path)) != value:
        raise OwnershipError(f"{name}는 정규화된 절대 경로여야 합니다: {value!r}")
    return path


def _validate_uuid(value: str, *, name: str) -> str:
    try:
        parsed = uuid.UUID(value)
    except (AttributeError, TypeError, ValueError) as exc:
        raise OwnershipError(f"{name}가 유효한 UUID가 아닙니다") from exc
    if str(parsed) != value:
        raise OwnershipError(f"{name}는 canonical UUID 문자열이어야 합니다")
    return value


def _validated_unique_paths(values: Iterable[str], *, name: str) -> tuple[Path, ...]:
    result = tuple(_require_absolute(value, name=name) for value in values)
    if len({str(path) for path in result}) != len(result):
        raise OwnershipError(f"{name} 경로가 중복됩니다")
    return result


def _ensure_no_symlink_ancestors(path: Path, *, boundary: Path) -> None:
    if not _within_or_equal(path, boundary):
        raise OwnershipError(f"경로가 ownership boundary 밖입니다: {path}")
    current = path.parent
    while _within_or_equal(current, boundary):
        if _lexists(current) and current.is_symlink():
            raise OwnershipError(f"경로 ancestor가 symlink입니다: {current}")
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
    "inventory_paths",
    "install_host_uninstaller_bundle",
    "ownership_install_uuid",
    "prepare_ownership_refresh",
    "sha256_file",
    "write_ownership_manifest",
]
