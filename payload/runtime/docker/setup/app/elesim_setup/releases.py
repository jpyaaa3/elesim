"""Immutable, install-scoped release manifests."""

from __future__ import annotations

import hashlib
import ctypes
import errno
import json
import os
import re
import shutil
import tempfile
import uuid
import fcntl
import stat
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Mapping

from .instance_identity import image_reference


_SOURCE_REVISION = re.compile(r"(?:git-[0-9a-f]{40}|sha256-[0-9a-f]{64})\Z")
_DIGEST = re.compile(r"[0-9a-f]{64}\Z")
_IMAGE_ID = re.compile(r"sha256:[0-9a-f]{64}\Z")
_ROLE = re.compile(r"[a-z][a-z0-9_]{0,62}\Z")
_PLATFORMS = frozenset(("linux/amd64", "linux/arm64"))
_DOCKER_ROLES = frozenset(("pilot", "sim", "ui"))
_FIELDS = (
    "schema_version",
    "install_uuid",
    "source_revision",
    "platform",
    "role_images",
    "image_ids",
    "build_fingerprints",
    "runtime_data_digest",
)


def _canonical_uuid(value: object) -> str:
    if not isinstance(value, str):
        raise ValueError("install_uuid must be a canonical UUID string")
    try:
        parsed = uuid.UUID(value)
    except (ValueError, AttributeError, TypeError) as exc:
        raise ValueError("install_uuid must be a canonical UUID string") from exc
    if str(parsed) != value:
        raise ValueError("install_uuid must be a canonical lowercase UUID string")
    return value


def _mapping(value: object, name: str) -> dict[str, str]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    result = {str(key): item for key, item in value.items()}
    if any(not isinstance(key, str) or not isinstance(item, str) for key, item in value.items()):
        raise ValueError(f"{name} must contain string keys and values")
    if any(_ROLE.fullmatch(key) is None for key in result):
        raise ValueError(f"{name} contains an invalid role")
    return dict(sorted(result.items()))


@dataclass(frozen=True)
class ReleaseManifest:
    install_uuid: str
    source_revision: str
    platform: str
    role_images: Mapping[str, str]
    image_ids: Mapping[str, str]
    build_fingerprints: Mapping[str, str]
    runtime_data_digest: str
    schema_version: int = 1

    def validate(self) -> "ReleaseManifest":
        if type(self.schema_version) is not int or self.schema_version != 1:
            raise ValueError("unsupported release manifest schema_version")
        install = _canonical_uuid(self.install_uuid)
        if (
            not isinstance(self.source_revision, str)
            or _SOURCE_REVISION.fullmatch(self.source_revision) is None
        ):
            raise ValueError(
                "source_revision must be git-<40 lowercase hex> or sha256-<64 lowercase hex>"
            )
        if not isinstance(self.platform, str) or self.platform not in _PLATFORMS:
            raise ValueError("platform must be linux/amd64 or linux/arm64")
        images = _mapping(self.role_images, "role_images")
        ids = _mapping(self.image_ids, "image_ids")
        fingerprints = _mapping(self.build_fingerprints, "build_fingerprints")
        roles = set(images)
        if not roles or roles != set(ids) or roles != set(fingerprints):
            raise ValueError("role_images, image_ids and build_fingerprints must have the same roles")
        if not roles <= _DOCKER_ROLES:
            raise ValueError("release manifests may contain only pilot, sim, and ui images")
        for role in sorted(roles):
            fingerprint = fingerprints[role]
            if _DIGEST.fullmatch(fingerprint) is None:
                raise ValueError(f"build_fingerprints[{role!r}] must be 64 lowercase hex characters")
            if images[role] != image_reference(install, role, fingerprint):
                raise ValueError(f"role_images[{role!r}] does not match its install and fingerprint")
            if _IMAGE_ID.fullmatch(ids[role]) is None:
                raise ValueError(f"image_ids[{role!r}] must be sha256:<64 lowercase hex characters>")
        if not isinstance(self.runtime_data_digest, str) or _DIGEST.fullmatch(self.runtime_data_digest) is None:
            raise ValueError("runtime_data_digest must be 64 lowercase hex characters")
        return ReleaseManifest(
            install_uuid=install,
            source_revision=self.source_revision,
            platform=self.platform,
            role_images=images,
            image_ids=ids,
            build_fingerprints=fingerprints,
            runtime_data_digest=self.runtime_data_digest,
            schema_version=1,
        )

    def fields(self) -> dict[str, object]:
        value = self.validate()
        return {
            "schema_version": value.schema_version,
            "install_uuid": value.install_uuid,
            "source_revision": value.source_revision,
            "platform": value.platform,
            "role_images": dict(value.role_images),
            "image_ids": dict(value.image_ids),
            "build_fingerprints": dict(value.build_fingerprints),
            "runtime_data_digest": value.runtime_data_digest,
        }

    def to_dict(self) -> dict[str, object]:
        fields = self.fields()
        return {**fields, "release_key": release_key(fields)}


def _canonical_json(value: Mapping[str, object]) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True) + "\n").encode()


def release_key(manifest: ReleaseManifest | Mapping[str, object]) -> str:
    """Hash the canonical manifest fields (never an embedded release key)."""

    if isinstance(manifest, ReleaseManifest):
        fields = manifest.fields()
    elif isinstance(manifest, Mapping):
        fields = dict(manifest)
    else:
        raise ValueError("release manifest must be an object")
    if set(fields) != set(_FIELDS):
        raise ValueError("release fields are incomplete or contain unknown fields")
    fields = ReleaseManifest(**fields).validate().fields()
    return hashlib.sha256(_canonical_json(fields)).hexdigest()


def _read_manifest(path: Path) -> ReleaseManifest:
    try:
        raw = json.loads(_read_regular_file(path).decode("utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid release manifest: {path}") from exc
    if not isinstance(raw, dict) or set(raw) != set(_FIELDS) | {"release_key"}:
        raise ValueError("release manifest fields are invalid")
    manifest = ReleaseManifest(**{key: raw[key] for key in _FIELDS}).validate()
    if raw["release_key"] != release_key(manifest):
        raise ValueError("release manifest content does not match its release key")
    return manifest


def _reject_symlinked_ancestors(path: Path) -> None:
    for current in (path, *path.parents):
        if current.is_symlink():
            raise ValueError("release path contains a symlinked ancestor")


def _read_regular_file(path: Path) -> bytes:
    """Read one owned regular file without following a replaced final path."""

    _reject_symlinked_ancestors(path)
    try:
        fd = os.open(
            path,
            os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC,
        )
    except OSError as exc:
        raise ValueError(f"release file is unsafe: {path}") from exc
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise ValueError(f"release file is not an unlinked regular file: {path}")
        with os.fdopen(fd, "rb") as stream:
            fd = -1
            return stream.read()
    finally:
        if fd != -1:
            os.close(fd)


def runtime_data_digest(root: Path) -> str:
    """Hash sorted relative file names and bytes from a runtime data tree."""

    source = Path(root)
    _reject_symlinked_ancestors(source)
    if source.is_symlink() or not source.is_dir():
        raise ValueError("runtime data root must be a real directory")
    entries: list[tuple[str, bytes]] = []
    for current, directories, files in os.walk(source, followlinks=False):
        current_path = Path(current)
        for name in (*directories, *files):
            path = current_path / name
            info = path.lstat()
            if path.is_symlink() or not (path.is_file() or path.is_dir()):
                raise ValueError(f"runtime data contains an unsupported entry: {path}")
        for name in files:
            path = current_path / name
            relative = path.relative_to(source).as_posix()
            entries.append((relative, _read_regular_file(path)))
    digest = hashlib.sha256()
    for relative, content in sorted(entries):
        name = relative.encode("utf-8")
        digest.update(len(name).to_bytes(8, "big"))
        digest.update(name)
        digest.update(len(content).to_bytes(8, "big"))
        digest.update(content)
    return digest.hexdigest()


def _copy_runtime_data(source: Path, destination: Path) -> None:
    destination.mkdir()
    for current, directories, files in os.walk(source, followlinks=False):
        relative = Path(current).relative_to(source)
        target = destination / relative
        for name in (*directories, *files):
            path = Path(current) / name
            info = path.lstat()
            if path.is_symlink() or not (path.is_file() or path.is_dir()):
                raise ValueError(f"runtime data contains an unsupported entry: {path}")
        for name in directories:
            (target / name).mkdir()
        for name in files:
            (target / name).write_bytes(_read_regular_file(Path(current) / name))


def _same_published_release(destination: Path, payload: bytes, digest: str) -> bool:
    manifest = destination / "manifest.json"
    data = destination / "data"
    try:
        return (
            manifest.is_file()
            and not manifest.is_symlink()
            and data.is_dir()
            and not data.is_symlink()
            and _read_regular_file(manifest) == payload
            and runtime_data_digest(data) == digest
        )
    except (OSError, ValueError):
        return False


def _rename_noreplace(source: Path, destination: Path) -> None:
    """Atomically publish a directory without replacing a race winner."""

    try:
        function = ctypes.CDLL(None, use_errno=True).renameat2
    except (AttributeError, OSError) as exc:
        raise ValueError("atomic no-replace directory publication is unavailable") from exc
    function.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    function.restype = ctypes.c_int
    if function(-100, os.fsencode(source), -100, os.fsencode(destination), 1) != 0:
        error = ctypes.get_errno()
        if error == errno.EEXIST:
            raise FileExistsError(destination)
        raise OSError(error, os.strerror(error), destination)


@contextmanager
def _publish_lock(root: Path, *, exclusive: bool) -> Iterator[None]:
    """Hold the release registry's validated publication lock.

    Both readers and writers use this exact lock boundary.  In particular,
    listing cannot race a writer while its ``.digest-*`` staging directory is
    visible, and callers do not need to nest lock acquisition (which would
    make the same-process path susceptible to a deadlock).
    """

    root = Path(root)
    _reject_symlinked_ancestors(root)
    if root.is_symlink() or not root.is_dir():
        raise ValueError("release root must be a real directory")
    lock_path = root / ".publish.lock"
    try:
        lock_fd = os.open(
            lock_path,
            os.O_CREAT
            | os.O_RDWR
            | os.O_NOFOLLOW
            | os.O_NONBLOCK
            | os.O_CLOEXEC,
            0o600,
        )
    except OSError as exc:
        raise ValueError("release lock is unsafe") from exc
    try:
        lock_info = os.fstat(lock_fd)
        if not stat.S_ISREG(lock_info.st_mode) or lock_info.st_nlink != 1:
            raise ValueError("release lock path must be a singly-linked regular file")
        lock = os.fdopen(lock_fd, "a+b")
    except BaseException:
        os.close(lock_fd)
        raise
    try:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH)
        yield
    finally:
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
        finally:
            lock.close()


def publish_release(prefix: Path, manifest: ReleaseManifest, data_source: Path) -> Path:
    """Publish a complete immutable release (manifest plus runtime data)."""

    value = manifest.validate()
    source = Path(data_source)
    digest = runtime_data_digest(source)
    if digest != value.runtime_data_digest:
        raise ValueError("runtime data does not match release manifest digest")
    key = release_key(value)
    root = Path(prefix) / "releases"
    _reject_symlinked_ancestors(root)
    root.mkdir(parents=True, exist_ok=True)
    _reject_symlinked_ancestors(root)
    if not root.is_dir() or root.is_symlink():
        raise ValueError("release root must be a real directory")
    with _publish_lock(root, exclusive=True):
        destination = root / key
        payload = _canonical_json(value.to_dict())
        if destination.exists() or destination.is_symlink():
            if not destination.is_symlink() and _same_published_release(destination, payload, digest):
                return destination
            raise ValueError("release already exists with different, corrupt, or incomplete content")
        temporary = Path(tempfile.mkdtemp(prefix=f".{key}-", dir=str(root)))
        try:
            _copy_runtime_data(source, temporary / "data")
            if runtime_data_digest(temporary / "data") != digest:
                raise ValueError("staged runtime data changed during publication")
            (temporary / "manifest.json").write_bytes(payload)
            try:
                stage_fd = os.open(temporary, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
                try:
                    os.fsync(stage_fd)
                finally:
                    os.close(stage_fd)
                _rename_noreplace(temporary, destination)
            except FileExistsError:
                if _same_published_release(destination, payload, digest):
                    return destination
                raise ValueError("release appeared with different, corrupt, or incomplete content")
            root_fd = os.open(root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
            try:
                os.fsync(root_fd)
            finally:
                os.close(root_fd)
            return destination
        finally:
            if temporary.exists():
                shutil.rmtree(temporary)
        


def load_release(path: Path) -> ReleaseManifest:
    """Load and verify ``manifest.json`` from a release directory or file."""

    source = Path(path)
    _reject_symlinked_ancestors(source)
    if source.is_dir():
        source /= "manifest.json"
    _reject_symlinked_ancestors(source)
    manifest = _read_manifest(source)
    if source.parent.name != release_key(manifest):
        raise ValueError("release directory does not match its content key")
    return manifest


def list_releases(prefix: Path, *, install_uuid: str | None = None) -> tuple[ReleaseManifest, ...]:
    """List complete releases while treating the publisher lock as metadata.

    Unknown entries still fail closed.  In particular, this does not silently
    skip abandoned staging directories or malformed release names.
    """

    root = Path(prefix) / "releases"
    _reject_symlinked_ancestors(root)
    if root.is_symlink() or not root.is_dir():
        raise FileNotFoundError(f"release registry is unavailable: {root}")
    if install_uuid is not None:
        install_uuid = _canonical_uuid(install_uuid)
    found: list[ReleaseManifest] = []
    with _publish_lock(root, exclusive=False):
        for child in sorted(root.iterdir(), key=lambda path: path.name):
            if child.name == ".publish.lock":
                # _publish_lock validates the lock after opening it.  Keep
                # the registry enumeration strict if the directory entry is
                # replaced while it is being scanned.
                try:
                    info = child.lstat()
                except OSError as exc:
                    raise ValueError("release publication lock is unsafe") from exc
                if child.is_symlink() or not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                    raise ValueError("release publication lock is unsafe")
                continue
            if _DIGEST.fullmatch(child.name) is None:
                raise ValueError(f"invalid release registry entry: {child}")
            if child.is_symlink() or not child.is_dir():
                raise ValueError(f"invalid release directory: {child}")
            manifest = load_release(child)
            if install_uuid is not None and manifest.install_uuid != install_uuid:
                raise ValueError(f"release belongs to another installation: {child}")
            found.append(manifest)
    return tuple(found)


__all__ = [
    "ReleaseManifest",
    "list_releases",
    "load_release",
    "publish_release",
    "release_key",
    "runtime_data_digest",
]
