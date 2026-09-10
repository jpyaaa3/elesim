"""System-scoped SROS2 views for release instances.

This module consumes an already provisioned, host-scoped SROS2 bundle.  It
does not create an Authority and deliberately copies only the common public
material and the enclave belonging to each endpoint in one ``InstanceState``.
The resulting tree is immutable by generation and selected through a
system-local ``current`` symlink.
"""

from __future__ import annotations

import fcntl
import ctypes
import errno
import hashlib
import json
import os
import re
import shutil
import stat
import tempfile
import uuid
from collections.abc import Iterator, Mapping
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any

from ._security_storage import SecurityAuthorityError, verify_bundle
from .instances import InstanceState


_ROLE = frozenset({"pilot", "sim", "ui"})
_GENERATION = re.compile(r"^[a-z0-9][a-z0-9_.-]{0,95}$")
_PRIVATE_AUTHORITY_FILES = frozenset(
    {"ca.key.pem", "identity_ca.key.pem", "permissions_ca.key.pem"}
)


@dataclass(frozen=True)
class InstanceSecurityResult:
    """Published instance security generation and endpoint mount paths."""

    system_id: str
    generation: str
    root: Path
    manifest: Path
    endpoint_keystores: Mapping[str, Path]

    @property
    def keystore_paths(self) -> Mapping[str, Path]:
        """Alias used by Compose preparation callers."""

        return self.endpoint_keystores

    @property
    def role_keystores(self) -> Mapping[str, Path]:
        """Return the same views keyed by role for role-oriented callers."""

        return {
            path.parts[-2]: path
            for path in self.endpoint_keystores.values()
        }


@dataclass(frozen=True)
class InstanceSecurityActivation:
    system_id: str
    generation: str
    current: Path
    previous_generation: str | None


def stage_instance_security(
    prefix: str | os.PathLike[str],
    install_uuid: str,
    instance: InstanceState,
    generation: str | None = None,
    source_role_views: Mapping[str, str | os.PathLike[str]] | None = None,
    *,
    release_key: str | None = None,
) -> InstanceSecurityResult:
    """Validate and publish one endpoint-scoped security generation.

    ``source_role_views`` may contain a schema-2 host bundle root, an
    ``apps/<role>`` directory, or its ``keystore`` directory.  A bundle root
    is verified with the exact Authority bundle validator.  Individual views
    are checked against the same canonical public/enclave layout before being
    copied.  Publication is no-replace: an existing generation is never
    overwritten.
    """

    _validate_install_uuid(install_uuid)
    instance.validate()
    roles = tuple(endpoint.role for endpoint in instance.endpoints)
    if any(role not in _ROLE for role in roles):
        raise SecurityAuthorityError("instance security does not support Robot")
    if len(set(roles)) != len(roles):
        raise SecurityAuthorityError("duplicate roles are not supported")
    generation_id = _resolve_generation(generation, release_key)
    if source_role_views is None or not isinstance(source_role_views, Mapping):
        raise ValueError("source_role_views must be an object")
    expected_roles = set(roles)
    if set(source_role_views) != expected_roles:
        raise SecurityAuthorityError(
            "source role views must exactly match instance endpoints"
        )

    # Validate all external inputs before creating anything below the instance
    # prefix.  The second validation is intentionally not delegated to a
    # caller: a malformed role view must not leave a half-created instance.
    validated_sources = {
        role: _source_view(Path(source_role_views[role]), role=role, instance=instance)
        for role in sorted(expected_roles)
    }
    for role, source in validated_sources.items():
        _validate_role_view(source, role=role, instance=instance)

    prefix_path = _secure_path(Path(prefix))
    instance_root = prefix_path / "instances" / instance.system_id
    security_root = instance_root / "security"
    with _system_lock(instance_root, security_root):
        _private_directory(security_root / "generations")
        staging_parent = security_root / ".staging"
        _private_directory(staging_parent)
        staging = staging_parent / f"{generation_id}-{uuid.uuid4().hex}"
        staging.mkdir(mode=0o700)
        try:
            for role in sorted(expected_roles):
                source = validated_sources[role]
                _copy_role_view(source, staging / "apps" / role / "keystore")

            files = {
                path.relative_to(staging).as_posix(): _sha256(path)
                for path in _regular_files(staging)
            }
            payload = {
                "schema_version": 1,
                "install_uuid": install_uuid,
                "system_id": instance.system_id,
                "generation": generation_id,
                "instance_release_key": instance.release_key,
                "endpoints": [
                    {
                        "role": endpoint.role,
                        "endpoint_id": endpoint.endpoint_id,
                        "enclave": _enclave_relative(
                            instance.system_id, endpoint.role, endpoint.endpoint_id
                        ).as_posix(),
                    }
                    for endpoint in instance.endpoints
                ],
                "files": dict(sorted(files.items())),
            }
            _write_json(staging / "manifest.json", payload)
            _validate_instance_manifest(
                payload,
                install_uuid=install_uuid,
                system_id=instance.system_id,
                generation=generation_id,
                instance=instance,
            )
            _validate_published(staging, payload)
            published = security_root / "generations" / generation_id
            if published.exists() or published.is_symlink():
                raise FileExistsError(f"instance security generation exists: {generation_id}")
            # The per-system lock makes this checked rename an atomic,
            # no-replace publication for all EleSim writers.
            _rename_noreplace(staging, published)
            _fsync_directory(published.parent)
        except BaseException:
            if staging.exists() and not staging.is_symlink():
                shutil.rmtree(staging)
            raise

    published = security_root / "generations" / generation_id
    endpoint_keystores = {
        endpoint.endpoint_id: published / "apps" / endpoint.role / "keystore"
        for endpoint in instance.endpoints
    }
    return InstanceSecurityResult(
        instance.system_id,
        generation_id,
        published,
        published / "manifest.json",
        endpoint_keystores,
    )


def activate_instance_security(
    prefix: str | os.PathLike[str],
    install_uuid: str,
    instance_or_system: InstanceState | str,
    generation: str,
) -> InstanceSecurityActivation:
    """Atomically select a previously published generation for one system."""

    _validate_install_uuid(install_uuid)
    system_id = (
        instance_or_system.system_id
        if isinstance(instance_or_system, InstanceState)
        else str(instance_or_system)
    )
    if not re.fullmatch(r"[a-z][a-z0-9_]{0,62}", system_id):
        raise ValueError("system_id must be a safe identifier")
    _validate_generation(generation)
    prefix_path = _secure_path(Path(prefix))
    instance_root = prefix_path / "instances" / system_id
    security_root = instance_root / "security"
    with _system_lock(instance_root, security_root):
        generation_root = security_root / "generations" / generation
        if generation_root.is_symlink() or not generation_root.is_dir():
            raise FileNotFoundError(generation_root)
        manifest_path = generation_root / "manifest.json"
        payload = _read_json(manifest_path)
        _validate_instance_manifest(
            payload,
            install_uuid=install_uuid,
            system_id=system_id,
            generation=generation,
            instance=instance_or_system if isinstance(instance_or_system, InstanceState) else None,
        )
        _validate_published(generation_root, payload)

        current = security_root / "current"
        previous: str | None = None
        if current.is_symlink():
            target = current.resolve()
            generations_root = (security_root / "generations").resolve()
            if generations_root not in target.parents:
                raise SecurityAuthorityError("instance security current escapes generations")
            previous = target.name
        elif current.exists():
            raise SecurityAuthorityError("instance security current must be a symlink")
        temporary = security_root / f".current-{uuid.uuid4().hex}"
        os.symlink(Path("generations") / generation, temporary)
        try:
            os.replace(temporary, current)
            _fsync_directory(security_root)
        finally:
            if temporary.is_symlink() or temporary.exists():
                temporary.unlink()
        return InstanceSecurityActivation(system_id, generation, current, previous)


def active_instance_security_path(
    prefix: str | os.PathLike[str],
    install_uuid: str,
    system_id: str,
    role: str,
) -> Path:
    """Return the active endpoint keystore mount path for Compose."""

    if not isinstance(role, str) or (
        role not in _ROLE
        and re.fullmatch(r"[a-z][a-z0-9_-]{0,62}", role) is None
    ):
        raise ValueError(f"unsupported instance role or endpoint: {role!r}")
    _validate_install_uuid(install_uuid)
    if not re.fullmatch(r"[a-z][a-z0-9_]{0,62}", str(system_id)):
        raise ValueError("system_id must be a safe identifier")
    current = _secure_path(Path(prefix)) / "instances" / str(system_id) / "security" / "current"
    if not current.is_symlink():
        raise FileNotFoundError(current)
    target = current.resolve()
    generations = (current.parent / "generations").resolve()
    if generations not in target.parents:
        raise SecurityAuthorityError("instance security current escapes generations")
    manifest_path = target / "manifest.json"
    manifest_payload = _read_json(manifest_path)
    generation = target.name
    _validate_instance_manifest(
        manifest_payload,
        install_uuid=install_uuid,
        system_id=str(system_id),
        generation=generation,
    )
    _validate_published(target, manifest_payload)
    # The public API accepts either the endpoint id (the stable instance
    # identity) or its role.  Existing role-oriented Compose callers can
    # therefore consume an endpoint-scoped generation without a compatibility
    # copy of the enclave.
    manifest = manifest_payload
    endpoints = manifest.get("endpoints")
    if not isinstance(endpoints, list):
        raise SecurityAuthorityError("instance security manifest has no endpoints")
    selected = next(
        (
            row
            for row in endpoints
            if isinstance(row, Mapping)
            and (row.get("endpoint_id") == role or row.get("role") == role)
        ),
        None,
    )
    if selected is None or not isinstance(selected.get("endpoint_id"), str) or not isinstance(selected.get("role"), str):
        raise FileNotFoundError(current)
    path = current / "apps" / selected["role"] / "keystore"
    if path.is_symlink() or not path.is_dir():
        raise FileNotFoundError(path)
    return path


def validate_instance_security_generation(
    root: str | os.PathLike[str],
    install_uuid: str,
    instance: InstanceState,
    generation: str | None = None,
) -> InstanceSecurityResult:
    """Read-only validation for a generation crossing into instance runtime.

    A caller-supplied directory is not trusted merely because its manifest
    repeats the expected identities.  The complete digest set, layout, role
    views, enclave selection, and absence of Authority private material are
    checked again immediately before the runtime copies it.
    """

    _validate_install_uuid(install_uuid)
    instance.validate()
    source = _secure_path(Path(root))
    if source.is_symlink() or not source.is_dir():
        raise SecurityAuthorityError("instance security generation is not a directory")
    generation_id = source.name if generation is None else generation
    _validate_generation(generation_id)
    if source.name != generation_id:
        raise SecurityAuthorityError("instance security generation path does not match its id")
    if instance.security_generation and instance.security_generation != generation_id:
        raise SecurityAuthorityError("instance security generation does not match instance state")

    manifest = source / "manifest.json"
    payload = _read_json(manifest)
    _validate_instance_manifest(
        payload,
        install_uuid=install_uuid,
        system_id=instance.system_id,
        generation=generation_id,
        instance=instance,
    )
    _validate_published(source, payload)

    children = {child.name: child for child in source.iterdir()}
    if set(children) != {"apps", "manifest.json"}:
        raise SecurityAuthorityError("instance security generation layout is invalid")
    apps = children["apps"]
    if apps.is_symlink() or not apps.is_dir():
        raise SecurityAuthorityError("instance security apps path is invalid")
    expected_roles = {endpoint.role for endpoint in instance.endpoints}
    role_entries = {child.name: child for child in apps.iterdir()}
    if set(role_entries) != expected_roles:
        raise SecurityAuthorityError("instance security role views do not match instance")

    endpoint_keystores: dict[str, Path] = {}
    for endpoint in instance.endpoints:
        role_root = role_entries[endpoint.role]
        if role_root.is_symlink() or not role_root.is_dir():
            raise SecurityAuthorityError("instance security role path is invalid")
        role_children = {child.name: child for child in role_root.iterdir()}
        if set(role_children) != {"keystore"}:
            raise SecurityAuthorityError("instance security role layout is invalid")
        keystore = role_children["keystore"]
        _validate_role_view(keystore, role=endpoint.role, instance=instance)
        endpoint_keystores[endpoint.endpoint_id] = keystore

    return InstanceSecurityResult(
        instance.system_id,
        generation_id,
        source,
        manifest,
        endpoint_keystores,
    )


def _source_view(path: Path, *, role: str, instance: InstanceState) -> Path:
    source = _secure_path(path)
    bundle_root: Path | None = None
    for candidate in (source, *source.parents):
        manifest = candidate / "manifest.json"
        if manifest.is_symlink():
            raise SecurityAuthorityError(f"security manifest must not be a symlink: {manifest}")
        if manifest.is_file():
            bundle_root = candidate
            break
        if candidate == source.parent.parent.parent.parent:
            break
    if bundle_root is not None:
        artifact = verify_bundle(bundle_root)
        if artifact.manifest.system_id != instance.system_id:
            raise SecurityAuthorityError("source bundle belongs to another system")
        identities = [identity for identity in artifact.manifest.enclaves if identity.role == role]
        endpoint = next(e for e in instance.endpoints if e.role == role)
        if len(identities) != 1 or identities[0].endpoint_id != endpoint.endpoint_id:
            raise SecurityAuthorityError("source bundle enclave does not match endpoint")
        candidate = bundle_root / "apps" / role / "keystore"
        if not candidate.is_dir():
            raise SecurityAuthorityError(f"source bundle has no role view: {role}")
        return _secure_path(candidate)
    if source.name == "keystore":
        return source
    if (source / "keystore").is_dir():
        return _secure_path(source / "keystore")
    return source


def _validate_role_view(source: Path, *, role: str, instance: InstanceState) -> None:
    if source.is_symlink() or not source.is_dir():
        raise SecurityAuthorityError(f"role security view is not a directory: {source}")
    children = {child.name for child in source.iterdir()}
    if children != {"public", "enclaves"}:
        raise SecurityAuthorityError("role view must contain exactly public/ and enclaves/")
    enclave = source / _enclave_relative(instance.system_id, role, next(e.endpoint_id for e in instance.endpoints if e.role == role))
    if not (source / "public").is_dir() or not enclave.is_dir():
        raise SecurityAuthorityError("role view is missing public material or its enclave")
    for path in _walk_paths(source):
        mode = path.stat(follow_symlinks=False).st_mode
        if stat.S_ISLNK(mode):
            raise SecurityAuthorityError(f"security view contains a symlink: {path}")
        if stat.S_ISDIR(mode):
            continue
        if not stat.S_ISREG(mode):
            raise SecurityAuthorityError(f"security view contains a special file: {path}")
        if path.stat(follow_symlinks=False).st_nlink != 1:
            raise SecurityAuthorityError(f"security view contains a hardlink: {path}")
        relative = path.relative_to(source)
        lowered = {part.casefold() for part in relative.parts}
        if "private" in lowered or path.name.casefold() in _PRIVATE_AUTHORITY_FILES:
            raise SecurityAuthorityError(f"authority private material is forbidden: {relative}")
        if "authority" in lowered and "private" in lowered:
            raise SecurityAuthorityError(f"authority private material is forbidden: {relative}")
        if relative.parts[0] == "public" and path.name.casefold().endswith(".key.pem"):
            raise SecurityAuthorityError(f"private key in public material: {relative}")
    expected_root = source / "enclaves" / "elesim" / instance.system_id / role
    for path in (source / "enclaves").rglob("*"):
        if path.is_dir() and path not in expected_root.parents and expected_root not in path.parents and path != expected_root:
            raise SecurityAuthorityError(f"role view contains an unrelated enclave path: {path}")
    if not any(path.is_file() for path in (source / "public").rglob("*")):
        raise SecurityAuthorityError("role view has no common public material")
    if not any(path.is_file() for path in enclave.rglob("*")):
        raise SecurityAuthorityError("role view has an empty enclave")


def _copy_role_view(source: Path, destination: Path) -> None:
    destination.mkdir(mode=0o700, parents=True)
    for path in _walk_paths(source):
        relative = path.relative_to(source)
        target = destination / relative
        if path.is_dir():
            target.mkdir(mode=0o700, exist_ok=True)
            target.chmod(0o700)
        else:
            target.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            _copy_regular_file(path, target)
            target.chmod(0o600)


def _copy_regular_file(source: Path, destination: Path) -> None:
    """Copy from an opened, no-follow descriptor, fencing replacement races."""

    source_fd = os.open(source, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        source_stat = os.fstat(source_fd)
        if not stat.S_ISREG(source_stat.st_mode) or source_stat.st_nlink != 1:
            raise SecurityAuthorityError(f"security source is not a unique regular file: {source}")
        target_fd = os.open(
            destination,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
            0o600,
        )
        try:
            while True:
                chunk = os.read(source_fd, 1024 * 1024)
                if not chunk:
                    break
                view = memoryview(chunk)
                while view:
                    written = os.write(target_fd, view)
                    view = view[written:]
            os.fsync(target_fd)
        finally:
            os.close(target_fd)
    finally:
        os.close(source_fd)


def _validate_published(root: Path, payload: Mapping[str, Any]) -> None:
    if root.is_symlink() or not root.is_dir():
        raise SecurityAuthorityError("published security generation is not a directory")
    expected = payload.get("files")
    if not isinstance(expected, Mapping):
        raise SecurityAuthorityError("security manifest files must be an object")
    for path in _walk_paths(root):
        mode = path.stat(follow_symlinks=False).st_mode
        if stat.S_ISLNK(mode):
            raise SecurityAuthorityError(f"published security contains a symlink: {path}")
        if not (stat.S_ISDIR(mode) or stat.S_ISREG(mode)):
            raise SecurityAuthorityError(f"published security contains a special file: {path}")
    for relative in expected:
        if not isinstance(relative, str):
            raise SecurityAuthorityError("security manifest path must be a string")
        pure = PurePosixPath(relative)
        if (
            not relative
            or "\\" in relative
            or pure.is_absolute()
            or any(part in {"", ".", ".."} for part in pure.parts)
            or pure.as_posix() != relative
        ):
            raise SecurityAuthorityError(f"invalid security manifest path: {relative}")
    actual = {
        path.relative_to(root).as_posix()
        for path in _regular_files(root)
        if path != root / "manifest.json"
    }
    if actual != set(expected):
        raise SecurityAuthorityError("security manifest file set mismatch")
    for relative, digest in expected.items():
        path = root.joinpath(*PurePosixPath(str(relative)).parts)
        if root not in path.parents or path.is_symlink() or not path.is_file():
            raise SecurityAuthorityError(f"invalid security manifest path: {relative}")
        if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise SecurityAuthorityError(f"invalid security digest: {relative}")
        if _sha256(path) != digest:
            raise SecurityAuthorityError(f"security digest mismatch: {relative}")
    if stat.S_IMODE(root.stat(follow_symlinks=False).st_mode) & 0o077:
        raise SecurityAuthorityError("published security generation is not private")
    for path in _walk_paths(root):
        if path.is_file() and path.stat(follow_symlinks=False).st_nlink != 1:
            raise SecurityAuthorityError(f"published security hardlink: {path}")


def _validate_instance_manifest(
    payload: Mapping[str, Any],
    *,
    install_uuid: str,
    system_id: str,
    generation: str,
    instance: InstanceState | None = None,
) -> None:
    """Validate identity, endpoint paths, and release binding metadata."""

    required = {
        "schema_version",
        "install_uuid",
        "system_id",
        "generation",
        "instance_release_key",
        "endpoints",
        "files",
    }
    if set(payload) != required or payload.get("schema_version") != 1:
        raise SecurityAuthorityError("instance security manifest fields are invalid")
    if payload.get("install_uuid") != install_uuid or payload.get("system_id") != system_id:
        raise SecurityAuthorityError("instance security manifest identity mismatch")
    if payload.get("generation") != generation:
        raise SecurityAuthorityError("instance security manifest generation mismatch")
    release = payload.get("instance_release_key")
    if not isinstance(release, str) or not re.fullmatch(r"[0-9a-f]{64}", release):
        raise SecurityAuthorityError("instance security release key is invalid")
    rows = payload.get("endpoints")
    if not isinstance(rows, list) or not rows:
        raise SecurityAuthorityError("instance security manifest endpoints are invalid")
    files = payload.get("files")
    if not isinstance(files, Mapping):
        raise SecurityAuthorityError("instance security manifest files are invalid")
    file_paths = tuple(files)
    if any(not isinstance(path, str) for path in file_paths):
        raise SecurityAuthorityError("instance security manifest file paths are invalid")
    seen_roles: set[str] = set()
    seen_endpoints: set[str] = set()
    for row in rows:
        if not isinstance(row, Mapping) or set(row) != {"role", "endpoint_id", "enclave"}:
            raise SecurityAuthorityError("instance security endpoint entry is invalid")
        role = row.get("role")
        endpoint_id = row.get("endpoint_id")
        if not isinstance(role, str) or role not in _ROLE:
            raise SecurityAuthorityError("instance security endpoint role is invalid")
        if role in seen_roles:
            raise SecurityAuthorityError("instance security manifest contains duplicate roles")
        if not isinstance(endpoint_id, str) or re.fullmatch(r"[a-z][a-z0-9_-]{0,62}", endpoint_id) is None:
            raise SecurityAuthorityError("instance security endpoint id is invalid")
        if endpoint_id in seen_endpoints:
            raise SecurityAuthorityError("instance security manifest contains duplicate endpoints")
        expected_enclave = _enclave_relative(system_id, role, endpoint_id).as_posix()
        if row.get("enclave") != expected_enclave:
            raise SecurityAuthorityError("instance security endpoint enclave path is invalid")
        enclave_prefix = f"apps/{role}/keystore/{expected_enclave}/"
        if not any(path.startswith(enclave_prefix) for path in file_paths):
            raise SecurityAuthorityError("instance security manifest omits endpoint enclave files")
        seen_roles.add(role)
        seen_endpoints.add(endpoint_id)
    if instance is not None:
        instance.validate()
        if instance.system_id != system_id or instance.release_key != release:
            raise SecurityAuthorityError("instance security release binding does not match instance")
        expected = [
            {
                "role": endpoint.role,
                "endpoint_id": endpoint.endpoint_id,
                "enclave": _enclave_relative(
                    system_id, endpoint.role, endpoint.endpoint_id
                ).as_posix(),
            }
            for endpoint in instance.endpoints
        ]
        if rows != expected:
            raise SecurityAuthorityError("instance security endpoints do not match instance")


def _rename_noreplace(source: Path, destination: Path) -> None:
    """Atomically publish a directory without replacing a race winner."""

    try:
        function = ctypes.CDLL(None, use_errno=True).renameat2
    except (AttributeError, OSError) as exc:
        raise SecurityAuthorityError(
            "atomic no-replace security publication is unavailable"
        ) from exc
    function.argtypes = [
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    ]
    function.restype = ctypes.c_int
    if function(
        -100,
        os.fsencode(source),
        -100,
        os.fsencode(destination),
        1,  # RENAME_NOREPLACE
    ) != 0:
        error = ctypes.get_errno()
        if error == errno.EEXIST:
            raise FileExistsError(destination)
        raise OSError(error, os.strerror(error), destination)


def _enclave_relative(system_id: str, role: str, endpoint_id: str) -> Path:
    endpoint = endpoint_id.replace("-", "_")[:63]
    return Path("enclaves", "elesim", system_id, role, endpoint)


def _resolve_generation(generation: str | None, release_key: str | None) -> str:
    if generation is not None and release_key is not None and generation != release_key:
        raise ValueError("generation and release_key disagree")
    value = generation if generation is not None else release_key
    if value is None:
        raise ValueError("generation is required")
    _validate_generation(value)
    return value


def _validate_generation(value: str) -> None:
    if not isinstance(value, str) or _GENERATION.fullmatch(value) is None:
        raise ValueError(f"invalid security generation: {value!r}")


def _validate_install_uuid(value: str) -> None:
    try:
        parsed = uuid.UUID(value)
    except (ValueError, AttributeError, TypeError) as exc:
        raise ValueError("install_uuid must be a canonical UUID string") from exc
    if not isinstance(value, str) or str(parsed) != value:
        raise ValueError("install_uuid must be a canonical UUID string")


def _secure_path(path: Path) -> Path:
    absolute = Path(os.path.abspath(path.expanduser()))
    current = Path(absolute.anchor)
    for component in absolute.parts[1:]:
        current /= component
        if current.is_symlink():
            raise SecurityAuthorityError(f"security path contains a symlink: {current}")
    return absolute


def _private_directory(path: Path) -> None:
    if path.is_symlink():
        raise SecurityAuthorityError(f"security directory must not be a symlink: {path}")
    path.mkdir(mode=0o700, parents=True, exist_ok=True)
    if not path.is_dir():
        raise SecurityAuthorityError(f"security path is not a directory: {path}")
    path.chmod(0o700)


def _walk_paths(root: Path) -> Iterator[Path]:
    for directory, names, files in os.walk(root, followlinks=False):
        parent = Path(directory)
        for name in sorted(names):
            yield parent / name
        for name in sorted(files):
            yield parent / name


def _regular_files(root: Path) -> tuple[Path, ...]:
    return tuple(sorted(path for path in _walk_paths(root) if path.is_file()))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=path.parent, delete=False) as handle:
        json.dump(payload, handle, ensure_ascii=False, sort_keys=True, indent=2)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
        temporary = Path(handle.name)
    temporary.chmod(0o600)
    os.replace(temporary, path)
    _fsync_directory(path.parent)


def _read_json(path: Path) -> dict[str, Any]:
    def pairs(items: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in items:
            if key in result:
                raise SecurityAuthorityError("security manifest contains a duplicate key")
            result[key] = value
        return result

    fd = -1
    try:
        fd = os.open(
            path,
            os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC,
        )
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_size > 1024 * 1024:
            raise SecurityAuthorityError(
                f"security manifest is not a bounded unlinked regular file: {path}"
            )
        with os.fdopen(fd, "r", encoding="utf-8") as handle:
            fd = -1
            payload = json.load(handle, object_pairs_hook=pairs)
    except SecurityAuthorityError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise SecurityAuthorityError(f"invalid security manifest: {path}") from exc
    finally:
        if fd != -1:
            os.close(fd)
    if not isinstance(payload, dict):
        raise SecurityAuthorityError("security manifest must be an object")
    return payload


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


@contextmanager
def _system_lock(instance_root: Path, security_root: Path) -> Iterator[None]:
    _private_directory(instance_root)
    _private_directory(security_root)
    # InstanceRuntime keeps a stable lock outside the replaceable system
    # directory.  Reuse it when present so direct security stage/activate and
    # runtime registration/rotation serialize on the same per-system lock.
    # Standalone security callers retain the historical in-tree lock; this is
    # also what keeps an unregistered security tombstone self-contained.
    stable_root = instance_root.parent / ".locks"
    if os.path.lexists(stable_root):
        if stable_root.is_symlink() or not stable_root.is_dir():
            raise SecurityAuthorityError("security lock metadata must be a real directory")
        lock = stable_root / f"{instance_root.name}.lock"
    else:
        lock = security_root / ".lock"
    if lock.is_symlink():
        raise SecurityAuthorityError(f"security lock must not be a symlink: {lock}")
    fd = os.open(
        lock,
        os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC,
        0o600,
    )
    try:
        lock_info = os.fstat(fd)
        if not stat.S_ISREG(lock_info.st_mode) or lock_info.st_nlink != 1:
            raise SecurityAuthorityError(
                "security lock must be a singly-linked regular file"
            )
        fcntl.flock(fd, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(fd, fcntl.LOCK_UN)
    finally:
        os.close(fd)


__all__ = [
    "InstanceSecurityActivation",
    "InstanceSecurityResult",
    "activate_instance_security",
    "active_instance_security_path",
    "stage_instance_security",
    "validate_instance_security_generation",
]
