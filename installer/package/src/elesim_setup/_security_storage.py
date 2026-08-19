"""Private filesystem and manifest primitives for the SROS2 authority."""

from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import stat
import tempfile
import uuid
from collections.abc import Mapping
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Any


SROS2_ROLES = frozenset({"pilot", "robot", "sim", "ui"})
_ROS_IDENTIFIER = re.compile(r"^[a-z][a-z0-9_]{0,62}$")
_ENDPOINT_IDENTIFIER = re.compile(r"^[a-z][a-z0-9_-]{0,127}$")
_GENERATION_IDENTIFIER = re.compile(r"^[a-z0-9][a-z0-9_.-]{0,95}$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")


class SecurityAuthorityError(RuntimeError):
    """Raised when an authority transaction or bundle is unsafe."""


@dataclass(frozen=True, order=True)
class EnclaveIdentity:
    """One deployable role identity in an EleSim DDS system."""

    system_id: str
    role: str
    endpoint_id: str

    def __post_init__(self) -> None:
        validate_system_id(self.system_id)
        if self.role not in SROS2_ROLES:
            raise ValueError(f"unsupported SROS2 role: {self.role!r}")
        if not _ENDPOINT_IDENTIFIER.fullmatch(self.endpoint_id):
            raise ValueError(
                "endpoint_id must start with a lowercase letter and contain only "
                "lowercase letters, digits, '-' or '_'"
            )

    @property
    def endpoint_key(self) -> str:
        """Return the ROS-safe endpoint component used by the enclave path."""

        return self.endpoint_id.replace("-", "_")[:63]

    @property
    def enclave(self) -> str:
        return f"/elesim/{self.system_id}/{self.role}/{self.endpoint_key}"

    @property
    def relative_enclave(self) -> Path:
        return Path(*PurePosixPath(self.enclave).parts[1:])

    def to_dict(self) -> dict[str, str]:
        return {
            "role": self.role,
            "endpoint_id": self.endpoint_id,
            "enclave": self.enclave,
        }


@dataclass(frozen=True)
class BundleManifest:
    schema_version: int
    system_id: str
    generation: str
    host_id: str
    created_at: str
    enclaves: tuple[EnclaveIdentity, ...]
    files: Mapping[str, str]

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "system_id": self.system_id,
            "generation": self.generation,
            "host_id": self.host_id,
            "created_at": self.created_at,
            "enclaves": [identity.to_dict() for identity in self.enclaves],
            "files": dict(sorted(self.files.items())),
        }


@dataclass(frozen=True)
class BundleArtifact:
    path: Path
    manifest: BundleManifest


def verify_bundle(root: Path) -> BundleArtifact:
    """Verify a bundle's shape, modes and SHA-256 manifest."""

    bundle = secure_absolute(root)
    validate_tree(bundle)
    raw = read_json_object(bundle / "manifest.json")
    manifest = _manifest_from_dict(raw)
    expected = dict(manifest.files)
    actual_paths = {
        path.relative_to(bundle).as_posix()
        for path in regular_files(bundle)
        if path != bundle / "manifest.json"
    }
    if actual_paths != set(expected):
        missing = sorted(set(expected) - actual_paths)
        extra = sorted(actual_paths - set(expected))
        raise SecurityAuthorityError(
            f"bundle manifest file set mismatch; missing={missing}, extra={extra}"
        )
    for relative, digest in expected.items():
        path = _safe_manifest_path(bundle, relative)
        if file_sha256(path) != digest:
            raise SecurityAuthorityError(f"bundle digest mismatch: {relative}")
    if (bundle / "keystore/private").exists():
        raise SecurityAuthorityError("bundle must not contain authority private material")
    for authority_key in (
        "ca.key.pem",
        "identity_ca.key.pem",
        "permissions_ca.key.pem",
    ):
        if any(path.name == authority_key for path in regular_files(bundle)):
            raise SecurityAuthorityError(
                f"bundle contains authority private material: {authority_key}"
            )

    public_root = bundle / "keystore/public"
    enclaves_root = bundle / "keystore/enclaves"
    for required in (public_root, enclaves_root):
        if required.is_symlink() or not required.is_dir():
            raise SecurityAuthorityError(
                f"bundle is missing {required.relative_to(bundle)}/"
            )
    allowed_roots = [
        public_root.resolve(),
        *[
            (enclaves_root / identity.relative_enclave).resolve()
            for identity in manifest.enclaves
        ],
    ]
    for enclave_root in allowed_roots[1:]:
        if not enclave_root.is_dir():
            raise SecurityAuthorityError(
                f"bundle is missing assigned enclave: {enclave_root.relative_to(bundle)}"
            )
    for identity in manifest.enclaves:
        role_keystore = bundle / "roles" / identity.role / "keystore"
        role_public = role_keystore / "public"
        role_enclave = (
            role_keystore / "enclaves" / identity.relative_enclave
        )
        for required in (role_public, role_enclave):
            if required.is_symlink() or not required.is_dir():
                raise SecurityAuthorityError(
                    f"bundle is missing role view: {required.relative_to(bundle)}/"
                )
        _require_matching_tree(public_root, role_public)
        _require_matching_tree(
            enclaves_root / identity.relative_enclave,
            role_enclave,
        )
        allowed_roots.extend((role_public.resolve(), role_enclave.resolve()))
    for relative in expected:
        path = _safe_manifest_path(bundle, relative).resolve()
        if not any(path == allowed or allowed in path.parents for allowed in allowed_roots):
            raise SecurityAuthorityError(f"bundle contains an unassigned path: {relative}")
    _reject_unassigned_directories(bundle, allowed_roots)
    require_private_modes(bundle)
    return BundleArtifact(bundle, manifest)


def _reject_unassigned_directories(
    bundle: Path,
    allowed_roots: list[Path],
) -> None:
    bundle_root = bundle.resolve()
    allowed_ancestors = {bundle_root}
    for allowed in allowed_roots:
        allowed_ancestors.update(
            parent
            for parent in allowed.parents
            if bundle_root in (parent, *parent.parents)
        )
    for directory in (path for path in bundle.rglob("*") if path.is_dir()):
        resolved = directory.resolve()
        if resolved in allowed_ancestors:
            continue
        if any(allowed == resolved or allowed in resolved.parents for allowed in allowed_roots):
            continue
        raise SecurityAuthorityError(
            f"bundle contains an unassigned directory: {directory.relative_to(bundle)}"
        )


def _require_matching_tree(source: Path, mirror: Path) -> None:
    source_files = {
        path.relative_to(source).as_posix(): file_sha256(path)
        for path in regular_files(source)
    }
    mirror_files = {
        path.relative_to(mirror).as_posix(): file_sha256(path)
        for path in regular_files(mirror)
    }
    if source_files != mirror_files:
        raise SecurityAuthorityError(
            f"role view does not mirror assigned keystore tree: {mirror}"
        )


def _manifest_from_dict(raw: Mapping[str, Any]) -> BundleManifest:
    unknown = sorted(
        set(raw)
        - {
            "schema_version",
            "system_id",
            "generation",
            "host_id",
            "created_at",
            "enclaves",
            "files",
        }
    )
    if unknown:
        raise SecurityAuthorityError(f"unknown security bundle manifest fields: {unknown}")
    if raw.get("schema_version") != 1:
        raise SecurityAuthorityError("unsupported security bundle manifest schema")
    system_id = str(raw.get("system_id", ""))
    generation = str(raw.get("generation", ""))
    host_id = str(raw.get("host_id", ""))
    validate_system_id(system_id)
    validate_generation(generation)
    validate_host_id(host_id)
    enclave_rows = raw.get("enclaves")
    files_raw = raw.get("files")
    if not isinstance(enclave_rows, list) or not isinstance(files_raw, Mapping):
        raise SecurityAuthorityError("invalid security bundle manifest shape")
    identities: list[EnclaveIdentity] = []
    for row in enclave_rows:
        if not isinstance(row, Mapping):
            raise SecurityAuthorityError("invalid enclave manifest entry")
        unknown_enclave = sorted(set(row) - {"role", "endpoint_id", "enclave"})
        if unknown_enclave:
            raise SecurityAuthorityError(
                f"unknown enclave manifest fields: {unknown_enclave}"
            )
        identity = EnclaveIdentity(
            system_id,
            str(row.get("role", "")),
            str(row.get("endpoint_id", "")),
        )
        if row.get("enclave") != identity.enclave:
            raise SecurityAuthorityError("manifest enclave path is not canonical")
        identities.append(identity)
    if not identities or len(set(identities)) != len(identities):
        raise SecurityAuthorityError("bundle enclave list must be non-empty and unique")
    if len({identity.role for identity in identities}) != len(identities):
        raise SecurityAuthorityError("bundle may contain only one enclave per role")
    files: dict[str, str] = {}
    for key, value in files_raw.items():
        relative = str(key)
        digest = str(value)
        _validate_manifest_relative(relative)
        if not _SHA256.fullmatch(digest):
            raise SecurityAuthorityError(f"invalid SHA-256 digest for {relative!r}")
        files[relative] = digest
    return BundleManifest(
        schema_version=1,
        system_id=system_id,
        generation=generation,
        host_id=host_id,
        created_at=str(raw.get("created_at", "")),
        enclaves=tuple(sorted(identities)),
        files=files,
    )


def validate_system_id(value: str) -> None:
    if not _ROS_IDENTIFIER.fullmatch(value):
        raise ValueError(
            "system_id must start with a lowercase letter and contain only "
            "lowercase letters, digits or '_'"
        )


def validate_host_id(value: str) -> None:
    if not _ENDPOINT_IDENTIFIER.fullmatch(value):
        raise ValueError(
            "host_id must start with a lowercase letter and contain only "
            "lowercase letters, digits, '-' or '_'"
        )


def validate_generation(value: str) -> None:
    if not _GENERATION_IDENTIFIER.fullmatch(value) or value in {".", ".."}:
        raise ValueError(f"invalid authority generation identifier: {value!r}")


def new_generation_id() -> str:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dt%H%M%S%fz")
    return f"g-{timestamp}-{uuid.uuid4().hex[:12]}"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def secure_absolute(path: Path) -> Path:
    expanded = Path(path).expanduser()
    absolute = Path(os.path.abspath(expanded))
    current = Path(absolute.anchor)
    for component in absolute.parts[1:]:
        current /= component
        if current.is_symlink():
            raise SecurityAuthorityError(
                f"security path must not contain symlinks: {current}"
            )
    return absolute


def ensure_private_directory(path: Path) -> None:
    secure = secure_absolute(path)
    secure.mkdir(mode=0o700, parents=True, exist_ok=True)
    if secure.is_symlink() or not secure.is_dir():
        raise SecurityAuthorityError(f"security path is not a directory: {secure}")
    secure.chmod(0o700)


def require_regular_file(path: Path) -> Path:
    secure = secure_absolute(path)
    if secure.is_symlink() or not secure.is_file():
        raise SecurityAuthorityError(f"security input is not a regular file: {secure}")
    if not stat.S_ISREG(secure.stat(follow_symlinks=False).st_mode):
        raise SecurityAuthorityError(f"security input is not a regular file: {secure}")
    return secure


def require_contained_directory(path: Path, owner: Path) -> None:
    secure = secure_absolute(path)
    boundary = secure_absolute(owner)
    if boundary not in secure.parents:
        raise SecurityAuthorityError(f"security path escapes its owner: {secure}")
    if secure.is_symlink() or not secure.is_dir():
        raise SecurityAuthorityError(f"required security directory is missing: {secure}")
    validate_tree(secure)


def validate_keystore(path: Path, *, require_private: bool) -> None:
    require_contained_directory(path, path.parent)
    for name in ("public", "enclaves"):
        child = path / name
        if child.is_symlink() or not child.is_dir():
            raise SecurityAuthorityError(f"SROS2 keystore is missing {name}/")
    private = path / "private"
    if require_private and (private.is_symlink() or not private.is_dir()):
        raise SecurityAuthorityError("SROS2 authority keystore is missing private/")


def materialize_keystore_symlinks(path: Path) -> None:
    """Replace the narrowly allowed symlinks emitted by ``ros2 security``.

    SROS2 intentionally links the two CA aliases to the canonical CA files and
    links each enclave to the shared public certificates and governance file.
    Authority generations and exported bundles are easier to validate safely
    when they contain regular files only, so resolve those links while the
    keystore is still in its private staging directory.  Links to another
    enclave, private material from a public/enclave tree, directories, broken
    paths, or anything outside the keystore remain fatal.
    """

    keystore = secure_absolute(path)
    if keystore.is_symlink() or not keystore.is_dir():
        raise SecurityAuthorityError(f"SROS2 keystore is not a directory: {keystore}")
    for name in ("public", "private", "enclaves"):
        child = keystore / name
        if child.is_symlink() or not child.is_dir():
            raise SecurityAuthorityError(f"SROS2 keystore is missing {name}/")

    replacements: list[tuple[Path, bytes]] = []
    for directory, names, files in os.walk(keystore, followlinks=False):
        parent = Path(directory)
        if parent.is_symlink():
            raise SecurityAuthorityError(
                f"SROS2 keystore contains a directory symlink: {parent}"
            )
        for name in names:
            child = parent / name
            mode = child.stat(follow_symlinks=False).st_mode
            if stat.S_ISLNK(mode):
                raise SecurityAuthorityError(
                    f"SROS2 keystore contains a directory symlink: {child}"
                )
            if not stat.S_ISDIR(mode):
                raise SecurityAuthorityError(
                    f"SROS2 keystore contains a special file: {child}"
                )
        for name in files:
            child = parent / name
            mode = child.stat(follow_symlinks=False).st_mode
            if stat.S_ISLNK(mode):
                source = resolve_keystore_file(child, keystore=keystore)
                replacements.append((child, source.read_bytes()))
            elif not stat.S_ISREG(mode):
                raise SecurityAuthorityError(
                    f"SROS2 keystore contains a special file: {child}"
                )

    for link, contents in replacements:
        with tempfile.NamedTemporaryFile(
            "wb",
            dir=link.parent,
            prefix=f".{link.name}.",
            delete=False,
        ) as handle:
            handle.write(contents)
            handle.flush()
            os.fsync(handle.fileno())
            temporary = Path(handle.name)
        temporary.chmod(0o600)
        os.replace(temporary, link)
        fsync_directory(link.parent)
    validate_tree(keystore)


def resolve_keystore_file(path: Path, *, keystore: Path) -> Path:
    """Return a regular source for an allowed SROS2 keystore file or link."""

    root = secure_absolute(keystore)
    logical = Path(os.path.abspath(path))
    if root not in logical.parents:
        raise SecurityAuthorityError(f"SROS2 keystore path escapes its root: {logical}")
    try:
        mode = logical.stat(follow_symlinks=False).st_mode
    except FileNotFoundError as exc:
        raise SecurityAuthorityError(
            f"SROS2 keystore file is missing: {logical}"
        ) from exc
    if stat.S_ISREG(mode):
        return logical
    if not stat.S_ISLNK(mode):
        raise SecurityAuthorityError(
            f"SROS2 keystore path is not a regular file: {logical}"
        )
    try:
        resolved = logical.resolve(strict=True)
    except (FileNotFoundError, OSError, RuntimeError) as exc:
        raise SecurityAuthorityError(
            f"SROS2 keystore contains a broken symlink: {logical}"
        ) from exc
    if not resolved.is_file() or not stat.S_ISREG(
        resolved.stat(follow_symlinks=False).st_mode
    ):
        raise SecurityAuthorityError(
            f"SROS2 keystore symlink does not target a regular file: {logical}"
        )

    public = root / "public"
    private = root / "private"
    enclaves = root / "enclaves"
    if public in logical.parents:
        allowed = public in resolved.parents
    elif private in logical.parents:
        allowed = private in resolved.parents
    elif enclaves in logical.parents:
        allowed = (
            public in resolved.parents
            or resolved == enclaves / "governance.p7s"
            or resolved.parent == logical.parent
        )
    else:
        allowed = False
    if not allowed:
        raise SecurityAuthorityError(
            f"SROS2 keystore symlink has a forbidden target: {logical} -> {resolved}"
        )
    return resolved


def validate_tree(root: Path) -> None:
    secure = secure_absolute(root)
    if secure.is_symlink() or not secure.is_dir():
        raise SecurityAuthorityError(f"security tree is not a directory: {secure}")
    for directory, names, files in os.walk(secure, followlinks=False):
        parent = Path(directory)
        if parent.is_symlink():
            raise SecurityAuthorityError(f"security tree contains a symlink: {parent}")
        for name in (*names, *files):
            path = parent / name
            mode = path.stat(follow_symlinks=False).st_mode
            if stat.S_ISLNK(mode):
                raise SecurityAuthorityError(f"security tree contains a symlink: {path}")
            if not (stat.S_ISDIR(mode) or stat.S_ISREG(mode)):
                raise SecurityAuthorityError(
                    f"security tree contains a special file: {path}"
                )


def regular_files(root: Path) -> tuple[Path, ...]:
    validate_tree(root)
    return tuple(sorted(path for path in root.rglob("*") if path.is_file()))


def harden_tree(root: Path) -> None:
    validate_tree(root)
    root.chmod(0o700)
    for path in root.rglob("*"):
        path.chmod(0o700 if path.is_dir() else 0o600)


def require_private_modes(root: Path) -> None:
    if root.stat().st_mode & 0o777 != 0o700:
        raise SecurityAuthorityError(f"security directory mode must be 0700: {root}")
    for path in root.rglob("*"):
        expected = 0o700 if path.is_dir() else 0o600
        if path.stat().st_mode & 0o777 != expected:
            raise SecurityAuthorityError(
                f"security path mode must be {expected:04o}: {path}"
            )


def copy_secure_tree(source: Path, destination: Path) -> None:
    validate_tree(source)
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(f"security destination already exists: {destination}")
    destination.mkdir(mode=0o700, parents=True)
    for source_path in sorted(source.rglob("*")):
        relative = source_path.relative_to(source)
        target = destination / relative
        if source_path.is_dir():
            target.mkdir(mode=0o700, exist_ok=True)
        else:
            target.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            shutil.copyfile(source_path, target)
            target.chmod(0o600)
    harden_tree(destination)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_manifest_relative(value: str) -> None:
    pure = PurePosixPath(value)
    if (
        not value
        or "\\" in value
        or pure.is_absolute()
        or any(part in {"", ".", ".."} for part in pure.parts)
        or pure.as_posix() != value
    ):
        raise SecurityAuthorityError(f"unsafe bundle manifest path: {value!r}")


def _safe_manifest_path(root: Path, relative: str) -> Path:
    _validate_manifest_relative(relative)
    path = root.joinpath(*PurePosixPath(relative).parts)
    if root not in path.parents:
        raise SecurityAuthorityError(f"bundle manifest path escapes root: {relative!r}")
    if path.is_symlink() or not path.is_file():
        raise SecurityAuthorityError(
            f"bundle manifest path is not a regular file: {relative!r}"
        )
    return path


def read_json_object(path: Path) -> dict[str, Any]:
    source = require_regular_file(path)
    try:
        raw = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise SecurityAuthorityError(f"invalid security JSON: {source}") from exc
    if not isinstance(raw, dict):
        raise SecurityAuthorityError(f"security JSON must be an object: {source}")
    return raw


def atomic_json(path: Path, payload: Mapping[str, object]) -> None:
    ensure_private_directory(path.parent)
    if path.is_symlink():
        raise SecurityAuthorityError(f"refusing to replace a symlink: {path}")
    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        dir=path.parent,
        prefix=f".{path.name}.",
        delete=False,
    ) as handle:
        json.dump(payload, handle, ensure_ascii=False, sort_keys=True, indent=2)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
        temporary = Path(handle.name)
    temporary.chmod(0o600)
    os.replace(temporary, path)
    fsync_directory(path.parent)


def fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def remove_owned_tree(path: Path, *, owner: Path) -> None:
    secure = secure_absolute(path)
    boundary = secure_absolute(owner)
    if boundary not in secure.parents or secure == boundary:
        raise SecurityAuthorityError(f"refusing to remove unowned path: {secure}")
    if secure.is_symlink():
        raise SecurityAuthorityError(f"refusing to remove a symlink: {secure}")
    shutil.rmtree(secure)
