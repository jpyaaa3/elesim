"""The no-Docker boundary for publishing install-scoped releases.

The host-side update wrapper builds images and writes a small evidence file.
This module runs in the setup/tools environment and consumes only that file
and an explicit, already-prepared runtime snapshot.  It deliberately has no
Docker client or subprocess dependency: publication is the only mutation it
performs.
"""

from __future__ import annotations

import json
import os
import re
import stat
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

from .instance_identity import image_reference, project_name
from .ownership import OwnershipManifest, append_docker_image_ownership
from .releases import ReleaseManifest, publish_release, release_key, runtime_data_digest
from .state import InstallState


_SOURCE_REVISION = re.compile(r"(?:git-[0-9a-f]{40}|sha256-[0-9a-f]{64})\Z")
_IMAGE_ID = re.compile(r"sha256:[0-9a-f]{64}\Z")
_FINGERPRINT = re.compile(r"[0-9a-f]{64}\Z")
_PLATFORMS = frozenset(("linux/amd64", "linux/arm64"))
_ROLES = frozenset(("pilot", "sim", "ui"))
_EVIDENCE_FIELDS = frozenset(("schema_version", "install_uuid", "project", "platform", "roles"))
_ROLE_FIELDS = frozenset(
    ("image_reference", "image_id", "install_uuid", "build_fingerprint", "project")
)
_EVIDENCE_SCHEMA_VERSION = 1
_MAX_EVIDENCE_BYTES = 64 * 1024


class ReleasePublicationError(ValueError):
    """Raised when host build evidence cannot safely become a release."""


@dataclass(frozen=True)
class PublicationResult:
    release_key: str
    release_path: Path

    def to_dict(self) -> dict[str, str]:
        return {"release_key": self.release_key, "release_path": str(self.release_path)}


def _reject_symlink_ancestors(path: Path) -> None:
    candidate = Path(os.path.abspath(os.fspath(path)))
    for current in (candidate, *candidate.parents):
        if current.is_symlink():
            raise ReleasePublicationError(f"path contains a symlink: {current}")


def _safe_regular_file(path: Path, name: str) -> bytes:
    _reject_symlink_ancestors(path)
    try:
        initial = path.lstat()
    except OSError as exc:
        raise ReleasePublicationError(f"{name} is not a readable regular file") from exc
    if (
        not stat.S_ISREG(initial.st_mode)
        or initial.st_nlink != 1
        or initial.st_size > _MAX_EVIDENCE_BYTES
    ):
        raise ReleasePublicationError(f"{name} must be an unlinked regular file")
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC)
    except OSError as exc:
        raise ReleasePublicationError(f"{name} is not a readable regular file") from exc
    try:
        info = os.fstat(fd)
        if (
            not stat.S_ISREG(info.st_mode)
            or info.st_nlink != 1
            or info.st_size > _MAX_EVIDENCE_BYTES
        ):
            raise ReleasePublicationError(f"{name} must be an unlinked regular file")
        with os.fdopen(fd, "rb") as stream:
            fd = -1
            return stream.read()
    finally:
        if fd != -1:
            os.close(fd)


def _safe_path_under(prefix: Path, candidate: Path, name: str) -> Path:
    """Return an absolute lexical child, refusing links and path escape."""

    root = Path(os.path.abspath(os.fspath(prefix)))
    value = Path(os.path.abspath(os.fspath(candidate)))
    _reject_symlink_ancestors(root)
    _reject_symlink_ancestors(value)
    try:
        value.relative_to(root)
    except ValueError as exc:
        raise ReleasePublicationError(f"{name} must be inside the install prefix") from exc
    return value


def _strict_json(path: Path, name: str) -> object:
    def pairs(items: list[tuple[str, object]]) -> dict[str, object]:
        result: dict[str, object] = {}
        for key, value in items:
            if key in result:
                raise ReleasePublicationError(f"{name} contains a duplicate JSON key")
            result[key] = value
        return result

    try:
        return json.loads(_safe_regular_file(path, name).decode("utf-8"), object_pairs_hook=pairs)
    except ReleasePublicationError:
        raise
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ReleasePublicationError(f"{name} is not valid UTF-8 JSON") from exc


def _text(value: object, name: str, *, pattern: re.Pattern[str] | None = None) -> str:
    if not isinstance(value, str) or value != value.strip() or not value:
        raise ReleasePublicationError(f"{name} must be a non-empty string")
    if any(ord(char) < 0x20 or char.isspace() for char in value):
        raise ReleasePublicationError(f"{name} must be a single-line string")
    if pattern is not None and pattern.fullmatch(value) is None:
        raise ReleasePublicationError(f"{name} has an invalid format")
    return value


def _validated_inputs(
    state: InstallState,
    ownership: OwnershipManifest,
    snapshot: Path,
    evidence: Path,
) -> tuple[InstallState, OwnershipManifest, Path, Mapping[str, Mapping[str, str]], str]:
    try:
        state = state.validate()
        ownership = ownership.validate()
    except (TypeError, ValueError) as exc:
        raise ReleasePublicationError("install-state or ownership manifest is invalid") from exc
    if state.install_mode != "container":
        raise ReleasePublicationError("immutable release publication requires a container install")
    docker = ownership.docker
    if docker is None:
        raise ReleasePublicationError("ownership manifest has no Docker identity")
    if ownership.prefix_path != state.prefix_path:
        raise ReleasePublicationError("install-state and ownership prefix do not match")
    expected_project = project_name(ownership.install_uuid)
    if docker.install_uuid != ownership.install_uuid or docker.project != expected_project:
        raise ReleasePublicationError(
            "install-state and ownership do not identify one scoped install"
        )
    # A release is an immutable snapshot of the installed capability, not of
    # the momentary topology assignment.  Otherwise changing assigned_roles
    # could silently make a later instance unable to select an installed role.
    roles = tuple(state.roles)
    if not roles or len(set(roles)) != len(roles) or not set(roles) <= _ROLES:
        raise ReleasePublicationError(
            "release publication supports only installed pilot/sim/ui roles"
        )
    snapshot = _safe_path_under(state.prefix_path, snapshot, "runtime snapshot")
    evidence = _safe_path_under(state.prefix_path, evidence, "build evidence")
    expected_snapshot = state.prefix_path / "containers" / "runtime-snapshot"
    if snapshot != expected_snapshot:
        raise ReleasePublicationError(
            f"runtime snapshot must be the installer-owned path: {expected_snapshot}"
        )
    if not snapshot.is_dir() or snapshot.is_symlink():
        raise ReleasePublicationError("runtime snapshot must be a real directory")
    required_directories = (
        Path("data"),
        Path("config"),
        *(Path("config") / role for role in roles),
    )
    for relative in required_directories:
        directory = snapshot / relative
        _reject_symlink_ancestors(directory)
        if not directory.is_dir() or directory.is_symlink():
            raise ReleasePublicationError(
                f"runtime snapshot is missing its real {relative.as_posix()} directory"
            )
    raw = _strict_json(evidence, "build evidence")
    if not isinstance(raw, dict) or set(raw) != _EVIDENCE_FIELDS:
        raise ReleasePublicationError(
            "build evidence fields are incomplete or contain unknown fields"
        )
    if (
        type(raw["schema_version"]) is not int
        or raw["schema_version"] != _EVIDENCE_SCHEMA_VERSION
    ):
        raise ReleasePublicationError("unsupported build evidence schema_version")
    install = _text(raw["install_uuid"], "evidence install_uuid")
    try:
        parsed = uuid.UUID(install)
    except (TypeError, ValueError, AttributeError) as exc:
        raise ReleasePublicationError("evidence install_uuid is not a UUID") from exc
    if str(parsed) != install:
        raise ReleasePublicationError("evidence install_uuid must be canonical lowercase UUID")
    project = _text(raw["project"], "evidence project")
    platform = _text(raw["platform"], "evidence platform")
    if platform not in _PLATFORMS:
        raise ReleasePublicationError("evidence platform is unsupported")
    role_values = raw["roles"]
    if not isinstance(role_values, dict) or set(role_values) != set(roles):
        raise ReleasePublicationError("evidence roles must exactly match installed roles")
    normalized: dict[str, Mapping[str, str]] = {}
    for role in roles:
        item = role_values[role]
        if not isinstance(item, dict) or set(item) != _ROLE_FIELDS:
            raise ReleasePublicationError(f"evidence for {role} has invalid fields")
        item_install = _text(item["install_uuid"], f"{role}.install_uuid")
        item_project = _text(item["project"], f"{role}.project")
        fingerprint = _text(
            item["build_fingerprint"],
            f"{role}.build_fingerprint",
            pattern=_FINGERPRINT,
        )
        image = _text(item["image_reference"], f"{role}.image_reference")
        image_id = _text(item["image_id"], f"{role}.image_id", pattern=_IMAGE_ID)
        if item_install != install or item_project != project:
            raise ReleasePublicationError(f"{role} evidence belongs to another install/project")
        if image != image_reference(install, role, fingerprint):
            raise ReleasePublicationError(f"{role} image reference does not match its fingerprint")
        normalized[role] = {
            "image_reference": image,
            "image_id": image_id,
            "install_uuid": item_install,
            "build_fingerprint": fingerprint,
            "project": item_project,
        }
    if install != ownership.install_uuid or project != expected_project:
        raise ReleasePublicationError("evidence belongs to another scoped installation")
    if not docker.context or not docker.engine_id:
        raise ReleasePublicationError("scoped release publication requires a pinned Docker daemon")
    return state, ownership, snapshot, normalized, platform


def _manifest_source_revision(source_revision: str) -> str:
    """Validate the revision authenticated by bootstrap."""

    if _SOURCE_REVISION.fullmatch(source_revision) is None:
        raise ReleasePublicationError("source_revision must be git-<40 hex> or sha256-<64 hex>")
    return source_revision


def publish_from_evidence(
    state: InstallState,
    ownership: OwnershipManifest,
    *,
    source_revision: str,
    runtime_snapshot: Path,
    evidence_path: Path,
) -> PublicationResult:
    """Validate host evidence and publish one immutable release.

    Publication is intentionally ordered before image ownership bookkeeping.
    If publication fails, ownership is untouched.  If ownership bookkeeping
    fails after publication, this raises an error while retaining the complete
    release; retrying the same evidence is safe because both publication and
    ownership append are idempotent.  This leaves a recoverable release rather
    than silently reporting success or deleting a valid immutable artifact.
    """

    state, ownership, snapshot, evidence, platform = _validated_inputs(
        state, ownership, Path(runtime_snapshot), Path(evidence_path)
    )
    source_revision = _manifest_source_revision(source_revision)
    install = ownership.install_uuid
    images = {role: values["image_reference"] for role, values in evidence.items()}
    image_ids = {role: values["image_id"] for role, values in evidence.items()}
    fingerprints = {role: values["build_fingerprint"] for role, values in evidence.items()}
    digest = runtime_data_digest(snapshot)
    try:
        manifest = ReleaseManifest(
            install_uuid=install,
            source_revision=source_revision,
            platform=platform,
            role_images=images,
            image_ids=image_ids,
            build_fingerprints=fingerprints,
            runtime_data_digest=digest,
        ).validate()
    except (TypeError, ValueError) as exc:
        raise ReleasePublicationError(
            "release manifest rejected build evidence"
        ) from exc
    # Publish the immutable release first.  A failed publication must not
    # leave image ownership pointing at a release that does not exist.
    destination = publish_release(state.prefix_path, manifest, snapshot)

    # A changed build receives a new install-scoped tag.  Record that exact
    # tag after publication so an ownership write failure leaves a complete,
    # discoverable release that can be repaired by rerunning this operation.
    # The evidence has already proved the install UUID/project and
    # image-reference shape above; this helper adds the manifest's atomic
    # ownership boundary.
    try:
        for image in images.values():
            append_docker_image_ownership(
                manifest_path=ownership.path,
                install_uuid=ownership.install_uuid,
                project=project_name(ownership.install_uuid),
                docker_context=ownership.docker.context if ownership.docker else "",
                docker_engine_id=ownership.docker.engine_id if ownership.docker else "",
                image=image,
            )
    except (OSError, ValueError) as exc:
        raise ReleasePublicationError(
            "release was published but image ownership could not be recorded; "
            f"retry publication to repair ownership ({release_key(manifest)})"
        ) from exc
    return PublicationResult(release_key(manifest), destination)


publish_release_from_evidence = publish_from_evidence


__all__ = [
    "PublicationResult",
    "ReleasePublicationError",
    "publish_from_evidence",
    "publish_release_from_evidence",
]
