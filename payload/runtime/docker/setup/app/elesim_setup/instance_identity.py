"""Pure, install-scoped Docker identity helpers.

These names deliberately contain the installation identity.  A second EleSim
installation therefore cannot address the first one's Compose project,
containers, or images by accident.
"""

from __future__ import annotations

import hashlib
import re
import uuid
from collections.abc import Mapping


_IDENTIFIER = re.compile(r"[a-z][a-z0-9_]{0,62}\Z")
_ENDPOINT = re.compile(r"[a-z][a-z0-9_-]{0,62}\Z")
_FINGERPRINT = re.compile(r"[0-9a-f]{64}\Z")
_SERVICE_MAX = 128
_CONTAINER_MAX = 128
_NAMED_TAG = re.compile(r"([a-z]{2,16}(?:_[a-z]{2,16}|[0-9]{0,6}))-([a-z]{2,16}(?:_[a-z]{2,16}|[0-9]{0,6}))\Z")
_NAMED_PROJECT = re.compile(r"elesim-([a-z]{2,16}(?:_[a-z]{2,16}|[0-9]{0,6}))\Z")
_SCOPED_IDENTITY_FIELDS = frozenset({"schema_version", "install_uuid", "project"})


def named_image_parts(image: str, role: str) -> tuple[str, str] | None:
    """Parse a readable tag; this is syntax, never proof of ownership."""
    prefix = f"elesim/{role}:"
    if not isinstance(image, str) or not image.startswith(prefix):
        return None
    match = _NAMED_TAG.fullmatch(image[len(prefix):])
    return match.groups() if match else None


def parse_native_identity(value: Mapping[str, object]) -> dict[str, str]:
    """Validate native identity without inventing a Compose project."""
    if not isinstance(value, Mapping) or set(value) != {
        "schema_version", "install_mode", "install_uuid", "prefix", "bin_dir"
    }:
        raise ValueError("invalid native installation identity fields")
    if type(value["schema_version"]) is not int or value["schema_version"] != 1:
        raise ValueError("unsupported native installation identity schema")
    if value["install_mode"] != "native":
        raise ValueError("invalid native installation mode")
    install_uuid = value["install_uuid"]
    if not isinstance(install_uuid, str) or str(uuid.UUID(install_uuid)) != install_uuid:
        raise ValueError("invalid native installation UUID")
    from pathlib import PurePosixPath

    for key in ("prefix", "bin_dir"):
        path = value[key]
        if (not isinstance(path, str) or not path.startswith("/")
                or ".." in PurePosixPath(path).parts
                or any(ord(char) < 32 for char in path)):
            raise ValueError("invalid native installation path")
    return {key: value[key] for key in ("install_uuid", "prefix", "bin_dir")}


def parse_scoped_identity(value: Mapping[str, object]) -> dict[str, str]:
    """Validate and normalize an ``elesim-net identity`` response.

    ``install_uuid`` and ``project`` are the enrollment proof.  Updated
    installations additionally expose ``install_name`` for human-facing
    display; it is deliberately not used as an ownership credential.  The
    optional field keeps the response backward-compatible with older
    installations that only returned the two machine identity values.
    """

    if not isinstance(value, Mapping):
        raise ValueError("scoped install identity must be an object")
    fields = set(value)
    if fields not in {
        _SCOPED_IDENTITY_FIELDS,
        _SCOPED_IDENTITY_FIELDS | {"install_name"},
    }:
        raise ValueError("scoped install identity fields are invalid")
    if type(value.get("schema_version")) is not int or value["schema_version"] != 1:
        raise ValueError("scoped install identity schema is unsupported")
    install_uuid = value.get("install_uuid")
    project = value.get("project")
    if not isinstance(install_uuid, str) or not isinstance(project, str):
        raise ValueError("scoped install identity values are invalid")
    if not is_scoped_project(install_uuid, project):
        raise ValueError(
            "scoped install identity project is not a valid EleSim namespace"
        )
    result = {"install_uuid": install_uuid, "project": project}
    if "install_name" in value:
        install_name = value.get("install_name")
        if not isinstance(install_name, str):
            raise ValueError("scoped install identity install_name is invalid")
        from .readable_names import NAME_PATTERN

        if NAME_PATTERN.fullmatch(install_name) is None:
            raise ValueError("scoped install identity install_name is invalid")
        # A readable project is self-describing only when it agrees with the
        # advertised display name. UUID-derived legacy projects are allowed to
        # acquire the display field during an update without being renamed.
        named_project = _NAMED_PROJECT.fullmatch(project)
        if named_project is not None and named_project.group(1) != install_name:
            raise ValueError(
                "scoped install identity install_name does not match project"
            )
        result["install_name"] = install_name
    return result


def _uuid_hex(install_uuid: str) -> str:
    if not isinstance(install_uuid, str):
        raise ValueError("install_uuid must be a canonical UUID string")
    try:
        parsed = uuid.UUID(install_uuid)
    except (ValueError, AttributeError, TypeError) as exc:
        raise ValueError("install_uuid must be a canonical UUID string") from exc
    if str(parsed) != install_uuid:
        raise ValueError("install_uuid must be a canonical lowercase UUID string")
    return parsed.hex


def _checked(value: str, pattern: re.Pattern[str], name: str) -> str:
    if not isinstance(value, str) or pattern.fullmatch(value) is None:
        raise ValueError(f"{name} must match {pattern.pattern}")
    return value


def project_name(install_uuid: str, *, install_name: str = "") -> str:
    """Return the Compose project name for one installation.

    The optional readable name is used by fresh installs.  Omitting it keeps
    the UUID namespace used by legacy manifests and by callers that only have
    the canonical internal identity.
    """

    _uuid_hex(install_uuid)
    if install_name:
        from .readable_names import NAME_PATTERN

        _checked(install_name, NAME_PATTERN, "install_name")
        return f"elesim-{install_name}"
    return f"elesim-runtime-{_uuid_hex(install_uuid)}"


def is_scoped_project(install_uuid: str, project: str) -> bool:
    """Return whether ``project`` is a valid scoped namespace for a UUID.

    UUID-derived projects are cryptographically bound to the installation.
    Readable projects intentionally carry only a reserved display name, so
    their UUID binding is proved by the ownership manifest/remote identity
    rather than by the project spelling alone.  This helper therefore checks
    only the strict namespace grammar and canonical UUID validity; callers
    must still compare the exact project returned by their ownership boundary.
    """

    try:
        canonical = project_name(install_uuid)
    except ValueError:
        return False
    return isinstance(project, str) and (
        project == canonical or _NAMED_PROJECT.fullmatch(project) is not None
    )


def service_key(system_id: str, endpoint_id: str) -> str:
    """Return a readable, bounded and collision-proof Compose service key."""

    system = _checked(system_id, _IDENTIFIER, "system_id")
    endpoint = _checked(endpoint_id, _ENDPOINT, "endpoint_id")
    digest = hashlib.sha256(f"{system}\0{endpoint}".encode("utf-8")).hexdigest()
    # Keep both components useful in Compose listings while reserving the
    # complete digest as the identity (not merely a truncated hash).
    return f"svc-{system[:24]}-{endpoint[:24]}-{digest}"[:_SERVICE_MAX]


def container_name(install_uuid: str, service: str) -> str:
    """Return a unique, bounded Docker container name for ``service``."""

    scope = _uuid_hex(install_uuid)
    _checked(service, re.compile(r"[a-z][a-z0-9_.-]{0,127}\Z"), "service")
    digest = hashlib.sha256(service.encode("utf-8")).hexdigest()
    prefix = f"elesim-{scope}-"
    suffix = f"-{digest}"
    readable_budget = _CONTAINER_MAX - len(prefix) - len(suffix)
    return f"{prefix}{service[:readable_budget]}{suffix}"


def manager_container_name(install_uuid: str, system_id: str) -> str:
    """Return the transient connection-manager name for one system workspace.

    The system identifier is included in the readable portion while the full
    service digest makes names collision-proof.  Keeping this in the shared
    identity module lets generated host wrappers and any later ownership
    bookkeeping derive exactly the same name without importing the manager.
    """

    system = _checked(system_id, _IDENTIFIER, "system_id")
    scope = _uuid_hex(install_uuid)
    # The complete validated system identifier is part of the name.  The
    # resulting maximum length is 111 characters, below Docker's 128-byte
    # limit, so no truncation or hash collision is necessary here.
    return f"elesim-{scope}-manager-{system}"


def image_reference(
    install_uuid: str,
    role: str,
    fingerprint: str,
    *,
    install_name: str = "",
    image_name: str = "",
) -> str:
    """Return an install-scoped immutable image tag (never ``:local``)."""

    scope = _uuid_hex(install_uuid)
    role = _checked(role, _IDENTIFIER, "role")
    fingerprint = _checked(fingerprint, _FINGERPRINT, "fingerprint")
    if install_name or image_name:
        from .readable_names import NAME_PATTERN

        _checked(install_name, NAME_PATTERN, "install_name")
        _checked(image_name, NAME_PATTERN, "image_name")
        return f"elesim/{role}:{install_name}-{image_name}"
    tag = f"{scope}-{fingerprint}"
    if len(tag) > 128:
        raise ValueError("fingerprint is too long for an install-scoped image tag")
    return f"elesim/{role}:{tag}"


__all__ = [
    "container_name",
    "image_reference",
    "is_scoped_project",
    "manager_container_name",
    "named_image_parts",
    "parse_scoped_identity",
    "project_name",
    "service_key",
]
