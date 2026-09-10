"""Pure, install-scoped Docker identity helpers.

These names deliberately contain the installation identity.  A second EleSim
installation therefore cannot address the first one's Compose project,
containers, or images by accident.
"""

from __future__ import annotations

import hashlib
import re
import uuid


_IDENTIFIER = re.compile(r"[a-z][a-z0-9_]{0,62}\Z")
_ENDPOINT = re.compile(r"[a-z][a-z0-9_-]{0,62}\Z")
_FINGERPRINT = re.compile(r"[0-9a-f]{64}\Z")
_SERVICE_MAX = 128
_CONTAINER_MAX = 128


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


def project_name(install_uuid: str) -> str:
    """Return the Compose project name for one installation."""

    return f"elesim-runtime-{_uuid_hex(install_uuid)}"


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


def image_reference(install_uuid: str, role: str, fingerprint: str) -> str:
    """Return an install-scoped immutable image tag (never ``:local``)."""

    scope = _uuid_hex(install_uuid)
    role = _checked(role, _IDENTIFIER, "role")
    fingerprint = _checked(fingerprint, _FINGERPRINT, "fingerprint")
    tag = f"{scope}-{fingerprint}"
    if len(tag) > 128:
        raise ValueError("fingerprint is too long for an install-scoped image tag")
    return f"elesim/{role}:{tag}"


__all__ = [
    "container_name",
    "image_reference",
    "manager_container_name",
    "project_name",
    "service_key",
]
