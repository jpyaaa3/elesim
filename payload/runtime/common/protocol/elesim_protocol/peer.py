"""Transport-independent peer identity and discovery state.

DDS discovery establishes reachability, not EleSim authority.  These value
objects keep the application identity and boot generation explicit while
``PeerDirectory`` derives a fail-closed local view from advertisements and
fresh heartbeats.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from .messages import ENDPOINT_ROLES


PEER_ID_MAX_LENGTH = 128
BOOT_ID_MAX_LENGTH = 128
CAPABILITY_MAX_LENGTH = 128
MAX_CAPABILITIES = 32


class PeerError(ValueError):
    pass


class PeerDirectoryError(PeerError):
    pass


class PeerAmbiguityError(PeerDirectoryError):
    def __init__(self, endpoint_id: str, candidates: tuple["PeerIdentity", ...]) -> None:
        self.endpoint_id = str(endpoint_id)
        self.candidates = tuple(candidates)
        boots = ", ".join(candidate.boot_id for candidate in self.candidates)
        super().__init__(
            f"endpoint_id {self.endpoint_id!r} has multiple active boot IDs: {boots}"
        )


def _identifier(value: object, *, name: str, maximum: int) -> str:
    text = str(value).strip()
    if not text or len(text) > maximum or any(character.isspace() for character in text):
        raise PeerError(
            f"{name} must contain 1..{maximum} non-whitespace characters"
        )
    return text


def _now(value: float) -> float:
    current = float(value)
    if not math.isfinite(current):
        raise PeerDirectoryError("peer directory time must be finite")
    return current


@dataclass(frozen=True, order=True)
class PeerIdentity:
    endpoint_id: str
    boot_id: str

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "endpoint_id",
            _identifier(
                self.endpoint_id,
                name="endpoint_id",
                maximum=PEER_ID_MAX_LENGTH,
            ),
        )
        object.__setattr__(
            self,
            "boot_id",
            _identifier(
                self.boot_id,
                name="boot_id",
                maximum=BOOT_ID_MAX_LENGTH,
            ),
        )


@dataclass(frozen=True)
class PeerDescriptor:
    identity: PeerIdentity
    role: str
    capabilities: tuple[str, ...] = ()
    descriptor_revision: int = 1
    service_prefix: str = ""
    topic_prefix: str = ""
    interface_hash: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.identity, PeerIdentity):
            raise PeerError("peer descriptor identity must be PeerIdentity")
        role = str(self.role).strip()
        if role not in ENDPOINT_ROLES:
            raise PeerError(f"unsupported peer role: {role!r}")
        object.__setattr__(self, "role", role)

        capabilities = tuple(
            _identifier(
                capability,
                name="capability",
                maximum=CAPABILITY_MAX_LENGTH,
            )
            for capability in self.capabilities
        )
        if len(capabilities) > MAX_CAPABILITIES:
            raise PeerError(
                f"peer descriptor supports at most {MAX_CAPABILITIES} capabilities"
            )
        if len(set(capabilities)) != len(capabilities):
            raise PeerError("peer descriptor capabilities must be unique")
        object.__setattr__(self, "capabilities", capabilities)

        if (
            isinstance(self.descriptor_revision, bool)
            or type(self.descriptor_revision) is not int
            or self.descriptor_revision < 1
        ):
            raise PeerError("descriptor_revision must be a positive integer")
        for field_name in ("service_prefix", "topic_prefix"):
            value = str(getattr(self, field_name)).strip()
            if value and (
                len(value) > 256
                or not value.startswith("/")
                or any(character.isspace() for character in value)
            ):
                raise PeerError(
                    f"{field_name} must be empty or an absolute ROS name "
                    "of at most 256 non-whitespace characters"
                )
            object.__setattr__(self, field_name, value)
        interface_hash = str(self.interface_hash).strip()
        if interface_hash and (
            len(interface_hash) > 128
            or any(character.isspace() for character in interface_hash)
        ):
            raise PeerError(
                "interface_hash must contain at most 128 non-whitespace characters"
            )
        object.__setattr__(self, "interface_hash", interface_hash)


@dataclass(frozen=True)
class PeerHeartbeat:
    identity: PeerIdentity
    descriptor_revision: int
    sequence: int

    def __post_init__(self) -> None:
        if not isinstance(self.identity, PeerIdentity):
            raise PeerError("peer heartbeat identity must be PeerIdentity")
        for name in ("descriptor_revision", "sequence"):
            value = getattr(self, name)
            minimum = 1 if name == "descriptor_revision" else 0
            if isinstance(value, bool) or type(value) is not int or value < minimum:
                raise PeerError(f"{name} must be an integer >= {minimum}")


@dataclass
class _DirectoryEntry:
    descriptor: PeerDescriptor
    announced_at: float
    last_heartbeat_at: Optional[float] = None
    heartbeat_sequence: int = -1


class PeerDirectory:
    """Build a local, non-authoritative view of live peer generations."""

    def __init__(self, heartbeat_timeout_s: float = 3.5) -> None:
        timeout = float(heartbeat_timeout_s)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise PeerDirectoryError("heartbeat_timeout_s must be positive and finite")
        self.heartbeat_timeout_s = timeout
        self._entries: dict[PeerIdentity, _DirectoryEntry] = {}

    def announce(self, descriptor: PeerDescriptor, *, now: float) -> None:
        if not isinstance(descriptor, PeerDescriptor):
            raise PeerDirectoryError("announcement requires PeerDescriptor")
        current_time = _now(now)
        entry = self._entries.get(descriptor.identity)
        if entry is None:
            self._entries[descriptor.identity] = _DirectoryEntry(
                descriptor=descriptor,
                announced_at=current_time,
            )
            return
        previous_revision = entry.descriptor.descriptor_revision
        if descriptor.descriptor_revision < previous_revision:
            raise PeerDirectoryError("stale peer descriptor revision")
        if (
            descriptor.descriptor_revision == previous_revision
            and descriptor != entry.descriptor
        ):
            raise PeerDirectoryError(
                "one descriptor revision must identify exactly one descriptor"
            )
        entry.descriptor = descriptor
        entry.announced_at = current_time
        if descriptor.descriptor_revision != previous_revision:
            entry.last_heartbeat_at = None
            entry.heartbeat_sequence = -1

    def heartbeat(self, heartbeat: PeerHeartbeat, *, now: float) -> bool:
        if not isinstance(heartbeat, PeerHeartbeat):
            raise PeerDirectoryError("heartbeat requires PeerHeartbeat")
        current_time = _now(now)
        entry = self._entries.get(heartbeat.identity)
        if entry is None:
            return False
        if heartbeat.descriptor_revision != entry.descriptor.descriptor_revision:
            return False
        if heartbeat.sequence <= entry.heartbeat_sequence:
            return False
        entry.last_heartbeat_at = current_time
        entry.heartbeat_sequence = heartbeat.sequence
        return True

    def active(self, identity: PeerIdentity, *, now: float) -> bool:
        entry = self._entries.get(identity)
        return entry is not None and self._entry_active(entry, now=_now(now))

    def get(self, identity: PeerIdentity) -> Optional[PeerDescriptor]:
        entry = self._entries.get(identity)
        return None if entry is None else entry.descriptor

    def resolve(
        self,
        endpoint_id: str,
        *,
        now: float,
        role: str = "",
        capability: str = "",
    ) -> Optional[PeerDescriptor]:
        endpoint = _identifier(
            endpoint_id,
            name="endpoint_id",
            maximum=PEER_ID_MAX_LENGTH,
        )
        matches = tuple(
            descriptor
            for descriptor in self.discover(
                now=now,
                role=role,
                capability=capability,
            )
            if descriptor.identity.endpoint_id == endpoint
        )
        if len(matches) > 1:
            raise PeerAmbiguityError(
                endpoint,
                tuple(descriptor.identity for descriptor in matches),
            )
        return None if not matches else matches[0]

    def discover(
        self,
        *,
        now: float,
        role: str = "",
        capability: str = "",
    ) -> tuple[PeerDescriptor, ...]:
        current_time = _now(now)
        role_filter = str(role).strip()
        if role_filter and role_filter not in ENDPOINT_ROLES:
            raise PeerDirectoryError(f"unsupported discovery role: {role_filter!r}")
        capability_filter = str(capability).strip()
        return tuple(
            entry.descriptor
            for _identity, entry in sorted(self._entries.items())
            if self._entry_active(entry, now=current_time)
            and (not role_filter or entry.descriptor.role == role_filter)
            and (
                not capability_filter
                or capability_filter in entry.descriptor.capabilities
            )
        )

    def expire(self, *, now: float) -> tuple[PeerIdentity, ...]:
        current_time = _now(now)
        expired = tuple(
            identity
            for identity, entry in sorted(self._entries.items())
            if current_time
            - (
                entry.announced_at
                if entry.last_heartbeat_at is None
                else entry.last_heartbeat_at
            )
            > self.heartbeat_timeout_s
        )
        for identity in expired:
            self._entries.pop(identity, None)
        return expired

    def _entry_active(self, entry: _DirectoryEntry, *, now: float) -> bool:
        return (
            entry.last_heartbeat_at is not None
            and now - entry.last_heartbeat_at <= self.heartbeat_timeout_s
        )


__all__ = [
    "BOOT_ID_MAX_LENGTH",
    "CAPABILITY_MAX_LENGTH",
    "MAX_CAPABILITIES",
    "PEER_ID_MAX_LENGTH",
    "PeerAmbiguityError",
    "PeerDescriptor",
    "PeerDirectory",
    "PeerDirectoryError",
    "PeerError",
    "PeerHeartbeat",
    "PeerIdentity",
]
