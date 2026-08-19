"""Peer-owned leases, sessions, fencing, and request idempotency.

The resource peer is the sole writer of these state machines.  DDS reliability,
discovery, and source timestamps do not grant authority and do not replace the
application fence checked here.
"""

from __future__ import annotations

import hmac
import math
import secrets
from collections import OrderedDict
from dataclasses import dataclass, replace
from typing import Callable, Generic, Hashable, Optional, TypeVar, cast

from .peer import PeerIdentity


REQUEST_ID_MAX_LENGTH = 128
TOKEN_MAX_LENGTH = 256
CHANNEL_MAX_LENGTH = 128


class AuthorityError(ValueError):
    pass


class IdempotencyConflict(AuthorityError):
    pass


T = TypeVar("T")


@dataclass(frozen=True)
class _Memo(Generic[T]):
    fingerprint: object
    value: T


class IdempotencyCache(Generic[T]):
    """A bounded cache that rejects reuse of one key for different input."""

    def __init__(self, max_entries: int = 256) -> None:
        if isinstance(max_entries, bool) or type(max_entries) is not int or max_entries < 1:
            raise AuthorityError("max_entries must be a positive integer")
        self.max_entries = max_entries
        self._entries: OrderedDict[Hashable, _Memo[T]] = OrderedDict()

    def recall(self, key: Hashable, fingerprint: object) -> tuple[bool, Optional[T]]:
        memo = self._entries.get(key)
        if memo is None:
            return False, None
        if memo.fingerprint != fingerprint:
            raise IdempotencyConflict(
                "one idempotency key cannot be reused for different input"
            )
        self._entries.move_to_end(key)
        return True, memo.value

    def remember(self, key: Hashable, fingerprint: object, value: T) -> None:
        hit, _existing = self.recall(key, fingerprint)
        if hit:
            return
        self._entries[key] = _Memo(fingerprint, value)
        while len(self._entries) > self.max_entries:
            self._entries.popitem(last=False)

    def execute(
        self,
        key: Hashable,
        fingerprint: object,
        factory: Callable[[], T],
    ) -> tuple[T, bool]:
        hit, value = self.recall(key, fingerprint)
        if hit:
            return cast(T, value), True
        created = factory()
        self.remember(key, fingerprint, created)
        return created, False

    def __len__(self) -> int:
        return len(self._entries)


def _bounded_text(value: object, *, name: str, maximum: int) -> str:
    text = str(value).strip()
    if not text or len(text) > maximum or any(character.isspace() for character in text):
        raise AuthorityError(
            f"{name} must contain 1..{maximum} non-whitespace characters"
        )
    return text


def _time(value: float) -> float:
    current = float(value)
    if not math.isfinite(current):
        raise AuthorityError("authority time must be finite")
    return current


@dataclass(frozen=True)
class LeaseFence:
    resource: PeerIdentity
    owner: PeerIdentity
    epoch: int
    token: str
    sequence: int

    def __post_init__(self) -> None:
        if not isinstance(self.resource, PeerIdentity):
            raise AuthorityError("fence resource must be PeerIdentity")
        if not isinstance(self.owner, PeerIdentity):
            raise AuthorityError("fence owner must be PeerIdentity")
        if isinstance(self.epoch, bool) or type(self.epoch) is not int or self.epoch < 1:
            raise AuthorityError("fence epoch must be a positive integer")
        if (
            isinstance(self.sequence, bool)
            or type(self.sequence) is not int
            or self.sequence < 0
        ):
            raise AuthorityError("fence sequence must be a non-negative integer")
        object.__setattr__(
            self,
            "token",
            _bounded_text(
                self.token,
                name="fence token",
                maximum=TOKEN_MAX_LENGTH,
            ),
        )


@dataclass(frozen=True)
class FenceDecision:
    accepted: bool
    reason: str


@dataclass(frozen=True)
class MotionLease:
    target: PeerIdentity
    pilot: PeerIdentity
    epoch: int
    token: str
    expires_at: float
    last_command_sequence: int = -1

    def fence(self, sequence: int) -> LeaseFence:
        return LeaseFence(
            resource=self.target,
            owner=self.pilot,
            epoch=self.epoch,
            token=self.token,
            sequence=sequence,
        )


@dataclass(frozen=True)
class LeaseDecision:
    accepted: bool
    reason: str
    lease: Optional[MotionLease] = None
    replayed: bool = False


@dataclass(frozen=True)
class SimulationSession:
    sim: PeerIdentity
    ui: PeerIdentity
    epoch: int
    token: str
    streams: tuple[str, ...]
    expires_at: float

    def fence(self, sequence: int) -> LeaseFence:
        return LeaseFence(
            resource=self.sim,
            owner=self.ui,
            epoch=self.epoch,
            token=self.token,
            sequence=sequence,
        )


@dataclass(frozen=True)
class SessionDecision:
    accepted: bool
    reason: str
    session: Optional[SimulationSession] = None
    replayed: bool = False


class _AuthorityBase:
    def __init__(
        self,
        resource: PeerIdentity,
        *,
        lease_ttl_s: float,
        min_ttl_s: float,
        max_ttl_s: float,
        token_factory: Optional[Callable[[], str]],
        idempotency_entries: int,
    ) -> None:
        if not isinstance(resource, PeerIdentity):
            raise AuthorityError("authority resource must be PeerIdentity")
        default_ttl = float(lease_ttl_s)
        minimum = float(min_ttl_s)
        maximum = float(max_ttl_s)
        if (
            not all(math.isfinite(value) for value in (default_ttl, minimum, maximum))
            or minimum <= 0.0
            or maximum < minimum
            or not minimum <= default_ttl <= maximum
        ):
            raise AuthorityError(
                "lease TTL values must be finite and satisfy 0 < min <= default <= max"
            )
        self.resource = resource
        self.lease_ttl_s = default_ttl
        self.min_ttl_s = minimum
        self.max_ttl_s = maximum
        self.token_factory = token_factory or (lambda: secrets.token_hex(32))
        self._idempotency: IdempotencyCache[object] = IdempotencyCache(
            idempotency_entries
        )
        self._epoch = 0

    @property
    def epoch(self) -> int:
        return self._epoch

    def _ttl(self, requested_ttl_s: Optional[float]) -> float:
        value = (
            self.lease_ttl_s
            if requested_ttl_s is None
            else float(requested_ttl_s)
        )
        if not math.isfinite(value) or value <= 0.0:
            raise AuthorityError("requested lease TTL must be positive and finite")
        return max(self.min_ttl_s, min(self.max_ttl_s, value))

    def _token(self) -> str:
        return _bounded_text(
            self.token_factory(),
            name="authority token",
            maximum=TOKEN_MAX_LENGTH,
        )

    @staticmethod
    def _request_id(value: str) -> str:
        return _bounded_text(
            value,
            name="request_id",
            maximum=REQUEST_ID_MAX_LENGTH,
        )

    @staticmethod
    def _peer(value: PeerIdentity, *, name: str) -> PeerIdentity:
        if not isinstance(value, PeerIdentity):
            raise AuthorityError(f"{name} must be PeerIdentity")
        return value

    def _next_epoch(self) -> int:
        self._epoch += 1
        return self._epoch


def _fence_reason(
    fence: LeaseFence,
    *,
    resource: PeerIdentity,
    owner: PeerIdentity,
    epoch: int,
    token: str,
) -> str:
    if not isinstance(fence, LeaseFence):
        return "invalid_fence"
    if fence.resource != resource:
        return "resource_mismatch"
    if fence.owner != owner:
        return "owner_mismatch"
    if fence.epoch != epoch:
        return "epoch_mismatch"
    if not hmac.compare_digest(fence.token, token):
        return "token_mismatch"
    return ""


class MotionLeaseAuthority(_AuthorityBase):
    """The target-owned, single-pilot motion lease."""

    def __init__(
        self,
        resource: PeerIdentity,
        *,
        lease_ttl_s: float = 3.5,
        min_ttl_s: float = 0.1,
        max_ttl_s: float = 60.0,
        token_factory: Optional[Callable[[], str]] = None,
        idempotency_entries: int = 256,
    ) -> None:
        super().__init__(
            resource,
            lease_ttl_s=lease_ttl_s,
            min_ttl_s=min_ttl_s,
            max_ttl_s=max_ttl_s,
            token_factory=token_factory,
            idempotency_entries=idempotency_entries,
        )
        self._lease: Optional[MotionLease] = None

    def acquire(
        self,
        owner: PeerIdentity,
        request_id: str,
        *,
        now: float,
        requested_ttl_s: Optional[float] = None,
    ) -> LeaseDecision:
        pilot = self._peer(owner, name="motion lease owner")
        request = self._request_id(request_id)
        current = _time(now)
        ttl = self._ttl(requested_ttl_s)
        key = ("motion.acquire", pilot, request)
        fingerprint = (pilot, ttl)
        replay = self._recall_lease(key, fingerprint, now=current)
        if replay is not None:
            return replay
        self._expire(current)
        if self._lease is not None:
            decision = LeaseDecision(False, "busy", self._lease)
        else:
            decision = LeaseDecision(
                True,
                "granted",
                MotionLease(
                    target=self.resource,
                    pilot=pilot,
                    epoch=self._next_epoch(),
                    token=self._token(),
                    expires_at=current + ttl,
                ),
            )
            self._lease = decision.lease
        self._idempotency.remember(key, fingerprint, decision)
        return decision

    def renew(
        self,
        owner: PeerIdentity,
        fence: LeaseFence,
        request_id: str,
        *,
        now: float,
        requested_ttl_s: Optional[float] = None,
    ) -> LeaseDecision:
        pilot = self._peer(owner, name="motion lease owner")
        request = self._request_id(request_id)
        current = _time(now)
        ttl = self._ttl(requested_ttl_s)
        key = ("motion.renew", pilot, request)
        fingerprint = (pilot, fence, ttl)
        replay = self._recall_lease(key, fingerprint, now=current)
        if replay is not None:
            return replay
        expired = self._expire(current)
        if expired is not None:
            decision = LeaseDecision(False, "lease_expired")
        else:
            reason = self._match(fence, owner=pilot)
            if reason:
                decision = LeaseDecision(False, reason, self._lease)
            else:
                assert self._lease is not None
                self._lease = replace(self._lease, expires_at=current + ttl)
                decision = LeaseDecision(True, "renewed", self._lease)
        self._idempotency.remember(key, fingerprint, decision)
        return decision

    def release(
        self,
        owner: PeerIdentity,
        fence: LeaseFence,
        request_id: str,
        *,
        now: float,
    ) -> LeaseDecision:
        pilot = self._peer(owner, name="motion lease owner")
        request = self._request_id(request_id)
        current = _time(now)
        key = ("motion.release", pilot, request)
        fingerprint = (pilot, fence)
        replay = self._recall_lease(key, fingerprint, now=current, allow_stale=True)
        if replay is not None:
            return replay
        expired = self._expire(current)
        if expired is not None:
            decision = LeaseDecision(False, "lease_expired")
        else:
            reason = self._match(fence, owner=pilot)
            if reason:
                decision = LeaseDecision(False, reason, self._lease)
            else:
                old = self._invalidate()
                decision = LeaseDecision(True, "released", old)
        self._idempotency.remember(key, fingerprint, decision)
        return decision

    def revoke(
        self,
        *,
        now: float,
        reason: str = "revoked",
    ) -> Optional[MotionLease]:
        _time(now)
        _bounded_text(reason, name="revocation reason", maximum=512)
        return self._invalidate()

    def active(self, *, now: float) -> Optional[MotionLease]:
        self._expire(_time(now))
        return self._lease

    def accept_command(self, fence: LeaseFence, *, now: float) -> FenceDecision:
        current = _time(now)
        expired = self._expire(current)
        if expired is not None:
            return FenceDecision(False, "lease_expired")
        if self._lease is None:
            return FenceDecision(False, "no_active_lease")
        if not isinstance(fence, LeaseFence):
            return FenceDecision(False, "invalid_fence")
        reason = self._match(fence, owner=fence.owner)
        if reason:
            return FenceDecision(False, reason)
        if fence.sequence <= self._lease.last_command_sequence:
            return FenceDecision(False, "stale_sequence")
        self._lease = replace(
            self._lease,
            last_command_sequence=fence.sequence,
        )
        return FenceDecision(True, "accepted")

    def _match(self, fence: LeaseFence, *, owner: PeerIdentity) -> str:
        if self._lease is None:
            return "no_active_lease"
        if owner != self._lease.pilot:
            return "owner_mismatch"
        return _fence_reason(
            fence,
            resource=self.resource,
            owner=self._lease.pilot,
            epoch=self._lease.epoch,
            token=self._lease.token,
        )

    def _expire(self, now: float) -> Optional[MotionLease]:
        if self._lease is None or now < self._lease.expires_at:
            return None
        return self._invalidate()

    def _invalidate(self) -> Optional[MotionLease]:
        old = self._lease
        if old is not None:
            self._lease = None
            self._next_epoch()
        return old

    def _recall_lease(
        self,
        key: Hashable,
        fingerprint: object,
        *,
        now: float,
        allow_stale: bool = False,
    ) -> Optional[LeaseDecision]:
        try:
            hit, raw = self._idempotency.recall(key, fingerprint)
        except IdempotencyConflict:
            return LeaseDecision(False, "idempotency_conflict")
        if not hit:
            return None
        assert isinstance(raw, LeaseDecision)
        if (
            raw.accepted
            and raw.lease is not None
            and not allow_stale
        ):
            if (
                self._lease is None
                or self._lease.epoch != raw.lease.epoch
                or self._lease.token != raw.lease.token
                or now >= self._lease.expires_at
            ):
                return LeaseDecision(False, "request_id_expired", replayed=True)
            return replace(raw, lease=self._lease, replayed=True)
        return replace(raw, replayed=True)


class SimulationSessionAuthority(_AuthorityBase):
    """The sim-owned, single-UI simulation and signaling session."""

    def __init__(
        self,
        sim: PeerIdentity,
        *,
        allowed_streams: tuple[str, ...] = ("observer", "hand_eye_preview"),
        lease_ttl_s: float = 3.5,
        min_ttl_s: float = 0.1,
        max_ttl_s: float = 60.0,
        token_factory: Optional[Callable[[], str]] = None,
        idempotency_entries: int = 256,
    ) -> None:
        super().__init__(
            sim,
            lease_ttl_s=lease_ttl_s,
            min_ttl_s=min_ttl_s,
            max_ttl_s=max_ttl_s,
            token_factory=token_factory,
            idempotency_entries=idempotency_entries,
        )
        streams = tuple(
            _bounded_text(stream, name="simulation stream", maximum=128)
            for stream in allowed_streams
        )
        if not streams or len(streams) > 8 or len(set(streams)) != len(streams):
            raise AuthorityError(
                "allowed_streams must contain 1..8 unique stream names"
            )
        self.allowed_streams = streams
        self._session: Optional[SimulationSession] = None
        self._last_sequence_by_channel: dict[str, int] = {}

    def open(
        self,
        ui: PeerIdentity,
        streams: tuple[str, ...],
        request_id: str,
        *,
        now: float,
        requested_ttl_s: Optional[float] = None,
    ) -> SessionDecision:
        owner = self._peer(ui, name="simulation session UI")
        requested_streams = self._streams(streams)
        request = self._request_id(request_id)
        current = _time(now)
        ttl = self._ttl(requested_ttl_s)
        key = ("simulation.open", owner, request)
        fingerprint = (owner, requested_streams, ttl)
        replay = self._recall_session(key, fingerprint, now=current)
        if replay is not None:
            return replay
        self._expire(current)
        if self._session is not None:
            if (
                self._session.ui == owner
                and self._session.streams == requested_streams
            ):
                decision = SessionDecision(
                    True,
                    "already_active",
                    self._session,
                )
            else:
                decision = SessionDecision(False, "busy", self._session)
        else:
            self._session = SimulationSession(
                sim=self.resource,
                ui=owner,
                epoch=self._next_epoch(),
                token=self._token(),
                streams=requested_streams,
                expires_at=current + ttl,
            )
            self._last_sequence_by_channel.clear()
            decision = SessionDecision(True, "opened", self._session)
        self._idempotency.remember(key, fingerprint, decision)
        return decision

    def renew(
        self,
        ui: PeerIdentity,
        fence: LeaseFence,
        request_id: str,
        *,
        now: float,
        requested_ttl_s: Optional[float] = None,
    ) -> SessionDecision:
        owner = self._peer(ui, name="simulation session UI")
        request = self._request_id(request_id)
        current = _time(now)
        ttl = self._ttl(requested_ttl_s)
        key = ("simulation.renew", owner, request)
        fingerprint = (owner, fence, ttl)
        replay = self._recall_session(key, fingerprint, now=current)
        if replay is not None:
            return replay
        expired = self._expire(current)
        if expired is not None:
            decision = SessionDecision(False, "session_expired")
        else:
            reason = self._match(fence, owner=owner)
            if reason:
                decision = SessionDecision(False, reason, self._session)
            else:
                assert self._session is not None
                self._session = replace(
                    self._session,
                    expires_at=current + ttl,
                )
                decision = SessionDecision(True, "renewed", self._session)
        self._idempotency.remember(key, fingerprint, decision)
        return decision

    def close(
        self,
        ui: PeerIdentity,
        fence: LeaseFence,
        request_id: str,
        *,
        now: float,
    ) -> SessionDecision:
        owner = self._peer(ui, name="simulation session UI")
        request = self._request_id(request_id)
        current = _time(now)
        key = ("simulation.close", owner, request)
        fingerprint = (owner, fence)
        replay = self._recall_session(
            key,
            fingerprint,
            now=current,
            allow_stale=True,
        )
        if replay is not None:
            return replay
        expired = self._expire(current)
        if expired is not None:
            decision = SessionDecision(False, "session_expired")
        else:
            reason = self._match(fence, owner=owner)
            if reason:
                decision = SessionDecision(False, reason, self._session)
            else:
                old = self._invalidate()
                decision = SessionDecision(True, "closed", old)
        self._idempotency.remember(key, fingerprint, decision)
        return decision

    def revoke(
        self,
        *,
        now: float,
        reason: str = "revoked",
    ) -> Optional[SimulationSession]:
        _time(now)
        _bounded_text(reason, name="revocation reason", maximum=512)
        return self._invalidate()

    def active(self, *, now: float) -> Optional[SimulationSession]:
        self._expire(_time(now))
        return self._session

    def accept(
        self,
        fence: LeaseFence,
        channel: str,
        *,
        now: float,
    ) -> FenceDecision:
        current = _time(now)
        expired = self._expire(current)
        if expired is not None:
            return FenceDecision(False, "session_expired")
        if self._session is None:
            return FenceDecision(False, "no_active_session")
        if not isinstance(fence, LeaseFence):
            return FenceDecision(False, "invalid_fence")
        reason = self._match(fence, owner=fence.owner)
        if reason:
            return FenceDecision(False, reason)
        channel_name = _bounded_text(
            channel,
            name="session fence channel",
            maximum=CHANNEL_MAX_LENGTH,
        )
        previous = self._last_sequence_by_channel.get(channel_name, -1)
        if fence.sequence <= previous:
            return FenceDecision(False, "stale_sequence")
        self._last_sequence_by_channel[channel_name] = fence.sequence
        return FenceDecision(True, "accepted")

    def _streams(self, streams: tuple[str, ...]) -> tuple[str, ...]:
        values = tuple(
            _bounded_text(stream, name="simulation stream", maximum=128)
            for stream in streams
        )
        if not values or len(values) > 8 or len(set(values)) != len(values):
            raise AuthorityError(
                "simulation session streams must contain 1..8 unique names"
            )
        unsupported = sorted(set(values) - set(self.allowed_streams))
        if unsupported:
            raise AuthorityError(
                "unsupported simulation streams: " + ", ".join(unsupported)
            )
        return values

    def _match(self, fence: LeaseFence, *, owner: PeerIdentity) -> str:
        if self._session is None:
            return "no_active_session"
        if owner != self._session.ui:
            return "owner_mismatch"
        return _fence_reason(
            fence,
            resource=self.resource,
            owner=self._session.ui,
            epoch=self._session.epoch,
            token=self._session.token,
        )

    def _expire(self, now: float) -> Optional[SimulationSession]:
        if self._session is None or now < self._session.expires_at:
            return None
        return self._invalidate()

    def _invalidate(self) -> Optional[SimulationSession]:
        old = self._session
        if old is not None:
            self._session = None
            self._last_sequence_by_channel.clear()
            self._next_epoch()
        return old

    def _recall_session(
        self,
        key: Hashable,
        fingerprint: object,
        *,
        now: float,
        allow_stale: bool = False,
    ) -> Optional[SessionDecision]:
        try:
            hit, raw = self._idempotency.recall(key, fingerprint)
        except IdempotencyConflict:
            return SessionDecision(False, "idempotency_conflict")
        if not hit:
            return None
        assert isinstance(raw, SessionDecision)
        if (
            raw.accepted
            and raw.session is not None
            and not allow_stale
        ):
            if (
                self._session is None
                or self._session.epoch != raw.session.epoch
                or self._session.token != raw.session.token
                or now >= self._session.expires_at
            ):
                return SessionDecision(False, "request_id_expired", replayed=True)
            return replace(raw, session=self._session, replayed=True)
        return replace(raw, replayed=True)


__all__ = [
    "AuthorityError",
    "CHANNEL_MAX_LENGTH",
    "FenceDecision",
    "IdempotencyCache",
    "IdempotencyConflict",
    "LeaseDecision",
    "LeaseFence",
    "MotionLease",
    "MotionLeaseAuthority",
    "REQUEST_ID_MAX_LENGTH",
    "SessionDecision",
    "SimulationSession",
    "SimulationSessionAuthority",
    "TOKEN_MAX_LENGTH",
]
