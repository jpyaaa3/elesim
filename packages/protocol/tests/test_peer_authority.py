from __future__ import annotations

import pytest

from elesim_protocol import (
    AuthorityError,
    IdempotencyCache,
    IdempotencyConflict,
    LeaseFence,
    MotionLeaseAuthority,
    PeerIdentity,
    SimulationSessionAuthority,
)


TARGET = PeerIdentity("robot-a", "robot-boot-a")
CONTROLLER_A = PeerIdentity("controller-a", "controller-boot-a")
CONTROLLER_B = PeerIdentity("controller-b", "controller-boot-b")
SIMULATOR = PeerIdentity("sim-a", "sim-boot-a")
UI_A = PeerIdentity("ui-a", "ui-boot-a")
UI_B = PeerIdentity("ui-b", "ui-boot-b")


def motion_authority() -> MotionLeaseAuthority:
    tokens = iter(("motion-token-a", "motion-token-b", "motion-token-c"))
    return MotionLeaseAuthority(
        TARGET,
        lease_ttl_s=3.5,
        token_factory=lambda: next(tokens),
    )


def session_authority() -> SimulationSessionAuthority:
    tokens = iter(("session-token-a", "session-token-b", "session-token-c"))
    return SimulationSessionAuthority(
        SIMULATOR,
        lease_ttl_s=3.5,
        token_factory=lambda: next(tokens),
    )


def test_idempotency_cache_replays_exact_input_and_rejects_key_reuse() -> None:
    cache: IdempotencyCache[str] = IdempotencyCache(max_entries=2)
    calls = 0

    def create() -> str:
        nonlocal calls
        calls += 1
        return "result"

    assert cache.execute("request-a", ("payload", 1), create) == ("result", False)
    assert cache.execute("request-a", ("payload", 1), create) == ("result", True)
    assert calls == 1
    with pytest.raises(IdempotencyConflict):
        cache.execute("request-a", ("different", 2), create)


def test_motion_target_grants_one_owner_and_replays_acquire_idempotently() -> None:
    authority = motion_authority()

    first = authority.acquire(CONTROLLER_A, "acquire-a", now=1.0)
    replay = authority.acquire(CONTROLLER_A, "acquire-a", now=1.1)
    busy = authority.acquire(CONTROLLER_B, "acquire-b", now=1.2)

    assert first.accepted is True
    assert first.lease is not None
    assert replay == type(replay)(
        accepted=True,
        reason="granted",
        lease=first.lease,
        replayed=True,
    )
    assert busy.accepted is False
    assert busy.reason == "busy"
    assert busy.lease == first.lease


def test_motion_request_id_conflict_does_not_change_active_lease() -> None:
    authority = motion_authority()
    granted = authority.acquire(
        CONTROLLER_A,
        "same-request",
        now=1.0,
        requested_ttl_s=2.0,
    )
    conflict = authority.acquire(
        CONTROLLER_A,
        "same-request",
        now=1.1,
        requested_ttl_s=4.0,
    )

    assert conflict.accepted is False
    assert conflict.reason == "idempotency_conflict"
    assert authority.active(now=1.1) == granted.lease


@pytest.mark.parametrize(
    ("fence_change", "reason"),
    (
        ({"resource": PeerIdentity("robot-a", "other-boot")}, "resource_mismatch"),
        ({"owner": CONTROLLER_B}, "owner_mismatch"),
        ({"epoch": 999}, "epoch_mismatch"),
        ({"token": "wrong-token"}, "token_mismatch"),
    ),
)
def test_motion_fence_rejects_wrong_generation_owner_epoch_or_token(
    fence_change: dict[str, object],
    reason: str,
) -> None:
    authority = motion_authority()
    granted = authority.acquire(CONTROLLER_A, "acquire-a", now=1.0)
    assert granted.lease is not None
    raw = {
        "resource": granted.lease.target,
        "owner": granted.lease.controller,
        "epoch": granted.lease.epoch,
        "token": granted.lease.token,
        "sequence": 1,
    }
    raw.update(fence_change)

    result = authority.accept_command(LeaseFence(**raw), now=1.1)

    assert result.accepted is False
    assert result.reason == reason


def test_motion_sequence_is_monotonic_inside_one_fence() -> None:
    authority = motion_authority()
    granted = authority.acquire(CONTROLLER_A, "acquire-a", now=1.0)
    assert granted.lease is not None

    assert authority.accept_command(granted.lease.fence(7), now=1.1).accepted
    stale = authority.accept_command(granted.lease.fence(7), now=1.2)
    assert stale.reason == "stale_sequence"
    assert authority.accept_command(granted.lease.fence(8), now=1.3).accepted


def test_motion_expiry_revokes_locally_and_old_request_cannot_resurrect() -> None:
    authority = motion_authority()
    granted = authority.acquire(
        CONTROLLER_A,
        "acquire-a",
        now=1.0,
        requested_ttl_s=1.0,
    )
    assert granted.lease is not None

    expired = authority.accept_command(granted.lease.fence(1), now=2.0)
    replay = authority.acquire(
        CONTROLLER_A,
        "acquire-a",
        now=2.1,
        requested_ttl_s=1.0,
    )

    assert expired.reason == "lease_expired"
    assert authority.active(now=2.0) is None
    assert replay.accepted is False
    assert replay.reason == "request_id_expired"


def test_motion_renew_extends_only_exact_fence_and_is_idempotent() -> None:
    authority = motion_authority()
    granted = authority.acquire(CONTROLLER_A, "acquire-a", now=1.0)
    assert granted.lease is not None
    fence = granted.lease.fence(0)

    renewed = authority.renew(
        CONTROLLER_A,
        fence,
        "renew-a",
        now=2.0,
        requested_ttl_s=5.0,
    )
    replay = authority.renew(
        CONTROLLER_A,
        fence,
        "renew-a",
        now=2.5,
        requested_ttl_s=5.0,
    )

    assert renewed.accepted is True
    assert renewed.lease is not None
    assert renewed.lease.expires_at == 7.0
    assert replay.lease == renewed.lease
    assert replay.replayed is True


def test_release_invalidates_before_next_grant_and_fences_old_controller() -> None:
    authority = motion_authority()
    first = authority.acquire(CONTROLLER_A, "acquire-a", now=1.0)
    assert first.lease is not None
    released = authority.release(
        CONTROLLER_A,
        first.lease.fence(0),
        "release-a",
        now=1.1,
    )
    second = authority.acquire(CONTROLLER_B, "acquire-b", now=1.2)

    assert released.accepted is True
    assert second.accepted is True
    assert second.lease is not None
    assert second.lease.epoch > first.lease.epoch
    assert authority.accept_command(
        first.lease.fence(1),
        now=1.3,
    ).reason in {"owner_mismatch", "epoch_mismatch"}
    assert authority.accept_command(second.lease.fence(1), now=1.3).accepted


def test_replayed_release_cannot_revoke_a_new_owner() -> None:
    authority = motion_authority()
    first = authority.acquire(CONTROLLER_A, "acquire-a", now=1.0)
    assert first.lease is not None
    released = authority.release(
        CONTROLLER_A,
        first.lease.fence(0),
        "release-a",
        now=1.1,
    )
    second = authority.acquire(CONTROLLER_B, "acquire-b", now=1.2)
    replay = authority.release(
        CONTROLLER_A,
        first.lease.fence(0),
        "release-a",
        now=1.3,
    )

    assert released.accepted is True
    assert replay.accepted is True
    assert replay.replayed is True
    assert authority.active(now=1.3) == second.lease


def test_ttl_is_bounded_by_target_policy() -> None:
    authority = MotionLeaseAuthority(
        TARGET,
        lease_ttl_s=2.0,
        min_ttl_s=1.0,
        max_ttl_s=4.0,
        token_factory=lambda: "token-a",
    )

    short = authority.acquire(
        CONTROLLER_A,
        "short",
        now=10.0,
        requested_ttl_s=0.1,
    )
    assert short.lease is not None
    assert short.lease.expires_at == 11.0


def test_simulator_grants_one_ui_and_validates_stream_allowlist() -> None:
    authority = session_authority()
    opened = authority.open(
        UI_A,
        ("observer", "hand_eye_preview"),
        "open-a",
        now=1.0,
    )
    busy = authority.open(UI_B, ("observer",), "open-b", now=1.1)

    assert opened.accepted is True
    assert busy.accepted is False
    assert busy.reason == "busy"
    with pytest.raises(AuthorityError, match="unsupported"):
        authority.open(UI_A, ("unknown",), "open-invalid", now=1.2)


def test_same_ui_open_is_idempotent_without_minting_another_session() -> None:
    authority = session_authority()
    first = authority.open(UI_A, ("observer",), "open-a", now=1.0)
    retry_with_new_request = authority.open(
        UI_A,
        ("observer",),
        "open-b",
        now=1.1,
    )

    assert first.session is not None
    assert retry_with_new_request.accepted is True
    assert retry_with_new_request.reason == "already_active"
    assert retry_with_new_request.session == first.session


def test_simulation_sequence_is_independent_per_transport_channel() -> None:
    authority = session_authority()
    opened = authority.open(UI_A, ("observer",), "open-a", now=1.0)
    assert opened.session is not None
    fence = opened.session.fence(1)

    assert authority.accept(fence, "simulation_command", now=1.1).accepted
    assert authority.accept(fence, "webrtc", now=1.1).accepted
    assert (
        authority.accept(fence, "simulation_command", now=1.2).reason
        == "stale_sequence"
    )
    assert authority.accept(
        opened.session.fence(2),
        "simulation_command",
        now=1.3,
    ).accepted


def test_simulation_close_and_boot_fence_reject_old_signaling() -> None:
    authority = session_authority()
    first = authority.open(UI_A, ("observer",), "open-a", now=1.0)
    assert first.session is not None
    closed = authority.close(
        UI_A,
        first.session.fence(0),
        "close-a",
        now=1.1,
    )
    second = authority.open(UI_B, ("observer",), "open-b", now=1.2)

    assert closed.accepted is True
    assert second.session is not None
    assert authority.accept(
        first.session.fence(1),
        "webrtc",
        now=1.3,
    ).reason in {"owner_mismatch", "epoch_mismatch"}
    assert authority.accept(
        second.session.fence(1),
        "webrtc",
        now=1.3,
    ).accepted


def test_replayed_session_close_cannot_close_a_new_ui_session() -> None:
    authority = session_authority()
    first = authority.open(UI_A, ("observer",), "open-a", now=1.0)
    assert first.session is not None
    authority.close(
        UI_A,
        first.session.fence(0),
        "close-a",
        now=1.1,
    )
    second = authority.open(UI_B, ("observer",), "open-b", now=1.2)
    replay = authority.close(
        UI_A,
        first.session.fence(0),
        "close-a",
        now=1.3,
    )

    assert replay.accepted is True
    assert replay.replayed is True
    assert authority.active(now=1.3) == second.session


def test_simulation_session_expires_without_sender_wall_clock() -> None:
    authority = session_authority()
    opened = authority.open(
        UI_A,
        ("observer",),
        "open-a",
        now=5.0,
        requested_ttl_s=1.0,
    )
    assert opened.session is not None

    rejected = authority.accept(
        opened.session.fence(1),
        "simulation_command",
        now=6.0,
    )

    assert rejected.reason == "session_expired"
    assert authority.active(now=6.0) is None
