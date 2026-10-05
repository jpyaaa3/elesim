from __future__ import annotations

import pytest

from elesim_pilot.pick.control_ownership import (
    ControlOwner,
    ControlOwnership,
    ControlOwnershipError,
    ControlState,
)


def test_acquire_release() -> None:
    gate = ControlOwnership()
    gate.acquire(ControlOwner.GAZE_TRACK, state=ControlState.GAZE_TRACK)
    assert gate.owner == ControlOwner.GAZE_TRACK
    assert gate.current_state() == ControlState.GAZE_TRACK
    gate.release(ControlOwner.GAZE_TRACK)
    assert gate.owner == ControlOwner.NONE
    assert gate.current_state() == ControlState.IDLE


def test_reject_conflicting_acquire() -> None:
    gate = ControlOwnership()
    gate.acquire(ControlOwner.AIM)
    try:
        gate.acquire(ControlOwner.GAZE_TRACK)
        raise AssertionError("expected ControlOwnershipError")
    except ControlOwnershipError:
        pass


def test_heartbeat_timeout() -> None:
    now = [0.0]
    gate = ControlOwnership(heartbeat_timeout_s=0.01, clock=lambda: now[0])
    gate.acquire(ControlOwner.GAZE_TRACK, state=ControlState.GAZE_TRACK)
    gate.heartbeat(ControlOwner.GAZE_TRACK)
    now[0] = 0.02
    assert gate.current_state() == ControlState.FAILED


def test_late_heartbeat_cannot_revive_expired_ownership() -> None:
    now = [0.0]
    gate = ControlOwnership(heartbeat_timeout_s=1.0, clock=lambda: now[0])
    gate.acquire(ControlOwner.AIM)
    now[0] = 1.01
    with pytest.raises(ControlOwnershipError, match="owner is none"):
        gate.heartbeat(ControlOwner.AIM)
    assert gate.current_state() == ControlState.FAILED
    assert gate.owner == ControlOwner.NONE
    gate.acquire(ControlOwner.AIM)
    gate.heartbeat(ControlOwner.AIM)
    assert gate.owner == ControlOwner.AIM


def test_default_timeout_uses_monotonic_time(monkeypatch) -> None:
    from elesim_pilot.pick import control_ownership

    now = [10.0]
    monkeypatch.setattr(control_ownership.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(control_ownership.time, "time", lambda: -1_000_000.0)
    gate = ControlOwnership(heartbeat_timeout_s=1.0)
    gate.acquire(ControlOwner.AIM)
    now[0] = 11.01
    assert gate.owner == ControlOwner.NONE


def test_heartbeat_refreshes_only_current_owner() -> None:
    now = [0.0]
    gate = ControlOwnership(heartbeat_timeout_s=1.0, clock=lambda: now[0])
    gate.acquire(ControlOwner.AIM)
    now[0] = 0.75
    gate.heartbeat(ControlOwner.AIM)
    with pytest.raises(ControlOwnershipError):
        gate.heartbeat(ControlOwner.LOOK)
    now[0] = 1.5
    assert gate.owner == ControlOwner.AIM
    now[0] = 1.76
    assert gate.owner == ControlOwner.NONE
