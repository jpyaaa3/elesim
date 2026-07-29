from __future__ import annotations

import time

from elesim_controller.pick.control_ownership import (
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
    gate = ControlOwnership(heartbeat_timeout_s=0.01)
    gate.acquire(ControlOwner.GAZE_TRACK, state=ControlState.GAZE_TRACK)
    gate.heartbeat(ControlOwner.GAZE_TRACK)
    time.sleep(0.02)
    assert gate.current_state() == ControlState.FAILED
