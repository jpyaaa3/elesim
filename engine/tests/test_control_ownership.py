from __future__ import annotations

from engine.controller.control_ownership import ControlOwner, ControlOwnership, ControlOwnershipError


def test_acquire_release() -> None:
    gate = ControlOwnership()
    gate.acquire(ControlOwner.GAZE_TRACK)
    assert gate.owner == ControlOwner.GAZE_TRACK
    gate.release(ControlOwner.GAZE_TRACK)
    assert gate.owner == ControlOwner.NONE


def test_reject_conflicting_acquire() -> None:
    gate = ControlOwnership()
    gate.acquire(ControlOwner.AIM)
    try:
        gate.acquire(ControlOwner.GAZE_TRACK)
        raise AssertionError("expected ControlOwnershipError")
    except ControlOwnershipError:
        pass
