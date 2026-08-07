from __future__ import annotations

import threading
import time
from enum import Enum


class ControlOwner(str, Enum):
    NONE = "none"
    LOOK = "look"
    AIM = "aim"
    GAZE_TRACK = "gaze_track"
    WALK_APPROACH = "walk_approach"
    GRASP_LJI = "grasp_lji"
    BLIND_FINISH = "blind_finish"
    PLANNED_MOVE = "planned_move"


class ControlState(str, Enum):
    IDLE = "idle"
    GAZE_TRACK = "gaze_track"
    WALK_APPROACH = "walk_approach"
    DONE = "done"
    FAILED = "failed"


class ControlOwnershipError(RuntimeError):
    pass


class ControlOwnership:
    """Single-writer arm control ownership gate with optional experiment FSM."""

    def __init__(self, *, heartbeat_timeout_s: float = 5.0) -> None:
        self._lock = threading.RLock()
        self._owner = ControlOwner.NONE
        self._state = ControlState.IDLE
        self._heartbeat_timeout_s = float(max(0.01, heartbeat_timeout_s))
        self._last_heartbeat_s = 0.0

    @property
    def owner(self) -> ControlOwner:
        with self._lock:
            self._check_heartbeat_locked()
            return self._owner

    def current_state(self) -> ControlState:
        with self._lock:
            self._check_heartbeat_locked()
            return self._state

    def current_owner(self) -> ControlOwner:
        return self.owner

    def _check_heartbeat_locked(self) -> None:
        if self._owner == ControlOwner.NONE:
            return
        if self._last_heartbeat_s <= 0.0:
            return
        if (time.time() - self._last_heartbeat_s) > self._heartbeat_timeout_s:
            self._owner = ControlOwner.NONE
            self._state = ControlState.FAILED

    def heartbeat(self, owner: ControlOwner) -> None:
        with self._lock:
            if self._owner != owner:
                raise ControlOwnershipError(
                    f"heartbeat from {owner.value} but owner is {self._owner.value}"
                )
            self._last_heartbeat_s = time.time()

    def acquire(
        self,
        owner: ControlOwner,
        *,
        state: ControlState | None = None,
        force: bool = False,
    ) -> None:
        with self._lock:
            self._check_heartbeat_locked()
            if self._owner not in (ControlOwner.NONE, owner) and not force:
                raise ControlOwnershipError(
                    f"cannot acquire {owner.value}: current owner is {self._owner.value}"
                )
            self._owner = owner
            if state is not None:
                self._state = state
            elif owner in (ControlOwner.GAZE_TRACK,):
                self._state = ControlState.GAZE_TRACK
            elif owner in (ControlOwner.WALK_APPROACH,):
                self._state = ControlState.WALK_APPROACH
            self._last_heartbeat_s = time.time()

    def release(self, owner: ControlOwner) -> None:
        with self._lock:
            if self._owner == owner:
                self._owner = ControlOwner.NONE
                self._state = ControlState.IDLE
                self._last_heartbeat_s = 0.0

    def require(self, owner: ControlOwner) -> None:
        with self._lock:
            self._check_heartbeat_locked()
            if self._owner != owner:
                raise ControlOwnershipError(
                    f"expected owner {owner.value}, got {self._owner.value}"
                )

    def can_start(self, owner: ControlOwner) -> bool:
        with self._lock:
            self._check_heartbeat_locked()
            return self._owner in (ControlOwner.NONE, owner)
