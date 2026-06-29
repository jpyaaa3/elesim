from __future__ import annotations

import threading
from enum import Enum


class ControlOwner(str, Enum):
    NONE = "none"
    LOOK = "look"
    AIM = "aim"
    GAZE_TRACK = "gaze_track"
    WALK_APPROACH = "walk_approach"
    GRASP_LJI = "grasp_lji"
    BLIND_FINISH = "blind_finish"


class ControlOwnershipError(RuntimeError):
    pass


class ControlOwnership:
    """Single-writer arm control ownership gate."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._owner = ControlOwner.NONE

    @property
    def owner(self) -> ControlOwner:
        with self._lock:
            return self._owner

    def acquire(self, owner: ControlOwner, *, force: bool = False) -> None:
        with self._lock:
            if self._owner not in (ControlOwner.NONE, owner) and not force:
                raise ControlOwnershipError(
                    f"cannot acquire {owner.value}: current owner is {self._owner.value}"
                )
            self._owner = owner

    def release(self, owner: ControlOwner) -> None:
        with self._lock:
            if self._owner == owner:
                self._owner = ControlOwner.NONE

    def require(self, owner: ControlOwner) -> None:
        with self._lock:
            if self._owner != owner:
                raise ControlOwnershipError(
                    f"expected owner {owner.value}, got {self._owner.value}"
                )

    def can_start(self, owner: ControlOwner) -> bool:
        with self._lock:
            return self._owner in (ControlOwner.NONE, owner)
