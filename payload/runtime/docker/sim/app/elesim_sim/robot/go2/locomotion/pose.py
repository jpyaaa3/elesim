"""Continuous joint-pose transitions for simulated GO2 locomotion."""

from __future__ import annotations

import math

import numpy as np


def smoothstep_quintic(value: float) -> float:
    """Return a clamped quintic blend with zero slope at both endpoints."""

    t = float(value)
    if not math.isfinite(t):
        raise ValueError("smoothstep input must be finite")
    t = min(1.0, max(0.0, t))
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t))


class JointPoseTransition:
    """Interpolate a joint vector with zero endpoint velocity and acceleration."""

    def __init__(self, duration_s: float) -> None:
        duration = float(duration_s)
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError("pose transition duration must be finite and positive")
        self.duration_s = duration
        self._start: np.ndarray | None = None
        self._target: np.ndarray | None = None
        self._current: np.ndarray | None = None
        self._elapsed_s = 0.0
        self._active = False

    @staticmethod
    def _pose(value, name: str) -> np.ndarray:
        pose = np.asarray(value, dtype=float).reshape(-1)
        if pose.size == 0 or not np.all(np.isfinite(pose)):
            raise ValueError(f"{name} must be a non-empty finite joint vector")
        return pose.copy()

    @property
    def active(self) -> bool:
        return self._active

    @property
    def current(self) -> np.ndarray | None:
        return None if self._current is None else self._current.copy()

    def reset(self, pose) -> None:
        current = self._pose(pose, "pose")
        self._start = current.copy()
        self._target = current.copy()
        self._current = current
        self._elapsed_s = 0.0
        self._active = False

    def begin(self, start, target) -> None:
        initial = self._pose(start, "start pose")
        final = self._pose(target, "target pose")
        if initial.shape != final.shape:
            raise ValueError("start and target poses must have the same joint count")
        self._start = initial
        self._target = final
        self._current = initial.copy()
        self._elapsed_s = 0.0
        self._active = not np.array_equal(initial, final)
        if not self._active:
            self._current = final.copy()

    def update(self, dt: float) -> tuple[np.ndarray, bool]:
        step_s = float(dt)
        if not math.isfinite(step_s) or step_s < 0.0:
            raise ValueError("pose transition dt must be finite and non-negative")
        if self._current is None or self._start is None or self._target is None:
            raise RuntimeError("pose transition must be reset or begun before update")
        if not self._active:
            return self._current.copy(), True

        self._elapsed_s = min(self.duration_s, self._elapsed_s + step_s)
        blend = smoothstep_quintic(self._elapsed_s / self.duration_s)
        self._current = self._start + (self._target - self._start) * blend
        done = self._elapsed_s >= self.duration_s
        if done:
            self._current = self._target.copy()
            self._active = False
        return self._current.copy(), done
