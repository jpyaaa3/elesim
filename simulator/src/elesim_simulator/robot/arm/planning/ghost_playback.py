"""Time-parameterize a sparse RRT waypoint list into a fixed-tick-rate stream
for animating the planned-move ghost (see ``elesim_simulator.runtime.SimScene
.start_ghost_preview``).

This deliberately duplicates the small algorithm in the Controller's
``elesim_controller.robot.arm.planning.trajectory`` rather than importing it
-- a deployment must not import a sibling deployment (see AGENTS.md). Unlike
the Controller's version (paced for streaming motion_command at a fixed
20 Hz), this resamples at the Simulator's own physics tick rate so ghost
playback stays in lock-step with ``SimScene.step_ghost_preview``.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np

# Not derived from a verified hardware register -- mirrors the same
# unverified default used by the Controller's trajectory.py.
DEFAULT_LINEAR_RATE_M_S = 0.02

_DOF = 4


def build_ghost_stream(
    waypoints: Sequence[Sequence[float]],
    *,
    roll_rate: float,
    bend_rate: float,
    linear_rate: float = DEFAULT_LINEAR_RATE_M_S,
    tick_hz: float,
) -> list[np.ndarray]:
    """Resample a piecewise-linear joint-space path at a fixed tick rate.

    Each segment's duration is bounded by its slowest joint (max-rate
    per-joint bound); the result is one ``q`` (linear, roll, theta1, theta2)
    per tick from the first waypoint through the last, inclusive.
    """
    if not waypoints:
        return []
    qs = [np.asarray(wp, dtype=float).reshape(_DOF) for wp in waypoints]
    if len(qs) == 1:
        return [qs[0].copy()]
    if tick_hz <= 0.0:
        raise ValueError("tick_hz must be positive")

    rates = np.array(
        [max(float(linear_rate), 1e-9), max(float(roll_rate), 1e-9), max(float(bend_rate), 1e-9), max(float(bend_rate), 1e-9)],
        dtype=float,
    )

    times = [0.0]
    for prev, curr in zip(qs[:-1], qs[1:]):
        delta = np.abs(curr - prev)
        times.append(times[-1] + float(np.max(delta / rates)))
    times_arr = np.asarray(times, dtype=float)
    duration = float(times_arr[-1])

    dt = 1.0 / float(tick_hz)
    n_ticks = int(np.floor(duration / dt + 1e-9)) + 1
    out: list[np.ndarray] = []
    for tick in range(n_ticks):
        t = min(tick * dt, duration)
        idx = int(np.clip(np.searchsorted(times_arr, t, side="right") - 1, 0, len(times_arr) - 2))
        t0, t1 = times_arr[idx], times_arr[idx + 1]
        span = t1 - t0
        alpha = 0.0 if span <= 1e-12 else float((t - t0) / span)
        out.append(qs[idx] + alpha * (qs[idx + 1] - qs[idx]))

    if not np.allclose(out[-1], qs[-1], atol=1e-9):
        out.append(qs[-1].copy())
    return out


__all__ = ["DEFAULT_LINEAR_RATE_M_S", "build_ghost_stream"]
