"""Time-parameterize a joint-space waypoint list for streaming as motion_command.

Robot's Dynamixel servos already ramp between consecutive position targets
using their own firmware profile-velocity (see the Dynamixel Profile
Velocity/Acceleration registers); this module only bounds how fast
consecutive *waypoints* may be handed to that ramp, it is not a substitute
for it and it does not do within-segment acceleration shaping.

The roll rate below is re-derived from the Dynamixel profile-velocity
register documented in ``simulator/src/elesim_simulator/robot/arm/rates.py``
for the default protocol ``SimMappingConfig`` (240 profile-velocity units at
0.229*6.0 deg/s per unit) -- it is recomputed here rather than imported,
because a deployment must not import a sibling deployment (see AGENTS.md).

The bend rate is deliberately set to match the roll rate, not the
hardware-derived ~0.28777 rad/s (60 profile-velocity units) -- confirmed
live that the hardware-realistic bend speed made intermediate (collision-
avoidance) waypoints impractical to actually track once the sim itself runs
below real time (see MIN_SIM_REALTIME_FACTOR in pick/planned_move.py): the
commanded reference had to be paced far slower in real time just to give a
hardware-realistic bend axis a chance to converge. Speeding bend up to
roll's rate keeps demo/sim runs responsive; a real deployment should
override this back down to a measured hardware value.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

DEFAULT_ROLL_RATE_RAD_S = 2.8777
DEFAULT_BEND_RATE_RAD_S = DEFAULT_ROLL_RATE_RAD_S
# Not derived from a verified hardware register -- no linear-axis profile
# velocity constant was found anywhere in the repo. Override with a
# measured value before relying on this for a real robot.
DEFAULT_LINEAR_RATE_M_S = 0.02

_DOF = 4


@dataclass(frozen=True)
class JointRateLimits:
    linear_m_s: float = DEFAULT_LINEAR_RATE_M_S
    roll_rad_s: float = DEFAULT_ROLL_RATE_RAD_S
    theta1_rad_s: float = DEFAULT_BEND_RATE_RAD_S
    theta2_rad_s: float = DEFAULT_BEND_RATE_RAD_S

    def as_array(self) -> np.ndarray:
        return np.array(
            [self.linear_m_s, self.roll_rad_s, self.theta1_rad_s, self.theta2_rad_s], dtype=float
        )


@dataclass(frozen=True)
class TrajectorySample:
    t_s: float
    q: np.ndarray


@dataclass(frozen=True)
class Trajectory:
    samples: list[TrajectorySample]

    @property
    def duration_s(self) -> float:
        return 0.0 if not self.samples else float(self.samples[-1].t_s)


def _segment_duration(q_a: np.ndarray, q_b: np.ndarray, rates: JointRateLimits) -> float:
    delta = np.abs(q_b - q_a)
    rate_arr = rates.as_array()
    per_joint = np.where(rate_arr > 0.0, delta / np.maximum(rate_arr, 1e-12), 0.0)
    return float(np.max(per_joint))


def time_parameterize(
    waypoints: Sequence[np.ndarray],
    *,
    rates: JointRateLimits = JointRateLimits(),
    hold_at_start_s: float = 0.0,
) -> Trajectory:
    """Assign an arrival time to each waypoint using a per-joint max-rate bound.

    Each segment's duration is the slowest joint's time-to-travel that
    segment (all four joints are assumed to arrive together). This bounds
    *average* velocity per segment; it intentionally leaves acceleration
    smoothing across segments to the receiving side's own servo ramp.
    """
    if not waypoints:
        return Trajectory(samples=[])
    first = np.asarray(waypoints[0], dtype=float).reshape(_DOF)
    samples = [TrajectorySample(t_s=float(hold_at_start_s), q=first)]
    t = float(hold_at_start_s)
    for prev, curr in zip(waypoints[:-1], waypoints[1:]):
        prev_arr = np.asarray(prev, dtype=float).reshape(_DOF)
        curr_arr = np.asarray(curr, dtype=float).reshape(_DOF)
        t += _segment_duration(prev_arr, curr_arr, rates)
        samples.append(TrajectorySample(t_s=t, q=curr_arr))
    return Trajectory(samples=samples)


def resample(trajectory: Trajectory, *, tick_hz: float) -> list[np.ndarray]:
    """Resample a piecewise-linear trajectory at a fixed tick rate for streaming.

    Returns one ``q`` per tick from ``t=0`` through the trajectory's final
    time inclusive, suitable for feeding straight into successive
    ``motion_command`` sends at ``tick_hz``.
    """
    if not trajectory.samples:
        return []
    if len(trajectory.samples) == 1:
        return [trajectory.samples[0].q.copy()]
    if tick_hz <= 0.0:
        raise ValueError("tick_hz must be positive")

    dt = 1.0 / float(tick_hz)
    duration = trajectory.duration_s
    times = np.asarray([sample.t_s for sample in trajectory.samples], dtype=float)
    qs = np.stack([sample.q for sample in trajectory.samples], axis=0)

    n_ticks = int(np.floor(duration / dt + 1e-9)) + 1
    out: list[np.ndarray] = []
    for tick in range(n_ticks):
        t = min(tick * dt, duration)
        idx = int(np.clip(np.searchsorted(times, t, side="right") - 1, 0, len(times) - 2))
        t0, t1 = times[idx], times[idx + 1]
        span = t1 - t0
        alpha = 0.0 if span <= 1e-12 else float((t - t0) / span)
        out.append(qs[idx] + alpha * (qs[idx + 1] - qs[idx]))

    if not np.allclose(out[-1], qs[-1], atol=1e-9):
        out.append(qs[-1].copy())
    return out


__all__ = [
    "DEFAULT_BEND_RATE_RAD_S",
    "DEFAULT_LINEAR_RATE_M_S",
    "DEFAULT_ROLL_RATE_RAD_S",
    "JointRateLimits",
    "Trajectory",
    "TrajectorySample",
    "resample",
    "time_parameterize",
]
