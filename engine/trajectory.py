from __future__ import annotations

from dataclasses import dataclass

import engine.protocol as proto


@dataclass(frozen=True)
class QuinticTimingConfig:
    enable: bool = True
    duration_s: float = 1.2
    min_duration_s: float = 0.25
    max_duration_s: float = 3.0
    linear_scale_m: float = 0.05
    angular_scale_rad: float = 0.35


@dataclass(frozen=True)
class TrajectoryState:
    active: bool
    q_start: proto.SimQ
    q_goal: proto.SimQ
    t0_s: float
    duration_s: float


@dataclass(frozen=True)
class TrajectoryStep:
    q_cmd: proto.SimQ
    done: bool


def _clamp(v: float, lo: float, hi: float) -> float:
    return float(max(float(lo), min(float(hi), float(v))))


def quintic_scale(tau: float) -> float:
    x = _clamp(float(tau), 0.0, 1.0)
    return float(10.0 * x**3 - 15.0 * x**4 + 6.0 * x**5)


def interpolate_q(q_start: proto.SimQ, q_goal: proto.SimQ, s: float) -> proto.SimQ:
    w = _clamp(float(s), 0.0, 1.0)
    return proto.SimQ(
        linear_m=float(q_start.linear_m + w * (q_goal.linear_m - q_start.linear_m)),
        roll_rad=float(q_start.roll_rad + w * (q_goal.roll_rad - q_start.roll_rad)),
        theta1_rad=float(q_start.theta1_rad + w * (q_goal.theta1_rad - q_start.theta1_rad)),
        theta2_rad=float(q_start.theta2_rad + w * (q_goal.theta2_rad - q_start.theta2_rad)),
    )


def estimate_duration_s(q_start: proto.SimQ, q_goal: proto.SimQ, cfg: QuinticTimingConfig) -> float:
    lin_scale = max(float(cfg.linear_scale_m), 1e-6)
    ang_scale = max(float(cfg.angular_scale_rad), 1e-6)
    linear_cost = abs(float(q_goal.linear_m) - float(q_start.linear_m)) / lin_scale
    angular_cost = max(
        abs(float(q_goal.roll_rad) - float(q_start.roll_rad)),
        abs(float(q_goal.theta1_rad) - float(q_start.theta1_rad)),
        abs(float(q_goal.theta2_rad) - float(q_start.theta2_rad)),
    ) / ang_scale
    scale = max(float(linear_cost), float(angular_cost))
    raw = float(cfg.duration_s) * max(1.0, float(scale))
    lo = min(float(cfg.min_duration_s), float(cfg.max_duration_s))
    hi = max(float(cfg.min_duration_s), float(cfg.max_duration_s))
    return _clamp(raw, lo, hi)


class QuinticTrajectoryRunner:
    def __init__(self, cfg: QuinticTimingConfig) -> None:
        self.cfg = cfg
        zero = proto.SimQ(0.0, 0.0, 0.0, 0.0)
        self._state = TrajectoryState(active=False, q_start=zero, q_goal=zero, t0_s=0.0, duration_s=max(float(cfg.duration_s), 1e-6))

    @property
    def active(self) -> bool:
        return bool(self._state.active)

    @property
    def goal(self) -> proto.SimQ:
        return self._state.q_goal

    def cancel(self) -> None:
        self._state = TrajectoryState(
            active=False,
            q_start=self._state.q_start,
            q_goal=self._state.q_goal,
            t0_s=self._state.t0_s,
            duration_s=self._state.duration_s,
        )

    def start(self, *, q_start: proto.SimQ, q_goal: proto.SimQ, now_s: float) -> None:
        duration = estimate_duration_s(q_start, q_goal, self.cfg)
        self._state = TrajectoryState(
            active=True,
            q_start=q_start,
            q_goal=q_goal,
            t0_s=float(now_s),
            duration_s=max(float(duration), 1e-6),
        )

    def step(self, *, now_s: float) -> TrajectoryStep:
        if not self._state.active or not bool(self.cfg.enable):
            return TrajectoryStep(q_cmd=self._state.q_goal, done=True)
        tau = (float(now_s) - float(self._state.t0_s)) / max(float(self._state.duration_s), 1e-6)
        done = bool(tau >= 1.0)
        if done:
            self.cancel()
            return TrajectoryStep(q_cmd=self._state.q_goal, done=True)
        s = quintic_scale(tau)
        return TrajectoryStep(
            q_cmd=interpolate_q(self._state.q_start, self._state.q_goal, s),
            done=False,
        )
