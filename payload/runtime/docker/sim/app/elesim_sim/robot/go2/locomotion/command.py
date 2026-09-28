"""Smooth planar velocity commands and settle trot stops on support."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from elesim_sim.robot.go2.locomotion.kinematics import TROT_PHASE_OFFSET
from elesim_sim.robot.go2.locomotion.types import ALL_LEGS, Go2Command


_TROT_PHASE_OFFSETS = tuple(float(TROT_PHASE_OFFSET[leg]) for leg in ALL_LEGS)


def trot_all_stance(time_s: float, *, gait_hz: float, duty: float) -> bool:
    """Whether the diagonal trot has its brief four-foot support window."""

    frequency = float(gait_hz)
    stance_duty = float(duty)
    timestamp = float(time_s)
    if (
        not math.isfinite(frequency)
        or not math.isfinite(stance_duty)
        or not math.isfinite(timestamp)
        or frequency <= 0.0
        or not 0.0 < stance_duty < 1.0
    ):
        raise ValueError(
            "trot support check needs finite time, positive gait_hz, and duty in (0, 1)"
        )
    phase = (timestamp * frequency) % 1.0
    return all((phase + offset) % 1.0 < stance_duty for offset in _TROT_PHASE_OFFSETS)


@dataclass
class Go2CommandShaper:
    """Acceleration-limit velocity changes and avoid stopping mid-swing.

    A zero request brakes the current command before the controller returns to
    its stand pose. A nonzero reversal keeps the gait active while the shaped
    command passes through zero.
    """

    linear_accel_mps2: float = 1.2
    yaw_accel_radps2: float = 3.0
    stop_dwell_s: float = 0.2
    current: Go2Command = field(default_factory=Go2Command)
    target: Go2Command = field(default_factory=Go2Command)
    _zero_since_s: float | None = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        self.linear_accel_mps2 = self._positive_finite(
            self.linear_accel_mps2, "linear_accel_mps2"
        )
        self.yaw_accel_radps2 = self._positive_finite(
            self.yaw_accel_radps2, "yaw_accel_radps2"
        )
        self.stop_dwell_s = self._non_negative_finite(self.stop_dwell_s, "stop_dwell_s")

    @staticmethod
    def _positive_finite(value: float, name: str) -> float:
        result = float(value)
        if not math.isfinite(result) or result <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return result

    @staticmethod
    def _non_negative_finite(value: float, name: str) -> float:
        result = float(value)
        if not math.isfinite(result) or result < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return result

    def set_target(self, command: Go2Command) -> None:
        self.target = command
        if not self.target.is_idle(0.0):
            self._zero_since_s = None

    def reset(self) -> None:
        self.current = Go2Command()
        self.target = Go2Command()
        self._zero_since_s = None

    def update(self, dt: float) -> Go2Command:
        step_s = float(dt)
        if not math.isfinite(step_s) or step_s < 0.0:
            raise ValueError("command-shaper dt must be finite and non-negative")

        dx = float(self.target.vx) - float(self.current.vx)
        dy = float(self.target.vy) - float(self.current.vy)
        distance = math.hypot(dx, dy)
        max_linear_delta = self.linear_accel_mps2 * step_s
        if distance > max_linear_delta and distance > 0.0:
            scale = max_linear_delta / distance
            dx *= scale
            dy *= scale

        dw = float(self.target.yaw_rate) - float(self.current.yaw_rate)
        max_yaw_delta = self.yaw_accel_radps2 * step_s
        dw = max(-max_yaw_delta, min(max_yaw_delta, dw))
        self.current = Go2Command(
            vx=float(self.current.vx) + dx,
            vy=float(self.current.vy) + dy,
            yaw_rate=float(self.current.yaw_rate) + dw,
        )
        return self.current

    def stop_ready(
        self,
        *,
        time_s: float,
        threshold: float,
        gait_hz: float,
        gait_duty: float,
    ) -> bool:
        """Allow stand-pose control after braking and a support window.

        If a gait has no four-foot support window (duty <= 0.5), wait at most
        one gait period after reaching zero so a stop cannot wait forever.
        """

        now = float(time_s)
        idle_threshold = float(threshold)
        if not math.isfinite(now) or not math.isfinite(idle_threshold) or idle_threshold < 0.0:
            raise ValueError("stop readiness needs finite time and non-negative threshold")
        if not self.target.is_idle(idle_threshold) or not self.current.is_idle(idle_threshold):
            self._zero_since_s = None
            return False

        if self._zero_since_s is None:
            self._zero_since_s = now
        if now - self._zero_since_s < self.stop_dwell_s:
            return False
        if trot_all_stance(now, gait_hz=gait_hz, duty=gait_duty):
            return True
        period_s = 1.0 / float(gait_hz)
        return now - self._zero_since_s >= period_s
