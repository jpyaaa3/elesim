from __future__ import annotations

from engine.go2.locomotion.config import Go2LocomotionConfig
from engine.go2.locomotion.kinematics import TROT_PHASE_OFFSET
from engine.go2.locomotion.types import LegId, LegPhase


class GaitScheduler:
    """Trot gait clock with diagonal pair phase offsets."""

    def __init__(self, config: Go2LocomotionConfig) -> None:
        self._config = config
        self._phase = 0.0

    @property
    def phase(self) -> float:
        return float(self._phase)

    def reset(self) -> None:
        self._phase = 0.0

    def step(self, dt: float) -> None:
        cycle = self._config.cycle_time_s
        if cycle <= 0.0:
            return
        self._phase = (self._phase + float(dt) / cycle) % 1.0

    def leg_phase(self, leg: LegId) -> float:
        return (self._phase + float(TROT_PHASE_OFFSET[leg])) % 1.0

    def leg_contact(self, leg: LegId) -> LegPhase:
        if self.leg_phase(leg) < self._config.stance_duty:
            return LegPhase.STANCE
        return LegPhase.SWING

    def swing_progress(self, leg: LegId) -> float:
        if self.leg_contact(leg) == LegPhase.STANCE:
            return 0.0
        duty = self._config.stance_duty
        swing_span = max(1e-9, 1.0 - duty)
        return float((self.leg_phase(leg) - duty) / swing_span)
