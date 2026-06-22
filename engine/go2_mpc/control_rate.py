from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ControlRateInfo:
    """Effective GO2 MPC leg-torque update rate (cannot exceed sim step rate)."""

    sim_hz: float
    ctrl_hz_config: float
    ctrl_decim: int
    ctrl_hz_effective: float

    @classmethod
    def from_sim_dt(cls, dt: float, ctrl_hz_config: float) -> ControlRateInfo:
        sim_hz = 1.0 / max(float(dt), 1e-9)
        ctrl_hz = max(1.0, float(ctrl_hz_config))
        decim = max(1, int(round(sim_hz / ctrl_hz)))
        effective = sim_hz / float(decim)
        return cls(
            sim_hz=float(sim_hz),
            ctrl_hz_config=float(ctrl_hz),
            ctrl_decim=int(decim),
            ctrl_hz_effective=float(effective),
        )
