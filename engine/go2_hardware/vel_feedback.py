from __future__ import annotations

from dataclasses import dataclass


def _clamp(x: float, lo: float, hi: float) -> float:
    return float(min(max(float(x), float(lo)), float(hi)))


@dataclass(frozen=True)
class Go2VelFeedbackGains:
    kp_vx: float = 0.8
    kp_vy: float = 0.8
    kp_wz: float = 1.0
    max_vx: float = 0.6
    max_vy: float = 0.6
    max_wz: float = 1.0
    max_corr_vx: float = 0.15
    max_corr_vy: float = 0.15
    max_corr_wz: float = 0.25
    axis_deadband: float = 0.02


def _axis_cmd(
    target: float,
    actual: float,
    *,
    kp: float,
    max_cmd: float,
    max_corr: float,
    axis_deadband: float,
) -> float:
    t = float(target)
    if abs(t) <= float(axis_deadband):
        return 0.0
    corr = float(kp) * (t - float(actual))
    corr = _clamp(corr, -float(max_corr), float(max_corr))
    return _clamp(t + corr, -float(max_cmd), float(max_cmd))


def compute_feedback_cmd(
    target_vx: float,
    target_vy: float,
    target_wz: float,
    actual_vx: float,
    actual_vy: float,
    actual_wz: float,
    *,
    gains: Go2VelFeedbackGains,
) -> tuple[float, float, float]:
    """Outer-loop correction on Sport Move using body-frame velocity feedback."""
    db = float(gains.axis_deadband)
    return (
        _axis_cmd(
            target_vx,
            actual_vx,
            kp=float(gains.kp_vx),
            max_cmd=float(gains.max_vx),
            max_corr=float(gains.max_corr_vx),
            axis_deadband=db,
        ),
        _axis_cmd(
            target_vy,
            actual_vy,
            kp=float(gains.kp_vy),
            max_cmd=float(gains.max_vy),
            max_corr=float(gains.max_corr_vy),
            axis_deadband=db,
        ),
        _axis_cmd(
            target_wz,
            actual_wz,
            kp=float(gains.kp_wz),
            max_cmd=float(gains.max_wz),
            max_corr=float(gains.max_corr_wz),
            axis_deadband=db,
        ),
    )
