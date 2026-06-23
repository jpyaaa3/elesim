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
    err_vx = float(target_vx) - float(actual_vx)
    err_vy = float(target_vy) - float(actual_vy)
    err_wz = float(target_wz) - float(actual_wz)
    cmd_vx = float(target_vx) + float(gains.kp_vx) * err_vx
    cmd_vy = float(target_vy) + float(gains.kp_vy) * err_vy
    cmd_wz = float(target_wz) + float(gains.kp_wz) * err_wz
    return (
        _clamp(cmd_vx, -float(gains.max_vx), float(gains.max_vx)),
        _clamp(cmd_vy, -float(gains.max_vy), float(gains.max_vy)),
        _clamp(cmd_wz, -float(gains.max_wz), float(gains.max_wz)),
    )
