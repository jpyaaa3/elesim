from __future__ import annotations

import math
from dataclasses import dataclass


def _clamp(x: float, lo: float, hi: float) -> float:
    return float(min(max(float(x), float(lo)), float(hi)))


def wrap_to_pi(angle: float) -> float:
    return float(math.atan2(math.sin(float(angle)), math.cos(float(angle))))


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
    heading_hold_kp: float = 1.5
    heading_hold_max_wz: float = 0.35


def linear_motion_active(target_vx: float, target_vy: float, *, axis_deadband: float) -> bool:
    db = float(axis_deadband)
    return abs(float(target_vx)) > db or abs(float(target_vy)) > db


def yaw_command_active(target_wz: float, *, axis_deadband: float) -> bool:
    return abs(float(target_wz)) > float(axis_deadband)


def compute_heading_hold_wz(
    held_yaw: float,
    current_yaw: float,
    *,
    kp: float,
    max_wz: float,
) -> float:
    err = wrap_to_pi(float(held_yaw) - float(current_yaw))
    return _clamp(float(kp) * err, -float(max_wz), float(max_wz))


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
    held_yaw: float | None = None,
    current_yaw: float | None = None,
    heading_hold_enable: bool = False,
) -> tuple[float, float, float]:
    """Outer-loop correction on Sport Move using body-frame velocity feedback."""
    db = float(gains.axis_deadband)
    cmd_vx = _axis_cmd(
        target_vx,
        actual_vx,
        kp=float(gains.kp_vx),
        max_cmd=float(gains.max_vx),
        max_corr=float(gains.max_corr_vx),
        axis_deadband=db,
    )
    cmd_vy = _axis_cmd(
        target_vy,
        actual_vy,
        kp=float(gains.kp_vy),
        max_cmd=float(gains.max_vy),
        max_corr=float(gains.max_corr_vy),
        axis_deadband=db,
    )
    if yaw_command_active(target_wz, axis_deadband=db):
        cmd_wz = _axis_cmd(
            target_wz,
            actual_wz,
            kp=float(gains.kp_wz),
            max_cmd=float(gains.max_wz),
            max_corr=float(gains.max_corr_wz),
            axis_deadband=db,
        )
    elif (
        bool(heading_hold_enable)
        and linear_motion_active(target_vx, target_vy, axis_deadband=db)
        and held_yaw is not None
        and current_yaw is not None
    ):
        cmd_wz = compute_heading_hold_wz(
            float(held_yaw),
            float(current_yaw),
            kp=float(gains.heading_hold_kp),
            max_wz=float(gains.heading_hold_max_wz),
        )
    else:
        cmd_wz = 0.0
    return (cmd_vx, cmd_vy, cmd_wz)
