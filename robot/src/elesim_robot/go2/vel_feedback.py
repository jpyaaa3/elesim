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
    heading_hold_kp: float = 2.5
    heading_hold_ki: float = 0.4
    heading_hold_kd: float = 0.15
    heading_hold_max_wz: float = 0.5
    heading_hold_integral_max: float = 0.35


@dataclass
class HeadingHoldController:
    integral: float = 0.0
    last_err: float = 0.0
    _t_last: float | None = None

    def reset(self) -> None:
        self.integral = 0.0
        self.last_err = 0.0
        self._t_last = None

    def compute(
        self,
        held_yaw: float,
        current_yaw: float,
        ang_vel_z: float,
        now_s: float,
        *,
        gains: Go2VelFeedbackGains,
    ) -> float:
        dt = 0.05
        if self._t_last is not None:
            dt = _clamp(float(now_s) - float(self._t_last), 1e-3, 0.2)
        self._t_last = float(now_s)

        err = wrap_to_pi(float(held_yaw) - float(current_yaw))
        self.last_err = float(err)
        self.integral += float(err) * float(dt)
        i_max = float(gains.heading_hold_integral_max)
        self.integral = _clamp(self.integral, -i_max, i_max)

        wz = (
            float(gains.heading_hold_kp) * float(err)
            + float(gains.heading_hold_ki) * float(self.integral)
            - float(gains.heading_hold_kd) * float(ang_vel_z)
        )
        return _clamp(wz, -float(gains.heading_hold_max_wz), float(gains.heading_hold_max_wz))


def linear_motion_active(target_vx: float, target_vy: float, *, axis_deadband: float) -> bool:
    db = float(axis_deadband)
    return abs(float(target_vx)) > db or abs(float(target_vy)) > db


def yaw_command_active(target_wz: float, *, axis_deadband: float) -> bool:
    return abs(float(target_wz)) > float(axis_deadband)


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
    heading_ctl: HeadingHoldController | None = None,
    now_s: float | None = None,
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
        and heading_ctl is not None
        and now_s is not None
    ):
        cmd_wz = heading_ctl.compute(
            float(held_yaw),
            float(current_yaw),
            float(actual_wz),
            float(now_s),
            gains=gains,
        )
    else:
        cmd_wz = 0.0
    return (cmd_vx, cmd_vy, cmd_wz)
