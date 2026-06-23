from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from engine.visual_servoing.uv_jacobian import solve_uv_control_delta


@dataclass(frozen=True)
class GazeStabilizerConfig:
    enable_feedback: bool = True
    enable_base_ff: bool = False
    uv_gain: float = 1.0
    base_ff_gain_pitch: float = 0.0
    base_ff_gain_roll: float = 0.0
    base_ff_gain_yaw: float = 0.0
    max_du_roll: float = 1.0
    max_du_s1: float = 1.0
    max_du_s2: float = 1.0
    jacobian_damping: float = 0.03
    hz: float = 20.0
    center_tol: float = 0.06
    center_u_gain: float = 18.0
    center_v_gain: float = 18.0
    center_roll_max: float = 8.0
    center_seg_max: float = 8.0
    step_scale: float = 1.0
    enable_roll: bool = False
    center_u_kd: float = 0.0
    center_v_kd: float = 4.0
    center_d_seg_max: float = 4.0
    d_filter_alpha: float = 0.35
    max_seg_du_per_tick: float = 1.5
    cmd_settle_s: float = 0.10
    center_u_seg_s2_scale: float = 0.55
    center_u_seg_s1_scale: float = 0.35
    fine_err_max: float = 0.11
    fine_settle_scale: float = 0.35
    fov_margin: float = 0.08
    clamp_go2_vel_on_large_error: bool = False
