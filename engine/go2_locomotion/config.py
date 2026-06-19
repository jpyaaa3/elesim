from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Go2LocomotionConfig:
    mode: str = "raibert_trot"
    stance_time_s: float = 0.30
    swing_time_s: float = 0.15
    raibert_kv: float = 0.05
    nominal_body_height_m: float = 0.30
    foot_swing_height_m: float = 0.06
    leg_kp: float = 80.0
    leg_kv: float = 4.0
    command_idle_threshold: float = 0.05
    ground_height_m: float = 0.0
    foot_radius_m: float = 0.022
    leg_max_rate_radps: float = 6.0
    foot_max_step_m: float = 0.008
    base_vel_blend: float = 0.0
    gait_hz: float = 2.5
    gait_duty: float = 0.6
    z_pos_des_m: float = 0.3
    mpc_steps_per_gait: int = 16
    torque_safety_scale: float = 0.9
    mpc_leg_kv_damping: float = 1.0
    mpc_ctrl_hz: float = 200.0
    mpc_command_ramp_s: float = 0.15
    mpc_torque_ramp_s: float = 0.12
    mpc_torque_warmup_s: float = 0.05
    mpc_ready_pose_s: float = 0.12
    mpc_ready_kp: float = 120.0
    mpc_ready_kv: float = 6.0
    mpc_aux_kp: float = 15.0
    mpc_aux_kv: float = 6.0
    mpc_tau_filter_alpha: float = 0.25
    mpc_force_filter_alpha: float = 0.35
    mpc_foot_placement_scale: float = 1.35
    mpc_payload_enable: bool = True
    mpc_payload_mass_kg: float = 0.0
    mpc_pitch_trim_gain_z: float = 0.55
    mpc_pitch_trim_z_ref_m: float = 0.12
    mpc_pitch_trim_max_rad: float = 0.10

    @property
    def cycle_time_s(self) -> float:
        return float(self.stance_time_s + self.swing_time_s)

    @property
    def stance_duty(self) -> float:
        cycle = self.cycle_time_s
        if cycle <= 0.0:
            return 0.5
        return float(self.stance_time_s / cycle)
