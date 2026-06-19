from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Go2MpcConfig:
    gait_hz: float = 2.5
    gait_duty: float = 0.6
    z_pos_des_m: float = 0.3
    mpc_steps_per_gait: int = 16
    command_idle_threshold: float = 0.05
    torque_safety_scale: float = 0.9
    leg_kv_damping: float = 1.0
    stand_kp: float = 80.0
    stand_kv: float = 4.0
    ctrl_hz: float = 200.0
    command_ramp_s: float = 0.15
    torque_ramp_s: float = 0.12
    torque_warmup_s: float = 0.05
    ready_pose_s: float = 0.12
    ready_kp: float = 120.0
    ready_kv: float = 6.0
    aux_kp: float = 15.0
    aux_kv: float = 6.0
    tau_filter_alpha: float = 0.25
    force_filter_alpha: float = 0.35
    foot_placement_scale: float = 1.35
    payload_enable: bool = True
    payload_mass_kg: float = 0.0
    pitch_trim_gain_z: float = 0.55
    pitch_trim_z_ref_m: float = 0.12
    pitch_trim_max_rad: float = 0.10

    @property
    def gait_period_s(self) -> float:
        return 1.0 / float(self.gait_hz)

    @property
    def mpc_dt_s(self) -> float:
        return float(self.gait_period_s / max(1, int(self.mpc_steps_per_gait)))
