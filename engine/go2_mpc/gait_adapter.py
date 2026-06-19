from __future__ import annotations

import numpy as np
from convex_mpc.gait import Gait, HEIGHT_SWING
from convex_mpc.go2_robot_data import PinGo2Model


class ScaledGait(Gait):
    """Gait with configurable foot touchdown placement scale (longer stride)."""

    def __init__(self, frequency_hz: float, duty: float, *, placement_scale: float = 1.0) -> None:
        super().__init__(frequency_hz, duty)
        self.placement_scale = max(0.5, float(placement_scale))

    def _scale_xy(self, vec) -> np.ndarray:
        out = np.asarray(vec, dtype=float).reshape(3).copy()
        out[0:2] *= self.placement_scale
        return out

    def compute_touchdown_world_for_traj_purpose_only(self, go2: PinGo2Model, leg: str):
        base_pos = go2.current_config.base_pos
        base_vel = go2.current_config.base_vel
        R_z = go2.R_z
        yaw_rate = go2.yaw_rate_des_world

        hip_offset = go2.get_hip_offset(leg)
        body_pos = np.array([base_pos[0], base_pos[1], 0])
        hip_pos_world = body_pos + R_z @ hip_offset

        t_swing = self.swing_time
        t_stance = self.stance_time
        T = t_swing + 0.5 * t_stance
        pred_time = T / 2.0

        pos_norminal_term = np.array([hip_pos_world[0], hip_pos_world[1], 0.02], dtype=float)
        pos_drift_term = self._scale_xy([base_vel[0] * pred_time, base_vel[1] * pred_time, 0])

        dtheta = yaw_rate * pred_time
        center_xy = np.array([base_pos[0], base_pos[1]])
        r_xy = pos_norminal_term[0:2] - center_xy
        rotation_correction_term = np.array(
            [-dtheta * r_xy[1], dtheta * r_xy[0], 0.0],
            dtype=float,
        )
        return pos_norminal_term + pos_drift_term + rotation_correction_term

    def compute_swing_traj_and_touchdown(self, go2: PinGo2Model, leg: str):
        base_pos = go2.current_config.base_pos
        pos_com_world = go2.pos_com_world
        vel_com_world = go2.vel_com_world
        R_z = go2.R_z
        yaw_rate = go2.yaw_rate_des_world

        hip_offset = go2.get_hip_offset(leg)
        foot_pos, _foot_vel = go2.get_single_foot_state_in_world(leg)
        body_pos = np.array([base_pos[0], base_pos[1], 0])
        hip_pos_world = body_pos + R_z @ hip_offset

        x_vel_des = go2.x_vel_des_world
        y_vel_des = go2.y_vel_des_world
        x_pos_des = go2.x_pos_des_world
        y_pos_des = go2.y_pos_des_world

        t_swing = self.swing_time
        t_stance = self.stance_time
        T = t_swing + 0.5 * t_stance
        pred_time = T / 2.0

        k_v_x = 0.4 * T
        k_p_x = 0.1
        k_v_y = 0.2 * T
        k_p_y = 0.05

        pos_norminal_term = np.array([hip_pos_world[0], hip_pos_world[1], 0.02], dtype=float)
        pos_drift_term = self._scale_xy([x_vel_des * pred_time, y_vel_des * pred_time, 0])
        pos_correction_term = self._scale_xy(
            [k_p_x * (pos_com_world[0] - x_pos_des), k_p_y * (pos_com_world[1] - y_pos_des), 0]
        )
        vel_correction_term = self._scale_xy(
            [k_v_x * (vel_com_world[0] - x_vel_des), k_v_y * (vel_com_world[1] - y_vel_des), 0]
        )

        dtheta = yaw_rate * pred_time
        center_xy = np.array([base_pos[0], base_pos[1]])
        r_xy = pos_norminal_term[0:2] - center_xy
        rotation_correction_term = np.array(
            [-dtheta * r_xy[1], dtheta * r_xy[0], 0.0],
            dtype=float,
        )

        pos_touchdown_world = (
            pos_norminal_term
            + pos_drift_term
            + pos_correction_term
            + vel_correction_term
            + rotation_correction_term
        )
        pos_foot_traj_eval_at_world = self.make_swing_trajectory(
            foot_pos, pos_touchdown_world, t_swing, h_sw=HEIGHT_SWING
        )
        return pos_foot_traj_eval_at_world, pos_touchdown_world
