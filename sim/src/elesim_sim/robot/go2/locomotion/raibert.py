from __future__ import annotations

import numpy as np

from elesim_sim.robot.go2.locomotion.config import Go2LocomotionConfig
from elesim_sim.robot.go2.locomotion.kinematics import HIP_OFFSET_BODY
from elesim_sim.robot.go2.locomotion.types import Go2Command, LegId


class RaibertFootPlacement:
    """Body-frame swing foot touchdown target using Raibert heuristic."""

    def __init__(self, config: Go2LocomotionConfig) -> None:
        self._config = config

    def compute_foot_target(
        self,
        leg: LegId,
        *,
        v_body: np.ndarray,
        cmd: Go2Command,
    ) -> np.ndarray:
        p_hip = HIP_OFFSET_BODY[leg]
        v_body_xy = np.asarray(v_body, dtype=float).reshape(3)
        v_cmd_xy = np.array([float(cmd.vx), float(cmd.vy), 0.0], dtype=float)

        stance_t = float(self._config.stance_time_s)
        kv = float(self._config.raibert_kv)

        p_des = (
            p_hip
            + 0.5 * stance_t * v_cmd_xy
            + kv * (v_cmd_xy - v_body_xy)
        )

        yaw_rate = float(cmd.yaw_rate)
        p_des[0] += -0.5 * stance_t * yaw_rate * float(p_hip[1])
        p_des[1] += 0.5 * stance_t * yaw_rate * float(p_hip[0])
        p_des[2] = -float(self._config.nominal_body_height_m)
        return p_des
