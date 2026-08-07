"""Genesis measurements converted to protocol-v4 telemetry payloads."""

from __future__ import annotations

import time
from typing import Any, Callable, Mapping, Optional, Sequence

import numpy as np

from elesim_protocol import SimQ


class RuntimeTelemetry:
    def __init__(self, publish: Callable[[Mapping[str, Any]], None]) -> None:
        self.publish = publish

    def send_actual_tip(
        self,
        actual_tip_xyz: Optional[np.ndarray],
        actual_tip_dir: Optional[np.ndarray] = None,
        *,
        arm_q: Optional[SimQ] = None,
        camera_origin: Optional[np.ndarray] = None,
        camera_look: Optional[np.ndarray] = None,
        camera_right: Optional[np.ndarray] = None,
        sim_time_s: Optional[float] = None,
        sim_wall_elapsed_s: Optional[float] = None,
        sim_realtime_factor: Optional[float] = None,
        sim_step_count: Optional[int] = None,
    ) -> None:
        if actual_tip_xyz is None and arm_q is None and camera_origin is None and sim_time_s is None:
            return
        payload: dict[str, Any] = {}
        if arm_q is not None:
            payload["q"] = [
                float(arm_q.linear_m),
                float(arm_q.roll_rad),
                float(arm_q.theta1_rad),
                float(arm_q.theta2_rad),
            ]
            payload["q_source"] = "simulated"
        if actual_tip_xyz is not None:
            payload["actual_tip"] = self._vec3(actual_tip_xyz)
        if actual_tip_dir is not None:
            direction = np.asarray(actual_tip_dir, dtype=float).reshape(3)
            norm = float(np.linalg.norm(direction))
            if norm > 1e-9:
                payload["actual_tip_dir"] = self._vec3(direction / norm)
        for key, value in (
            ("camera_world_origin", camera_origin),
            ("camera_world_look", camera_look),
            ("camera_world_right", camera_right),
        ):
            if value is not None:
                payload[key] = self._vec3(value)
        self._add_clock(
            payload,
            sim_time_s=sim_time_s,
            sim_wall_elapsed_s=sim_wall_elapsed_s,
            sim_realtime_factor=sim_realtime_factor,
            sim_step_count=sim_step_count,
        )
        self.publish(payload)

    def send_go2_base(
        self,
        go2_entity: Any,
        *,
        sim_time_s: Optional[float] = None,
        sim_wall_elapsed_s: Optional[float] = None,
        sim_realtime_factor: Optional[float] = None,
        sim_step_count: Optional[int] = None,
        leg_dof_idx: Optional[Sequence[int]] = None,
    ) -> None:
        from scipy.spatial.transform import Rotation

        from elesim_simulator.simulation.genesis.utils import (
            quat_wxyz_to_xyzw,
            to_numpy_1d,
        )

        base = go2_entity.get_link("base")
        position = to_numpy_1d(base.get_pos())[:3]
        quaternion = quat_wxyz_to_xyzw(to_numpy_1d(base.get_quat())[:4])
        rotation = Rotation.from_quat(quaternion)
        rpy = rotation.as_euler("xyz", degrees=False)
        linear_body = rotation.inv().apply(to_numpy_1d(base.get_vel())[:3])
        angular_body = rotation.inv().apply(to_numpy_1d(base.get_ang())[:3])
        payload: dict[str, Any] = {
            "go2_base_pos": self._vec3(position),
            "go2_base_rpy": self._vec3(rpy),
            "go2_base_lin_vel_body": self._vec3(linear_body),
            "go2_base_ang_vel": self._vec3(angular_body),
            "go2_base_timestamp_s": float(time.time()),
        }
        if leg_dof_idx:
            leg_q = to_numpy_1d(go2_entity.get_dofs_position(dofs_idx_local=list(leg_dof_idx)))
            payload["go2_leg_q"] = [float(x) for x in leg_q.reshape(-1)]
        self._add_clock(
            payload,
            sim_time_s=sim_time_s,
            sim_wall_elapsed_s=sim_wall_elapsed_s,
            sim_realtime_factor=sim_realtime_factor,
            sim_step_count=sim_step_count,
        )
        self.publish(payload)

    def send_planned_move_target(
        self,
        xyz: Optional[np.ndarray],
        *,
        sim_time_s: Optional[float] = None,
        sim_wall_elapsed_s: Optional[float] = None,
        sim_realtime_factor: Optional[float] = None,
        sim_step_count: Optional[int] = None,
    ) -> None:
        if xyz is None:
            return
        payload: dict[str, Any] = {"planned_move_target": self._vec3(xyz)}
        self._add_clock(
            payload,
            sim_time_s=sim_time_s,
            sim_wall_elapsed_s=sim_wall_elapsed_s,
            sim_realtime_factor=sim_realtime_factor,
            sim_step_count=sim_step_count,
        )
        self.publish(payload)

    def close(self) -> None:
        return None

    @staticmethod
    def _vec3(value: object) -> list[float]:
        array = np.asarray(value, dtype=float).reshape(3)
        return [float(array[0]), float(array[1]), float(array[2])]

    @staticmethod
    def _add_clock(
        payload: dict[str, Any],
        *,
        sim_time_s: Optional[float],
        sim_wall_elapsed_s: Optional[float],
        sim_realtime_factor: Optional[float],
        sim_step_count: Optional[int],
    ) -> None:
        if sim_time_s is not None:
            payload["sim_time_s"] = float(sim_time_s)
        if sim_wall_elapsed_s is not None:
            payload["sim_wall_elapsed_s"] = float(sim_wall_elapsed_s)
        if sim_realtime_factor is not None:
            payload["sim_realtime_factor"] = float(sim_realtime_factor)
        if sim_step_count is not None:
            payload["sim_step_count"] = int(sim_step_count)


__all__ = ["RuntimeTelemetry"]
