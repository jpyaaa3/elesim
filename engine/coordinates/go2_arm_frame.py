"""Sim world ↔ IK arm-base frame when the arm rides on a moving GO2 base."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from engine.perception_bridge.transforms import make_transform_from_world_pose, transform_point


def _world_offset(
    pos: Sequence[float],
    euler_deg: Sequence[float],
    local_offset: Sequence[float],
) -> np.ndarray:
    world_off = Rot.from_euler("xyz", np.asarray(euler_deg, dtype=float), degrees=True).apply(
        np.asarray(local_offset, dtype=float).reshape(3)
    )
    p = np.asarray(pos, dtype=float).reshape(3)
    return p + world_off


def _normalize3(v: Sequence[float]) -> Optional[np.ndarray]:
    arr = np.asarray(v, dtype=float).reshape(3)
    n = float(np.linalg.norm(arr))
    if n <= 1e-9:
        return None
    return arr / n


@dataclass(frozen=True)
class Go2ArmFrameConfig:
    """Static mount geometry: IK spawn frame vs GO2 body-mounted arm."""

    use_go2: bool
    ik_spawn_xyz: tuple[float, float, float]
    ik_spawn_euler_deg: tuple[float, float, float]
    go2_spawn_xyz: tuple[float, float, float]
    go2_spawn_euler_deg: tuple[float, float, float]
    mount_offset_body_m: tuple[float, float, float]
    mount_pos_body: tuple[float, float, float]
    mount_rot_body_quat_xyzw: tuple[float, float, float, float]

    @classmethod
    def from_context(
        cls,
        *,
        use_go2: bool,
        spawn_xyz: Sequence[float],
        spawn_euler_deg: Sequence[float],
        go2_spawn_height: float,
        go2_spawn_euler_deg: Sequence[float],
        mount_offset_body_m: Sequence[float],
        ik_context: dict[str, Any],
    ) -> Optional["Go2ArmFrameConfig"]:
        if not bool(use_go2):
            return None
        ik_spawn = tuple(float(x) for x in np.asarray(ik_context["spawn_xyz"], dtype=float).reshape(3))
        ik_euler = tuple(float(x) for x in np.asarray(ik_context["spawn_euler_deg"], dtype=float).reshape(3))
        go2_spawn = (
            float(spawn_xyz[0]),
            float(spawn_xyz[1]),
            float(go2_spawn_height),
        )
        go2_euler = tuple(float(x) for x in np.asarray(go2_spawn_euler_deg, dtype=float).reshape(3))
        mount = tuple(float(x) for x in np.asarray(mount_offset_body_m, dtype=float).reshape(3))
        go2_pos = np.asarray(go2_spawn, dtype=float)
        R_go2 = Rot.from_euler("xyz", go2_euler, degrees=True)
        arm_pos_init = _world_offset(go2_spawn, go2_euler, mount)
        mount_pos_body = R_go2.inv().apply(arm_pos_init - go2_pos)
        R_arm_init = Rot.from_euler("xyz", ik_euler, degrees=True)
        mount_rot_body = R_go2.inv() * R_arm_init
        q = mount_rot_body.as_quat()
        return cls(
            use_go2=True,
            ik_spawn_xyz=ik_spawn,
            ik_spawn_euler_deg=ik_euler,
            go2_spawn_xyz=go2_spawn,
            go2_spawn_euler_deg=go2_euler,
            mount_offset_body_m=mount,
            mount_pos_body=(
                float(mount_pos_body[0]),
                float(mount_pos_body[1]),
                float(mount_pos_body[2]),
            ),
            mount_rot_body_quat_xyzw=(
                float(q[0]),
                float(q[1]),
                float(q[2]),
                float(q[3]),
            ),
        )

    def ik_spawn_transform(self) -> np.ndarray:
        pos = np.asarray(self.ik_spawn_xyz, dtype=float).reshape(3)
        rot = Rot.from_euler("xyz", np.asarray(self.ik_spawn_euler_deg, dtype=float), degrees=True)
        return make_transform_from_world_pose(pos, rot.as_matrix())

    def _mount_rot_body(self) -> Rot:
        return Rot.from_quat(np.asarray(self.mount_rot_body_quat_xyzw, dtype=float).reshape(4))

    def arm_base_transform(
        self,
        go2_pos: Sequence[float],
        go2_rpy_rad: Sequence[float],
    ) -> np.ndarray:
        pos_go2 = np.asarray(go2_pos, dtype=float).reshape(3)
        R_go2 = Rot.from_euler("xyz", np.asarray(go2_rpy_rad, dtype=float).reshape(3), degrees=False)
        mount_pos = np.asarray(self.mount_pos_body, dtype=float).reshape(3)
        arm_pos = pos_go2 + R_go2.apply(mount_pos)
        R_arm = R_go2 * self._mount_rot_body()
        return make_transform_from_world_pose(arm_pos, R_arm.as_matrix())

    def default_go2_pose(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        rpy = tuple(
            float(x)
            for x in np.radians(np.asarray(self.go2_spawn_euler_deg, dtype=float).reshape(3))
        )
        return self.go2_spawn_xyz, rpy


def sim_point_to_ik_frame(
    point_sim: Sequence[float],
    *,
    T_ik_spawn: np.ndarray,
    T_arm_current: np.ndarray,
) -> np.ndarray:
    """Map a sim-world point into the fixed IK arm-base frame."""
    T = np.asarray(T_ik_spawn, dtype=float).reshape(4, 4) @ np.linalg.inv(np.asarray(T_arm_current, dtype=float).reshape(4, 4))
    return transform_point(T, point_sim)


def ik_point_to_sim_frame(
    point_ik: Sequence[float],
    *,
    T_ik_spawn: np.ndarray,
    T_arm_current: np.ndarray,
) -> np.ndarray:
    """Map an IK-frame point into sim world."""
    T = np.asarray(T_arm_current, dtype=float).reshape(4, 4) @ np.linalg.inv(np.asarray(T_ik_spawn, dtype=float).reshape(4, 4))
    return transform_point(T, point_ik)


def sim_direction_to_ik_frame(
    direction_sim: Sequence[float],
    *,
    T_ik_spawn: np.ndarray,
    T_arm_current: np.ndarray,
) -> Optional[np.ndarray]:
    R = (
        np.asarray(T_ik_spawn, dtype=float).reshape(4, 4)[:3, :3]
        @ np.linalg.inv(np.asarray(T_arm_current, dtype=float).reshape(4, 4))[:3, :3]
    )
    return _normalize3(R @ np.asarray(direction_sim, dtype=float).reshape(3))


def ik_direction_to_sim_frame(
    direction_ik: Sequence[float],
    *,
    T_ik_spawn: np.ndarray,
    T_arm_current: np.ndarray,
) -> Optional[np.ndarray]:
    R = (
        np.asarray(T_arm_current, dtype=float).reshape(4, 4)[:3, :3]
        @ np.linalg.inv(np.asarray(T_ik_spawn, dtype=float).reshape(4, 4))[:3, :3]
    )
    return _normalize3(R @ np.asarray(direction_ik, dtype=float).reshape(3))
