"""GO2 body mount geometry for the arm base.

This module deliberately does not transform IK targets.  It only answers:
"where is the arm base in world coordinates right now?"
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation as Rot


def _make_transform(pos: Sequence[float], rot: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = np.asarray(rot, dtype=float).reshape(3, 3)
    T[:3, 3] = np.asarray(pos, dtype=float).reshape(3)
    return T


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


@dataclass(frozen=True)
class Go2ArmMount:
    """Static GO2-to-arm mount geometry."""

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
        go2_spawn_height: float,
        go2_spawn_euler_deg: Sequence[float],
        mount_offset_body_m: Sequence[float],
        ik_context: dict[str, Any],
    ) -> Optional["Go2ArmMount"]:
        if not bool(use_go2):
            return None

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

    def _mount_rot_body(self) -> Rot:
        return Rot.from_quat(np.asarray(self.mount_rot_body_quat_xyzw, dtype=float).reshape(4))

    def arm_base_world_transform(
        self,
        go2_pos: Sequence[float],
        go2_rpy_rad: Sequence[float],
    ) -> np.ndarray:
        pos_go2 = np.asarray(go2_pos, dtype=float).reshape(3)
        R_go2 = Rot.from_euler("xyz", np.asarray(go2_rpy_rad, dtype=float).reshape(3), degrees=False)
        mount_pos = np.asarray(self.mount_pos_body, dtype=float).reshape(3)
        arm_pos = pos_go2 + R_go2.apply(mount_pos)
        R_arm = R_go2 * self._mount_rot_body()
        return _make_transform(arm_pos, R_arm.as_matrix())

    def default_go2_pose(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        rpy = tuple(
            float(x)
            for x in np.radians(np.asarray(self.go2_spawn_euler_deg, dtype=float).reshape(3))
        )
        return self.go2_spawn_xyz, rpy


__all__ = ["Go2ArmMount"]
