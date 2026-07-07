from __future__ import annotations

import unittest

import numpy as np

from engine.robot.arm.mounts.go2_mount import Go2ArmMount
from engine.robot.arm.iklib.kinematics import (
    _forward_grasp_direction_world,
    _forward_grasp_world,
    with_base_world_transform,
)


def _minimal_context() -> dict:
    return {
        "linear_joint_name": "linear",
        "roll_joint_name": "roll",
        "bend_joint_names": ["bend1", "bend2"],
        "n_seg": 1,
        "sag_model": {},
        "part_pose_root": {
            "root": np.zeros(3, dtype=float),
            "tip": np.zeros(3, dtype=float),
        },
        "part_rot_root": {
            "root": np.eye(3, dtype=float),
            "tip": np.eye(3, dtype=float),
        },
        "fk_root_link": "root",
        "fk_joint_chain": [
            {
                "name": "fixed_tip",
                "type": "fixed",
                "parent": "root",
                "child": "tip",
                "origin_parent": np.array([1.0, 0.0, 0.0], dtype=float),
                "axis_parent": np.array([1.0, 0.0, 0.0], dtype=float),
                "child_rot_parent": np.eye(3, dtype=float),
            }
        ],
        "spawn_xyz": np.array([0.0, 0.0, 0.0], dtype=float),
        "spawn_euler_deg": np.array([0.0, 0.0, 0.0], dtype=float),
        "terminal_link_name": "tip",
        "old_tip_local_offset": np.zeros(3, dtype=float),
        "grasp_offset_node_local": np.zeros(3, dtype=float),
        "approach_axis_local": np.array([1.0, 0.0, 0.0], dtype=float),
        "approach_rot_tip": np.eye(3, dtype=float),
    }


class TestGo2Mount(unittest.TestCase):
    def _mount(self) -> Go2ArmMount:
        mount = Go2ArmMount.from_context(
            use_go2=True,
            spawn_xyz=(0.0, 0.0, 0.0),
            go2_spawn_height=0.32,
            go2_spawn_euler_deg=(0.0, 0.0, 0.0),
            mount_offset_body_m=(0.35, 0.0, 0.08),
            ik_context={"spawn_euler_deg": (0.0, 0.0, 0.0)},
        )
        assert mount is not None
        return mount

    def test_mount_default_pose_places_arm_base(self) -> None:
        mount = self._mount()
        pos, rpy = mount.default_go2_pose()
        T = mount.arm_base_world_transform(pos, rpy)
        np.testing.assert_allclose(T[:3, 3], [0.35, 0.0, 0.40], atol=1e-6)
        np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-6)

    def test_mount_tracks_go2_translation_and_yaw(self) -> None:
        mount = self._mount()
        T = mount.arm_base_world_transform((0.0, 0.0, 0.32), (0.0, 0.0, np.pi / 2.0))
        np.testing.assert_allclose(T[:3, 3], [0.0, 0.35, 0.40], atol=1e-6)
        np.testing.assert_allclose(T[:3, :3] @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-6)

    def test_ik_context_base_transform_moves_fk_world_pose(self) -> None:
        ctx = _minimal_context()
        q = np.zeros(4, dtype=float)
        np.testing.assert_allclose(_forward_grasp_world(ctx, q), [1.0, 0.0, 0.0], atol=1e-6)

        T = np.eye(4, dtype=float)
        T[:3, 3] = [0.5, 0.1, 0.0]
        moved = with_base_world_transform(ctx, T)
        np.testing.assert_allclose(_forward_grasp_world(moved, q), [1.5, 0.1, 0.0], atol=1e-6)

    def test_ik_context_base_transform_rotates_fk_world_direction(self) -> None:
        ctx = _minimal_context()
        q = np.zeros(4, dtype=float)
        T = np.eye(4, dtype=float)
        T[:3, :3] = np.array(
            [
                [0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        rotated = with_base_world_transform(ctx, T)
        np.testing.assert_allclose(_forward_grasp_world(rotated, q), [0.0, 1.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(_forward_grasp_direction_world(rotated, q), [0.0, 1.0, 0.0], atol=1e-6)


if __name__ == "__main__":
    unittest.main()
