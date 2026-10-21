from __future__ import annotations

import unittest
from types import SimpleNamespace as NS

import numpy as np

from elesim_pilot.pick.actions import ControlService
from elesim_pilot.remote_state import RemoteState
from elesim_pilot.robot.arm.mounts.go2_mount import Go2ArmMount
from elesim_pilot.robot.arm.iklib.kinematics import (
    _forward_grasp_direction_world,
    _forward_grasp_world,
    with_base_world_transform,
)
from elesim_protocol import SimMappingConfig


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

    def test_pilot_ik_mount_uses_nested_robot_go2_pose_instead_of_spawn_pose(self) -> None:
        state = RemoteState(SimMappingConfig(), clock=lambda: 10.0)
        state.peer_connected(True)
        state.target_changed("robot-go2")
        state.accept_telemetry({"go2": {
            "position": [1.0, -0.2, 0.44],
            "rpy": [0.1, -0.15, 0.6],
            "leg_q": [0.1] * 12,
        }})
        host = state.snapshot(tx_seq=0)
        service = object.__new__(ControlService)
        service._go2_arm_mount = self._mount()

        actual = ControlService._current_arm_base_world_T(service, host)
        expected = self._mount().arm_base_world_transform(
            host.go2_base_pos, host.go2_base_rpy
        )
        spawn_pos, spawn_rpy = self._mount().default_go2_pose()
        spawn = self._mount().arm_base_world_transform(spawn_pos, spawn_rpy)
        np.testing.assert_allclose(actual, expected, atol=1e-9)
        assert not np.allclose(actual, spawn)

    def test_pilot_ik_rejects_partial_go2_pose_instead_of_using_spawn_pose(self) -> None:
        service = object.__new__(ControlService)
        service._go2_arm_mount = self._mount()
        with self.assertRaisesRegex(RuntimeError, "both position and orientation"):
            ControlService._current_arm_base_world_T(
                service,
                NS(go2_base_pos=(1.0, 0.0, 0.4), go2_base_rpy=None, go2_leg_q=(0.0,) * 12),
            )

    def test_pilot_ik_does_not_fall_back_to_spawn_pose_for_robot_telemetry(self) -> None:
        service = object.__new__(ControlService)
        service._go2_arm_mount = self._mount()
        with self.assertRaisesRegex(RuntimeError, "no measured GO2 body pose"):
            ControlService._current_arm_base_world_T(
                service,
                NS(device="/dev/ttyUSB0", connected=True,
                   go2_base_pos=None, go2_base_rpy=None, go2_leg_q=None),
            )

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
