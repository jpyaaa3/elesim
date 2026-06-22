from __future__ import annotations

import json
import unittest
from unittest.mock import MagicMock

import numpy as np

from engine.go2_hardware.config import Go2HardwareConfig
from engine.go2_hardware.odom_parser import odom_msg_to_sample, parse_odom_pose, world_to_elesim
from engine.go2_hardware.sport_api import (
    API_BALANCE_STAND,
    API_DAMP,
    API_MOVE,
    API_STOP_MOVE,
    build_move_parameter,
    shutdown_api_id,
    stand_api_id,
    velocity_below_deadband,
)
from engine.go2_hardware.unitree_ros2_bridge import UnitreeRos2Bridge, create_go2_bridge_if_enabled


class TestSportApi(unittest.TestCase):
    def test_build_move_parameter(self) -> None:
        raw = build_move_parameter(0.3, -0.1, 0.5)
        data = json.loads(raw)
        self.assertEqual(data, {"x": 0.3, "y": -0.1, "z": 0.5})

    def test_velocity_below_deadband(self) -> None:
        self.assertTrue(velocity_below_deadband(0.0, 0.0, 0.0, 0.02))
        self.assertTrue(velocity_below_deadband(0.01, -0.01, 0.02, 0.02))
        self.assertFalse(velocity_below_deadband(0.03, 0.0, 0.0, 0.02))

    def test_stand_and_shutdown_ids(self) -> None:
        self.assertEqual(stand_api_id("balance"), API_BALANCE_STAND)
        self.assertEqual(shutdown_api_id("damp"), API_DAMP)
        self.assertEqual(shutdown_api_id("stop"), API_STOP_MOVE)


class TestOdomParser(unittest.TestCase):
    def test_parse_odom_pose_identity(self) -> None:
        pos, rpy = parse_odom_pose((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
        self.assertAlmostEqual(pos[0], 1.0)
        self.assertAlmostEqual(pos[1], 2.0)
        self.assertAlmostEqual(pos[2], 3.0)
        self.assertAlmostEqual(rpy[0], 0.0, places=5)
        self.assertAlmostEqual(rpy[1], 0.0, places=5)
        self.assertAlmostEqual(rpy[2], 0.0, places=5)

    def test_world_to_elesim_yaw_offset(self) -> None:
        pos, rpy = world_to_elesim((1.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.5, 0.0, 0.0), 90.0)
        self.assertAlmostEqual(pos[0], 0.5, places=5)
        self.assertAlmostEqual(pos[1], 1.0, places=5)
        self.assertAlmostEqual(rpy[2], np.pi / 2.0, places=5)

    def test_odom_msg_to_sample_body_vel(self) -> None:
        sample = odom_msg_to_sample(
            position=(0.0, 0.0, 0.0),
            orientation_quat_xyzw=(0.0, 0.0, 0.0, 1.0),
            lin_vel_world=(1.0, 0.0, 0.0),
            ang_vel_world=(0.0, 0.0, 0.5),
            timestamp_s=1.25,
            offset_xyz=(0.0, 0.0, 0.0),
            yaw_deg=0.0,
        )
        self.assertAlmostEqual(sample.lin_vel_body[0], 1.0, places=5)
        self.assertAlmostEqual(sample.ang_vel_body[2], 0.5, places=5)
        self.assertAlmostEqual(sample.timestamp_s, 1.25, places=5)


class TestGo2HardwareConfig(unittest.TestCase):
    def test_is_active_requires_all_flags(self) -> None:
        cfg = Go2HardwareConfig(enabled=True, backend="unitree_ros2")
        self.assertTrue(cfg.is_active(use_go2=True))
        self.assertFalse(cfg.is_active(use_go2=False))
        self.assertFalse(Go2HardwareConfig(enabled=False).is_active(use_go2=True))


class TestBridgeMock(unittest.TestCase):
    def test_set_velocity_publishes_move_and_stop(self) -> None:
        cfg = Go2HardwareConfig(enabled=True, vel_deadband=0.02, stop_on_zero_vel=True)
        bridge = UnitreeRos2Bridge(cfg)
        bridge._started = True
        bridge._Request = MagicMock()
        bridge._pub = MagicMock()
        published: list[tuple[int, str]] = []

        def _capture(api_id: int, parameter: str) -> None:
            published.append((int(api_id), str(parameter)))

        bridge._publish_api = _capture  # type: ignore[method-assign]

        bridge.set_velocity(0.3, 0.0, 0.2)
        self.assertEqual(published[-1][0], API_MOVE)
        self.assertEqual(json.loads(published[-1][1]), {"x": 0.3, "y": 0.0, "z": 0.2})

        bridge.set_velocity(0.0, 0.0, 0.0)
        self.assertEqual(published[-1][0], API_STOP_MOVE)

    def test_create_bridge_if_disabled(self) -> None:
        cfg = Go2HardwareConfig(enabled=False)
        self.assertIsNone(create_go2_bridge_if_enabled(cfg, use_go2=True))


if __name__ == "__main__":
    unittest.main()
