from __future__ import annotations

import json
import unittest
from unittest.mock import MagicMock

import numpy as np

from engine.go2.hardware.config import Go2HardwareConfig
from engine.go2.hardware.odom_parser import odom_msg_to_sample, parse_odom_pose, world_to_elesim
from engine.go2.hardware.sport_api import (
    API_BALANCE_STAND,
    API_DAMP,
    API_MOVE,
    API_RECOVERY_STAND,
    API_STAND_DOWN,
    API_STAND_UP,
    API_STATIC_WALK,
    API_STOP_MOVE,
    API_TROT_RUN,
    build_move_parameter,
    fill_unitree_request,
    gait_on_start_api_id,
    normalize_gait_on_start,
    normalize_go2_sport_pose,
    shutdown_api_id,
    sport_pose_api_id,
    stand_api_id,
    velocity_below_deadband,
)
from engine.go2.hardware.sport_state_parser import sportmodestate_to_sample
from engine.go2.hardware.odom_parser import OdomSample
from engine.go2.hardware.unitree_ros2_bridge import UnitreeRos2Bridge, _ros_topic, create_go2_bridge_if_enabled


class TestSportApi(unittest.TestCase):
    def test_fill_unitree_request_header(self) -> None:
        class _Policy:
            priority = -1
            noreply = False

        class _Lease:
            id = -1

        class _Identity:
            id = -1
            api_id = -1

        class _Header:
            identity = _Identity()
            lease = _Lease()
            policy = _Policy()

        class _Request:
            header = _Header()
            parameter = ""
            binary = [1]

        req = _Request()
        fill_unitree_request(req, api_id=1008, parameter='{"x":0.2,"y":0.0,"z":0.0}')
        self.assertEqual(req.header.identity.id, 0)
        self.assertEqual(req.header.identity.api_id, 1008)
        self.assertEqual(req.header.lease.id, 0)
        self.assertEqual(req.header.policy.priority, 0)
        self.assertTrue(req.header.policy.noreply)
        self.assertEqual(req.parameter, '{"x":0.2,"y":0.0,"z":0.0}')
        self.assertEqual(req.binary, [])

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

    def test_sport_pose_ids(self) -> None:
        self.assertEqual(sport_pose_api_id("balance_stand"), API_BALANCE_STAND)
        self.assertEqual(sport_pose_api_id("stand_up"), API_STAND_UP)
        self.assertEqual(sport_pose_api_id("stand_down"), API_STAND_DOWN)
        self.assertEqual(sport_pose_api_id("recovery_stand"), API_RECOVERY_STAND)
        self.assertEqual(sport_pose_api_id("static_walk"), API_STATIC_WALK)
        self.assertEqual(normalize_go2_sport_pose("stand"), "stand_up")
        self.assertEqual(normalize_go2_sport_pose("sit"), "stand_down")
        self.assertEqual(normalize_go2_sport_pose("lie-down"), "stand_down")
        self.assertIsNone(sport_pose_api_id("dance"))

    def test_gait_on_start_ids(self) -> None:
        self.assertEqual(gait_on_start_api_id("static_walk"), API_STATIC_WALK)
        self.assertEqual(gait_on_start_api_id("static"), API_STATIC_WALK)
        self.assertEqual(gait_on_start_api_id("trot_run"), API_TROT_RUN)
        self.assertIsNone(gait_on_start_api_id("none"))
        self.assertEqual(normalize_gait_on_start("static-walk"), "static_walk")


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


class TestSportModeStateParser(unittest.TestCase):
    def test_sportmodestate_to_sample_from_user_echo(self) -> None:
        class _Stamp:
            sec = 1782134663
            nanosec = 28960662

        class _Imu:
            rpy = [0.01275869831442833, -0.0844469740986824, -0.005300145596265793]
            gyroscope = [-0.011717907153069973, 0.027696872130036354, -0.017044229432940483]

        class _Msg:
            stamp = _Stamp()
            position = [-0.007525680121034384, 0.0019341090228408575, 0.049848105758428574]
            velocity = [2.432839352195515e-08, 3.750767252341802e-09, 2.831849030826561e-07]
            yaw_speed = -0.017044229432940483
            imu_state = _Imu()

        sample = sportmodestate_to_sample(_Msg(), offset_xyz=(0.0, 0.0, 0.0), yaw_deg=0.0)
        self.assertAlmostEqual(sample.pos[0], -0.007525680121034384, places=6)
        self.assertAlmostEqual(sample.pos[2], 0.049848105758428574, places=6)
        self.assertAlmostEqual(sample.rpy[1], -0.0844469740986824, places=6)
        self.assertAlmostEqual(sample.ang_vel_body[2], -0.017044229432940483, places=6)


class TestGo2HardwareConfig(unittest.TestCase):
    def test_is_active_requires_all_flags(self) -> None:
        cfg = Go2HardwareConfig(enabled=True, backend="unitree_ros2")
        self.assertTrue(cfg.is_active(use_go2=True))
        self.assertFalse(cfg.is_active(use_go2=False))
        self.assertFalse(Go2HardwareConfig(enabled=False).is_active(use_go2=True))


class TestBridgeMock(unittest.TestCase):
    def test_ros_topic_normalization(self) -> None:
        self.assertEqual(_ros_topic("api/sport/request"), "/api/sport/request")
        self.assertEqual(_ros_topic("/sportmodestate"), "/sportmodestate")

    def test_maybe_log_status_no_crash(self) -> None:
        cfg = Go2HardwareConfig(enabled=True, status_log_interval_s=0.0)
        bridge = UnitreeRos2Bridge(cfg)
        bridge.maybe_log_status(1.0)

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

    def test_set_velocity_with_feedback_corrects_from_body_vel(self) -> None:
        cfg = Go2HardwareConfig(
            enabled=True,
            vel_deadband=0.02,
            vel_feedback_enable=True,
            vel_feedback_kp_vx=1.0,
            vel_feedback_max_vx=0.6,
        )
        bridge = UnitreeRos2Bridge(cfg)
        bridge._started = True
        bridge._latest = OdomSample(
            pos=(0.0, 0.0, 0.0),
            rpy=(0.0, 0.0, 0.0),
            lin_vel_body=(0.1, 0.0, 0.0),
            ang_vel_body=(0.0, 0.0, 0.0),
            timestamp_s=1.0,
        )
        published: list[tuple[int, str]] = []

        def _capture(api_id: int, parameter: str) -> None:
            published.append((int(api_id), str(parameter)))

        bridge._publish_api = _capture  # type: ignore[method-assign]
        bridge.set_velocity(0.3, 0.0, 0.0)
        self.assertEqual(published[-1][0], API_MOVE)
        data = json.loads(published[-1][1])
        self.assertAlmostEqual(data["x"], 0.45, places=5)
        self.assertAlmostEqual(data["y"], 0.0, places=5)
        self.assertAlmostEqual(data["z"], 0.0, places=5)

    def test_call_sport_pose_publishes_pose_and_stop(self) -> None:
        cfg = Go2HardwareConfig(enabled=True, stop_on_zero_vel=True)
        bridge = UnitreeRos2Bridge(cfg)
        bridge._started = True
        bridge._Request = MagicMock()
        bridge._pub = MagicMock()
        published: list[tuple[int, str]] = []

        def _capture(api_id: int, parameter: str) -> None:
            published.append((int(api_id), str(parameter)))

        bridge._publish_api = _capture  # type: ignore[method-assign]

        bridge.call_sport_pose("recovery_stand")
        self.assertEqual(published[0][0], API_STOP_MOVE)
        self.assertEqual(published[-1][0], API_RECOVERY_STAND)

    def test_create_bridge_if_disabled(self) -> None:
        cfg = Go2HardwareConfig(enabled=False)
        self.assertIsNone(create_go2_bridge_if_enabled(cfg, use_go2=True))


if __name__ == "__main__":
    unittest.main()
