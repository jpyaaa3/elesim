from __future__ import annotations

import unittest
from unittest.mock import MagicMock

import host as host_mod
import engine.protocol as proto
from engine.go2.hardware.odom_parser import OdomSample


class TestHostGo2Bridge(unittest.TestCase):
    def _make_host(self, *, with_bridge: bool, hw=None) -> host_mod.ControlHost:
        bridge = MagicMock() if with_bridge else None
        server = host_mod.ControlHost(
            bind_addr="tcp://127.0.0.1:0",
            sim_pub_addr="tcp://127.0.0.1:0",
            sim_feedback_addr="tcp://127.0.0.1:0",
            hw=hw,
            direction_by_id={},
            device="",
            hardware_cfg=None,
            go2_bridge=bridge,
        )
        self.addCleanup(server.close)
        return server

    def test_go2_vel_forwards_to_bridge(self) -> None:
        server = self._make_host(with_bridge=True)
        ident = b"client-1"
        server._handle_msg(
            ident,
            {
                "t": "target",
                "ts": 1.0,
                "seq": 1,
                "source": "target",
                "go2_vel": [0.2, 0.1, -0.3],
            },
        )
        server._go2_bridge.set_velocity.assert_called_once_with(0.2, 0.1, -0.3)

    def test_go2_sport_pose_forwards_to_bridge(self) -> None:
        server = self._make_host(with_bridge=True)
        ident = b"client-1"
        server._handle_msg(
            ident,
            {
                "t": "target",
                "ts": 1.0,
                "seq": 2,
                "source": "target",
                "go2_sport_pose": "balance_stand",
            },
        )
        server._go2_bridge.call_sport_pose.assert_called_once_with("balance_stand")
        self.assertEqual(server.last_go2_vel, (0.0, 0.0, 0.0))
        self.assertEqual(server.last_go2_sport_pose, "balance_stand")
        self.assertEqual(server.last_go2_sport_pose_seq, 1)

    def test_go2_sport_pose_records_for_sim_without_bridge(self) -> None:
        server = self._make_host(with_bridge=False)
        ident = b"client-1"
        server._handle_msg(
            ident,
            {
                "t": "target",
                "ts": 1.0,
                "seq": 2,
                "source": "target",
                "go2_sport_pose": "lie-down",
            },
        )
        self.assertEqual(server.last_go2_sport_pose, "stand_down")
        self.assertEqual(server.last_go2_sport_pose_seq, 1)

    def test_go2_stand_alias_forwards_stand_up_to_bridge(self) -> None:
        server = self._make_host(with_bridge=True)
        ident = b"client-1"
        server._handle_msg(
            ident,
            {
                "t": "target",
                "ts": 1.0,
                "seq": 2,
                "source": "target",
                "go2_sport_pose": "stand",
            },
        )
        server._go2_bridge.call_sport_pose.assert_called_once_with("stand_up")
        self.assertEqual(server.last_go2_sport_pose, "stand_up")
        self.assertEqual(server.last_go2_sport_pose_seq, 1)

    def test_go2_obstacles_avoid_forwards_to_bridge(self) -> None:
        server = self._make_host(with_bridge=True)
        ident = b"client-1"
        server._handle_msg(
            ident,
            {
                "t": "target",
                "ts": 1.0,
                "seq": 3,
                "source": "target",
                "go2_obstacles_avoid_enable": False,
            },
        )
        server._go2_bridge.set_obstacles_avoid.assert_called_once_with(False)
        self.assertFalse(server.last_go2_obstacles_avoid_enabled)
        self.assertEqual(server.last_go2_obstacles_avoid_seq, 1)

    def test_sim_target_records_and_broadcasts(self) -> None:
        server = self._make_host(with_bridge=False)
        server._broadcast_state_now = MagicMock()  # type: ignore[method-assign]
        ident = b"client-1"
        server._handle_msg(
            ident,
            {
                "t": "target",
                "ts": 1.0,
                "seq": 4,
                "source": "target",
                "sim_target": [0.7, 0.0, 0.08],
            },
        )
        self.assertEqual(server.last_sim_target_xyz, (0.7, 0.0, 0.08))
        server._broadcast_state_now.assert_called_once()

    def test_sim_feedback_q_updates_virtual_host(self) -> None:
        server = self._make_host(with_bridge=False)
        server._handle_sim_feedback(
            {
                "t": "sim_state",
                "ts": 2.0,
                "q": {
                    "linear_m": -0.12,
                    "roll_rad": 0.1,
                    "theta1_rad": 0.2,
                    "theta2_rad": -0.3,
                },
            }
        )
        self.assertIsNotNone(server.last_q)
        self.assertAlmostEqual(server.last_q.linear_m, -0.12)
        self.assertAlmostEqual(server.last_q.roll_rad, 0.1)
        self.assertIsNotNone(server.last_u)

    def test_sim_feedback_q_does_not_override_hardware_host(self) -> None:
        server = self._make_host(with_bridge=False, hw=MagicMock())
        server.last_q = proto.SimQ(linear_m=0.01, roll_rad=0.0, theta1_rad=0.0, theta2_rad=0.0)
        server._handle_sim_feedback(
            {
                "t": "sim_state",
                "ts": 2.0,
                "q": {
                    "linear_m": -0.12,
                    "roll_rad": 0.1,
                    "theta1_rad": 0.2,
                    "theta2_rad": -0.3,
                },
            }
        )
        self.assertIsNotNone(server.last_q)
        self.assertAlmostEqual(server.last_q.linear_m, 0.01)

    def test_sim_feedback_ignores_go2_base_when_bridge_active(self) -> None:
        server = self._make_host(with_bridge=True)
        server.last_go2_base_pos = (1.0, 2.0, 3.0)
        server._handle_sim_feedback(
            {
                "t": "sim_state",
                "ts": 1.0,
                "go2_base_pos": [9.0, 9.0, 9.0],
                "go2_base_rpy": [0.1, 0.2, 0.3],
                "go2_base_lin_vel_body": [1.0, 0.0, 0.0],
                "go2_base_ang_vel": [0.0, 0.0, 0.5],
                "go2_base_timestamp_s": 2.0,
            }
        )
        self.assertEqual(server.last_go2_base_pos, (1.0, 2.0, 3.0))

    def test_apply_go2_base_from_odom(self) -> None:
        server = self._make_host(with_bridge=False)
        sample = OdomSample(
            pos=(1.0, 2.0, 3.0),
            rpy=(0.1, 0.2, 0.3),
            lin_vel_body=(0.5, 0.0, 0.0),
            ang_vel_body=(0.0, 0.0, 0.1),
            timestamp_s=4.5,
            leg_q=tuple(float(i) for i in range(12)),
            leg_dq=tuple(float(i) * 0.1 for i in range(12)),
            leg_torque_nm=tuple(float(i) * 0.2 for i in range(12)),
        )
        server._apply_go2_base_from_odom(sample)
        self.assertEqual(server.last_go2_base_pos, (1.0, 2.0, 3.0))
        self.assertEqual(server.last_go2_base_lin_vel_body, (0.5, 0.0, 0.0))
        self.assertEqual(server.last_go2_leg_q, tuple(float(i) for i in range(12)))
        self.assertEqual(server.last_go2_leg_dq, tuple(float(i) * 0.1 for i in range(12)))
        self.assertEqual(server.last_go2_leg_torque_nm, tuple(float(i) * 0.2 for i in range(12)))
        self.assertAlmostEqual(server.last_go2_base_timestamp_s, 4.5)


if __name__ == "__main__":
    unittest.main()
