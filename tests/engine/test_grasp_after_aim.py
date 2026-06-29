from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.core.config_loader import PickConfig
from engine.behaviors.pick.actions import ControlService
from engine.vision.pick.core import ObjectPickPhase
from engine.behaviors.pick.state import HostState, PanelState
from engine.core.protocol import SimQ


class TestGraspAfterAim(unittest.TestCase):
    def test_start_grasp_rejects_without_any_object(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc.client.last_object_world_xyz = None
        ok = svc._start_grasp_to_object(internal=True)
        self.assertFalse(ok)
        self.assertTrue(svc.state.pick_failed)
        self.assertIn("grasp missing object", svc.state.pick_status_msg)

    def test_start_grasp_works_after_look_only(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc._pick_cfg = PickConfig(grasp_standoff_m=0.05, grasp_guided_enabled=False)
        svc._pick_look_object_world_xyz = (0.33, 0.01, 0.92)
        svc._pick_look_dir_world = (1.0, 0.0, 0.0)
        svc.state.raw_sag_model = {"seg1_equal_offset_deg": 0.0}
        with patch.object(svc, "_start_ready_pose_resolve_and_solve") as mock_solve:
            ok = svc._start_grasp_to_object(internal=True)
        self.assertTrue(ok)
        kwargs = mock_solve.call_args.kwargs
        self.assertEqual(kwargs["object_world"], (0.33, 0.01, 0.92))
        self.assertFalse(kwargs["corrected"])
        self.assertFalse(kwargs["resolve_dir"])
        self.assertTrue(kwargs["close_gripper_after"])

    def test_start_grasp_calls_direct_ik_with_standoff_and_close_claw(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc._pick_cfg = PickConfig(grasp_standoff_m=0.05, grasp_guided_enabled=False)
        svc._pick_centered_object_world_xyz = (0.33, 0.01, 0.92)
        svc._pick_equal_sag_model = {"seg1_equal_offset_deg": 0.8, "seg2_equal_offset_deg": 1.1}
        with patch.object(
            svc,
            "_pick_ready_direction",
            return_value=(1.0, 0.0, 0.0),
        ), patch.object(svc, "_start_ready_pose_resolve_and_solve") as mock_solve:
            ok = svc._start_grasp_to_object(internal=True)
        self.assertTrue(ok)
        kwargs = mock_solve.call_args.kwargs
        self.assertEqual(kwargs["object_world"], (0.33, 0.01, 0.92))
        self.assertAlmostEqual(kwargs["target_world"][0], 0.28, places=3)
        self.assertTrue(kwargs["corrected"])
        self.assertTrue(kwargs["close_gripper_after"])

    def test_start_grasp_public_delegates(self) -> None:
        svc = ControlService(PanelState())
        with patch.object(svc, "_start_grasp_to_object", return_value=True) as mock_grasp:
            svc.start_grasp()
        mock_grasp.assert_called_once_with()

    def test_close_gripper_blocked_when_precontact_not_reached(self) -> None:
        svc = ControlService(PanelState(), use_hardware=True)
        svc.client = MagicMock()
        host_state = HostState(
            connected=True,
            tx_seq=1,
            rx_age_s=0.0,
            device="test",
            ports=(),
            torque_enabled=True,
            claw_current=0,
            motor_currents_ma={},
            safety_fault="",
            actual_tip_xyz=None,
            actual_tip_dir=None,
            perceived_object_label="",
            perceived_object_confidence=0.0,
            perceived_object_camera_xyz=None,
            perceived_center_uv=None,
            perceived_scale=None,
            perceived_timestamp_s=0.0,
            reply_ok=True,
            reply_reason="ok",
            q=SimQ(linear_m=0.1, roll_rad=0.0, theta1_rad=0.0, theta2_rad=0.0),
            u=None,
        )
        with patch.object(
            svc,
            "_wait_until_grasp_target_reached",
            return_value=(False, 0.042, host_state),
        ):
            ok, _ = svc._close_gripper_after_grasp_arrival(
                host_state=host_state,
                q_cmd=np.array([0.1, 0.0, 0.0, 0.0], dtype=float),
                target_world=np.array([0.28, 0.01, 0.92], dtype=float),
                sag_model={},
                label="grasp pre-contact",
            )
        self.assertFalse(ok)
        self.assertTrue(svc.state.pick_failed)
        self.assertIn("pre-contact not reached", str(svc.state.pick_status_msg))
        self.assertIn("gripper kept open", str(svc.state.pick_status_msg))
        svc.client.send_claw_command.assert_not_called()

    def test_close_gripper_only_after_precontact_reached(self) -> None:
        svc = ControlService(PanelState(), use_hardware=True)
        svc.client = MagicMock()
        host_state = HostState(
            connected=True,
            tx_seq=1,
            rx_age_s=0.0,
            device="test",
            ports=(),
            torque_enabled=True,
            claw_current=0,
            motor_currents_ma={},
            safety_fault="",
            actual_tip_xyz=None,
            actual_tip_dir=None,
            perceived_object_label="",
            perceived_object_confidence=0.0,
            perceived_object_camera_xyz=None,
            perceived_center_uv=None,
            perceived_scale=None,
            perceived_timestamp_s=0.0,
            reply_ok=True,
            reply_reason="ok",
            q=SimQ(linear_m=0.1, roll_rad=0.0, theta1_rad=0.0, theta2_rad=0.0),
            u=None,
        )
        with patch.object(
            svc,
            "_wait_until_grasp_target_reached",
            return_value=(True, 0.003, host_state),
        ):
            ok, msg = svc._close_gripper_after_grasp_arrival(
                host_state=host_state,
                q_cmd=np.array([0.1, 0.0, 0.0, 0.0], dtype=float),
                target_world=np.array([0.28, 0.01, 0.92], dtype=float),
                sag_model={},
                label="grasp pre-contact",
            )
        self.assertTrue(ok)
        self.assertIn("claw closed", msg)
        svc.client.send_claw_command.assert_called_once_with(claw_closed=True, source="target")


if __name__ == "__main__":
    unittest.main()
