from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.config_loader import PickConfig
from engine.controller.actions import ControlService
from engine.controller.object_pick import ObjectPickPhase
from engine.controller.state import PanelState


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
        svc._pick_cfg = PickConfig(grasp_standoff_m=0.05)
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
        svc._pick_cfg = PickConfig(grasp_standoff_m=0.05)
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


if __name__ == "__main__":
    unittest.main()
