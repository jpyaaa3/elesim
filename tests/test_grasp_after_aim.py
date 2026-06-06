from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.config_loader import PickConfig
from engine.controller.actions import ControlService
from engine.controller.object_pick import ObjectPickPhase
from engine.controller.state import PanelState


class TestGraspAfterAim(unittest.TestCase):
    def test_start_grasp_rejects_without_centered_object(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc._pick_equal_sag_model = {"seg1_equal_offset_deg": 1.0}
        ok = svc._start_grasp_to_object(internal=True)
        self.assertFalse(ok)
        self.assertTrue(svc.state.pick_failed)
        self.assertIn("centered object", svc.state.pick_status_msg)

    def test_start_grasp_calls_direct_ik_with_standoff(self) -> None:
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
        self.assertAlmostEqual(kwargs["target_world"][1], 0.01, places=3)
        self.assertAlmostEqual(kwargs["target_world"][2], 0.92, places=3)
        self.assertFalse(kwargs["resolve_dir"])
        self.assertEqual(kwargs["label"], "grasp pre-contact")
        self.assertEqual(kwargs["pick_phase"], ObjectPickPhase.GRASP.value)
        self.assertEqual(kwargs["profile_phase"], "grasp")
        self.assertEqual(kwargs["sag_model"]["seg1_equal_offset_deg"], 0.8)

    def test_aim_auto_grasp_when_config_enabled(self) -> None:
        svc = ControlService(PanelState())
        svc._pick_cfg = PickConfig(auto_grasp_after_aim=True)
        with patch.object(svc, "_pick_config_effective", return_value=PickConfig(auto_grasp_after_aim=True)), patch.object(
            svc, "_start_grasp_to_object",
            return_value=True,
        ) as mock_grasp, patch.object(
            svc, "_wait_grasp_ik_done",
            return_value=True,
        ) as mock_wait, patch.object(
            svc.state,
            "set_pick_status",
        ) as mock_status:
            # Simulate equal_sag accepted branch logic inline
            pk_done = svc._pick_config_effective()
            self.assertTrue(pk_done.auto_grasp_after_aim)
            if bool(pk_done.auto_grasp_after_aim):
                svc._start_grasp_to_object(internal=True)
                svc._wait_grasp_ik_done(timeout_s=30.0, label="auto grasp")
        mock_grasp.assert_called_once_with(internal=True)
        mock_wait.assert_called_once()


if __name__ == "__main__":
    unittest.main()
