from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "AGENTS.md").exists())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from elesim_pilot.config import PickConfig
from elesim_pilot.pick.actions import ControlService
from elesim_pilot.pick.state import PanelState


class TestReadyPoseAlign(unittest.TestCase):
    def test_ready_ik_align_kwargs_use_ready_config(self) -> None:
        svc = ControlService(PanelState())
        svc._pick_cfg = PickConfig(
            ready_pose_align_mode="full",
            ready_pose_align_skip_under_deg=3.0,
            ik_align_mode="lite",
            ik_align_skip_under_deg=10.0,
            ik_align_rounds=6,
        )
        kwargs = svc._ready_ik_align_kwargs()
        self.assertEqual(kwargs["align_mode"], "full")
        self.assertAlmostEqual(float(kwargs["align_skip_under_deg"]), 3.0)
        self.assertEqual(int(kwargs["tweak_rounds"]), 6)

    def test_start_ready_passes_resolve_dir_from_config(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc._pick_look_object_world_xyz = (0.5, 0.0, 0.2)
        svc._pick_look_ready_pose_world_xyz = (0.20, 0.0, 0.2)
        svc._pick_look_dir_world = (1.0, 0.0, 0.0)
        svc._pick_cfg = PickConfig(ready_pose_resolve_dir=True)
        with patch.object(svc, "_start_ready_pose_resolve_and_solve") as mock_solve:
            svc.start_ready_pose()
        self.assertTrue(mock_solve.call_args.kwargs["resolve_dir"])

    def test_start_ready_corrected_uses_equal_sag_model(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc._pick_look_object_world_xyz = (0.5, 0.0, 0.2)
        svc._pick_look_ready_pose_world_xyz = (0.20, 0.0, 0.2)
        svc._pick_look_dir_world = (1.0, 0.0, 0.0)
        svc._pick_corrected_object_world_xyz = (0.51, 0.0, 0.2)
        svc._pick_centered_object_world_xyz = (0.51, 0.0, 0.2)
        svc._pick_centered_ready_pose_world_xyz = (0.31, 0.0, 0.2)
        svc._pick_equal_sag_model = {"seg1_equal_offset_deg": 1.5}
        with patch.object(
            svc,
            "_pick_corrected_ready_pose",
            return_value=(0.31, 0.0, 0.2),
        ), patch.object(
            svc,
            "_pick_ready_direction",
            return_value=(1.0, 0.0, 0.0),
        ), patch.object(svc, "_start_ready_pose_resolve_and_solve") as mock_solve:
            svc.start_ready_pose()
        kwargs = mock_solve.call_args.kwargs
        self.assertTrue(kwargs["corrected"])
        self.assertEqual(kwargs["label"], "corrected pre-grasp")
        self.assertEqual(kwargs["sag_model"]["seg1_equal_offset_deg"], 1.5)
        self.assertEqual(kwargs["object_world"], (0.51, 0.0, 0.2))
        self.assertEqual(kwargs["target_world"], (0.31, 0.0, 0.2))
        self.assertTrue(kwargs["resolve_dir"])
        self.assertAlmostEqual(float(kwargs["accept_best_effort_dir_error_deg"]), 15.0)

    def test_ready_pose_marker_is_small_sphere_with_line_to_object(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc._send_ready_pose_markers(
            object_world=(0.50, 0.0, 0.20),
            target=np.array([0.30, 0.0, 0.20], dtype=float),
            direction=np.array([1.0, 0.0, 0.0], dtype=float),
            actual_offset_m=0.20,
            corrected=False,
        )
        markers = svc.client.send_debug_markers.call_args.args[0]
        self.assertEqual([m["name"] for m in markers], ["ready_pose", "ready_pose_dir"])
        self.assertEqual(markers[0]["pos"], [0.30, 0.0, 0.20])
        self.assertNotIn("dir", markers[0])
        self.assertAlmostEqual(float(markers[0]["radius"]), 0.005)
        self.assertEqual(markers[1]["pos"], [0.30, 0.0, 0.20])
        self.assertEqual(markers[1]["dir"], [0.20, 0.0, 0.0])
        self.assertAlmostEqual(float(markers[1]["length"]), 0.20)

    def test_tweak_wrapper_delegates_to_start_ready_pose(self) -> None:
        svc = ControlService(PanelState())
        svc._pick_equal_sag_model = {"seg1_equal_offset_deg": 1.0}
        svc._pick_equal_sag_estimate = MagicMock(accepted=True)
        with patch.object(
            svc,
            "_pick_corrected_ready_pose",
            return_value=(0.31, 0.0, 0.2),
        ), patch.object(svc, "start_ready_pose") as mock_ready:
            svc.start_equal_sag_tweak()
        mock_ready.assert_called_once()

    def test_tweak_wrapper_rejects_without_accepted_estimate(self) -> None:
        svc = ControlService(PanelState())
        svc._pick_equal_sag_model = {"seg1_equal_offset_deg": 1.0}
        svc._pick_equal_sag_estimate = MagicMock(accepted=False, reason="offset_too_large")
        with patch.object(
            svc,
            "_pick_corrected_ready_pose",
            return_value=(0.31, 0.0, 0.2),
        ), patch.object(svc, "start_ready_pose") as mock_ready:
            svc.start_equal_sag_tweak()
        mock_ready.assert_not_called()
        self.assertTrue(svc.state.pick_failed)


if __name__ == "__main__":
    unittest.main()
