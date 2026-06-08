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
from engine.controller.state import PanelState
from engine.visual_servoing.equal_sag_probe import EqualSagEstimate
from engine.visual_servoing.grasp_trajectory import GraspWaypoint


class TestGraspGuidedHelpers(unittest.TestCase):
    def test_nominal_endpoint_shifts_with_object(self) -> None:
        e0 = ControlService._compute_grasp_nominal_endpoint(
            (0.30, 0.0, 0.90),
            (1.0, 0.0, 0.0),
            standoff_m=0.05,
        )
        e1 = ControlService._compute_grasp_nominal_endpoint(
            (0.32, 0.0, 0.90),
            (1.0, 0.0, 0.0),
            standoff_m=0.05,
        )
        self.assertAlmostEqual(e1[0] - e0[0], 0.02, places=4)

    def test_axial_distance_along_approach_dir(self) -> None:
        tip = (0.20, 0.0, 0.90)
        nominal = (0.28, 0.0, 0.90)
        dist = ControlService._grasp_axial_distance(tip, nominal, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(dist, 0.08, places=4)

    def test_sag_clip_limits_per_waypoint_delta(self) -> None:
        svc = ControlService(PanelState())
        base = {"seg1_equal_offset_deg": 0.0, "seg2_equal_offset_deg": 0.0}
        estimate = EqualSagEstimate(
            accepted=True,
            seg1_equal_offset_deg=5.0,
            seg2_equal_offset_deg=-4.0,
            drift_world=(0.01, 0.0, 0.0),
            reconstructed_drift_world=(0.01, 0.0, 0.0),
            residual_m=0.001,
            condition=2.0,
            reason="accepted",
        )
        updated = svc._grasp_clip_sag_update(base, base, estimate, max_step_deg=2.0)
        self.assertAlmostEqual(float(updated["seg1_equal_offset_deg"]), 2.0, places=3)
        self.assertAlmostEqual(float(updated["seg2_equal_offset_deg"]), -2.0, places=3)

    def test_waypoint_step_capped_by_blind_margin(self) -> None:
        tip = (0.10, 0.0, 0.90)
        nominal = (0.16, 0.0, 0.90)
        blind_start = 0.06
        step_m = 0.03
        dist = ControlService._grasp_axial_distance(tip, nominal, (1.0, 0.0, 0.0))
        margin = max(0.0, dist - blind_start)
        travel = min(step_m, margin)
        self.assertAlmostEqual(dist, 0.06, places=4)
        self.assertAlmostEqual(travel, 0.0, places=4)

    def test_visual_recover_disabled_in_mock_mode(self) -> None:
        from engine.config_loader import PerceptionConfig

        svc = ControlService(PanelState())
        svc._pick_cfg = PickConfig(grasp_skip_aim_recover_in_mock=True)
        svc._perception_cfg = PerceptionConfig(mode="mock")
        svc._perception_capture = MagicMock()
        svc._perception_capture.is_running.return_value = True
        self.assertFalse(svc._grasp_visual_recover_supported())

    def test_visual_recover_disabled_in_sim(self) -> None:
        from engine.config_loader import PerceptionConfig

        svc = ControlService(PanelState())
        svc._pick_cfg = PickConfig(grasp_skip_aim_recover_in_mock=True)
        svc._perception_cfg = PerceptionConfig(mode="camera")
        svc._use_hardware = False
        svc._perception_capture = MagicMock()
        svc._perception_capture.is_running.return_value = True
        self.assertFalse(svc._grasp_visual_recover_supported())

    def test_aim_latched_direction_prefers_resolved(self) -> None:
        svc = ControlService(PanelState())
        svc._pick_resolved_ready_dir_world = (0.0, 0.0, 1.0)
        svc._pick_look_dir_world = (1.0, 0.0, 0.0)
        got = svc._grasp_aim_latched_direction()
        self.assertEqual(got, (0.0, 0.0, 1.0))

    def test_start_grasp_guided_when_enabled(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc._pick_cfg = PickConfig(grasp_guided_enabled=True)
        svc._pick_look_object_world_xyz = (0.33, 0.01, 0.92)
        svc._pick_look_dir_world = (1.0, 0.0, 0.0)
        with patch.object(svc, "_start_grasp_guided_approach", return_value=True) as mock_guided:
            ok = svc._start_grasp_to_object(internal=True)
        self.assertTrue(ok)
        mock_guided.assert_called_once()

    def test_guided_worker_move_aim_sag_order(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc._pick_cfg = PickConfig(
            grasp_guided_enabled=True,
            grasp_waypoint_step_m=0.03,
            grasp_blind_start_m=0.06,
            grasp_blind_approach_m=0.02,
            grasp_max_waypoints=2,
            grasp_standoff_m=0.02,
        )
        svc._grasp_nominal_dir = (1.0, 0.0, 0.0)
        svc._pick_centered_ready_pose_world_xyz = (0.10, 0.0, 0.90)
        svc._perception_capture = MagicMock()
        svc._perception_capture.is_running.return_value = True

        sample_wp = GraspWaypoint(
            position_world=(0.13, 0.0, 0.90),
            direction_world=(1.0, 0.0, 0.0),
            standoff_m=0.27,
        )
        call_order: list[str] = []

        def _ik(**_kwargs):
            call_order.append("ik")
            return True, np.zeros(4), None, 0.0

        def _aim(**_kwargs):
            call_order.append("aim")
            return True, MagicMock(center_uv=(0.0, 0.0), scale=0.1), None

        def _sag(**_kwargs):
            call_order.append("sag")
            return 0.0, 0.0

        def _stop():
            call_order.append("stop")

        svc.stop_perception_capture = _stop  # type: ignore[method-assign]

        with patch.object(
            svc,
            "_pick_current_tip_world",
            side_effect=[
                (0.10, 0.0, 0.90),
                (0.13, 0.0, 0.90),
                (0.15, 0.0, 0.90),
                (0.05, 0.0, 0.90),
            ],
        ), patch.object(
            svc,
            "_pick_grasp_object_world",
            return_value=(0.33, 0.01, 0.92),
        ), patch(
            "engine.controller.actions.plan_grasp_feasible_next_waypoint",
            side_effect=[sample_wp, None],
        ), patch(
            "engine.controller.actions.plan_grasp_feasible_trajectory",
            return_value=[sample_wp],
        ), patch(
            "engine.controller.actions.plan_grasp_approach_trajectory",
            return_value=[sample_wp],
        ), patch.object(
            svc, "_grasp_feasible_plan_callbacks", return_value=(MagicMock(), MagicMock())
        ), patch.object(
            svc, "_grasp_visual_recover_supported", return_value=True
        ), patch.object(svc, "_grasp_ik_to_waypoint", side_effect=_ik), patch.object(
            svc, "_grasp_aim_recover_after_move", side_effect=_aim
        ), patch.object(svc, "_grasp_update_online_sag_bias", side_effect=_sag), patch.object(
            svc,
            "_grasp_align_to_approach_dir",
        ) as mock_align, patch.object(
            svc,
            "_grasp_blind_final_approach",
            return_value=(True, np.zeros(4), None, (0.31, 0.01, 0.92)),
        ), patch.object(svc, "send_grasp_meta"), patch.object(
            svc, "_send_grasp_target_markers"
        ), patch.object(
            svc,
            "_close_gripper_after_grasp_arrival",
            return_value=(True, "claw closed"),
        ):
            svc._run_grasp_guided_approach_worker(
                object_world=(0.33, 0.01, 0.92),
                approach_dir=np.array([1.0, 0.0, 0.0]),
            )

        mock_align.assert_not_called()
        self.assertEqual(call_order[:3], ["ik", "aim", "sag"])
        self.assertIn("stop", call_order)
        self.assertEqual(str(svc.state.pick_phase), "done")


if __name__ == "__main__":
    unittest.main()
