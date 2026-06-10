from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.config_loader import IkConfig, PickConfig
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
        obj = (0.16, 0.0, 0.90)
        blind_start = 0.06
        step_m = 0.03
        remain = ControlService._grasp_approach_remaining_m(tip, obj, 0.0)
        travel = min(step_m, max(0.0, remain - blind_start))
        self.assertAlmostEqual(remain, 0.06, places=4)
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

    def test_grasp_wait_waypoint_settle_dwells(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        q = np.array([0.1, 0.0, 0.2, 0.1], dtype=float)
        with patch.object(
            svc, "_wait_until_q_settled", return_value=(MagicMock(), True)
        ) as mock_q, patch("engine.controller.actions.time.sleep") as mock_sleep:
            out = svc._grasp_wait_waypoint_settle(
                q_cmd=q,
                host_state=None,
                label="wp 1/5",
                settle_s=0.4,
                settle_timeout_s=3.0,
            )
        self.assertIsNotNone(out)
        mock_q.assert_called()
        mock_sleep.assert_called()

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

    def test_grasp_trajectory_end_is_pre_contact_not_object(self) -> None:
        obj = (0.33, 0.01, 0.92)
        end = ControlService._pick_grasp_trajectory_end_position(
            ControlService(PanelState()),
            obj,
            (1.0, 0.0, 0.0),
            standoff_m=0.02,
        )
        self.assertAlmostEqual(end[0], 0.31, places=3)
        self.assertNotAlmostEqual(end[0], obj[0], places=3)

    def test_grasp_trajectory_start_uses_look_pose(self) -> None:
        svc = ControlService(PanelState())
        svc._pick_look_ready_pose_world_xyz = (0.03, 0.0, 0.90)
        svc._pick_resolved_ready_pose_world_xyz = (0.10, 0.0, 0.90)
        self.assertEqual(svc._pick_grasp_trajectory_start_position(), (0.03, 0.0, 0.90))

    def test_grasp_waypoint_behind_tip_on_approach_axis(self) -> None:
        wp = GraspWaypoint(
            position_world=(0.03, 0.0, 0.90),
            direction_world=(1.0, 0.0, 0.0),
            standoff_m=0.30,
        )
        tip = (0.10, 0.0, 0.90)
        nominal = (0.31, 0.0, 0.90)
        self.assertTrue(
            ControlService._grasp_waypoint_behind_tip(
                wp,
                tip,
                nominal,
                (1.0, 0.0, 0.0),
            )
        )
        wp_ahead = GraspWaypoint(
            position_world=(0.13, 0.0, 0.90),
            direction_world=(1.0, 0.0, 0.0),
            standoff_m=0.18,
        )
        self.assertFalse(
            ControlService._grasp_waypoint_behind_tip(
                wp_ahead,
                tip,
                nominal,
                (1.0, 0.0, 0.0),
            )
        )

    def test_guided_worker_move_aim_sag_ik_order(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc._ik_cfg = IkConfig(tol=0.001)
        svc._pick_cfg = PickConfig(
            grasp_guided_enabled=True,
            grasp_waypoint_step_m=0.03,
            grasp_blind_start_m=0.06,
            grasp_blind_approach_m=0.02,
            grasp_max_waypoints=2,
            grasp_standoff_m=0.02,
        )
        svc._grasp_nominal_dir = (1.0, 0.0, 0.0)
        svc._grasp_traj_start = (0.10, 0.0, 0.90)
        svc._grasp_look_anchor = (0.03, 0.0, 0.90)
        svc._perception_capture = MagicMock()
        svc._perception_capture.is_running.return_value = True
        call_order: list[str] = []

        def _advance(**_kwargs):
            call_order.append("ik")
            return True, np.zeros(4), None

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
                (0.31, 0.0, 0.90),
            ],
        ), patch.object(
            svc,
            "_pick_grasp_object_world",
            return_value=(0.33, 0.01, 0.92),
        ), patch.object(
            svc, "_q_array_from_state", return_value=np.zeros(4)
        ), patch.object(
            svc, "_grasp_visual_recover_supported", return_value=True
        ), patch.object(
            svc, "_grasp_advance_waypoint_ik", side_effect=_advance
        ), patch.object(
            svc, "_grasp_aim_recover_after_move", side_effect=_aim
        ), patch.object(
            svc, "_grasp_update_online_sag_bias", side_effect=_sag
        ), patch.object(
            svc, "_grasp_blind_final_approach",
            return_value=(True, np.zeros(4), None, (0.31, 0.0, 0.90)),
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
                nominal_world=(0.31, 0.0, 0.90),
            )

        self.assertEqual(call_order[:3], ["aim", "sag", "ik"])
        self.assertIn("stop", call_order)
        self.assertEqual(str(svc.state.pick_phase), "done")

    def test_blind_final_one_shot_with_latched_look(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc._ik_cfg = IkConfig(tol=0.001)
        object_world = (0.33, 0.0, 0.90)
        look_dir = (1.0, 0.0, 0.0)
        advance_labels: list[str] = []

        def _advance(**kwargs):
            advance_labels.append(str(kwargs.get("label", "")))
            return True, 0.06, np.array([0.1, 0.0, 0.0, 0.0]), None

        with patch.object(
            svc,
            "_pick_current_tip_world",
            side_effect=[
                (0.25, 0.0, 0.90),
                (0.308, 0.0, 0.90),
            ],
        ), patch.object(
            svc, "_grasp_align_to_approach_dir", return_value=(True, None)
        ), patch.object(
            svc, "_grasp_cartesian_advance_along_dir", side_effect=_advance
        ), patch.object(
            svc,
            "_wait_until_grasp_target_reached",
            return_value=(True, 0.002, MagicMock(reply_ok=True, q=MagicMock())),
        ), patch.object(
            svc, "_q_array_from_state", return_value=np.zeros(4)
        ), patch.object(
            svc,
            "_pick_reach_model",
        ) as mock_model:
            mock_model.return_value.grasp_position.return_value = np.array(
                [0.25, 0.0, 0.90]
            )
            mock_model.return_value.grasp_direction.return_value = np.array(
                [1.0, 0.0, 0.0]
            )
            ok, _, _, target = svc._grasp_blind_final_approach(
                object_world=object_world,
                look_dir=look_dir,
                sag_model={},
                host_state=MagicMock(),
                grasp_standoff_m=0.02,
                approach_dir=np.array([1.0, 0.0, 0.0]),
                nominal_world=(0.31, 0.0, 0.90),
            )
        self.assertTrue(ok)
        self.assertAlmostEqual(target[0], 0.31, places=3)
        self.assertEqual(len(advance_labels), 1)
        self.assertEqual(advance_labels[0], "grasp blind")
        mock_model.return_value.grasp_position.assert_called()


if __name__ == "__main__":
    unittest.main()
