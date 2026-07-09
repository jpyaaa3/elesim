from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "host.py").exists())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.core.config_loader import IkConfig, PickConfig
from engine.behaviors.pick.actions import ControlService
from engine.vision.perception.capture import PerceptionSnapshot
from engine.behaviors.pick.state import HostState, PanelState
from engine.core.protocol import SimQ
from engine.vision.visual_servoing.equal_sag_probe import EqualSagEstimate
from engine.vision.visual_servoing.grasp_trajectory import GraspWaypoint


class TestGraspGuidedHelpers(unittest.TestCase):
    def _host_state(
        self,
        *,
        q: SimQ,
        sim_q: SimQ | None = None,
    ) -> HostState:
        return HostState(
            connected=True,
            tx_seq=1,
            rx_age_s=0.0,
            device="test",
            ports=(),
            torque_enabled=False,
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
            q=q,
            u=None,
            sim_q=sim_q,
        )

    def test_motion_feedback_prefers_sim_q_over_command_q(self) -> None:
        svc = ControlService(PanelState())
        host = self._host_state(
            q=SimQ(linear_m=0.20, roll_rad=0.30, theta1_rad=0.40, theta2_rad=0.50),
            sim_q=SimQ(linear_m=0.01, roll_rad=0.02, theta1_rad=0.03, theta2_rad=0.04),
        )
        got = svc._q_array_for_motion_feedback(host)
        np.testing.assert_allclose(got, np.array([0.01, 0.02, 0.03, 0.04]))

    def test_lji_motion_fraction_wait_uses_sim_q(self) -> None:
        svc = ControlService(PanelState(), use_hardware=False)
        svc.client = MagicMock()
        svc.client.refresh_state.return_value = self._host_state(
            q=SimQ(linear_m=0.10, roll_rad=0.0, theta1_rad=0.0, theta2_rad=0.0),
            sim_q=SimQ(linear_m=0.004, roll_rad=0.0, theta1_rad=0.0, theta2_rad=0.0),
        )
        with patch("engine.behaviors.pick.actions.time.sleep"):
            out = svc._grasp_lji_wait_motion_fraction(
                q_before=np.zeros(4),
                dq_cmd=np.array([0.01, 0.0, 0.0, 0.0]),
                timeout_s=0.05,
                min_frac=0.30,
            )
        self.assertIsNotNone(out)

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

    def test_grasp_filtered_object_ema(self) -> None:
        svc = ControlService(PanelState())
        svc._pick_cfg = PickConfig(
            grasp_object_filter_alpha=0.5,
            grasp_approach_filter_alpha=0.0,
        )
        svc._grasp_init_filtered_tracking((1.0, 0.0, 1.0), (1.0, 0.0, 0.0))
        svc._perception_capture = MagicMock()
        svc._perception_capture.snapshot.return_value = PerceptionSnapshot(
            running=True,
            failed=False,
            status_msg="ok",
            frame_idx=1,
            label="obj",
            confidence=0.9,
            p_camera=(0.0, 0.0, 0.5),
            p_world=(0.0, 0.0, 1.0),
            last_update_s=0.0,
            depth_valid=True,
        )
        obj, _ = svc._grasp_update_filtered_tracking(
            tip_world=(0.5, 0.0, 0.5),
            pk=svc._pick_cfg,
        )
        self.assertAlmostEqual(obj[0], 0.5, places=4)
        self.assertAlmostEqual(obj[2], 1.0, places=4)

    def test_grasp_uv_only_publish_skips_depth(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc.client.send_perception_observation.return_value = (0.33, 0.01, 0.92)
        svc._grasp_uv_only_mode = True
        svc._publish_perception_to_host(
            object_camera_xyz=(0.0, 0.0, 0.5),
            label="obj",
            confidence=0.9,
            image_center_uv=(0.0, 0.0),
            image_scale=0.2,
            depth_valid=True,
        )
        kwargs = svc.client.send_perception_observation.call_args.kwargs
        self.assertFalse(bool(kwargs["depth_valid"]))

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
        from engine.core.config_loader import PerceptionConfig

        svc = ControlService(PanelState())
        svc._pick_cfg = PickConfig(grasp_skip_aim_recover_in_mock=True)
        svc._perception_cfg = PerceptionConfig(mode="mock")
        svc._perception_capture = MagicMock()
        svc._perception_capture.is_running.return_value = True
        self.assertFalse(svc._grasp_visual_recover_supported())

    def test_visual_recover_disabled_in_sim(self) -> None:
        from engine.core.config_loader import PerceptionConfig

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
        ) as mock_q, patch("engine.behaviors.pick.actions.time.sleep") as mock_sleep:
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
            local_img_jacobian_enabled=False,
            grasp_waypoint_step_m=0.03,
            grasp_blind_start_m=0.06,
            grasp_blind_approach_m=0.02,
            grasp_max_waypoints=2,
            grasp_standoff_m=0.02,
        )
        svc._grasp_nominal_dir = (1.0, 0.0, 0.0)
        svc._grasp_traj_start = (0.10, 0.0, 0.90)
        svc._grasp_look_anchor = (0.03, 0.0, 0.90)
        svc._grasp_init_filtered_tracking((0.33, 0.01, 0.92), (1.0, 0.0, 0.0))
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

        def _stop(*_args, **_kwargs):
            call_order.append("stop")

        svc.stop_perception_capture = _stop  # type: ignore[method-assign]

        tip_iter = iter(
            [
                (0.10, 0.0, 0.90),
                (0.13, 0.0, 0.90),
                (0.15, 0.0, 0.90),
            ]
        )

        def _tip_world(**_kwargs):
            try:
                return next(tip_iter)
            except StopIteration:
                return (0.15, 0.0, 0.90)

        with patch.object(
            svc,
            "_pick_current_tip_world",
            side_effect=_tip_world,
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
            svc,
            "_grasp_wait_waypoint_settle",
            side_effect=lambda **kw: kw.get("host_state") or MagicMock(),
        ), patch.object(
            svc, "_grasp_blind_final_approach",
            return_value=(True, np.zeros(4), None, (0.31, 0.0, 0.90)),
        ), patch.object(
            svc,
            "perception_snapshot",
            return_value=PerceptionSnapshot(
                running=True,
                failed=False,
                status_msg="ok",
                frame_idx=1,
                label="obj",
                confidence=0.9,
                p_camera=(0.0, 0.0, 0.5),
                p_world=(0.33, 0.01, 0.92),
                last_update_s=0.0,
                depth_valid=True,
            ),
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

    def test_guided_stop_keeps_perception_running(self) -> None:
        svc = ControlService(PanelState())
        svc._ik_cfg = IkConfig(tol=0.001)
        svc._pick_cfg = PickConfig(
            grasp_guided_enabled=True,
            local_img_jacobian_enabled=False,
        )
        svc._grasp_traj_start = (0.10, 0.0, 0.90)
        svc._grasp_look_anchor = (0.03, 0.0, 0.90)
        svc._perception_capture = MagicMock()
        svc._perception_capture.is_running.return_value = True
        svc._pick_stop_event.set()
        with patch.object(
            svc, "_pick_current_tip_world", return_value=(0.10, 0.0, 0.90)
        ), patch.object(
            svc,
            "_grasp_update_filtered_tracking",
            return_value=((0.33, 0.01, 0.92), (1.0, 0.0, 0.0)),
        ), patch.object(svc, "stop_perception_capture") as mock_stop:
            svc._run_grasp_guided_approach_worker(
                object_world=(0.33, 0.01, 0.92),
                approach_dir=np.array([1.0, 0.0, 0.0]),
                nominal_world=(0.31, 0.0, 0.90),
            )
        mock_stop.assert_not_called()
        self.assertFalse(bool(svc.state.pick_running))
        self.assertFalse(bool(svc.state.pick_failed))

    def test_lji_stop_keeps_perception_running(self) -> None:
        svc = ControlService(PanelState())
        svc._ik_cfg = IkConfig(tol=0.001)
        svc._pick_cfg = PickConfig(
            grasp_guided_enabled=True,
            local_img_jacobian_enabled=True,
        )
        svc._grasp_traj_start = (0.10, 0.0, 0.90)
        svc._grasp_look_anchor = (0.03, 0.0, 0.90)
        svc._grasp_init_lji_controller(svc._pick_cfg)
        svc._perception_capture = MagicMock()
        svc._perception_capture.is_running.return_value = True
        svc._pick_stop_event.set()
        with patch.object(svc, "stop_perception_capture") as mock_stop:
            svc._run_grasp_guided_approach_worker(
                object_world=(0.33, 0.01, 0.92),
                approach_dir=np.array([1.0, 0.0, 0.0]),
                nominal_world=(0.31, 0.0, 0.90),
            )
        mock_stop.assert_not_called()
        self.assertFalse(bool(svc.state.pick_running))
        self.assertFalse(bool(svc.state.pick_failed))

    def test_blind_final_one_shot_with_latched_look(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc._ik_cfg = IkConfig(tol=0.001)
        object_world = (0.33, 0.0, 0.90)
        look_dir = (1.0, 0.0, 0.0)
        advance_labels: list[str] = []

        def _advance(*args, **kwargs):
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

    def test_lji_far_step_uses_seg_joints(self) -> None:
        svc = ControlService(PanelState())
        pk = PickConfig(
            lij_uv_handoff_m=0.10,
            lij_far_linear_cap_m=0.01,
            lij_far_z_gain=0.2,
            lij_gain_z=0.45,
            lij_z_bend_gain=0.2,
        )
        svc._grasp_init_lji_controller(pk)
        servo = svc._grasp_lji_servo_3d
        assert servo is not None
        s = np.array([0.05, -0.08, 0.35], dtype=float)
        q = np.array([0.0, 0.0, -0.1, 0.1], dtype=float)
        approach = np.array([1.0, 0.0, 0.0], dtype=float)
        with patch.object(svc, "_pick_reach_model") as mock_model:
            mock_model.return_value.position_jacobian.return_value = np.array(
                [
                    [0.9, 0.0, 0.05, 0.08],
                    [0.0, 0.2, 0.3, 0.4],
                    [0.0, 0.1, -0.2, 0.3],
                ],
                dtype=float,
            )
            dq, _, _, _, _, avail, tag = svc._grasp_lji_compute_step_dq(
                servo,
                s,
                q=q,
                approach_dir=approach,
                sag_model={},
                remain_m=0.35,
                pk=pk,
                close_tol_m=0.003,
            )
        self.assertEqual(tag, "local_img_jacobian")
        self.assertTrue(avail)
        self.assertGreater(float(dq[0]), 0.002)
        self.assertNotEqual(float(dq[2]), 0.0)
        self.assertNotEqual(float(dq[3]), 0.0)

    def test_lji_centered_uv_uses_null_space_approach_bias(self) -> None:
        svc = ControlService(PanelState())
        s = np.array([0.0, 0.0, 0.35], dtype=float)
        q = np.array([0.0, 0.0, -0.1, 0.1], dtype=float)
        approach = np.array([1.0, 0.0, 0.0], dtype=float)
        position_j = np.array(
            [
                [1.0, 0.0, 0.02, 0.02],
                [0.0, 0.2, 0.3, 0.4],
                [0.0, 0.1, -0.2, 0.3],
            ],
            dtype=float,
        )

        def compute(pk: PickConfig) -> tuple[np.ndarray, np.ndarray]:
            svc._grasp_init_lji_controller(pk)
            servo = svc._grasp_lji_servo_3d
            assert servo is not None
            with patch.object(svc, "_pick_reach_model") as mock_model:
                mock_model.return_value.position_jacobian.return_value = position_j
                dq, _, _, _, _, avail, _ = svc._grasp_lji_compute_step_dq(
                    servo,
                    s,
                    q=q,
                    approach_dir=approach,
                    sag_model={},
                    remain_m=0.35,
                    pk=pk,
                    close_tol_m=0.003,
                )
            self.assertTrue(avail)
            z_row = -position_j[0, :]
            return dq, z_row

        base_pk = PickConfig(
            lij_uv_handoff_m=0.10,
            lij_far_linear_cap_m=0.01,
            lij_far_z_gain=0.2,
            lij_max_dq_theta1=0.02,
            lij_max_dq_angle=0.02,
            lij_approach_bias_gain=0.0,
            lij_approach_seed_q_delta=(0.0, 0.0, 0.02, 0.02),
        )
        bias_pk = PickConfig(
            lij_uv_handoff_m=0.10,
            lij_far_linear_cap_m=0.01,
            lij_far_z_gain=0.2,
            lij_max_dq_theta1=0.02,
            lij_max_dq_angle=0.02,
            lij_approach_bias_gain=1.0,
            lij_approach_seed_q_delta=(0.0, 0.0, 0.02, 0.02),
        )
        dq_base, z_row = compute(base_pk)
        dq_bias, _ = compute(bias_pk)
        base_seg = float(np.linalg.norm(dq_base[2:4]))
        bias_seg = float(np.linalg.norm(dq_bias[2:4]))
        self.assertGreater(bias_seg, base_seg + 1e-4)
        self.assertLess(float(np.dot(z_row, dq_bias)), 0.0)

    def test_lji_command_horizon_leads_target_without_changing_controller_step(self) -> None:
        svc = ControlService(PanelState())
        pk = PickConfig(lij_command_horizon=3.0)
        dq = np.array([0.0, 0.002, 0.003, -0.004], dtype=float)
        got = svc._grasp_lji_apply_command_horizon(
            dq,
            q_before=np.zeros(4, dtype=float),
            pk=pk,
        )
        np.testing.assert_allclose(got, 3.0 * dq)

    def test_lji_command_horizon_clamps_to_joint_limits(self) -> None:
        svc = ControlService(PanelState())
        pk = PickConfig(lij_command_horizon=8.0)
        seg1_max = float(svc.control_mapping().seg1_q_max_rad)
        q_before = np.array([0.0, 0.0, seg1_max - 0.002, 0.0], dtype=float)
        got = svc._grasp_lji_apply_command_horizon(
            np.array([0.0, 0.0, 0.004, 0.0], dtype=float),
            q_before=q_before,
            pk=pk,
        )
        self.assertAlmostEqual(float(q_before[2] + got[2]), seg1_max)

    def test_lji_worker_does_not_update_online_sag(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc._ik_cfg = IkConfig(tol=0.001)
        svc._pick_cfg = PickConfig(
            grasp_guided_enabled=True,
            local_img_jacobian_enabled=True,
            grasp_max_waypoints=1,
            grasp_close_tol_m=0.003,
            lij_min_samples=1,
            lij_condition_max=1000.0,
        )
        svc._grasp_traj_start = (0.10, 0.0, 0.90)
        svc._grasp_look_anchor = (0.03, 0.0, 0.90)
        svc._grasp_init_filtered_tracking((0.33, 0.01, 0.92), (1.0, 0.0, 0.0))
        svc._grasp_init_lji_controller(svc._pick_cfg)
        svc._perception_capture = MagicMock()
        svc._perception_capture.is_running.return_value = True
        sag_calls: list[str] = []

        def _sag(**kwargs):
            sag_calls.append(str(kwargs.get("label", "")))
            return (0.0, 0.0)

        with patch.object(
            svc, "_pick_current_tip_world", return_value=(0.10, 0.0, 0.90)
        ), patch.object(
            svc, "_grasp_visual_recover_supported", return_value=False
        ), patch.object(
            svc, "_grasp_lji_fk_z_row", return_value=np.array([1.0, 0.0, 0.0, 0.0])
        ), patch.object(
            svc, "_grasp_apply_q_delta", return_value=(np.zeros(4), MagicMock())
        ), patch.object(
            svc,
            "current_visual_observation",
            return_value=MagicMock(center_uv=(0.0, 0.0), scale=0.1),
        ), patch.object(
            svc,
            "perception_snapshot",
            return_value=PerceptionSnapshot(
                running=True,
                failed=False,
                status_msg="ok",
                frame_idx=1,
                label="obj",
                confidence=0.9,
                p_camera=(0.0, 0.0, 0.5),
                p_world=(0.33, 0.0, 0.90),
                last_update_s=0.0,
                depth_valid=True,
            ),
        ), patch.object(
            svc,
            "_grasp_update_filtered_tracking",
            return_value=((0.33, 0.01, 0.92), (1.0, 0.0, 0.0)),
        ), patch.object(
            svc, "_grasp_update_online_sag_bias", side_effect=_sag
        ), patch.object(
            svc, "_grasp_wait_waypoint_settle", side_effect=lambda **kw: kw.get("host_state")
        ), patch.object(
            svc, "_grasp_complete_precontact_and_close", return_value=True
        ), patch.object(svc, "stop_perception_capture"):
            svc._run_grasp_guided_approach_worker(
                object_world=(0.33, 0.01, 0.92),
                approach_dir=np.array([1.0, 0.0, 0.0]),
                nominal_world=(0.31, 0.0, 0.90),
            )
        self.assertEqual(sag_calls, [])

    def test_lji_worker_skips_legacy_axial_ik(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc._ik_cfg = IkConfig(tol=0.001)
        svc._pick_cfg = PickConfig(
            grasp_guided_enabled=True,
            local_img_jacobian_enabled=True,
            grasp_max_waypoints=3,
            grasp_standoff_m=0.02,
            blind_micro_start_m=0.06,
            grasp_close_tol_m=0.003,
            lij_min_samples=1,
            lij_condition_max=1000.0,
        )
        svc._grasp_traj_start = (0.10, 0.0, 0.90)
        svc._grasp_look_anchor = (0.03, 0.0, 0.90)
        svc._grasp_init_filtered_tracking((0.33, 0.01, 0.92), (1.0, 0.0, 0.0))
        svc._grasp_init_lji_controller(svc._pick_cfg)
        svc._perception_capture = MagicMock()
        svc._perception_capture.is_running.return_value = True
        calls: list[str] = []
        remain_seq = iter([0.28, 0.28, 0.055, 0.055])

        def _advance(**_kwargs):
            calls.append("axial_ik")
            return True, np.zeros(4), None

        def _apply(dq, **kwargs):
            calls.append("lji_apply")
            return np.zeros(4), MagicMock()

        def _remain(*_args, **_kwargs):
            return float(next(remain_seq, 0.055))

        with patch.object(
            svc,
            "_pick_current_tip_world",
            return_value=(0.10, 0.0, 0.90),
        ), patch.object(
            svc, "_grasp_axial_distance", side_effect=_remain
        ), patch.object(
            svc, "_grasp_visual_recover_supported", return_value=False
        ), patch.object(
            svc, "_grasp_lji_fk_z_row", return_value=np.array([1.0, 0.0, 0.0, 0.0])
        ), patch.object(
            svc, "_grasp_advance_waypoint_ik", side_effect=_advance
        ), patch.object(
            svc, "_grasp_blind_final_approach",
            return_value=(True, np.zeros(4), None, (0.31, 0.0, 0.90)),
        ), patch.object(
            svc, "_grasp_apply_q_delta", side_effect=_apply
        ), patch.object(
            svc,
            "current_visual_observation",
            return_value=MagicMock(center_uv=(0.0, 0.0), scale=0.1),
        ), patch.object(
            svc,
            "perception_snapshot",
            return_value=PerceptionSnapshot(
                running=True,
                failed=False,
                status_msg="ok",
                frame_idx=1,
                label="obj",
                confidence=0.9,
                p_camera=(0.0, 0.0, 0.5),
                p_world=(0.33, 0.0, 0.90),
                last_update_s=0.0,
                depth_valid=True,
            ),
        ), patch.object(
            svc,
            "_grasp_update_filtered_tracking",
            return_value=((0.33, 0.01, 0.92), (1.0, 0.0, 0.0)),
        ), patch.object(
            svc, "_grasp_update_online_sag_bias", return_value=(0.0, 0.0)
        ), patch.object(
            svc, "_grasp_wait_waypoint_settle", side_effect=lambda **kw: kw.get("host_state")
        ), patch.object(
            svc, "_grasp_complete_precontact_and_close", return_value=True
        ), patch.object(svc, "stop_perception_capture"):
            svc._run_grasp_guided_approach_worker(
                object_world=(0.33, 0.01, 0.92),
                approach_dir=np.array([1.0, 0.0, 0.0]),
                nominal_world=(0.31, 0.0, 0.90),
            )
        self.assertIn("lji_apply", calls)
        self.assertNotIn("axial_ik", calls)

    def test_lji_remain_within_blind_threshold_triggers_blind_finish(self) -> None:
        svc = ControlService(PanelState())
        pk = PickConfig(
            grasp_guided_enabled=True,
            local_img_jacobian_enabled=True,
            grasp_max_waypoints=3,
            grasp_standoff_m=0.02,
            grasp_close_tol_m=0.003,
            blind_micro_start_m=0.06,
            lij_min_samples=1,
            lij_condition_max=1000.0,
        )
        svc._pick_cfg = pk
        svc._ik_cfg = IkConfig(tol=0.001)
        svc._grasp_traj_start = (0.10, 0.0, 0.90)
        svc._grasp_look_anchor = (0.03, 0.0, 0.90)
        svc._grasp_init_filtered_tracking((0.33, 0.01, 0.92), (1.0, 0.0, 0.0))
        svc._grasp_init_lji_controller(pk)
        svc._grasp_lji_latch_reliable_state(
            object_world=(0.33, 0.01, 0.92),
            approach_dir=np.array([1.0, 0.0, 0.0]),
            remain_m=0.055,
            host_state=None,
        )
        svc._perception_capture = MagicMock()
        svc._perception_capture.is_running.return_value = True
        calls: list[str] = []

        def _blind(**_kwargs):
            calls.append("blind_finish")
            return True, np.zeros(4), MagicMock(), (0.31, 0.0, 0.90)

        with patch.object(
            svc, "_pick_current_tip_world", return_value=(0.30, 0.0, 0.90)
        ), patch.object(
            svc, "_grasp_axial_distance", return_value=0.055
        ), patch.object(
            svc, "_grasp_visual_recover_supported", return_value=False
        ), patch.object(
            svc, "_grasp_lji_fk_z_row", return_value=np.array([1.0, 0.0, 0.0, 0.0])
        ), patch.object(
            svc, "_grasp_blind_final_approach", side_effect=_blind
        ), patch.object(
            svc, "_grasp_apply_q_delta",
            return_value=(np.zeros(4), MagicMock()),
        ), patch.object(
            svc,
            "current_visual_observation",
            return_value=MagicMock(center_uv=(0.0, 0.0), scale=0.1),
        ), patch.object(
            svc,
            "perception_snapshot",
            return_value=PerceptionSnapshot(
                running=True,
                failed=False,
                status_msg="ok",
                frame_idx=1,
                label="obj",
                confidence=0.9,
                p_camera=(0.0, 0.0, 0.5),
                p_world=(0.33, 0.0, 0.90),
                last_update_s=0.0,
                depth_valid=True,
            ),
        ), patch.object(
            svc,
            "_grasp_update_filtered_tracking",
            return_value=((0.33, 0.01, 0.92), (1.0, 0.0, 0.0)),
        ), patch.object(
            svc, "_grasp_update_online_sag_bias", return_value=(0.0, 0.0)
        ), patch.object(
            svc, "_grasp_wait_waypoint_settle", side_effect=lambda **kw: kw.get("host_state")
        ), patch.object(
            svc, "_grasp_complete_precontact_and_close", return_value=True
        ), patch.object(svc, "stop_perception_capture"):
            svc._run_grasp_guided_approach_worker(
                object_world=(0.33, 0.01, 0.92),
                approach_dir=np.array([1.0, 0.0, 0.0]),
                nominal_world=(0.31, 0.0, 0.90),
            )
        self.assertIn("blind_finish", calls)

    def test_lji_gain_scale_drops_near_contact(self) -> None:
        svc = ControlService(PanelState())
        pk = PickConfig(
            lij_gain_scale_ref_m=0.30,
            lij_gain_scale_min=0.12,
        )
        far = svc._grasp_lji_gain_scale(0.28, pk, close_tol_m=0.003)
        near = svc._grasp_lji_gain_scale(0.05, pk, close_tol_m=0.003)
        self.assertGreater(far, near)
        self.assertAlmostEqual(far, 1.0, places=2)
        self.assertAlmostEqual(near, 0.12, places=2)

    def test_lji_remain_far_keeps_lji_then_fails_without_blind(self) -> None:
        svc = ControlService(PanelState())
        pk = PickConfig(
            grasp_guided_enabled=True,
            local_img_jacobian_enabled=True,
            grasp_max_waypoints=2,
            grasp_close_tol_m=0.003,
            blind_micro_start_m=0.06,
            lij_min_samples=1,
            lij_uv_handoff_m=0.10,
        )
        svc._pick_cfg = pk
        svc._ik_cfg = IkConfig(tol=0.001)
        svc._grasp_traj_start = (0.10, 0.0, 0.90)
        svc._grasp_init_filtered_tracking((0.33, 0.01, 0.92), (1.0, 0.0, 0.0))
        svc._grasp_init_lji_controller(pk)
        svc._grasp_lji_latch_reliable_state(
            object_world=(0.33, 0.01, 0.92),
            approach_dir=np.array([1.0, 0.0, 0.0]),
            remain_m=0.28,
            host_state=None,
        )
        svc._perception_capture = MagicMock()
        svc._perception_capture.is_running.return_value = True
        calls: list[str] = []

        def _apply(*_args, **_kwargs):
            calls.append("lji_apply")
            return np.zeros(4), MagicMock()

        with patch.object(
            svc, "_pick_current_tip_world", return_value=(0.10, 0.0, 0.90)
        ), patch.object(
            svc, "_grasp_axial_distance", return_value=0.28
        ), patch.object(
            svc, "_grasp_apply_q_delta", side_effect=_apply
        ), patch.object(
            svc, "_grasp_lji_fk_z_row", return_value=np.array([1.0, 0.0, 0.0, 0.0])
        ), patch.object(
            svc,
            "current_visual_observation",
            return_value=MagicMock(center_uv=(0.0, 0.0), scale=0.1),
        ), patch.object(
            svc,
            "perception_snapshot",
            return_value=PerceptionSnapshot(
                running=True,
                failed=False,
                status_msg="ok",
                frame_idx=1,
                label="obj",
                confidence=0.9,
                p_camera=(0.0, 0.0, 0.5),
                p_world=(0.33, 0.0, 0.90),
                last_update_s=0.0,
                depth_valid=True,
            ),
        ), patch.object(
            svc,
            "_grasp_update_filtered_tracking",
            return_value=((0.33, 0.01, 0.92), (1.0, 0.0, 0.0)),
        ), patch.object(
            svc, "_grasp_wait_waypoint_settle", side_effect=lambda **kw: kw.get("host_state")
        ), patch.object(svc, "stop_perception_capture"):
            svc._run_grasp_guided_approach_worker(
                object_world=(0.33, 0.01, 0.92),
                approach_dir=np.array([1.0, 0.0, 0.0]),
                nominal_world=(0.31, 0.0, 0.90),
            )
        self.assertIn("lji_apply", calls)
        self.assertTrue(svc.state.pick_failed)

    def test_lji_depth_z_std_stable_camera_z(self) -> None:
        svc = ControlService(PanelState())
        pk = PickConfig(
            lij_depth_std_max_m=0.012,
            lij_depth_settled_remain_delta_m=0.005,
        )
        for z_cam in (0.850, 0.851, 0.849, 0.850):
            svc._grasp_depth_history.append((True, float(z_cam), 0.30))
        stable, reason = svc._grasp_lji_eval_depth_stability(pk, remain_m=0.30)
        self.assertTrue(stable, msg=reason)
        svc._grasp_depth_history.clear()
        for z_cam in (0.850, 0.870, 0.840, 0.880):
            svc._grasp_depth_history.append((True, float(z_cam), 0.30))
        stable2, reason2 = svc._grasp_lji_eval_depth_stability(pk, remain_m=0.30)
        self.assertFalse(stable2)
        self.assertEqual(reason2, "z_std")

    def test_lji_blind_finish_threshold_is_remain_only(self) -> None:
        svc = ControlService(PanelState())
        pk = PickConfig(blind_micro_start_m=0.06)
        self.assertFalse(svc._grasp_lji_should_blind_finish(0.12, pk))
        self.assertTrue(svc._grasp_lji_should_blind_finish(0.06, pk))
        self.assertTrue(svc._grasp_lji_should_blind_finish(0.049, pk))

    def test_lji_smooth_dq_blends_with_previous(self) -> None:
        svc = ControlService(PanelState())
        pk = PickConfig(lij_dq_smooth_alpha=0.5)
        svc._grasp_lji_last_dq_cmd = np.array([1.0, 0.0, 0.0, 0.0])
        out = svc._grasp_lji_smooth_dq(np.array([0.0, 1.0, 0.0, 0.0]), pk=pk)
        self.assertAlmostEqual(float(out[0]), 0.5, places=4)
        self.assertAlmostEqual(float(out[1]), 0.5, places=4)

    def test_lji_axial_retract_uses_negative_distance(self) -> None:
        svc = ControlService(PanelState())
        pk = PickConfig(lij_reacquire_axial_step_m=0.012)
        q0 = np.array([0.1, 0.2, 0.3, 0.4])
        calls: list[float] = []

        def _axial(**kwargs):
            calls.append(float(kwargs["distance_m"]))
            return True, q0 - np.array([0.012, 0.0, 0.0, 0.0])

        with patch.object(svc, "_grasp_solve_axial_ik_q", side_effect=_axial):
            dq = svc._grasp_lji_compute_axial_retract_dq(
                pk=pk,
                approach_dir=np.array([1.0, 0.0, 0.0]),
                object_world=(0.33, 0.0, 0.92),
                sag_model={},
                host_state=MagicMock(),
                q_before=q0,
            )
        self.assertEqual(len(calls), 1)
        self.assertLess(float(calls[0]), 0.0)
        self.assertIsNotNone(dq)

    def test_lji_retract_dq_to_last_good_q(self) -> None:
        svc = ControlService(PanelState())
        pk = PickConfig(lij_reacquire_axial_step_m=0.012)
        q_before = np.array([0.10, 0.20, 0.30, 0.40])
        svc._grasp_lji_last_good_q = np.array([0.04, 0.18, 0.28, 0.38])
        dq = svc._grasp_lji_retract_dq_to_last_good_q(q_before=q_before, pk=pk)
        self.assertIsNotNone(dq)
        assert dq is not None
        self.assertLess(float(dq[0]), 0.0)

    def test_lji_should_reacquire_only_on_object_lost(self) -> None:
        svc = ControlService(PanelState())
        pk = PickConfig(lij_reacquire_max_steps=8)
        self.assertFalse(
            svc._grasp_lji_should_reacquire(
                object_lost=False,
                remain_m=0.20,
                close_tol_m=0.003,
                pk=pk,
            )
        )
        self.assertTrue(
            svc._grasp_lji_should_reacquire(
                object_lost=True,
                remain_m=0.20,
                close_tol_m=0.003,
                pk=pk,
            )
        )

    def test_lji_visual_tracking_lost_on_v_divergence(self) -> None:
        svc = ControlService(PanelState())
        pk = PickConfig(lij_reacquire_v_err_m=0.45)
        svc._grasp_lji_v_err_hist = [0.20, 0.28, 0.36, 0.44]
        s = np.array([0.05, -0.52, 0.15], dtype=float)
        self.assertTrue(svc._grasp_lji_visual_tracking_lost(s, pk=pk))

    def test_lji_object_lost_triggers_reacquire_reverse_dq(self) -> None:
        svc = ControlService(PanelState())
        pk = PickConfig(
            grasp_guided_enabled=True,
            local_img_jacobian_enabled=True,
            grasp_max_waypoints=4,
            grasp_standoff_m=0.02,
            grasp_close_tol_m=0.003,
            lij_min_samples=1,
            lij_condition_max=1000.0,
            lij_reacquire_max_steps=3,
            lij_reacquire_retrace_gain=1.0,
        )
        svc._pick_cfg = pk
        svc.client = MagicMock()
        svc._ik_cfg = IkConfig(tol=0.001)
        svc._grasp_traj_start = (0.10, 0.0, 0.90)
        svc._grasp_look_anchor = (0.03, 0.0, 0.90)
        svc._grasp_init_filtered_tracking((0.33, 0.01, 0.92), (1.0, 0.0, 0.0))
        svc._grasp_init_lji_controller(pk)
        svc._perception_capture = MagicMock()
        svc._perception_capture.is_running.return_value = True
        applied: list[np.ndarray] = []
        obs_calls = {"n": 0}

        def _apply(dq, **_kwargs):
            arr = np.asarray(dq, dtype=float).reshape(4).copy()
            applied.append(arr)
            return np.zeros(4) + arr, MagicMock()

        def _obs(*_args, **_kwargs):
            obs_calls["n"] += 1
            if obs_calls["n"] == 1:
                return MagicMock(center_uv=(0.0, 0.0), scale=0.1)
            return None

        with patch.object(
            svc, "_pick_current_tip_world", return_value=(0.10, 0.0, 0.90)
        ), patch.object(
            svc, "_grasp_axial_distance", return_value=0.15
        ), patch.object(
            svc, "_grasp_visual_recover_supported", return_value=False
        ), patch.object(
            svc, "_grasp_lji_fk_z_row", return_value=np.array([1.0, 0.0, 0.0, 0.0])
        ), patch.object(
            svc, "_grasp_apply_q_delta", side_effect=_apply
        ), patch.object(
            svc, "current_visual_observation", side_effect=_obs
        ), patch.object(
            svc,
            "perception_snapshot",
            return_value=PerceptionSnapshot(
                running=True,
                failed=False,
                status_msg="ok",
                frame_idx=1,
                label="obj",
                confidence=0.9,
                p_camera=(0.0, 0.0, 0.5),
                p_world=(0.33, 0.0, 0.90),
                last_update_s=0.0,
                depth_valid=True,
            ),
        ), patch.object(
            svc,
            "_grasp_update_filtered_tracking",
            return_value=((0.33, 0.01, 0.92), (1.0, 0.0, 0.0)),
        ), patch.object(
            svc, "_grasp_update_online_sag_bias", return_value=(0.0, 0.0)
        ), patch.object(
            svc, "_grasp_wait_waypoint_settle", side_effect=lambda **kw: kw.get("host_state")
        ), patch.object(
            svc, "_grasp_blind_final_approach",
            return_value=(True, np.zeros(4), MagicMock(), (0.31, 0.0, 0.90)),
        ), patch.object(
            svc, "_grasp_complete_precontact_and_close", return_value=True
        ), patch.object(svc, "stop_perception_capture"):
            svc._run_grasp_guided_approach_worker(
                object_world=(0.33, 0.01, 0.92),
                approach_dir=np.array([1.0, 0.0, 0.0]),
                nominal_world=(0.31, 0.0, 0.90),
            )
        self.assertGreaterEqual(len(applied), 1)

    def test_null_space_compose_via_module(self) -> None:
        from engine.vision.visual_servoing.local_image_jacobian import compose_dq_align_and_approach

        j = np.array([[0.0, 2.0, 0.0, 0.0], [0.0, 0.0, 1.0, 1.0]], dtype=float)
        seed = np.array([0.01, 0.02, 0.03, 0.04], dtype=float)
        dq, _, dq_app = compose_dq_align_and_approach(
            j_uv=j,
            s_uv=[0.01, 0.0],
            dq_approach_seed=seed,
            damping=0.05,
            gain_u=0.5,
            gain_v=0.5,
            approach_bias_gain=1.0,
            enable_approach=True,
            max_dq_linear=0.01,
            max_dq_angle=0.05,
        )
        self.assertGreater(float(np.linalg.norm(dq_app)), 0.0)
        from engine.vision.visual_servoing.local_image_jacobian import null_space_projector

        n_proj = null_space_projector(j, damping=0.05)
        self.assertTrue(np.allclose(j @ (n_proj @ seed), np.zeros(2), atol=1e-4))


if __name__ == "__main__":
    unittest.main()
