from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "AGENTS.md").exists())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from elesim_controller.config import PickConfig
from elesim_controller.pick.actions import ControlService
from elesim_controller.vision.perception.observation import VisualObservation
from elesim_controller.pick.state import PanelState
from elesim_protocol import ControlU


def _obs(*, u: float, v: float, scale: float = 0.05) -> VisualObservation:
    return VisualObservation(
        label="ball",
        confidence=0.9,
        center_uv=(float(u), float(v)),
        scale=float(scale),
        timestamp_s=float(time.time()),
    )


class TestAimGainFallback(unittest.TestCase):
    def test_aim_config_caps_motion_below_pick_caps(self) -> None:
        svc = ControlService(PanelState())
        svc._pick_cfg = PickConfig(
            target_uv_u=0.0,
            target_uv_v=0.0,
            center_tol=0.16,
            aim_center_tol=0.02,
            center_roll_max=6.0,
            center_seg_max=6.0,
        )

        aim_cfg = svc._pick_config_for_aim()

        self.assertAlmostEqual(float(aim_cfg.center_tol), 0.02)
        self.assertLessEqual(float(aim_cfg.center_roll_max), 3.0)
        self.assertLessEqual(float(aim_cfg.center_seg_max), 3.0)
        self.assertLess(float(svc._pick_aim_step_scale), 1.0)

    def test_large_u_error_uses_roll_not_only_seg(self) -> None:
        svc = ControlService(PanelState())
        cfg = PickConfig(
            target_uv_u=0.4,
            target_uv_v=0.1,
            center_tol=0.04,
            center_u_gain=12.0,
            center_v_gain=12.0,
            center_roll_max=6.0,
            center_seg_max=6.0,
        )
        current = ControlU(u_linear=0.0, u_roll=5.0, u_s1=0.0, u_s2=0.0)
        obs = _obs(u=-0.55, v=-0.09)

        next_u, mode, roll_req, seg_req = svc._apply_pick_center_step(
            obs,
            current,
            cfg=cfg,
            fallback_gains=True,
        )

        self.assertEqual(mode, "gain_fallback")
        self.assertLess(float(roll_req), -0.5)
        self.assertLess(float(next_u.u_roll), float(current.u_roll))


class TestAimCoupledAxes(unittest.TestCase):
    def test_v_only_freezes_roll_and_boosts_seg(self) -> None:
        svc = ControlService(PanelState())
        cfg = PickConfig(
            target_uv_u=0.4,
            target_uv_v=0.0,
            center_tol=0.04,
            center_u_gain=12.0,
            center_v_gain=12.0,
            center_roll_max=6.0,
            center_seg_max=6.0,
        )
        current = ControlU(u_linear=0.0, u_roll=5.0, u_s1=50.0, u_s2=50.0)
        # u within tol, v still out -> seg-only finish (no roll fighting v).
        obs = _obs(u=0.387, v=-0.22)

        next_u, mode, roll_req, seg_req = svc._apply_pick_center_step(
            obs,
            current,
            cfg=cfg,
            fallback_gains=True,
            coupled_axes=True,
        )

        self.assertEqual(mode, "gain_v_only")
        self.assertAlmostEqual(float(roll_req), 0.0, places=6)
        self.assertAlmostEqual(float(next_u.u_roll), float(current.u_roll), places=6)
        self.assertGreaterEqual(float(seg_req), float(svc._pick_aim_v_min_seg_step))
        self.assertLess(float(next_u.u_s1), float(current.u_s1))

    def test_roll_keeps_correcting_when_u_in_tol_but_v_out(self) -> None:
        svc = ControlService(PanelState())
        cfg = PickConfig(
            target_uv_u=0.4,
            target_uv_v=0.0,
            center_tol=0.04,
            center_u_gain=12.0,
            center_v_gain=12.0,
            center_roll_max=6.0,
            center_seg_max=6.0,
        )
        current = ControlU(u_linear=0.0, u_roll=5.0, u_s1=50.0, u_s2=50.0)
        # u within tol -> v-only; roll frozen, seg drives v.
        obs = _obs(u=0.363, v=-0.29)

        next_u, mode, roll_req, seg_req = svc._apply_pick_center_step(
            obs,
            current,
            cfg=cfg,
            fallback_gains=True,
            coupled_axes=True,
        )

        self.assertEqual(mode, "gain_v_only")
        self.assertAlmostEqual(float(roll_req), 0.0, places=6)
        self.assertGreaterEqual(float(seg_req), float(svc._pick_aim_v_min_seg_step))

    def test_v_only_respects_reduced_step_scale(self) -> None:
        svc = ControlService(PanelState())
        cfg = PickConfig(
            target_uv_u=0.4,
            target_uv_v=0.0,
            center_tol=0.04,
            center_u_gain=12.0,
            center_v_gain=12.0,
            center_roll_max=6.0,
            center_seg_max=6.0,
        )
        current = ControlU(u_linear=0.0, u_roll=5.0, u_s1=50.0, u_s2=50.0)
        obs = _obs(u=0.387, v=-0.22)

        next_u, mode, roll_req, seg_req = svc._apply_pick_center_step(
            obs,
            current,
            cfg=cfg,
            fallback_gains=True,
            coupled_axes=True,
            step_scale=0.25,
        )

        self.assertEqual(mode, "gain_v_only")
        self.assertAlmostEqual(float(roll_req), 0.0, places=6)
        self.assertLessEqual(float(seg_req), float(cfg.center_seg_max) * 0.25 + 1e-6)
        self.assertLess(float(abs(next_u.u_s1 - current.u_s1)), 2.0)

    def test_aim_step_cap_tapers_with_remaining_uv_error(self) -> None:
        svc = ControlService(PanelState())
        cfg = PickConfig(
            target_uv_u=0.0,
            target_uv_v=0.0,
            center_tol=0.02,
            center_u_gain=100.0,
            center_v_gain=100.0,
            center_roll_max=3.0,
            center_seg_max=3.0,
        )
        current = ControlU(u_linear=0.0, u_roll=5.0, u_s1=50.0, u_s2=50.0)
        obs = _obs(u=0.175, v=0.0)

        next_u, mode, roll_req, _seg_req = svc._apply_pick_center_step(
            obs,
            current,
            cfg=cfg,
            fallback_gains=True,
            coupled_axes=True,
            step_scale=0.45,
        )

        self.assertEqual(mode, "gain_fallback")
        # err=0.175 is half of taper_ref=0.35, so cap is 3.0 * 0.45 * 0.5.
        self.assertAlmostEqual(float(roll_req), 0.675, places=6)
        self.assertAlmostEqual(float(next_u.u_roll - current.u_roll), 0.675, places=6)


class TestLookPreAimRough(unittest.TestCase):
    def test_uses_fixed_offset_target_without_full_centering(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc.client.refresh_state.return_value = None
        pk = PickConfig(
            look_pre_aim_enabled=True,
            look_pre_aim_max_steps=1,
            look_pre_aim_target_uv_u=0.10,
            look_pre_aim_target_uv_v=0.0,
            look_pre_aim_tol=0.04,
            look_pre_aim_awful_tol=0.45,
            look_pre_aim_step_scale=0.35,
        )
        current = ControlU(u_linear=0.0, u_roll=5.0, u_s1=50.0, u_s2=50.0)
        next_u = ControlU(u_linear=0.0, u_roll=5.1, u_s1=49.9, u_s2=50.1)
        seen_targets: list[tuple[float, float, float]] = []

        def _step(obs, current_u, *, cfg, **_kwargs):
            seen_targets.append(
                (
                    float(cfg.target_uv_u),
                    float(cfg.target_uv_v),
                    float(cfg.center_tol),
                )
            )
            return next_u, "test", 0.0, 0.0

        with patch.object(svc, "_wait_for_track_lock", return_value=True):
            with patch.object(svc, "current_visual_observation", return_value=_obs(u=-0.40, v=0.0)):
                with patch.object(svc, "current_control_u", return_value=current):
                    with patch.object(svc, "_apply_pick_center_step", side_effect=_step):
                        with patch.object(svc, "apply_control_u") as apply_mock:
                            with patch.object(svc, "send_current_target") as send_mock:
                                ok, _, reason = svc._look_pre_aim_rough(pk=pk, host_state=None)

        self.assertTrue(ok, reason)
        self.assertEqual(seen_targets, [(0.10, 0.0, 0.04)])
        apply_mock.assert_called_once()
        send_mock.assert_called_once_with(source="look_pre_aim")


class TestAimProgressStall(unittest.TestCase):
    def test_stuck_counter_resets_on_improvement(self) -> None:
        svc = ControlService(PanelState())
        svc._pick_aim_best_uv_err = 0.60
        svc._pick_aim_stuck_iters = 5
        svc._pick_aim_progress_eps = 0.015

        err_mag = 0.50
        eps = float(svc._pick_aim_progress_eps)
        if err_mag < float(svc._pick_aim_best_uv_err) - eps:
            svc._pick_aim_best_uv_err = float(err_mag)
            svc._pick_aim_stuck_iters = 0

        self.assertEqual(svc._pick_aim_stuck_iters, 0)
        self.assertAlmostEqual(svc._pick_aim_best_uv_err, 0.50, places=6)

    def test_divergence_detection_and_step_scale_reduction(self) -> None:
        svc = ControlService(PanelState())
        svc._reset_pick_aim_progress()
        svc._pick_aim_last_command_err = 0.20

        self.assertFalse(svc._aim_error_diverged(0.22))
        self.assertTrue(svc._aim_error_diverged(0.30))
        self.assertTrue(svc._reduce_aim_step_scale())
        self.assertLess(float(svc._pick_aim_runtime_step_scale), float(svc._pick_aim_step_scale))


if __name__ == "__main__":
    unittest.main()
