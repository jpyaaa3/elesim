from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.config_loader import PickConfig
from engine.controller.actions import ControlService
from engine.controller.perception import VisualObservation
from engine.controller.state import PanelState
from engine.protocol import ControlU


def _obs(*, u: float, v: float, scale: float = 0.05) -> VisualObservation:
    return VisualObservation(
        label="ball",
        confidence=0.9,
        center_uv=(float(u), float(v)),
        scale=float(scale),
        timestamp_s=float(time.time()),
    )


class TestAimGainFallback(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
