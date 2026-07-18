from __future__ import annotations

import unittest

from elesim_controller.config import PickConfig
from elesim_controller.vision.pick.core import evaluate_pick_convergence
from elesim_controller.vision.perception.observation import VisualObservation


class PickConvergenceTests(unittest.TestCase):
    def test_reports_uv_delta_not_raw_uv(self) -> None:
        cfg = PickConfig(
            target_uv_u=0.5,
            target_uv_v=0.0,
            center_tol=0.1,
            target_scale=0.16,
            scale_tol=0.02,
        )
        obs = VisualObservation(
            label="ball",
            confidence=0.9,
            center_uv=(0.62, -0.08),
            scale=0.10,
            timestamp_s=1.0,
        )

        conv = evaluate_pick_convergence(obs, cfg=cfg)

        self.assertAlmostEqual(conv.u_err, 0.12, places=6)
        self.assertAlmostEqual(conv.v_err, -0.08, places=6)
        self.assertFalse(conv.center_ok)
        self.assertFalse(conv.scale_ok)


if __name__ == "__main__":
    unittest.main()
