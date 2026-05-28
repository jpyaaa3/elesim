from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from host import PickContext, PickStage, filtered_camera_stats, should_pass_confidence_gate, should_stage_timeout


class TestPickEstimator(unittest.TestCase):
    def test_filtered_camera_stats_rejects_outlier(self) -> None:
        samples = np.array(
            [
                [0.01, -0.01, 0.30],
                [0.02, -0.02, 0.31],
                [0.00, -0.01, 0.29],
                [0.40, 0.35, 1.40],
            ],
            dtype=float,
        )
        mu, cov = filtered_camera_stats(samples, outlier_zscore=1.4)
        self.assertIsNotNone(mu)
        self.assertIsNotNone(cov)
        assert mu is not None
        assert cov is not None
        self.assertLess(float(mu[2]), 0.5)
        self.assertEqual(cov.shape, (3, 3))


class TestPickGate(unittest.TestCase):
    def test_confidence_gate_threshold(self) -> None:
        self.assertTrue(
            should_pass_confidence_gate(
                error_m=0.005,
                uncertainty=0.0005,
                error_threshold_m=0.01,
                uncertainty_threshold=0.001,
            )
        )
        self.assertFalse(
            should_pass_confidence_gate(
                error_m=0.03,
                uncertainty=0.0005,
                error_threshold_m=0.01,
                uncertainty_threshold=0.001,
            )
        )


class TestPickContext(unittest.TestCase):
    def test_context_defaults_stage(self) -> None:
        ctx = PickContext()
        self.assertEqual(ctx.stage, PickStage.SEARCH)
        self.assertIsNone(ctx.object_world_latest)

    def test_stage_timeout_transition_rule(self) -> None:
        self.assertFalse(should_stage_timeout(stage_elapsed_s=1.0, timeout_s=2.0))
        self.assertTrue(should_stage_timeout(stage_elapsed_s=2.1, timeout_s=2.0))


if __name__ == "__main__":
    unittest.main()
