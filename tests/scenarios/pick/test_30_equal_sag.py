from __future__ import annotations

import unittest

import numpy as np

from engine.robot.arm.sag_model import segment_errors_from_model
from engine.vision.visual_servoing.equal_sag_probe import solve_equal_sag_offsets


class EqualSagModelTests(unittest.TestCase):
    def test_equal_offset_is_added_to_empty_model(self) -> None:
        err = segment_errors_from_model(
            {"seg1_equal_offset_deg": 2.5},
            seg_index=1,
            count=3,
        )

        np.testing.assert_allclose(err, [2.5, 2.5, 2.5])

    def test_equal_offset_is_added_to_legacy_model(self) -> None:
        err = segment_errors_from_model(
            {
                "seg2_distribution": "1,1",
                "seg2_amplitude": "2",
                "seg2_equal_offset_deg": -0.5,
            },
            seg_index=2,
            count=2,
        )

        np.testing.assert_allclose(err, [1.5, 1.5])


class EqualSagSolverTests(unittest.TestCase):
    def test_recovers_offsets_from_full_rank_sensitivity(self) -> None:
        sensitivity = np.array(
            [
                [0.010, 0.000],
                [0.000, 0.020],
                [0.000, 0.000],
            ],
            dtype=float,
        )

        est = solve_equal_sag_offsets(
            drift_world=(0.020, -0.040, 0.0),
            sensitivity_m_per_deg=sensitivity,
            min_drift_m=0.0,
        )

        self.assertTrue(est.accepted)
        self.assertAlmostEqual(est.seg1_equal_offset_deg, 2.0)
        self.assertAlmostEqual(est.seg2_equal_offset_deg, -2.0)
        self.assertAlmostEqual(est.residual_m, 0.0)

    def test_rejects_tiny_drift(self) -> None:
        est = solve_equal_sag_offsets(
            drift_world=(0.0001, 0.0, 0.0),
            sensitivity_m_per_deg=np.eye(3, 2),
            min_drift_m=0.002,
        )

        self.assertFalse(est.accepted)
        self.assertEqual(est.reason, "drift_too_small")

    def test_rejects_singular_sensitivity(self) -> None:
        sensitivity = np.array(
            [
                [0.010, 0.020],
                [0.000, 0.000],
                [0.000, 0.000],
            ],
            dtype=float,
        )

        est = solve_equal_sag_offsets(
            drift_world=(0.020, 0.0, 0.0),
            sensitivity_m_per_deg=sensitivity,
            min_drift_m=0.0,
        )

        self.assertFalse(est.accepted)
        self.assertEqual(est.reason, "singular_sensitivity")


if __name__ == "__main__":
    unittest.main()
