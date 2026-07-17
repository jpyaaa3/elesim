from __future__ import annotations

import unittest

import numpy as np

from engine.gaze.preview_lite import PitchLeadEstimator, resolve_pitch_rate
from engine.gaze.preview_mpc import solve_preview_du


class PreviewMpcLiteTests(unittest.TestCase):
    def test_solve_known_jacobian(self) -> None:
        j = np.diag([1.0, 1.0, 1.0]).astype(float)[:2, :]
        result = solve_preview_du(
            jacobian=j,
            s=np.array([0.1, 0.2], dtype=float),
            preview_term=np.array([0.0, 0.05], dtype=float),
            q_u=1.0,
            q_v=1.0,
            r_roll=0.01,
            r_s1=0.01,
            r_s2=0.01,
        )
        self.assertTrue(result.ok)
        self.assertAlmostEqual(float(result.preview_term_v), 0.05, places=6)
        self.assertTrue(np.all(np.isfinite(result.du)))

    def test_solve_non_finite_fails(self) -> None:
        j = np.eye(2, 3, dtype=float)
        result = solve_preview_du(
            jacobian=j,
            s=np.array([float("inf"), 0.0], dtype=float),
            preview_term=np.zeros(2, dtype=float),
            q_u=1.0,
            q_v=1.0,
            r_roll=0.01,
            r_s1=0.01,
            r_s2=0.01,
        )
        self.assertFalse(result.ok)
        self.assertIn("solve_fail", result.reason)

    def test_preview_term_sign(self) -> None:
        j = np.eye(2, 3, dtype=float)
        pos = solve_preview_du(
            jacobian=j,
            s=np.zeros(2, dtype=float),
            preview_term=np.array([0.0, 0.1], dtype=float),
            q_u=1.0,
            q_v=1.0,
            r_roll=0.1,
            r_s1=0.1,
            r_s2=0.1,
        )
        neg = solve_preview_du(
            jacobian=j,
            s=np.zeros(2, dtype=float),
            preview_term=np.array([0.0, -0.1], dtype=float),
            q_u=1.0,
            q_v=1.0,
            r_roll=0.1,
            r_s1=0.1,
            r_s2=0.1,
        )
        self.assertTrue(pos.ok and neg.ok)
        self.assertFalse(np.allclose(pos.du, neg.du))


class PitchLeadEstimatorTests(unittest.TestCase):
    def test_uses_sim_timestamp_dt(self) -> None:
        est = PitchLeadEstimator()
        out1 = est.update(
            pitch_rate=0.0,
            go2_base_timestamp_s=1.0,
            worker_period_s=0.1,
            tau_s=0.08,
            lowpass_alpha=1.0,
        )
        self.assertTrue(out1.ok)
        out2 = est.update(
            pitch_rate=0.2,
            go2_base_timestamp_s=1.05,
            worker_period_s=0.1,
            tau_s=0.08,
            lowpass_alpha=1.0,
        )
        self.assertTrue(out2.ok)
        self.assertAlmostEqual(out2.preview_dt_s, 0.05, places=6)
        self.assertAlmostEqual(out2.pitch_acc_est, 0.2 / 0.05, places=4)

    def test_missing_ts_fails(self) -> None:
        est = PitchLeadEstimator()
        out = est.update(
            pitch_rate=0.1,
            go2_base_timestamp_s=0.0,
            worker_period_s=0.1,
            tau_s=0.08,
            lowpass_alpha=0.35,
        )
        self.assertFalse(out.ok)
        self.assertEqual(out.reason, "missing_ts")

    def test_resolve_pitch_rate_from_ang_vel(self) -> None:
        class Host:
            go2_base_timestamp_s = 1.0
            go2_base_ang_vel_body = (0.0, 0.42, 0.0)
            go2_base_rpy = None

        rate, pitch = resolve_pitch_rate(Host(), prev_pitch_rad=None, prev_go2_base_timestamp_s=None, worker_period_s=0.1)
        self.assertAlmostEqual(float(rate), 0.42, places=6)
        self.assertIsNone(pitch)


if __name__ == "__main__":
    unittest.main()
