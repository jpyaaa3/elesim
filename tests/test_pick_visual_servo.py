from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.pick_visual_servo import (
    JacobianLookGains,
    LookAlignLimits,
    LookGains,
    advance_allowed,
    apply_q_delta,
    apply_q_delta_to_tuple,
    camera_xy_error,
    compute_advance_delta_q,
    compute_jacobian_look_delta_q,
    compute_look_delta_q,
    damped_pseudoinverse,
    error_vector_2d,
    estimate_jacobian_column,
    jacobian_column_usable,
    look_align_ok,
    should_send_look_command,
    stack_jacobian,
)


class TestCameraXyError(unittest.TestCase):
    def test_error_at_desired(self) -> None:
        ex, ey, norm_xy = camera_xy_error((0.0, 0.0, 0.5), (0.0, 0.0))
        self.assertAlmostEqual(ex, 0.0)
        self.assertAlmostEqual(ey, 0.0)
        self.assertAlmostEqual(norm_xy, 0.0)

    def test_error_offset(self) -> None:
        ex, ey, _ = camera_xy_error((0.02, -0.01, 0.5), (0.0, 0.0))
        self.assertAlmostEqual(ex, 0.02)
        self.assertAlmostEqual(ey, -0.01)

    def test_error_vector_2d(self) -> None:
        e = error_vector_2d(0.02, -0.01)
        self.assertEqual(e.shape, (2,))
        self.assertAlmostEqual(float(e[0]), 0.02)
        self.assertAlmostEqual(float(e[1]), -0.01)


class TestLookAlignGates(unittest.TestCase):
    def setUp(self) -> None:
        self.limits = LookAlignLimits(xy_threshold_m=0.010, xy_deadband_m=0.008)

    def test_look_ok_inside_threshold(self) -> None:
        self.assertTrue(look_align_ok(0.005, -0.004, self.limits))

    def test_advance_blocked_when_xy_large(self) -> None:
        self.assertFalse(advance_allowed(0.02, 0.0, self.limits))

    def test_deadband_blocks_small_command(self) -> None:
        self.assertFalse(should_send_look_command(0.002, 0.001, self.limits))


class TestLookDeltaQ(unittest.TestCase):
    def test_look_only_changes_bend_axes(self) -> None:
        limits = LookAlignLimits()
        gains = LookGains(theta1_per_error_x=1.0, theta2_per_error_y=1.0, max_step_rad=0.02)
        delta = compute_look_delta_q(0.02, -0.01, gains, limits=limits, use_roll=False)
        lin, roll, t1, t2 = apply_q_delta(0.5, 0.1, 0.2, 0.3, delta)
        self.assertAlmostEqual(lin, 0.5)
        self.assertAlmostEqual(roll, 0.1)
        self.assertNotAlmostEqual(t1, 0.2)
        self.assertNotAlmostEqual(t2, 0.3)

    def test_look_heuristic_roll_for_ex(self) -> None:
        limits = LookAlignLimits()
        gains = LookGains(
            roll_per_error_x=-1.0,
            theta2_per_error_y=-1.0,
            max_step_rad=0.02,
            max_step_roll_rad=0.02,
        )
        delta = compute_look_delta_q(0.02, -0.01, gains, limits=limits, use_roll=True)
        self.assertAlmostEqual(delta.roll_rad, +0.02)
        self.assertAlmostEqual(delta.theta1_rad, 0.0)
        self.assertAlmostEqual(delta.theta2_rad, -0.01)

    def test_advance_only_changes_linear(self) -> None:
        delta = compute_advance_delta_q(0.003)
        lin, roll, t1, t2 = apply_q_delta(0.5, 0.1, 0.2, 0.3, delta)
        self.assertAlmostEqual(lin, 0.503)
        self.assertAlmostEqual(roll, 0.1)
        self.assertAlmostEqual(t1, 0.2)
        self.assertAlmostEqual(t2, 0.3)


class TestJacobianMath(unittest.TestCase):
    def test_damped_pseudoinverse_recovers_error_direction(self) -> None:
        j = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], dtype=float)
        e = np.array([0.02, -0.01], dtype=float)
        j_pinv = damped_pseudoinverse(j, 0.01)
        recovered = j @ (j_pinv @ e)
        np.testing.assert_allclose(recovered, e, atol=1e-4)

    def test_estimate_jacobian_column(self) -> None:
        e0 = np.array([0.01, 0.0])
        e1 = np.array([0.02, 0.0])
        col = estimate_jacobian_column(e0, e1, 0.01)
        np.testing.assert_allclose(col, [1.0, 0.0], atol=1e-6)

    def test_stack_jacobian_shape(self) -> None:
        cols = [np.array([1.0, 0.0]), np.array([0.0, 1.0]), np.array([0.5, 0.5])]
        j = stack_jacobian(cols)
        self.assertEqual(j.shape, (2, 3))

    def test_jacobian_column_usable(self) -> None:
        self.assertTrue(jacobian_column_usable([0.01, 0.0], norm_min=1e-5))
        self.assertFalse(jacobian_column_usable([0.0, 0.0], norm_min=1e-5))

    def test_jacobian_look_delta_reduces_error_diagonal(self) -> None:
        limits = LookAlignLimits(xy_deadband_m=0.0)
        gains = JacobianLookGains(gain=0.5, damping=0.01, max_step_roll_rad=0.05, max_step_theta_rad=0.05)
        # roll -> ex, theta1 -> ey
        j = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=float)
        e = np.array([0.02, -0.01], dtype=float)
        delta = compute_jacobian_look_delta_q(e, j, gains, limits=limits, include_roll=True)
        self.assertAlmostEqual(delta.linear_m, 0.0)
        self.assertLess(delta.roll_rad, 0.0)
        self.assertGreater(delta.theta1_rad, 0.0)

    def test_jacobian_look_delta_theta12_only(self) -> None:
        limits = LookAlignLimits(xy_deadband_m=0.0)
        gains = JacobianLookGains(gain=0.5, damping=0.01, max_step_theta_rad=0.05)
        j = np.array([[0.17, 0.10], [-0.14, -0.03]], dtype=float)
        e = np.array([0.15, 0.10], dtype=float)
        delta = compute_jacobian_look_delta_q(e, j, gains, limits=limits, include_roll=False)
        self.assertAlmostEqual(delta.roll_rad, 0.0)
        self.assertNotAlmostEqual(delta.theta1_rad, 0.0)
        self.assertNotAlmostEqual(delta.theta2_rad, 0.0)

    def test_apply_q_delta_to_tuple(self) -> None:
        q = (0.5, 0.1, 0.2, 0.3)
        out = apply_q_delta_to_tuple(q, compute_advance_delta_q(0.01))
        self.assertAlmostEqual(out[0], 0.51)


if __name__ == "__main__":
    unittest.main()
