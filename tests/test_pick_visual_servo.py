from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.pick_visual_servo import (
    LookAlignLimits,
    LookGains,
    advance_allowed,
    apply_q_delta,
    camera_xy_error,
    compute_advance_delta_q,
    compute_look_delta_q,
    look_align_ok,
    should_send_look_command,
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
        delta = compute_look_delta_q(0.02, -0.01, gains, limits=limits)
        lin, roll, t1, t2 = apply_q_delta(0.5, 0.1, 0.2, 0.3, delta)
        self.assertAlmostEqual(lin, 0.5)
        self.assertAlmostEqual(roll, 0.1)
        self.assertNotAlmostEqual(t1, 0.2)
        self.assertNotAlmostEqual(t2, 0.3)

    def test_advance_only_changes_linear(self) -> None:
        delta = compute_advance_delta_q(0.003)
        lin, roll, t1, t2 = apply_q_delta(0.5, 0.1, 0.2, 0.3, delta)
        self.assertAlmostEqual(lin, 0.503)
        self.assertAlmostEqual(roll, 0.1)
        self.assertAlmostEqual(t1, 0.2)
        self.assertAlmostEqual(t2, 0.3)


if __name__ == "__main__":
    unittest.main()
