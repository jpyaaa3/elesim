from __future__ import annotations

import unittest

from engine.go2_hardware.vel_feedback import (
    Go2VelFeedbackGains,
    HeadingHoldController,
    compute_feedback_cmd,
)


class TestGo2VelFeedback(unittest.TestCase):
    def test_no_error_returns_target(self) -> None:
        gains = Go2VelFeedbackGains()
        cmd = compute_feedback_cmd(0.3, 0.0, 0.2, 0.3, 0.0, 0.2, gains=gains)
        self.assertAlmostEqual(cmd[0], 0.3, places=6)
        self.assertAlmostEqual(cmd[1], 0.0, places=6)
        self.assertAlmostEqual(cmd[2], 0.2, places=6)

    def test_positive_error_increases_cmd_within_corr_cap(self) -> None:
        gains = Go2VelFeedbackGains(kp_vx=1.0, max_vx=0.6, max_corr_vx=0.15)
        cmd = compute_feedback_cmd(0.3, 0.0, 0.0, 0.1, 0.0, 0.0, gains=gains)
        self.assertAlmostEqual(cmd[0], 0.45, places=6)

    def test_cmd_clamped_to_max(self) -> None:
        gains = Go2VelFeedbackGains(kp_vx=2.0, max_vx=0.5, max_corr_vx=0.5)
        cmd = compute_feedback_cmd(0.3, 0.0, 0.0, 0.0, 0.0, 0.0, gains=gains)
        self.assertAlmostEqual(cmd[0], 0.5, places=6)

    def test_inactive_axes_stay_zero(self) -> None:
        gains = Go2VelFeedbackGains(kp_vy=1.0, kp_wz=1.0)
        cmd = compute_feedback_cmd(0.3, 0.0, 0.0, 0.0, 0.05, 0.2, gains=gains)
        self.assertAlmostEqual(cmd[0], 0.45, places=6)
        self.assertAlmostEqual(cmd[1], 0.0, places=6)
        self.assertAlmostEqual(cmd[2], 0.0, places=6)

    def test_heading_hold_applies_wz_for_linear_only(self) -> None:
        gains = Go2VelFeedbackGains(
            heading_hold_kp=2.0,
            heading_hold_ki=0.0,
            heading_hold_kd=0.0,
            heading_hold_max_wz=0.5,
        )
        ctl = HeadingHoldController()
        cmd = compute_feedback_cmd(
            0.25,
            0.0,
            0.0,
            0.2,
            0.0,
            0.0,
            gains=gains,
            held_yaw=0.0,
            current_yaw=0.2,
            heading_hold_enable=True,
            heading_ctl=ctl,
            now_s=1.0,
        )
        self.assertAlmostEqual(cmd[2], -0.4, places=6)

    def test_heading_hold_integral_reduces_steady_state_error(self) -> None:
        gains = Go2VelFeedbackGains(
            heading_hold_kp=0.0,
            heading_hold_ki=1.0,
            heading_hold_kd=0.0,
            heading_hold_max_wz=0.5,
            heading_hold_integral_max=0.5,
        )
        ctl = HeadingHoldController()
        ctl.integral = 0.2
        wz = ctl.compute(0.0, 0.0, 0.0, 1.0, gains=gains)
        self.assertAlmostEqual(wz, 0.2, places=6)

    def test_heading_hold_disabled_keeps_wz_zero(self) -> None:
        gains = Go2VelFeedbackGains()
        cmd = compute_feedback_cmd(
            0.25,
            0.0,
            0.0,
            0.2,
            0.0,
            0.1,
            gains=gains,
            held_yaw=0.0,
            current_yaw=0.2,
            heading_hold_enable=False,
        )
        self.assertAlmostEqual(cmd[2], 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
