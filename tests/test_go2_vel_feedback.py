from __future__ import annotations

import unittest

from engine.go2_hardware.vel_feedback import Go2VelFeedbackGains, compute_feedback_cmd


class TestGo2VelFeedback(unittest.TestCase):
    def test_no_error_returns_target(self) -> None:
        gains = Go2VelFeedbackGains()
        cmd = compute_feedback_cmd(0.3, 0.0, 0.2, 0.3, 0.0, 0.2, gains=gains)
        self.assertAlmostEqual(cmd[0], 0.3, places=6)
        self.assertAlmostEqual(cmd[1], 0.0, places=6)
        self.assertAlmostEqual(cmd[2], 0.2, places=6)

    def test_positive_error_increases_cmd(self) -> None:
        gains = Go2VelFeedbackGains(kp_vx=1.0, max_vx=0.6)
        cmd = compute_feedback_cmd(0.3, 0.0, 0.0, 0.1, 0.0, 0.0, gains=gains)
        self.assertAlmostEqual(cmd[0], 0.5, places=6)

    def test_cmd_clamped_to_max(self) -> None:
        gains = Go2VelFeedbackGains(kp_vx=2.0, max_vx=0.5)
        cmd = compute_feedback_cmd(0.3, 0.0, 0.0, 0.0, 0.0, 0.0, gains=gains)
        self.assertAlmostEqual(cmd[0], 0.5, places=6)


if __name__ == "__main__":
    unittest.main()
