from __future__ import annotations

import unittest

import engine.core.protocol as proto
from engine.core.trajectory import (
    QuinticTimingConfig,
    QuinticTrajectoryRunner,
    estimate_duration_s,
    quintic_scale,
)


class QuinticTrajectoryTests(unittest.TestCase):
    def test_quintic_scale_endpoints(self) -> None:
        self.assertAlmostEqual(quintic_scale(0.0), 0.0)
        self.assertAlmostEqual(quintic_scale(1.0), 1.0)
        self.assertAlmostEqual(quintic_scale(-1.0), 0.0)
        self.assertAlmostEqual(quintic_scale(2.0), 1.0)

    def test_estimate_duration_scales_and_clamps(self) -> None:
        cfg = QuinticTimingConfig(duration_s=1.2, min_duration_s=0.25, max_duration_s=3.0, linear_scale_m=0.05, angular_scale_rad=0.35)
        q0 = proto.SimQ(0.0, 0.0, 0.0, 0.0)
        q_far = proto.SimQ(0.25, 1.5, 0.0, 0.0)
        d = estimate_duration_s(q0, q_far, cfg)
        self.assertAlmostEqual(d, 3.0)
        q_near = proto.SimQ(0.001, 0.001, 0.001, 0.001)
        d2 = estimate_duration_s(q0, q_near, cfg)
        self.assertAlmostEqual(d2, 1.2)

    def test_runner_start_step_done(self) -> None:
        cfg = QuinticTimingConfig(duration_s=1.0, min_duration_s=0.25, max_duration_s=2.0, linear_scale_m=1.0, angular_scale_rad=1.0)
        runner = QuinticTrajectoryRunner(cfg)
        q0 = proto.SimQ(0.0, 0.0, 0.0, 0.0)
        q1 = proto.SimQ(1.0, 0.5, 0.2, -0.2)
        runner.start(q_start=q0, q_goal=q1, now_s=10.0)
        mid = runner.step(now_s=10.5)
        self.assertFalse(mid.done)
        self.assertGreater(mid.q_cmd.linear_m, 0.0)
        self.assertLess(mid.q_cmd.linear_m, 1.0)
        end = runner.step(now_s=11.1)
        self.assertTrue(end.done)
        self.assertAlmostEqual(end.q_cmd.linear_m, 1.0)
        self.assertFalse(runner.active)

    def test_runner_preempt_replaces_goal(self) -> None:
        cfg = QuinticTimingConfig(duration_s=1.0, min_duration_s=0.25, max_duration_s=2.0, linear_scale_m=1.0, angular_scale_rad=1.0)
        runner = QuinticTrajectoryRunner(cfg)
        q0 = proto.SimQ(0.0, 0.0, 0.0, 0.0)
        q1 = proto.SimQ(1.0, 0.0, 0.0, 0.0)
        q2 = proto.SimQ(0.2, 0.0, 0.0, 0.0)
        runner.start(q_start=q0, q_goal=q1, now_s=0.0)
        _ = runner.step(now_s=0.3)
        runner.start(q_start=q0, q_goal=q2, now_s=0.3)
        end = runner.step(now_s=1.4)
        self.assertTrue(end.done)
        self.assertAlmostEqual(end.q_cmd.linear_m, 0.2)


if __name__ == "__main__":
    unittest.main()
