from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "payload").is_dir())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from elesim_pilot.vision.visual_servoing.local_image_jacobian import (
    ImageJacobianEstimator3D,
    LocalImageJacobianServo3D,
    LocalImageJacobianServoGains,
    check_sample_quality,
    default_j_lji_seed,
)


class TestLjiGraspGate(unittest.TestCase):
    @staticmethod
    def _servo(
        *,
        min_rank: int = 3,
        condition_max: float = 100.0,
    ) -> LocalImageJacobianServo3D:
        seed_j = default_j_lji_seed(
            center_u_gain=0.1,
            center_v_gain=0.1,
            command_direction=(-1, -1, 1, -1),
        )
        est = ImageJacobianEstimator3D(
            window_size=8,
            seed_j=seed_j,
            min_measured_samples=4,
            condition_max=condition_max,
            min_rank=min_rank,
        )
        gains = LocalImageJacobianServoGains(
            damping=0.05,
            gain_u=0.35,
            gain_v=0.35,
            gain_z=0.45,
            max_dq_linear=0.01,
            max_dq_angle=0.012,
        )
        return LocalImageJacobianServo3D(
            estimator=est,
            gains=gains,
            min_samples=4,
            condition_max=condition_max,
            min_rank=min_rank,
            command_direction=(-1, -1, 1, -1),
        )

    def test_fk_z_patch_makes_seed_ready_for_grasp_step(self) -> None:
        servo = self._servo()
        z_row = np.array([-0.9, -0.1, 0.05, 0.08], dtype=float)

        dq, _, j, rank, cond, available = servo.compute_dq(
            [0.05, -0.02, 0.15],
            z_row=z_row,
        )

        self.assertTrue(available)
        self.assertGreaterEqual(rank, 3)
        self.assertTrue(np.isfinite(cond))
        self.assertTrue(np.allclose(j[2, :], z_row))
        self.assertLessEqual(abs(float(dq[0])), 0.01)
        self.assertLessEqual(float(np.max(np.abs(dq[1:]))), 0.012)

    def test_without_fk_z_patch_seed_is_not_grasp_ready(self) -> None:
        servo = self._servo()

        _, _, j, rank, cond, available = servo.compute_dq([0.05, -0.02, 0.15])

        self.assertFalse(available)
        self.assertLess(rank, 3)
        self.assertFalse(np.isfinite(cond))
        self.assertTrue(np.allclose(j[2, :], 0.0))

    def test_bad_measured_samples_fall_back_to_seed_then_fk_patch(self) -> None:
        servo = self._servo()
        for _ in range(4):
            servo.estimator.push([0.005, 0.03, -0.03, -0.03], [0.001, -0.002, -0.004])
        z_row = np.array([-0.8, 0.0, -0.1, 0.15], dtype=float)

        _, _, j, rank, cond, available = servo.compute_dq(
            [0.02, -0.04, 0.12],
            z_row=z_row,
        )

        self.assertTrue(available)
        self.assertGreaterEqual(rank, 3)
        self.assertTrue(np.isfinite(cond))
        self.assertTrue(np.allclose(j[2, :], z_row))

    def test_grasp_sample_gate_rejects_tiny_motion_before_update(self) -> None:
        ok, reason = check_sample_quality(
            delta_q=[1e-8, 0.0, 0.0, 0.0],
            min_dq_norm=1e-4,
            object_lost=False,
            settle_ok=True,
            joint_saturated=False,
        )

        self.assertFalse(ok)
        self.assertEqual(reason.value, "dq_too_small")


if __name__ == "__main__":
    unittest.main()
