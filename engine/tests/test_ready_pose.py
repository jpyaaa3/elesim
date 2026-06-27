from __future__ import annotations

import unittest

from engine.vision.pick.core import compute_ready_pose_target


class ReadyPoseTests(unittest.TestCase):
    def test_ready_pose_offsets_against_target_direction(self) -> None:
        target = compute_ready_pose_target(
            (1.0, 2.0, 3.0),
            (2.0, 0.0, 0.0),
            standoff_m=0.20,
        )

        self.assertAlmostEqual(target[0], 0.80)
        self.assertAlmostEqual(target[1], 2.0)
        self.assertAlmostEqual(target[2], 3.0)

    def test_ready_pose_rejects_zero_direction(self) -> None:
        with self.assertRaises(ValueError):
            compute_ready_pose_target(
                (1.0, 2.0, 3.0),
                (0.0, 0.0, 0.0),
                standoff_m=0.20,
            )

    def test_ready_pose_normalizes_diagonal_direction(self) -> None:
        target = compute_ready_pose_target(
            (1.0, 1.0, 1.0),
            (3.0, 4.0, 0.0),
            standoff_m=0.20,
        )

        self.assertAlmostEqual(target[0], 0.88)
        self.assertAlmostEqual(target[1], 0.84)
        self.assertAlmostEqual(target[2], 1.0)


if __name__ == "__main__":
    unittest.main()
