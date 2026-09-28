from __future__ import annotations

import unittest

from elesim_pilot.robot.arm.iklib.kinematics import _build_q_map


class NominalKinematicsTests(unittest.TestCase):
    def test_legacy_sag_context_does_not_change_joint_map(self) -> None:
        context = {
            "linear_joint_name": "linear",
            "roll_joint_name": "roll",
            "bend_joint_names": ["a", "b", "c", "d"],
            "n_seg": 2,
            "sag_model": {"seg1_equal_offset_deg": 12.0, "seg2_equal_offset_deg": -12.0},
        }
        result = _build_q_map(context, [0.01, 0.2, -0.3, 0.4])
        self.assertEqual(result, {"linear": 0.01, "roll": 0.2, "a": -0.3, "b": -0.3, "c": 0.4, "d": 0.4})


if __name__ == "__main__":
    unittest.main()
