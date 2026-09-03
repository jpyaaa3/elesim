from __future__ import annotations

import unittest

from elesim_robot.go2.joints import GO2_LEG_JOINTS
from elesim_robot.go2.lowstate_parser import lowstate_leg_q_genesis_order, lowstate_motor_sample_genesis_order


class LowStateParserTests(unittest.TestCase):
    def test_maps_unitree_motors_to_genesis_order(self) -> None:
        class _Motor:
            def __init__(self, q: float) -> None:
                self.q = q

        motors = [_Motor(float(i) * 0.1) for i in range(12)]

        class _Msg:
            motor_state = motors

        leg_q = lowstate_leg_q_genesis_order(_Msg())
        self.assertEqual(len(leg_q), 12)
        # FL_hip_joint uses unitree motor index 3
        fl_hip_idx = GO2_LEG_JOINTS.index("FL_hip_joint")
        self.assertAlmostEqual(leg_q[fl_hip_idx], 0.3, places=6)
        # FR_hip_joint uses unitree motor index 0
        fr_hip_idx = GO2_LEG_JOINTS.index("FR_hip_joint")
        self.assertAlmostEqual(leg_q[fr_hip_idx], 0.0, places=6)

    def test_extracts_velocity_and_torque_when_available(self) -> None:
        class _Motor:
            def __init__(self, i: int) -> None:
                self.q = float(i) * 0.1
                self.dq = float(i) * 0.01
                self.tau_est = float(i) * 0.2

        motors = [_Motor(i) for i in range(12)]

        class _Msg:
            motor_state = motors

        sample = lowstate_motor_sample_genesis_order(_Msg())
        self.assertEqual(len(sample.q), 12)
        self.assertIsNotNone(sample.dq)
        self.assertIsNotNone(sample.torque_nm)
        assert sample.dq is not None
        assert sample.torque_nm is not None
        fl_hip_idx = GO2_LEG_JOINTS.index("FL_hip_joint")
        self.assertAlmostEqual(sample.dq[fl_hip_idx], 0.03, places=6)
        self.assertAlmostEqual(sample.torque_nm[fl_hip_idx], 0.6, places=6)

    def test_torque_is_optional(self) -> None:
        class _Motor:
            def __init__(self, q: float) -> None:
                self.q = q

        motors = [_Motor(float(i)) for i in range(12)]

        class _Msg:
            motor_state = motors

        sample = lowstate_motor_sample_genesis_order(_Msg())
        self.assertEqual(len(sample.q), 12)
        self.assertIsNone(sample.dq)
        self.assertIsNone(sample.torque_nm)


if __name__ == "__main__":
    unittest.main()
