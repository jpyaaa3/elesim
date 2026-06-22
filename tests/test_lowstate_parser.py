from __future__ import annotations

import unittest

from engine.go2_locomotion.kinematics import GO2_LEG_JOINTS
from engine.go2_hardware.lowstate_parser import lowstate_leg_q_genesis_order


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


if __name__ == "__main__":
    unittest.main()
