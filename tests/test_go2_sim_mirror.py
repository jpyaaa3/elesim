from __future__ import annotations

import os
import unittest

from engine.config_loader import load_app_config_from_ini
from engine.go2_locomotion.config import Go2LocomotionConfig
import engine.protocol as proto


class Go2SimMirrorConfigTests(unittest.TestCase):
    def test_pc_config_enables_mirror(self) -> None:
        root = os.path.dirname(os.path.dirname(__file__))
        bundle = load_app_config_from_ini(os.path.join(root, "config.pc.ini"))
        self.assertTrue(bundle.go2_locomotion_config.mirror_from_host)

    def test_local_config_disables_mirror(self) -> None:
        root = os.path.dirname(os.path.dirname(__file__))
        bundle = load_app_config_from_ini(os.path.join(root, "config.ini"))
        self.assertFalse(bundle.go2_locomotion_config.mirror_from_host)

    def test_default_mirror_false(self) -> None:
        self.assertFalse(Go2LocomotionConfig().mirror_from_host)


class Go2SimMirrorStateTests(unittest.TestCase):
    def test_state_msg_roundtrip_go2_base(self) -> None:
        msg = {
            "t": "state",
            "ts": 1.0,
            "go2_base_pos": [1.5, -0.2, 0.32],
            "go2_base_rpy": [0.0, 0.05, 1.2],
        }
        decoded = proto.loads_msg(proto.dumps_msg(msg))
        pos_raw = decoded.get("go2_base_pos", None)
        rpy_raw = decoded.get("go2_base_rpy", None)
        self.assertEqual(
            (float(pos_raw[0]), float(pos_raw[1]), float(pos_raw[2])),
            (1.5, -0.2, 0.32),
        )
        self.assertEqual(
            (float(rpy_raw[0]), float(rpy_raw[1]), float(rpy_raw[2])),
            (0.0, 0.05, 1.2),
        )

    def test_pack_state_includes_go2_motor_state(self) -> None:
        msg = proto.pack_state(
            go2_leg_q=tuple(float(i) for i in range(12)),
            go2_leg_dq=tuple(float(i) * 0.1 for i in range(12)),
            go2_leg_torque_nm=tuple(float(i) * 0.2 for i in range(12)),
        )
        decoded = proto.loads_msg(proto.dumps_msg(msg))
        self.assertEqual(len(decoded["go2_leg_q"]), 12)
        self.assertEqual(len(decoded["go2_leg_dq"]), 12)
        self.assertEqual(len(decoded["go2_leg_torque_nm"]), 12)


if __name__ == "__main__":
    unittest.main()
