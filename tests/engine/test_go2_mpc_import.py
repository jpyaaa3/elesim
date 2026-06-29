from __future__ import annotations

import importlib
import importlib.util
import sys
import unittest
from pathlib import Path


class Go2MpcImportTests(unittest.TestCase):
    def test_genesis_pin_bridge_import_without_convex_mpc(self) -> None:
        sys.modules.pop("convex_mpc", None)
        sys.modules.pop("engine.robot.go2.mpc.genesis_pin_bridge", None)
        mod = importlib.import_module("engine.robot.go2.mpc.genesis_pin_bridge")
        self.assertTrue(callable(mod._quat_wxyz_to_xyzw))
        self.assertNotIn("convex_mpc", sys.modules)

    def test_controller_patches_convex_mpc_go2_urdf_path(self) -> None:
        if importlib.util.find_spec("convex_mpc") is None:
            self.skipTest("convex_mpc is not installed")
        controller = importlib.import_module("engine.robot.go2.mpc.controller")
        controller._require_convex_mpc()
        data = importlib.import_module("convex_mpc.go2_robot_data")
        urdf_path = Path(data.URDF_PATH)
        self.assertTrue(urdf_path.is_file())
        self.assertEqual(urdf_path.name, "go2.urdf")
        self.assertEqual(urdf_path.parent.name, "go2")
        pin_model = data.PinGo2Model()
        self.assertGreaterEqual(pin_model.base_id, 0)


if __name__ == "__main__":
    unittest.main()
