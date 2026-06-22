from __future__ import annotations

import importlib
import sys
import unittest


class Go2MpcImportTests(unittest.TestCase):
    def test_genesis_pin_bridge_import_without_convex_mpc(self) -> None:
        sys.modules.pop("convex_mpc", None)
        sys.modules.pop("engine.go2_mpc.genesis_pin_bridge", None)
        mod = importlib.import_module("engine.go2_mpc.genesis_pin_bridge")
        self.assertTrue(callable(mod._quat_wxyz_to_xyzw))
        self.assertNotIn("convex_mpc", sys.modules)

    def test_sim_camera_pose_import_without_convex_mpc(self) -> None:
        sys.modules.pop("convex_mpc", None)
        sys.modules.pop("engine.sim_camera.pose", None)
        mod = importlib.import_module("engine.sim_camera.pose")
        self.assertTrue(callable(mod.camera_point_to_world_from_axes))
        self.assertNotIn("convex_mpc", sys.modules)


if __name__ == "__main__":
    unittest.main()
