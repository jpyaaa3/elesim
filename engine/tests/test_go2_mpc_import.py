from __future__ import annotations

import importlib
import sys
import unittest


class Go2MpcImportTests(unittest.TestCase):
    def test_genesis_pin_bridge_import_without_convex_mpc(self) -> None:
        sys.modules.pop("convex_mpc", None)
        sys.modules.pop("engine.go2.mpc.genesis_pin_bridge", None)
        mod = importlib.import_module("engine.go2.mpc.genesis_pin_bridge")
        self.assertTrue(callable(mod._quat_wxyz_to_xyzw))
        self.assertNotIn("convex_mpc", sys.modules)


if __name__ == "__main__":
    unittest.main()
