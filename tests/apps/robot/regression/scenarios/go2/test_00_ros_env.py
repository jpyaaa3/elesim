from __future__ import annotations

import os
import shutil
import sys
import unittest

from elesim_robot.go2.ros_env import _dist_package_dirs, bootstrap_ros_python_path, ros_import_hint


class TestRosEnv(unittest.TestCase):
    def test_ros_import_hint_mentions_setup_bash(self) -> None:
        hint = ros_import_hint(config_workspace="~/unitree_ros2")
        self.assertIn("setup.bash", hint)
        self.assertIn("unitree_api", hint)

    def test_bootstrap_no_crash_when_paths_missing(self) -> None:
        added = bootstrap_ros_python_path(config_workspace="/tmp/does-not-exist-elesim")
        self.assertIsInstance(added, list)

    def test_dist_package_dirs_for_fake_tree(self) -> None:
        root = os.path.join(os.path.dirname(__file__), "_ros_env_fixture")
        tag = f"python{sys.version_info.major}.{sys.version_info.minor}"
        dist = os.path.join(root, "install", "unitree_api", "local", "lib", tag, "dist-packages")
        os.makedirs(dist, exist_ok=True)
        try:
            paths = _dist_package_dirs(root)
            self.assertTrue(any(p.endswith("dist-packages") for p in paths))
        finally:
            shutil.rmtree(root, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
