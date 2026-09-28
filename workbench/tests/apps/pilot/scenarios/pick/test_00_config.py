from __future__ import annotations

import sys
import unittest
from dataclasses import replace
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "payload").is_dir())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from elesim_pilot.config import PickConfig, load_app_config


class TestPickConfigEffectivePattern(unittest.TestCase):
    """Runtime replace preserves deployment config fields not overridden by UI."""

    def test_replace_preserves_grasp_knobs(self) -> None:
        loaded = PickConfig(
            grasp_waypoint_max_dir_error_deg=5.0,
            grasp_waypoint_max_approach_drift_deg=12.0,
            grasp_skip_aim_recover_in_mock=True,
        )
        effective = replace(
            loaded,
            target_scale=0.14,
            center_tol=0.16,
            look_pose_standoff_m=0.30,
        )
        self.assertAlmostEqual(effective.grasp_waypoint_max_dir_error_deg, 5.0)
        self.assertAlmostEqual(effective.grasp_waypoint_max_approach_drift_deg, 12.0)
        self.assertTrue(effective.grasp_skip_aim_recover_in_mock)

class TestHardwareOffsetConfig(unittest.TestCase):
    def test_real_profiles_preload_roll_offset(self) -> None:
        jetson = load_app_config(str(ROOT / "payload/config/pilot/config.yaml"), mode="jetson")
        pc = load_app_config(str(ROOT / "payload/config/pilot/config.yaml"), mode="pc")

        self.assertAlmostEqual(jetson.hardware_config.u_offset_roll, -9.0)
        self.assertAlmostEqual(pc.hardware_config.u_offset_roll, -9.0)


if __name__ == "__main__":
    unittest.main()
