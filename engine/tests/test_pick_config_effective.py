from __future__ import annotations

import sys
import unittest
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.config_loader import PickConfig


class TestPickConfigEffectivePattern(unittest.TestCase):
    """Regression: runtime replace must preserve config.ini fields not overridden by UI."""

    def test_replace_preserves_sag_drift_knobs(self) -> None:
        loaded = PickConfig(
            sag_drift_max_dir_error_deg=18.0,
            sag_drift_max_lateral_m=0.1,
            sag_drift_axial_only=True,
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
        self.assertAlmostEqual(effective.sag_drift_max_dir_error_deg, 18.0)
        self.assertAlmostEqual(effective.sag_drift_max_lateral_m, 0.1)
        self.assertTrue(effective.sag_drift_axial_only)
        self.assertAlmostEqual(effective.grasp_waypoint_max_dir_error_deg, 5.0)
        self.assertAlmostEqual(effective.grasp_waypoint_max_approach_drift_deg, 12.0)
        self.assertTrue(effective.grasp_skip_aim_recover_in_mock)

    def test_rebuild_without_sag_fields_resets_defaults(self) -> None:
        """Old bug: reconstructing PickConfig dropped sag_drift → default 12deg."""
        loaded = PickConfig(sag_drift_max_dir_error_deg=18.0)
        rebuilt = PickConfig(
            enabled=bool(loaded.enabled),
            target_scale=0.14,
            grasp_online_sag_max_step_deg=float(loaded.grasp_online_sag_max_step_deg),
        )
        self.assertAlmostEqual(rebuilt.sag_drift_max_dir_error_deg, 12.0)


if __name__ == "__main__":
    unittest.main()
