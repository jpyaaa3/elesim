from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "AGENTS.md").exists())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from elesim_controller.vision.visual_servoing.grasp_trajectory import GraspWaypoint, build_grasp_trajectory_markers


class TestGraspTrajectoryMarkers(unittest.TestCase):
    def test_build_markers_includes_start_nominal_and_waypoints(self) -> None:
        start = (0.10, 0.0, 0.90)
        end = (0.31, 0.0, 0.90)
        obj = (0.33, 0.0, 0.90)
        wp = GraspWaypoint(
            position_world=(0.20, 0.0, 0.90),
            direction_world=(1.0, 0.0, 0.0),
            standoff_m=0.13,
        )
        markers = build_grasp_trajectory_markers(
            start_position=start,
            end_position=end,
            object_world=obj,
            waypoints=[wp],
            highlight_idx=0,
            look_anchor_position=(0.03, 0.0, 0.90),
        )
        names = {str(m.get("name", "")) for m in markers}
        self.assertIn("grasp_traj_start", names)
        self.assertIn("grasp_traj_nominal", names)
        self.assertIn("grasp_traj_object", names)
        self.assertIn("grasp_traj_wp_0", names)
        self.assertIn("grasp_traj_look_pose", names)

    def test_empty_waypoints_still_draws_nominal_segment(self) -> None:
        markers = build_grasp_trajectory_markers(
            start_position=(0.1, 0.0, 0.9),
            end_position=(0.3, 0.0, 0.9),
            object_world=(0.32, 0.0, 0.9),
            waypoints=[],
        )
        self.assertGreaterEqual(len(markers), 3)


if __name__ == "__main__":
    unittest.main()
