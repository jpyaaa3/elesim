from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

_SPEC = importlib.util.spec_from_file_location(
    "grasp_trajectory",
    ROOT / "engine" / "visual_servoing" / "grasp_trajectory.py",
)
assert _SPEC is not None and _SPEC.loader is not None
_GRASP_TRAJECTORY = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = _GRASP_TRAJECTORY
_SPEC.loader.exec_module(_GRASP_TRAJECTORY)
plan_grasp_approach_trajectory = _GRASP_TRAJECTORY.plan_grasp_approach_trajectory
plan_grasp_next_waypoint = _GRASP_TRAJECTORY.plan_grasp_next_waypoint
build_grasp_trajectory_markers = _GRASP_TRAJECTORY.build_grasp_trajectory_markers


class TestGraspTrajectoryPlanner(unittest.TestCase):
    def test_plan_monotonic_standoff_along_lerp(self) -> None:
        obj = (0.40, 0.0, 0.90)
        start = (0.20, 0.0, 1.10)
        end = (0.38, 0.0, 0.88)
        direction = (1.0, 0.0, 0.0)
        waypoints = plan_grasp_approach_trajectory(
            start_position=start,
            end_position=end,
            start_direction=direction,
            end_direction=direction,
            object_world=obj,
            step_m=0.03,
            blind_start_m=0.06,
            grasp_standoff_m=0.02,
            max_waypoints=20,
        )
        self.assertGreaterEqual(len(waypoints), 2)
        standoffs = [float(wp.standoff_m) for wp in waypoints]
        for i in range(1, len(standoffs)):
            self.assertLessEqual(standoffs[i], standoffs[i - 1] + 1e-4)
        first = waypoints[0].position_world
        self.assertGreater(first[0], start[0] - 1e-3)
        self.assertLess(first[0], end[0] + 1e-2)

    def test_plan_stops_before_blind_zone(self) -> None:
        obj = (0.40, 0.0, 0.90)
        start = (0.30, 0.0, 1.10)
        end = (0.38, 0.0, 0.88)
        direction = (1.0, 0.0, 0.0)
        blind = 0.06
        waypoints = plan_grasp_approach_trajectory(
            start_position=start,
            end_position=end,
            start_direction=direction,
            end_direction=direction,
            object_world=obj,
            step_m=0.01,
            blind_start_m=blind,
            grasp_standoff_m=0.02,
            max_waypoints=50,
        )
        self.assertGreater(len(waypoints), 0)
        end_arr = end
        for wp in waypoints:
            axial = (
                (end_arr[0] - wp.position_world[0]) * direction[0]
                + (end_arr[1] - wp.position_world[1]) * direction[1]
                + (end_arr[2] - wp.position_world[2]) * direction[2]
            )
            self.assertGreater(axial, blind - 1e-3)

    def test_next_waypoint_returns_single_step(self) -> None:
        obj = (0.40, 0.0, 0.90)
        start = (0.30, 0.0, 1.10)
        end = (0.38, 0.0, 0.88)
        direction = (1.0, 0.0, 0.0)
        nxt = plan_grasp_next_waypoint(
            start_position=start,
            end_position=end,
            start_direction=direction,
            end_direction=direction,
            object_world=obj,
            step_m=0.03,
            blind_start_m=0.06,
            grasp_standoff_m=0.02,
        )
        self.assertIsNotNone(nxt)
        assert nxt is not None
        all_wp = plan_grasp_approach_trajectory(
            start_position=start,
            end_position=end,
            start_direction=direction,
            end_direction=direction,
            object_world=obj,
            step_m=0.03,
            blind_start_m=0.06,
            grasp_standoff_m=0.02,
            max_waypoints=1,
        )
        self.assertEqual(len(all_wp), 1)
        self.assertAlmostEqual(nxt.position_world[0], all_wp[0].position_world[0], places=4)

    def test_trajectory_markers_include_path_nodes(self) -> None:
        obj = (0.40, 0.0, 0.90)
        start = (0.20, 0.0, 1.10)
        end = (0.38, 0.0, 0.88)
        direction = (1.0, 0.0, 0.0)
        waypoints = plan_grasp_approach_trajectory(
            start_position=start,
            end_position=end,
            start_direction=direction,
            end_direction=direction,
            object_world=obj,
            step_m=0.03,
            blind_start_m=0.06,
            grasp_standoff_m=0.02,
            max_waypoints=20,
        )
        markers = build_grasp_trajectory_markers(
            start_position=start,
            end_position=end,
            object_world=obj,
            waypoints=waypoints,
        )
        names = {str(m["name"]) for m in markers}
        self.assertIn("grasp_traj_start", names)
        self.assertIn("grasp_traj_end", names)
        self.assertIn("grasp_traj_object", names)
        self.assertGreaterEqual(len([n for n in names if n.startswith("grasp_traj_wp_")]), 1)
        self.assertGreaterEqual(len([n for n in names if n.startswith("grasp_traj_seg_")]), 1)


if __name__ == "__main__":
    unittest.main()
