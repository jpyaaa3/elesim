from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np

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
plan_grasp_feasible_trajectory = _GRASP_TRAJECTORY.plan_grasp_feasible_trajectory
plan_grasp_next_waypoint = _GRASP_TRAJECTORY.plan_grasp_next_waypoint
build_grasp_trajectory_markers = _GRASP_TRAJECTORY.build_grasp_trajectory_markers
GraspWaypoint = _GRASP_TRAJECTORY.GraspWaypoint

_SPEC_STATS = importlib.util.spec_from_file_location(
    "pick_timing",
    ROOT / "engine" / "profile" / "pick_timing.py",
)
assert _SPEC_STATS is not None and _SPEC_STATS.loader is not None
_PICK_TIMING = importlib.util.module_from_spec(_SPEC_STATS)
sys.modules[_SPEC_STATS.name] = _PICK_TIMING
_SPEC_STATS.loader.exec_module(_PICK_TIMING)
GraspPlanStats = _PICK_TIMING.GraspPlanStats


class TestGraspTrajectoryPlanner(unittest.TestCase):
    def test_plan_monotonic_standoff_along_kinematic_geom(self) -> None:
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
        end_arr = np.asarray(end, dtype=float)
        for wp in waypoints:
            wp_pos = np.asarray(wp.position_world, dtype=float)
            wp_dir = np.asarray(wp.direction_world, dtype=float)
            axial = float(np.dot(end_arr - wp_pos, wp_dir))
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
        self.assertIn("grasp_traj_nominal", names)
        self.assertIn("grasp_traj_object", names)
        self.assertIn("grasp_traj_standoff", names)
        nominal = next(m for m in markers if m["name"] == "grasp_traj_nominal")
        obj_marker = next(m for m in markers if m["name"] == "grasp_traj_object")
        self.assertNotAlmostEqual(nominal["pos"][0], obj_marker["pos"][0], places=3)
        self.assertGreaterEqual(len([n for n in names if n.startswith("grasp_traj_wp_")]), 1)
        self.assertGreaterEqual(len([n for n in names if n.startswith("grasp_traj_seg_")]), 1)

    def test_feasible_kinematic_chain_stops_when_ik_fails(self) -> None:
        obj = (0.40, 0.0, 0.90)
        start = (0.20, 0.0, 1.10)
        end = (0.38, 0.0, 0.88)
        direction = (1.0, 0.0, 0.0)
        fail_after = 4
        ik_ok = {"n": 0}

        class _IkResult:
            def __init__(
                self,
                success: bool,
                q=None,
                position_error_m=0.0,
                direction_angle_rad=0.0,
                reason="",
            ):
                self.success = success
                self.q = q
                self.position_error_m = position_error_m
                self.direction_angle_rad = direction_angle_rad
                self.reason = reason

        def ik_fn(**kwargs):
            if ik_ok["n"] >= fail_after:
                return _IkResult(
                    False,
                    position_error_m=0.0037,
                    reason="position tolerance not reached",
                )
            target = kwargs["target_world"]
            ik_ok["n"] += 1
            return _IkResult(True, q=[float(target[0]), 0.0, 0.2, 0.1])

        def fk_fn(q):
            q_arr = q
            return type(
                "FkTip",
                (),
                {
                    "position_world": (float(q_arr[0]), 0.0, 0.90),
                    "direction_world": direction,
                },
            )()

        feasible = plan_grasp_feasible_trajectory(
            start_position=start,
            end_position=end,
            start_direction=direction,
            end_direction=direction,
            object_world=obj,
            q_seed=(0.20, 0.0, 0.2, 0.1),
            step_m=0.03,
            blind_start_m=0.06,
            ik_fn=ik_fn,
            fk_fn=fk_fn,
            grasp_standoff_m=0.02,
            max_waypoints=20,
        )
        self.assertEqual(len(feasible), fail_after)
        for wp in feasible:
            self.assertIsNotNone(wp.q_seed)
            self.assertIsNotNone(wp.achieved_position_world)

    def test_feasible_bisect_reaches_smaller_step_after_ik_fail(self) -> None:
        obj = (0.40, 0.0, 0.90)
        start = (0.30, 0.0, 1.10)
        end = (0.38, 0.0, 0.88)
        direction = (1.0, 0.0, 0.0)

        class _IkResult:
            def __init__(
                self,
                success: bool,
                q=None,
                position_error_m=0.0,
                direction_angle_rad=0.0,
                reason="",
            ):
                self.success = success
                self.q = q
                self.position_error_m = position_error_m
                self.direction_angle_rad = direction_angle_rad
                self.reason = reason

        calls: list[float] = []
        attempt = {"n": 0}

        def ik_fn(**kwargs):
            target = kwargs["target_world"]
            calls.append(float(target[0]))
            attempt["n"] += 1
            if attempt["n"] == 1:
                return _IkResult(False, position_error_m=0.004, reason="fail")
            return _IkResult(True, q=[float(target[0]), 0.0, 0.2, 0.1])

        def fk_fn(q):
            return type(
                "FkTip",
                (),
                {
                    "position_world": (float(q[0]), 0.0, 0.90),
                    "direction_world": direction,
                },
            )()

        feasible = plan_grasp_feasible_trajectory(
            start_position=start,
            end_position=end,
            start_direction=direction,
            end_direction=direction,
            object_world=obj,
            q_seed=(0.30, 0.0, 0.2, 0.1),
            step_m=0.03,
            blind_start_m=0.06,
            ik_fn=ik_fn,
            fk_fn=fk_fn,
            max_waypoints=5,
        )
        self.assertGreaterEqual(len(feasible), 1)
        self.assertGreater(len(calls), 1)

    def test_feasible_plan_rejects_excessive_direction_error(self) -> None:
        import math

        obj = (0.40, 0.0, 0.90)
        start = (0.20, 0.0, 1.10)
        end = (0.38, 0.0, 0.88)
        direction = (1.0, 0.0, 0.0)

        class _IkResult:
            def __init__(self, success, q, direction_angle_rad=0.0):
                self.success = success
                self.q = q
                self.position_error_m = 0.0
                self.direction_angle_rad = direction_angle_rad
                self.reason = ""

        def ik_fn(**_kwargs):
            return _IkResult(
                True,
                q=[0.25, 0.0, 0.2, 0.1],
                direction_angle_rad=math.radians(20.0),
            )

        def fk_fn(q):
            return type(
                "FkTip",
                (),
                {
                    "position_world": (float(q[0]), 0.0, 0.90),
                    "direction_world": direction,
                },
            )()

        feasible = plan_grasp_feasible_trajectory(
            start_position=start,
            end_position=end,
            start_direction=direction,
            end_direction=direction,
            object_world=obj,
            q_seed=(0.20, 0.0, 0.2, 0.1),
            step_m=0.03,
            blind_start_m=0.06,
            ik_fn=ik_fn,
            fk_fn=fk_fn,
            max_waypoints=3,
            max_dir_error_deg=12.0,
            max_approach_drift_deg=18.0,
        )
        self.assertEqual(len(feasible), 0)

    def test_feasible_plan_stores_look_at_object_direction(self) -> None:
        obj = (0.40, 0.10, 0.90)
        start = (0.20, 0.0, 1.10)
        end = (0.38, 0.0, 0.88)
        lerp_dir = (1.0, 0.0, 0.0)
        fk_dir = (0.96, 0.28, 0.0)

        class _IkResult:
            def __init__(self):
                self.success = True
                self.q = [0.25, 0.0, 0.2, 0.1]
                self.position_error_m = 0.0
                self.direction_angle_rad = 0.05
                self.reason = ""

        def ik_fn(**_kwargs):
            return _IkResult()

        def fk_fn(q):
            return type(
                "FkTip",
                (),
                {
                    "position_world": (float(q[0]), 0.0, 0.90),
                    "direction_world": fk_dir,
                },
            )()

        feasible = plan_grasp_feasible_trajectory(
            start_position=start,
            end_position=end,
            start_direction=lerp_dir,
            end_direction=lerp_dir,
            object_world=obj,
            q_seed=(0.20, 0.0, 0.2, 0.1),
            step_m=0.03,
            blind_start_m=0.06,
            ik_fn=ik_fn,
            fk_fn=fk_fn,
            max_waypoints=1,
            max_approach_drift_deg=45.0,
        )
        self.assertEqual(len(feasible), 1)
        got = feasible[0].direction_world
        expected = np.asarray(obj, dtype=float) - np.asarray(
            feasible[0].position_world,
            dtype=float,
        )
        expected = expected / float(np.linalg.norm(expected))
        self.assertAlmostEqual(got[0], float(expected[0]), places=4)
        self.assertAlmostEqual(got[1], float(expected[1]), places=4)
        self.assertAlmostEqual(got[2], float(expected[2]), places=4)

    def test_feasible_kinematic_monotonic_axial_progress(self) -> None:
        obj = (0.40, 0.0, 0.90)
        end = (0.38, 0.0, 0.88)
        direction = (1.0, 0.0, 0.0)

        class _IkResult:
            def __init__(self, q):
                self.success = True
                self.q = q
                self.position_error_m = 0.0
                self.direction_angle_rad = 0.0
                self.reason = ""

        def ik_fn(**kwargs):
            target = kwargs["target_world"]
            return _IkResult([float(target[0]), 0.0, 0.2, 0.1])

        def fk_fn(q):
            return type(
                "FkTip",
                (),
                {
                    "position_world": (float(q[0]), 0.0, 0.90),
                    "direction_world": direction,
                },
            )()

        feasible = plan_grasp_feasible_trajectory(
            start_position=(0.20, 0.0, 1.10),
            end_position=end,
            start_direction=direction,
            end_direction=direction,
            object_world=obj,
            q_seed=(0.20, 0.0, 0.2, 0.1),
            step_m=0.03,
            blind_start_m=0.06,
            ik_fn=ik_fn,
            fk_fn=fk_fn,
            max_waypoints=10,
        )
        self.assertGreaterEqual(len(feasible), 2)
        end_arr = np.asarray(end, dtype=float)
        prev_axial = float("inf")
        for wp in feasible:
            pos = np.asarray(wp.position_world, dtype=float)
            axial = float(np.dot(end_arr - pos, direction))
            self.assertLess(axial, prev_axial + 1e-4)
            prev_axial = axial

    def test_kinematic_plan_records_stats_counters(self) -> None:
        obj = (0.40, 0.0, 0.90)
        end = (0.38, 0.0, 0.88)
        direction = (1.0, 0.0, 0.0)
        stats = GraspPlanStats()
        fail_after = 2
        n = {"ik": 0}

        class _IkResult:
            def __init__(self, ok: bool, q=None):
                self.success = ok
                self.q = q
                self.position_error_m = 0.0
                self.direction_angle_rad = 0.0
                self.reason = ""

        def ik_fn(**kwargs):
            n["ik"] += 1
            if n["ik"] > fail_after:
                return _IkResult(False)
            target = kwargs["target_world"]
            return _IkResult(True, q=[float(target[0]), 0.0, 0.2, 0.1])

        def fk_fn(q):
            return type(
                "FkTip",
                (),
                {
                    "position_world": (float(q[0]), 0.0, 0.90),
                    "direction_world": direction,
                },
            )()

        waypoints = plan_grasp_feasible_trajectory(
            start_position=(0.20, 0.0, 1.10),
            end_position=end,
            start_direction=direction,
            end_direction=direction,
            object_world=obj,
            q_seed=(0.20, 0.0, 0.2, 0.1),
            step_m=0.03,
            blind_start_m=0.06,
            ik_fn=ik_fn,
            fk_fn=fk_fn,
            max_waypoints=10,
            stats=stats,
        )
        self.assertEqual(len(waypoints), fail_after)
        self.assertGreater(stats.feasible_ik_attempts, 0)
        self.assertGreater(stats.bisect_iters, 0)
        self.assertEqual(stats.kinematic_steps_ok, fail_after)
        self.assertEqual(stats.kinematic_steps_fail, 1)

    def test_geom_plan_uses_look_at_object_direction(self) -> None:
        obj = (0.40, 0.10, 0.90)
        start = (0.20, 0.0, 1.10)
        end = (0.38, 0.0, 0.88)
        waypoints = plan_grasp_approach_trajectory(
            start_position=start,
            end_position=end,
            start_direction=(1.0, 0.0, 0.0),
            end_direction=(1.0, 0.0, 0.0),
            object_world=obj,
            step_m=0.03,
            blind_start_m=0.06,
            grasp_standoff_m=0.02,
            max_waypoints=5,
        )
        self.assertGreaterEqual(len(waypoints), 1)
        pos = np.asarray(waypoints[0].position_world, dtype=float)
        expected = (np.asarray(obj, dtype=float) - pos) / float(
            np.linalg.norm(np.asarray(obj, dtype=float) - pos)
        )
        got = np.asarray(waypoints[0].direction_world, dtype=float)
        self.assertAlmostEqual(float(np.dot(got, expected)), 1.0, places=4)


if __name__ == "__main__":
    unittest.main()
