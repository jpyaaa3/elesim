from __future__ import annotations

import threading
from pathlib import Path

import numpy as np
import pytest

from elesim_controller.robot.arm.iklib.kinematics import Q_NEUTRAL
from elesim_controller.robot.arm.iklib.solver import load_solver_context
from elesim_controller.robot.arm.joint_defs import JointLimit
from elesim_controller.robot.arm.planning.collision import CollisionModel, WorldBox, check_configuration
from elesim_controller.robot.arm.planning.rrt import (
    RrtConfig,
    joint_bounds_from_context,
    maximize_clearance,
    plan_rrt_connect,
    shortcut_path,
)

FAKE_LIMIT = JointLimit(roll_min_deg=-90.0, roll_max_deg=90.0, bend_deg=40.0)
FAKE_CONTEXT = {"linear_min_m": -0.23, "linear_max_m": 0.01, "limit": FAKE_LIMIT}

CONFIG_PATH = Path(__file__).parents[2] / "config" / "config.yaml"


@pytest.fixture(scope="module")
def ik_context() -> dict:
    _bundle, context = load_solver_context(str(CONFIG_PATH))
    return context


def _always_valid(_q: np.ndarray) -> bool:
    return True


def test_joint_bounds_from_context_matches_joint_limit() -> None:
    lo, hi = joint_bounds_from_context(FAKE_CONTEXT)
    assert lo == pytest.approx([-0.23, np.radians(-90.0), np.radians(-40.0), np.radians(-40.0)])
    assert hi == pytest.approx([0.01, np.radians(90.0), np.radians(40.0), np.radians(40.0)])


def test_plan_rrt_connect_finds_direct_path_with_no_obstacles() -> None:
    lo, hi = joint_bounds_from_context(FAKE_CONTEXT)
    start = lo + 0.2 * (hi - lo)
    goal = lo + 0.8 * (hi - lo)

    result = plan_rrt_connect(
        start_q=start,
        goal_q=goal,
        context=FAKE_CONTEXT,
        validity_fn=_always_valid,
        config=RrtConfig(max_iters=500, seed=1),
    )

    assert result.success
    assert result.waypoints[0] == pytest.approx(start)
    assert result.waypoints[-1] == pytest.approx(goal)
    # RRT-Connect's greedy connect should link the two trees on the very
    # first sample when nothing blocks a direct route.
    assert result.iterations == 1


def test_plan_rrt_connect_fails_when_start_in_collision() -> None:
    lo, hi = joint_bounds_from_context(FAKE_CONTEXT)
    start = lo.copy()
    goal = hi.copy()
    result = plan_rrt_connect(
        start_q=start,
        goal_q=goal,
        context=FAKE_CONTEXT,
        validity_fn=lambda q: False,
        config=RrtConfig(max_iters=10, seed=1),
    )
    assert not result.success
    assert result.reason == "start_in_collision"
    assert result.waypoints == []


def test_plan_rrt_connect_fails_when_goal_in_collision() -> None:
    lo, hi = joint_bounds_from_context(FAKE_CONTEXT)
    start = lo.copy()
    goal = hi.copy()

    def _valid(q: np.ndarray) -> bool:
        return bool(np.allclose(q, start, atol=1e-6))

    result = plan_rrt_connect(
        start_q=start,
        goal_q=goal,
        context=FAKE_CONTEXT,
        validity_fn=_valid,
        config=RrtConfig(max_iters=10, seed=1),
    )
    assert not result.success
    assert result.reason == "goal_in_collision"


def _wall_with_roll_gap(q: np.ndarray) -> bool:
    """Blocks the mid-linear plane unless roll has swung past +-0.5 rad."""
    linear, roll = float(q[0]), float(q[1])
    if -0.12 < linear < -0.10:
        return abs(roll) > 0.5
    return True


def test_plan_rrt_connect_routes_around_a_blocking_wall() -> None:
    lo, hi = joint_bounds_from_context(FAKE_CONTEXT)
    start = np.array([lo[0], 0.0, 0.0, 0.0], dtype=float)
    goal = np.array([hi[0], 0.0, 0.0, 0.0], dtype=float)
    assert _wall_with_roll_gap(start) and _wall_with_roll_gap(goal)

    # A straight interpolation between start and goal keeps roll pinned at 0,
    # so it must fail at the wall -- any success has to come from a detour.
    mid = 0.5 * (start + goal)
    assert not _wall_with_roll_gap(mid)

    result = plan_rrt_connect(
        start_q=start,
        goal_q=goal,
        context=FAKE_CONTEXT,
        validity_fn=_wall_with_roll_gap,
        config=RrtConfig(max_iters=4000, step_size=0.1, collision_check_resolution=0.02, seed=7),
    )

    assert result.success
    assert any(abs(float(q[1])) > 0.5 for q in result.waypoints)


def test_plan_rrt_connect_returns_cancelled_when_cancel_is_already_set() -> None:
    """A Cancel click during a long search must interrupt it -- checked once
    per iteration, so an already-set event stops the search before it does
    any real work at all, rather than running the full max_iters budget."""
    lo, hi = joint_bounds_from_context(FAKE_CONTEXT)
    start = lo.copy()
    goal = hi.copy()
    cancel = threading.Event()
    cancel.set()

    result = plan_rrt_connect(
        start_q=start,
        goal_q=goal,
        context=FAKE_CONTEXT,
        validity_fn=_always_valid,
        config=RrtConfig(max_iters=10000, seed=1),
        cancel=cancel,
    )

    assert not result.success
    assert result.reason == "cancelled"
    assert result.waypoints == []
    assert result.iterations == 0


def test_plan_rrt_connect_ignores_cancel_when_not_set() -> None:
    lo, hi = joint_bounds_from_context(FAKE_CONTEXT)
    start = lo + 0.2 * (hi - lo)
    goal = lo + 0.8 * (hi - lo)
    cancel = threading.Event()

    result = plan_rrt_connect(
        start_q=start,
        goal_q=goal,
        context=FAKE_CONTEXT,
        validity_fn=_always_valid,
        config=RrtConfig(max_iters=500, seed=1),
        cancel=cancel,
    )

    assert result.success


def test_shortcut_path_collapses_unnecessary_detour_when_unobstructed() -> None:
    lo, hi = joint_bounds_from_context(FAKE_CONTEXT)
    start = lo + 0.1 * (hi - lo)
    end = lo + 0.9 * (hi - lo)
    zigzag = [
        start,
        start + 0.3 * (end - start) + np.array([0.01, -0.2, 0.1, -0.1]),
        start + 0.6 * (end - start) + np.array([-0.01, 0.2, -0.1, 0.1]),
        end,
    ]
    shortened = shortcut_path(zigzag, context=FAKE_CONTEXT, validity_fn=_always_valid, iterations=200, seed=3)
    assert len(shortened) == 2
    assert shortened[0] == pytest.approx(start)
    assert shortened[-1] == pytest.approx(end)


def test_shortcut_path_stops_early_when_cancel_is_already_set() -> None:
    lo, hi = joint_bounds_from_context(FAKE_CONTEXT)
    start = lo + 0.1 * (hi - lo)
    end = lo + 0.9 * (hi - lo)
    zigzag = [
        start,
        start + 0.3 * (end - start) + np.array([0.01, -0.2, 0.1, -0.1]),
        start + 0.6 * (end - start) + np.array([-0.01, 0.2, -0.1, 0.1]),
        end,
    ]
    cancel = threading.Event()
    cancel.set()
    shortened = shortcut_path(
        zigzag, context=FAKE_CONTEXT, validity_fn=_always_valid, iterations=200, seed=3, cancel=cancel
    )
    # Cancelled before the first iteration -- path comes back untouched.
    assert len(shortened) == len(zigzag)
    for a, b in zip(shortened, zigzag):
        assert a == pytest.approx(b)


def test_shortcut_path_never_introduces_a_wall_violation() -> None:
    lo, hi = joint_bounds_from_context(FAKE_CONTEXT)
    start = np.array([lo[0], 0.0, 0.0, 0.0], dtype=float)
    goal = np.array([hi[0], 0.0, 0.0, 0.0], dtype=float)
    result = plan_rrt_connect(
        start_q=start,
        goal_q=goal,
        context=FAKE_CONTEXT,
        validity_fn=_wall_with_roll_gap,
        config=RrtConfig(max_iters=4000, step_size=0.1, collision_check_resolution=0.02, seed=7),
    )
    assert result.success

    shortened = shortcut_path(
        result.waypoints,
        context=FAKE_CONTEXT,
        validity_fn=_wall_with_roll_gap,
        iterations=300,
        resolution=0.02,
        seed=11,
    )

    span = hi - lo
    for a, b in zip(shortened[:-1], shortened[1:]):
        steps = 20
        for i in range(steps + 1):
            t = i / steps
            q = a + t * (b - a)
            assert _wall_with_roll_gap(q)


def _model_with_box_touching_node9(ik_context: dict, *, gap_m: float) -> tuple[CollisionModel, float]:
    """A default_radius=0.001 model (see test_collision.py's tiny-default-radius
    convention -- avoids spurious self-collision from treating every link as a
    0.03m point) plus one obstacle box positioned exactly ``gap_m`` past
    node9's Q_NEUTRAL position along X, small enough (0.02m half-extent) not
    to also reach node8 or gripper_base, both 0.05m further away."""
    node9_x = 0.478
    default_radius = 0.001
    box_half_x = 0.02
    box_center_x = node9_x + default_radius + gap_m + box_half_x
    model = CollisionModel(
        link_capsules={},
        default_radius=default_radius,
        obstacle_boxes=(
            WorldBox(
                center_world=(box_center_x, 0.0, 0.155),
                half_extents_world=(box_half_x, 0.3, 0.3),
                label="wall",
            ),
        ),
    )
    return model, node9_x


def test_maximize_clearance_improves_a_tight_interior_waypoint(ik_context: dict) -> None:
    model, _node9_x = _model_with_box_touching_node9(ik_context, gap_m=0.002)
    baseline = check_configuration(context=ik_context, q=Q_NEUTRAL, model=model)
    assert baseline.ok is True
    assert baseline.min_clearance_m == pytest.approx(0.002)

    # Neighbors retracted (linear=-0.1) rather than pinned at the same tight
    # Q_NEUTRAL point: retracting shifts every downstream node ~10cm back
    # along X (confirmed: node9 lands at x=0.378, comfortably clear of the
    # box at x>=0.481), giving the segment-to-neighbor check real room to
    # accept an improving candidate. Using Q_NEUTRAL on *all three* points
    # (tried first) pins both neighbors exactly at the 2mm boundary too,
    # which made almost every improving candidate's connecting segment fail
    # -- a degenerate, not representative, setup.
    retracted = np.array([-0.1, 0.0, 0.0, 0.0])
    waypoints = [retracted.copy(), Q_NEUTRAL.copy(), retracted.copy()]
    # A generous search budget (production tunes these down for speed on
    # real, many-waypoint paths -- see maximize_clearance's docstring) so
    # this test reliably demonstrates the mechanism regardless of that
    # speed/thoroughness tuning.
    smoothed = maximize_clearance(
        waypoints, context=ik_context, model=model, iterations=25, candidates_per_waypoint=20, seed=0
    )

    assert np.allclose(smoothed[0], retracted)
    assert np.allclose(smoothed[-1], retracted)
    improved = check_configuration(context=ik_context, q=smoothed[1], model=model)
    assert improved.ok is True
    assert improved.min_clearance_m > baseline.min_clearance_m


def test_maximize_clearance_stops_early_when_cancel_is_already_set(ik_context: dict) -> None:
    model, _node9_x = _model_with_box_touching_node9(ik_context, gap_m=0.002)
    retracted = np.array([-0.1, 0.0, 0.0, 0.0])
    waypoints = [retracted.copy(), Q_NEUTRAL.copy(), retracted.copy()]
    cancel = threading.Event()
    cancel.set()

    smoothed = maximize_clearance(
        waypoints,
        context=ik_context,
        model=model,
        iterations=25,
        candidates_per_waypoint=20,
        seed=0,
        cancel=cancel,
    )

    # Cancelled before the first round -- waypoints come back untouched.
    for got, expected in zip(smoothed, waypoints):
        assert np.allclose(got, expected)


def test_maximize_clearance_never_moves_the_start_or_goal(ik_context: dict) -> None:
    model, _node9_x = _model_with_box_touching_node9(ik_context, gap_m=0.002)
    waypoints = [Q_NEUTRAL.copy(), Q_NEUTRAL.copy(), Q_NEUTRAL.copy(), Q_NEUTRAL.copy()]
    smoothed = maximize_clearance(waypoints, context=ik_context, model=model, iterations=10, seed=2)
    assert np.allclose(smoothed[0], Q_NEUTRAL)
    assert np.allclose(smoothed[-1], Q_NEUTRAL)


def test_maximize_clearance_is_a_noop_for_a_two_point_path(ik_context: dict) -> None:
    model, _node9_x = _model_with_box_touching_node9(ik_context, gap_m=0.002)
    waypoints = [Q_NEUTRAL.copy(), Q_NEUTRAL.copy()]
    smoothed = maximize_clearance(waypoints, context=ik_context, model=model, iterations=10, seed=0)
    assert len(smoothed) == 2
    assert np.allclose(smoothed[0], Q_NEUTRAL)
    assert np.allclose(smoothed[1], Q_NEUTRAL)
