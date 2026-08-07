"""Joint-space RRT-Connect planner for the 4-DOF arm.

Plans directly in q-space (linear, roll, theta1, theta2) rather than
Cartesian space: with only 4 dimensions and hard joint bounds, uniform
q-space sampling is cheap and avoids running inverse kinematics per sample.
The planner itself is generic -- it takes a validity predicate rather than
importing collision.py directly, so it can be unit-tested against toy
obstacles without a real arm model.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, Optional, Sequence

import numpy as np

from elesim_controller.robot.arm.iklib.kinematics import _ReachModel
from elesim_controller.robot.arm.planning.collision import CollisionModel, check_configuration

ValidityCheck = Callable[[np.ndarray], bool]

_DOF = 4


def joint_bounds_from_context(context: Mapping[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    model = _ReachModel(context=dict(context), limit=context["limit"])
    lo = np.array([model.linear_min, model.roll_min, -model.bend_lim, -model.bend_lim], dtype=float)
    hi = np.array([model.linear_max, model.roll_max, model.bend_lim, model.bend_lim], dtype=float)
    return lo, hi


def make_collision_validity_fn(
    *,
    context: Mapping[str, Any],
    model: CollisionModel,
    go2_pos: Optional[Sequence[float]] = None,
    go2_rpy_rad: Optional[Sequence[float]] = None,
    leg_q: Optional[Sequence[float]] = None,
    clearance_m: float = 0.0,
    environment_clearance_m: Optional[float] = None,
) -> ValidityCheck:
    """``leg_q`` (GO2's live 12-value leg joint vector) is held fixed for the whole
    search -- a planning-time snapshot, same as ``go2_pos``/``go2_rpy_rad`` already are;
    it does not model the legs continuing to walk while the arm moves.
    See ``check_configuration`` for ``environment_clearance_m``."""

    def _valid(q: np.ndarray) -> bool:
        result = check_configuration(
            context=context,
            q=q,
            model=model,
            go2_pos=go2_pos,
            go2_rpy_rad=go2_rpy_rad,
            leg_q=leg_q,
            clearance_m=clearance_m,
            environment_clearance_m=environment_clearance_m,
        )
        return bool(result.ok)

    return _valid


@dataclass(frozen=True)
class RrtConfig:
    max_iters: int = 20000
    step_size: float = 0.02
    goal_bias: float = 0.1
    goal_tol: float = 1e-3
    # Both step_size and this are fractions of the *full joint range* (unit
    # cube, see joint_bounds_from_context), not meters -- a large-leverage
    # bend-angle change can move the tip several cm in Cartesian space per
    # unit-space step, so a coarse resolution can step clean over a thin
    # obstacle (confirmed live: a ~3cm-thick wall) without ever sampling a
    # point inside it. Kept fine enough that even a thin obstacle gets
    # actually sampled along every edge.
    collision_check_resolution: float = 0.01
    seed: int = 0
    shortcut_iters: int = 200


@dataclass(frozen=True)
class RrtResult:
    success: bool
    waypoints: list[np.ndarray]
    iterations: int
    reason: str = ""


@dataclass
class _Tree:
    nodes: list[np.ndarray] = field(default_factory=list)
    parents: list[int] = field(default_factory=list)

    def add(self, q_unit: np.ndarray, parent: int) -> int:
        self.nodes.append(q_unit)
        self.parents.append(parent)
        return len(self.nodes) - 1

    def nearest(self, q_unit: np.ndarray) -> int:
        stacked = np.stack(self.nodes, axis=0)
        dists = np.linalg.norm(stacked - q_unit.reshape(1, _DOF), axis=1)
        return int(np.argmin(dists))

    def path_to_root(self, index: int) -> list[np.ndarray]:
        out = []
        cursor: Optional[int] = index
        while cursor is not None:
            out.append(self.nodes[cursor])
            cursor = self.parents[cursor] if self.parents[cursor] >= 0 else None
        return list(reversed(out))


def _segment_is_valid(
    a_unit: np.ndarray,
    b_unit: np.ndarray,
    *,
    lo: np.ndarray,
    hi: np.ndarray,
    validity_fn: ValidityCheck,
    resolution: float,
) -> bool:
    span = float(np.linalg.norm(b_unit - a_unit))
    steps = max(1, int(np.ceil(span / max(resolution, 1e-6))))
    for i in range(1, steps + 1):
        t = float(i) / float(steps)
        q_unit = a_unit + t * (b_unit - a_unit)
        q_real = lo + q_unit * (hi - lo)
        if not validity_fn(q_real):
            return False
    return True


def _extend(
    tree: _Tree,
    target_unit: np.ndarray,
    *,
    lo: np.ndarray,
    hi: np.ndarray,
    validity_fn: ValidityCheck,
    step_size: float,
    resolution: float,
) -> tuple[str, int]:
    """Grow ``tree`` one bounded step toward ``target_unit``.

    Returns ("reached" | "advanced" | "trapped", new_node_index).
    """
    nearest_idx = tree.nearest(target_unit)
    nearest = tree.nodes[nearest_idx]
    delta = target_unit - nearest
    dist = float(np.linalg.norm(delta))
    if dist <= 1e-12:
        return "reached", nearest_idx
    if dist <= step_size:
        candidate = target_unit
        status = "reached"
    else:
        candidate = nearest + (delta / dist) * step_size
        status = "advanced"
    candidate = np.clip(candidate, 0.0, 1.0)
    if not _segment_is_valid(nearest, candidate, lo=lo, hi=hi, validity_fn=validity_fn, resolution=resolution):
        return "trapped", nearest_idx
    new_idx = tree.add(candidate, nearest_idx)
    return status, new_idx


def _connect(
    tree: _Tree,
    target_unit: np.ndarray,
    *,
    lo: np.ndarray,
    hi: np.ndarray,
    validity_fn: ValidityCheck,
    step_size: float,
    resolution: float,
) -> tuple[str, int]:
    status, idx = "advanced", -1
    while status == "advanced":
        status, idx = _extend(
            tree, target_unit, lo=lo, hi=hi, validity_fn=validity_fn, step_size=step_size, resolution=resolution
        )
    return status, idx


def plan_rrt_connect(
    *,
    start_q: Sequence[float],
    goal_q: Sequence[float],
    context: Mapping[str, Any],
    validity_fn: ValidityCheck,
    config: RrtConfig = RrtConfig(),
    cancel: Optional[threading.Event] = None,
) -> RrtResult:
    """Plan a collision-checked joint-space path from ``start_q`` to ``goal_q``.

    ``cancel``, if given, is checked once per iteration so a Cancel click can
    interrupt a long search (up to ``config.max_iters``) instead of forcing
    the caller to wait it out -- see ``PlannedMoveExecutor.generate``.
    """
    lo, hi = joint_bounds_from_context(context)
    span = hi - lo
    if np.any(span <= 0.0):
        return RrtResult(success=False, waypoints=[], iterations=0, reason="degenerate_joint_bounds")

    start_arr = np.asarray(start_q, dtype=float).reshape(_DOF)
    goal_arr = np.asarray(goal_q, dtype=float).reshape(_DOF)
    start_unit = np.clip((start_arr - lo) / span, 0.0, 1.0)
    goal_unit = np.clip((goal_arr - lo) / span, 0.0, 1.0)

    if not validity_fn(start_arr):
        return RrtResult(success=False, waypoints=[], iterations=0, reason="start_in_collision")
    if not validity_fn(goal_arr):
        return RrtResult(success=False, waypoints=[], iterations=0, reason="goal_in_collision")

    rng = np.random.default_rng(config.seed)
    tree_a = _Tree(nodes=[start_unit], parents=[-1])
    tree_b = _Tree(nodes=[goal_unit], parents=[-1])
    a_is_start = True

    for iteration in range(1, int(config.max_iters) + 1):
        if cancel is not None and cancel.is_set():
            return RrtResult(success=False, waypoints=[], iterations=iteration - 1, reason="cancelled")
        if rng.random() < config.goal_bias:
            sample = tree_b.nodes[0]
        else:
            sample = rng.uniform(0.0, 1.0, size=_DOF)

        status, new_idx = _extend(
            tree_a,
            sample,
            lo=lo,
            hi=hi,
            validity_fn=validity_fn,
            step_size=config.step_size,
            resolution=config.collision_check_resolution,
        )
        if status != "trapped":
            new_node = tree_a.nodes[new_idx]
            connect_status, connect_idx = _connect(
                tree_b,
                new_node,
                lo=lo,
                hi=hi,
                validity_fn=validity_fn,
                step_size=config.step_size,
                resolution=config.collision_check_resolution,
            )
            if connect_status == "reached":
                path_a = tree_a.path_to_root(new_idx)
                path_b = tree_b.path_to_root(connect_idx)
                if not a_is_start:
                    path_a, path_b = path_b, path_a
                units = path_a + list(reversed(path_b))
                waypoints = [lo + u * span for u in units]
                return RrtResult(success=True, waypoints=waypoints, iterations=iteration, reason="connected")

        tree_a, tree_b = tree_b, tree_a
        a_is_start = not a_is_start

    return RrtResult(success=False, waypoints=[], iterations=int(config.max_iters), reason="max_iters_exceeded")


def shortcut_path(
    waypoints: Sequence[np.ndarray],
    *,
    context: Mapping[str, Any],
    validity_fn: ValidityCheck,
    iterations: int = 200,
    resolution: float = 0.05,
    seed: int = 0,
    cancel: Optional[threading.Event] = None,
) -> list[np.ndarray]:
    """Random-shortcut post-processing: splice out detours that a straight segment can replace.

    ``cancel``, if set mid-loop, stops refinement early and returns whatever
    the path looks like so far -- the caller (``PlannedMoveExecutor.generate``)
    discards it and reports the whole generate as cancelled either way, so
    this only needs to exit promptly, not produce a usable partial result.
    """
    if len(waypoints) <= 2:
        return list(waypoints)
    lo, hi = joint_bounds_from_context(context)
    span = hi - lo
    path = [np.asarray(q, dtype=float).reshape(_DOF) for q in waypoints]
    rng = np.random.default_rng(seed)
    for _ in range(int(iterations)):
        if cancel is not None and cancel.is_set():
            break
        if len(path) <= 2:
            break
        i, j = sorted(rng.integers(0, len(path), size=2))
        if j - i < 2:
            continue
        a_unit = (path[i] - lo) / span
        b_unit = (path[j] - lo) / span
        if _segment_is_valid(a_unit, b_unit, lo=lo, hi=hi, validity_fn=validity_fn, resolution=resolution):
            path = path[: i + 1] + path[j:]
    return path


def maximize_clearance(
    waypoints: Sequence[np.ndarray],
    *,
    context: Mapping[str, Any],
    model: CollisionModel,
    go2_pos: Optional[Sequence[float]] = None,
    go2_rpy_rad: Optional[Sequence[float]] = None,
    leg_q: Optional[Sequence[float]] = None,
    environment_clearance_m: Optional[float] = None,
    iterations: int = 4,
    candidates_per_waypoint: int = 4,
    step_scale: float = 0.08,
    segment_resolution: float = 0.02,
    good_enough_clearance_m: float = 0.02,
    seed: int = 0,
    cancel: Optional[threading.Event] = None,
) -> list[np.ndarray]:
    """Nudge each *interior* waypoint toward locally higher clearance.

    Neither ``plan_rrt_connect`` nor ``shortcut_path`` optimizes for margin --
    a waypoint that just barely clears an obstacle (clearance ~0) is treated
    identically to one with centimeters to spare, as long as both are >= 0.
    Threading a narrow opening (e.g. a hole in a wall) this way finds *a*
    valid path that can trace right along the tight boundary the whole way
    through (confirmed live: a wall-threading path visibly grazed the wall
    face). This is a repeated local hill-climb, not a real gradient-based
    optimizer -- for each interior waypoint, try several random nearby
    joint-space perturbations, keep whichever one both (a) reports higher
    ``min_clearance_m`` via the same ``check_configuration`` the rest of
    collision checking already uses, and (b) keeps the segments to its
    (unmoved) neighbors valid, so nudging one point can't reopen a
    collision next to it. The path's start and goal are never touched --
    those are the arm's actual current pose and the specific configuration
    IK solved for the requested task-space target, not free variables.

    Skips any waypoint whose clearance is already >= ``good_enough_clearance_m``
    (nothing to fix there) and stops early once a whole round improves
    nothing -- a real, un-gated version of this loop measured ~90s on a
    23-waypoint path threading a tight opening (each candidate needs a full
    ``check_configuration`` plus up to two segment re-validations), which is
    far too slow for an interactive Generate click.

    ``cancel``, if set mid-loop, stops refinement early -- see
    ``shortcut_path``'s docstring for why a partial result here is fine.
    """
    if len(waypoints) <= 2:
        return list(waypoints)
    lo, hi = joint_bounds_from_context(context)
    span = hi - lo
    path = [np.asarray(q, dtype=float).reshape(_DOF) for q in waypoints]
    rng = np.random.default_rng(seed)
    validity_fn = make_collision_validity_fn(
        context=context,
        model=model,
        go2_pos=go2_pos,
        go2_rpy_rad=go2_rpy_rad,
        leg_q=leg_q,
        environment_clearance_m=environment_clearance_m,
    )

    def _clearance(q: np.ndarray) -> float:
        result = check_configuration(
            context=context,
            q=q,
            model=model,
            go2_pos=go2_pos,
            go2_rpy_rad=go2_rpy_rad,
            leg_q=leg_q,
            environment_clearance_m=environment_clearance_m,
        )
        return float(result.min_clearance_m) if result.ok else float("-inf")

    def _segment_ok(a: np.ndarray, b: np.ndarray) -> bool:
        a_unit = (a - lo) / span
        b_unit = (b - lo) / span
        return _segment_is_valid(a_unit, b_unit, lo=lo, hi=hi, validity_fn=validity_fn, resolution=segment_resolution)

    for _ in range(int(iterations)):
        if cancel is not None and cancel.is_set():
            break
        improved_any = False
        for i in range(1, len(path) - 1):
            original_clearance = _clearance(path[i])
            if original_clearance >= good_enough_clearance_m:
                continue
            best_q, best_clearance = path[i], original_clearance
            for _ in range(int(candidates_per_waypoint)):
                candidate = np.clip(best_q + rng.normal(scale=step_scale * span), lo, hi)
                candidate_clearance = _clearance(candidate)
                if candidate_clearance <= best_clearance:
                    continue
                if not _segment_ok(path[i - 1], candidate) or not _segment_ok(candidate, path[i + 1]):
                    continue
                best_q, best_clearance = candidate, candidate_clearance
            if best_clearance > original_clearance:
                improved_any = True
            path[i] = best_q
        if not improved_any:
            break
    return path


__all__ = [
    "RrtConfig",
    "RrtResult",
    "joint_bounds_from_context",
    "make_collision_validity_fn",
    "maximize_clearance",
    "plan_rrt_connect",
    "shortcut_path",
]
