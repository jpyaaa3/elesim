"""Kinematic grasp approach trajectory for continuum guided grasp.

Feasible planning chains short IK steps from the current FK tip toward the object
(look-at-object axis), not a Cartesian lerp chord.  ``grasp_nominal`` is the
pre-contact point ``object - approach_dir * grasp_standoff_m``, not the object center.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Callable, Optional, Protocol, Sequence

if TYPE_CHECKING:
    from engine.profile.pick_timing import GraspPlanStats

import numpy as np


class IkPlanResult(Protocol):
    success: bool
    q: Any
    position_error_m: float
    direction_angle_rad: float
    reason: str


class FkTipResult(Protocol):
    position_world: tuple[float, float, float]
    direction_world: tuple[float, float, float]


@dataclass(frozen=True)
class GraspWaypoint:
    position_world: tuple[float, float, float]
    direction_world: tuple[float, float, float]
    standoff_m: float
    q_seed: tuple[float, float, float, float] | None = None
    achieved_position_world: tuple[float, float, float] | None = None


def _direction_angle_rad(a: Sequence[float], b: Sequence[float]) -> float:
    a_u = _unit_vec3(a)
    b_u = _unit_vec3(b)
    return float(np.arccos(float(np.clip(float(np.dot(a_u, b_u)), -1.0, 1.0))))


def _unit_vec3(vec: Sequence[float]) -> np.ndarray:
    v = np.asarray(vec, dtype=float).reshape(3)
    norm = float(np.linalg.norm(v))
    if norm <= 1e-9:
        raise ValueError("direction must be nonzero")
    return v / norm


def _axial_dist_to_end(
    position: np.ndarray,
    end_position: np.ndarray,
    direction: np.ndarray,
) -> float:
    return float(np.dot(end_position - position, _unit_vec3(direction)))


def _standoff_at(
    position: np.ndarray,
    object_world: np.ndarray,
    direction: np.ndarray,
) -> float:
    return float(np.dot(object_world - position, _unit_vec3(direction)))


def _look_at_object_dir(
    position: Sequence[float] | np.ndarray,
    object_world: Sequence[float] | np.ndarray,
) -> np.ndarray:
    """Unit direction from ``position`` toward the target object (Look-style view axis)."""
    obj = np.asarray(object_world, dtype=float).reshape(3)
    pos = np.asarray(position, dtype=float).reshape(3)
    return _unit_vec3(obj - pos)


def plan_grasp_approach_trajectory(
    *,
    start_position: Sequence[float],
    end_position: Sequence[float],
    start_direction: Sequence[float],
    end_direction: Sequence[float],
    object_world: Sequence[float],
    step_m: float,
    blind_start_m: float,
    grasp_standoff_m: float = 0.0,
    max_waypoints: int = 20,
) -> list[GraspWaypoint]:
    """Geometric reference polyline: repeated look-at-object steps without IK.

    Each step advances along the view axis from the previous point (FK-free preview).
    """
    _ = float(grasp_standoff_m)
    _ = _unit_vec3(start_direction)
    approach_axis = _unit_vec3(end_direction)
    step = float(max(step_m, 0.005))
    blind = float(max(blind_start_m, 0.0))
    cap = max(1, int(max_waypoints))
    pos = np.asarray(start_position, dtype=float).reshape(3).copy()
    end = np.asarray(end_position, dtype=float).reshape(3)
    obj = np.asarray(object_world, dtype=float).reshape(3)

    waypoints: list[GraspWaypoint] = []
    prev_standoff: float | None = None
    while len(waypoints) < cap:
        direction = _look_at_object_dir(pos, obj)
        axial = _axial_dist_to_end(pos, end, approach_axis)
        if axial <= blind + 1e-4:
            break
        travel = min(step, axial - blind)
        if travel < 0.005 - 1e-6:
            break
        nxt = pos + direction * travel
        standoff = _standoff_at(nxt, obj, direction)
        if prev_standoff is not None and standoff > prev_standoff + 1e-4:
            break
        prev_standoff = standoff
        look_dir = _look_at_object_dir(nxt, obj)
        waypoints.append(
            GraspWaypoint(
                position_world=(float(nxt[0]), float(nxt[1]), float(nxt[2])),
                direction_world=(
                    float(look_dir[0]),
                    float(look_dir[1]),
                    float(look_dir[2]),
                ),
                standoff_m=float(standoff),
            )
        )
        pos = nxt

    return waypoints


def _waypoint_from_pose(
    *,
    pos: np.ndarray,
    direction: np.ndarray,
    object_world: np.ndarray,
    approach_axis: np.ndarray,
    q_seed: np.ndarray | None = None,
    achieved_position: np.ndarray | None = None,
) -> GraspWaypoint:
    q_tuple: tuple[float, float, float, float] | None = None
    if q_seed is not None:
        q_arr = np.asarray(q_seed, dtype=float).reshape(4)
        q_tuple = (
            float(q_arr[0]),
            float(q_arr[1]),
            float(q_arr[2]),
            float(q_arr[3]),
        )
    achieved_tuple: tuple[float, float, float] | None = None
    if achieved_position is not None:
        achieved_arr = np.asarray(achieved_position, dtype=float).reshape(3)
        achieved_tuple = (
            float(achieved_arr[0]),
            float(achieved_arr[1]),
            float(achieved_arr[2]),
        )
    standoff = _standoff_at(pos, object_world, approach_axis)
    direc = _unit_vec3(direction)
    return GraspWaypoint(
        position_world=(float(pos[0]), float(pos[1]), float(pos[2])),
        direction_world=(
            float(direc[0]),
            float(direc[1]),
            float(direc[2]),
        ),
        standoff_m=float(standoff),
        q_seed=q_tuple,
        achieved_position_world=achieved_tuple,
    )


def _waypoint_from_fk(
    *,
    fk: FkTipResult,
    object_world: np.ndarray,
    q_seed: np.ndarray,
) -> GraspWaypoint:
    """Build a waypoint from FK achieved tip pose (executable, not lerp geometry)."""
    pos = np.asarray(fk.position_world, dtype=float).reshape(3)
    look_dir = _look_at_object_dir(pos, object_world)
    return _waypoint_from_pose(
        pos=pos,
        direction=look_dir,
        object_world=object_world,
        approach_axis=look_dir,
        q_seed=q_seed,
        achieved_position=pos,
    )


def _lateral_offset_candidates(
    direction: np.ndarray,
    *,
    offset_m: float,
) -> list[np.ndarray]:
    if offset_m <= 1e-9:
        return []
    d = _unit_vec3(direction)
    ref = np.array([0.0, 0.0, 1.0], dtype=float)
    if abs(float(np.dot(ref, d))) > 0.95:
        ref = np.array([0.0, 1.0, 0.0], dtype=float)
    lateral0 = np.cross(d, ref)
    n0 = float(np.linalg.norm(lateral0))
    if n0 <= 1e-9:
        return []
    lateral0 = lateral0 / n0
    lateral1 = np.cross(d, lateral0)
    n1 = float(np.linalg.norm(lateral1))
    if n1 <= 1e-9:
        return [lateral0 * offset_m]
    lateral1 = lateral1 / n1
    return [
        lateral0 * offset_m,
        -lateral0 * offset_m,
        lateral1 * offset_m,
        -lateral1 * offset_m,
    ]


def _try_ik_at_pose(
    *,
    pos: np.ndarray,
    direction: np.ndarray,
    q_seed: np.ndarray,
    ik_fn: Callable[..., IkPlanResult],
    ik_kwargs: dict[str, Any],
) -> IkPlanResult | None:
    try:
        return ik_fn(
            target_world=(float(pos[0]), float(pos[1]), float(pos[2])),
            target_dir_world=(
                float(direction[0]),
                float(direction[1]),
                float(direction[2]),
            ),
            current_seed=np.asarray(q_seed, dtype=float).reshape(4),
            **ik_kwargs,
        )
    except Exception:
        return None


def _attempt_feasible_ik(
    *,
    pos: np.ndarray,
    desired_dir: np.ndarray,
    q_seed: np.ndarray,
    object_world: np.ndarray,
    approach_axis: np.ndarray,
    ik_fn: Callable[..., IkPlanResult],
    fk_fn: Callable[[np.ndarray], FkTipResult],
    ik_kwargs: dict[str, Any],
    max_dir_error_rad: float,
    max_approach_drift_rad: float,
    stats: Optional["GraspPlanStats"] = None,
) -> GraspWaypoint | None:
    """Try IK with look-at-object dir, then dir-hold; accept only FK-achievable poses."""
    if stats is not None:
        stats.feasible_ik_attempts += 1
    desired_u = _look_at_object_dir(pos, object_world)
    _ = _unit_vec3(approach_axis)
    dir_candidates: list[tuple[np.ndarray, bool]] = [(desired_u, True)]

    fk_seed = fk_fn(q_seed)
    dir_seed = np.asarray(fk_seed.direction_world, dtype=float).reshape(3)
    if float(np.linalg.norm(dir_seed)) > 1e-9:
        hold_u = _unit_vec3(dir_seed)
        if float(np.dot(hold_u, desired_u)) < 0.9999:
            dir_candidates.append((hold_u, True))

    for dir_try, enforce_dir_gate in dir_candidates:
        result = _try_ik_at_pose(
            pos=pos,
            direction=dir_try,
            q_seed=q_seed,
            ik_fn=ik_fn,
            ik_kwargs=ik_kwargs,
        )
        if result is None or not bool(result.success) or result.q is None:
            continue
        dir_err = float(getattr(result, "direction_angle_rad", 0.0))
        if enforce_dir_gate and dir_err > float(max_dir_error_rad):
            continue
        q_arr = np.asarray(result.q, dtype=float).reshape(4)
        fk = fk_fn(q_arr)
        fk_dir = np.asarray(fk.direction_world, dtype=float).reshape(3)
        if float(np.linalg.norm(fk_dir)) <= 1e-9:
            continue
        look_dir = _look_at_object_dir(fk.position_world, object_world)
        drift = _direction_angle_rad(fk_dir, look_dir)
        if drift > float(max_approach_drift_rad):
            continue
        return _waypoint_from_fk(
            fk=fk,
            object_world=object_world,
            q_seed=q_arr,
        )
    return None


def _kinematic_step_at_travel(
    *,
    tip: np.ndarray,
    travel_m: float,
    object_world: np.ndarray,
    nominal_world: np.ndarray,
    approach_axis: np.ndarray,
    q_seed: np.ndarray,
    blind: float,
    prev_standoff: float | None,
    prev_axial: float,
    ik_fn: Callable[..., IkPlanResult],
    fk_fn: Callable[[np.ndarray], FkTipResult],
    ik_kwargs: dict[str, Any],
    min_step_m: float,
    lateral_offset_m: float,
    max_dir_error_rad: float,
    max_approach_drift_rad: float,
    stats: Optional["GraspPlanStats"] = None,
) -> GraspWaypoint | None:
    """One kinematic step: advance along look-at-object from FK tip, bisect travel on IK fail."""
    look_dir = _look_at_object_dir(tip, object_world)
    travel_hi = float(max(travel_m, 0.0))
    travel_lo = 0.0
    min_travel = float(max(min_step_m, 0.005))
    best: GraspWaypoint | None = None

    for _ in range(12):
        if stats is not None:
            stats.bisect_iters += 1
        if travel_hi <= travel_lo + 1e-9:
            break
        if travel_hi - travel_lo < min_travel - 1e-6:
            break

        travel_try = float(travel_hi)
        pos = tip + look_dir * travel_try
        axial_target = _axial_dist_to_end(pos, nominal_world, approach_axis)
        if axial_target <= blind + 1e-4:
            travel_hi = 0.5 * (travel_lo + travel_hi)
            continue

        direction = _look_at_object_dir(pos, object_world)
        standoff_target = _standoff_at(pos, object_world, direction)
        if prev_standoff is not None and standoff_target > prev_standoff + 1e-4:
            travel_hi = 0.5 * (travel_lo + travel_hi)
            continue

        best = _attempt_feasible_ik(
            pos=pos,
            desired_dir=direction,
            q_seed=q_seed,
            object_world=object_world,
            approach_axis=approach_axis,
            ik_fn=ik_fn,
            fk_fn=fk_fn,
            ik_kwargs=ik_kwargs,
            max_dir_error_rad=max_dir_error_rad,
            max_approach_drift_rad=max_approach_drift_rad,
            stats=stats,
        )
        if best is None:
            offsets = _lateral_offset_candidates(
                direction,
                offset_m=float(lateral_offset_m),
            )
            for off in offsets:
                if stats is not None:
                    stats.lateral_tries += 1
                best = _attempt_feasible_ik(
                    pos=pos + off,
                    desired_dir=direction,
                    q_seed=q_seed,
                    object_world=object_world,
                    approach_axis=approach_axis,
                    ik_fn=ik_fn,
                    fk_fn=fk_fn,
                    ik_kwargs=ik_kwargs,
                    max_dir_error_rad=max_dir_error_rad,
                    max_approach_drift_rad=max_approach_drift_rad,
                    stats=stats,
                )
                if best is not None:
                    break

        if best is not None:
            achieved = np.asarray(
                best.achieved_position_world or best.position_world,
                dtype=float,
            ).reshape(3)
            new_axial = _axial_dist_to_end(achieved, nominal_world, approach_axis)
            if new_axial >= float(prev_axial) - 1e-4:
                best = None
                travel_hi = 0.5 * (travel_lo + travel_hi)
                continue
            if prev_standoff is not None and float(best.standoff_m) > prev_standoff + 1e-4:
                best = None
                travel_hi = 0.5 * (travel_lo + travel_hi)
                continue
            break

        travel_hi = 0.5 * (travel_lo + travel_hi)

    return best


def plan_grasp_kinematic_trajectory(
    *,
    q_seed: Sequence[float],
    nominal_world: Sequence[float],
    object_world: Sequence[float],
    approach_axis: Sequence[float],
    step_m: float,
    blind_start_m: float,
    ik_fn: Callable[..., IkPlanResult],
    fk_fn: Callable[[np.ndarray], FkTipResult],
    ik_kwargs: dict[str, Any] | None = None,
    max_waypoints: int = 20,
    min_step_m: float = 0.005,
    lateral_offset_m: float = 0.003,
    max_dir_error_deg: float = 12.0,
    max_approach_drift_deg: float = 18.0,
    stats: Optional["GraspPlanStats"] = None,
) -> list[GraspWaypoint]:
    """Chain IK steps from the current FK tip toward ``nominal_world`` (no Cartesian lerp)."""
    step = float(max(step_m, 0.005))
    blind = float(max(blind_start_m, 0.0))
    cap = max(1, int(max_waypoints))
    max_dir_error_rad = float(np.deg2rad(max(0.0, max_dir_error_deg)))
    max_approach_drift_rad = float(np.deg2rad(max(0.0, max_approach_drift_deg)))
    nominal = np.asarray(nominal_world, dtype=float).reshape(3)
    obj = np.asarray(object_world, dtype=float).reshape(3)
    axis = _unit_vec3(approach_axis)
    q_prev = np.asarray(q_seed, dtype=float).reshape(4).copy()
    kwargs = dict(ik_kwargs or {})

    waypoints: list[GraspWaypoint] = []
    prev_standoff: float | None = None
    while len(waypoints) < cap:
        fk = fk_fn(q_prev)
        tip = np.asarray(fk.position_world, dtype=float).reshape(3)
        axial = _axial_dist_to_end(tip, nominal, axis)
        if axial <= blind + 1e-4:
            break

        travel = min(step, axial - blind)
        if travel < float(min_step_m) - 1e-6:
            break

        wp = _kinematic_step_at_travel(
            tip=tip,
            travel_m=travel,
            object_world=obj,
            nominal_world=nominal,
            approach_axis=axis,
            q_seed=q_prev,
            blind=blind,
            prev_standoff=prev_standoff,
            prev_axial=axial,
            ik_fn=ik_fn,
            fk_fn=fk_fn,
            ik_kwargs=kwargs,
            min_step_m=float(min_step_m),
            lateral_offset_m=float(lateral_offset_m),
            max_dir_error_rad=max_dir_error_rad,
            max_approach_drift_rad=max_approach_drift_rad,
            stats=stats,
        )
        if wp is None:
            if stats is not None:
                stats.kinematic_steps_fail += 1
            break

        if stats is not None:
            stats.kinematic_steps_ok += 1
        waypoints.append(wp)
        prev_standoff = float(wp.standoff_m)
        if wp.q_seed is not None:
            q_prev = np.asarray(wp.q_seed, dtype=float).reshape(4)

    return waypoints


def plan_grasp_feasible_trajectory(
    *,
    start_position: Sequence[float],
    end_position: Sequence[float],
    start_direction: Sequence[float],
    end_direction: Sequence[float],
    object_world: Sequence[float],
    q_seed: Sequence[float],
    step_m: float,
    blind_start_m: float,
    ik_fn: Callable[..., IkPlanResult],
    fk_fn: Callable[[np.ndarray], FkTipResult],
    ik_kwargs: dict[str, Any] | None = None,
    grasp_standoff_m: float = 0.0,
    max_waypoints: int = 20,
    min_step_m: float = 0.005,
    lateral_offset_m: float = 0.003,
    max_dir_error_deg: float = 12.0,
    max_approach_drift_deg: float = 18.0,
    stats: Optional["GraspPlanStats"] = None,
) -> list[GraspWaypoint]:
    """Kinematic IK chain from ``q_seed`` FK tip; Cartesian start/end are reference only."""
    _ = start_position
    _ = start_direction
    _ = float(grasp_standoff_m)
    return plan_grasp_kinematic_trajectory(
        q_seed=q_seed,
        nominal_world=end_position,
        object_world=object_world,
        approach_axis=end_direction,
        step_m=step_m,
        blind_start_m=blind_start_m,
        ik_fn=ik_fn,
        fk_fn=fk_fn,
        ik_kwargs=ik_kwargs,
        max_waypoints=max_waypoints,
        min_step_m=min_step_m,
        lateral_offset_m=lateral_offset_m,
        max_dir_error_deg=max_dir_error_deg,
        max_approach_drift_deg=max_approach_drift_deg,
        stats=stats,
    )


def plan_grasp_feasible_next_waypoint(
    *,
    start_position: Sequence[float],
    end_position: Sequence[float],
    start_direction: Sequence[float],
    end_direction: Sequence[float],
    object_world: Sequence[float],
    q_seed: Sequence[float],
    step_m: float,
    blind_start_m: float,
    ik_fn: Callable[..., IkPlanResult],
    fk_fn: Callable[[np.ndarray], FkTipResult],
    ik_kwargs: dict[str, Any] | None = None,
    grasp_standoff_m: float = 0.0,
    min_step_m: float = 0.005,
    lateral_offset_m: float = 0.003,
    max_dir_error_deg: float = 12.0,
    max_approach_drift_deg: float = 18.0,
) -> GraspWaypoint | None:
    """Single receding-horizon feasible waypoint from ``start_position``."""
    planned = plan_grasp_feasible_trajectory(
        start_position=start_position,
        end_position=end_position,
        start_direction=start_direction,
        end_direction=end_direction,
        object_world=object_world,
        q_seed=q_seed,
        step_m=step_m,
        blind_start_m=blind_start_m,
        ik_fn=ik_fn,
        fk_fn=fk_fn,
        ik_kwargs=ik_kwargs,
        grasp_standoff_m=grasp_standoff_m,
        max_waypoints=1,
        min_step_m=min_step_m,
        lateral_offset_m=lateral_offset_m,
        max_dir_error_deg=max_dir_error_deg,
        max_approach_drift_deg=max_approach_drift_deg,
    )
    if not planned:
        return None
    return planned[0]


def plan_grasp_next_waypoint(
    *,
    start_position: Sequence[float],
    end_position: Sequence[float],
    start_direction: Sequence[float],
    end_direction: Sequence[float],
    object_world: Sequence[float],
    step_m: float,
    blind_start_m: float,
    grasp_standoff_m: float = 0.0,
) -> GraspWaypoint | None:
    """Return the next single waypoint along the approach curve (receding horizon)."""
    planned = plan_grasp_approach_trajectory(
        start_position=start_position,
        end_position=end_position,
        start_direction=start_direction,
        end_direction=end_direction,
        object_world=object_world,
        step_m=step_m,
        blind_start_m=blind_start_m,
        grasp_standoff_m=grasp_standoff_m,
        max_waypoints=1,
    )
    if not planned:
        return None
    return planned[0]


def trajectory_path_length_m(
    waypoints: Sequence[GraspWaypoint],
    *,
    start_position: Sequence[float] | None = None,
) -> float:
    chain: list[np.ndarray] = []
    if start_position is not None:
        chain.append(np.asarray(start_position, dtype=float).reshape(3))
    for wp in waypoints:
        chain.append(np.asarray(wp.position_world, dtype=float).reshape(3))
    if len(chain) < 2:
        return 0.0
    total = 0.0
    for idx in range(1, len(chain)):
        total += float(np.linalg.norm(chain[idx] - chain[idx - 1]))
    return total


def _segment_marker(
    *,
    name: str,
    start: np.ndarray,
    end: np.ndarray,
    color: list[float],
    ttl_ms: int,
) -> dict[str, Any] | None:
    delta = end - start
    length = float(np.linalg.norm(delta))
    if length <= 1e-6:
        return None
    return {
        "name": str(name),
        "frame": "world",
        "pos": [float(start[0]), float(start[1]), float(start[2])],
        "dir": [float(delta[0]), float(delta[1]), float(delta[2])],
        "color": list(color),
        "radius": 0.004,
        "length": length,
        "ttl_ms": int(ttl_ms),
    }


def build_grasp_trajectory_markers(
    *,
    start_position: Sequence[float],
    end_position: Sequence[float],
    object_world: Sequence[float],
    waypoints: Sequence[GraspWaypoint],
    highlight_idx: int = -1,
    look_anchor_position: Sequence[float] | None = None,
    ttl_ms: int = 120000,
) -> list[dict[str, Any]]:
    """Debug markers: FK kinematic polyline (start→waypoints), not a chord to nominal."""
    ttl = int(max(1000, ttl_ms))
    start = np.asarray(start_position, dtype=float).reshape(3)
    end = np.asarray(end_position, dtype=float).reshape(3)
    obj = np.asarray(object_world, dtype=float).reshape(3)
    standoff_vec = obj - end
    standoff_len = float(np.linalg.norm(standoff_vec))
    markers: list[dict[str, Any]] = [
        {
            "name": "grasp_traj_object",
            "frame": "world",
            "pos": [float(obj[0]), float(obj[1]), float(obj[2])],
            "color": [1.0, 0.92, 0.15, 0.55],
            "radius": 0.009,
            "ttl_ms": ttl,
        },
        {
            "name": "grasp_traj_start",
            "frame": "world",
            "pos": [float(start[0]), float(start[1]), float(start[2])],
            "color": [0.25, 1.0, 0.40, 0.95],
            "radius": 0.015,
            "ttl_ms": ttl,
        },
    ]
    if look_anchor_position is not None:
        look = np.asarray(look_anchor_position, dtype=float).reshape(3)
        markers.append(
            {
                "name": "grasp_traj_look_anchor",
                "frame": "world",
                "pos": [float(look[0]), float(look[1]), float(look[2])],
                "color": [0.35, 0.85, 1.0, 0.55],
                "radius": 0.010,
                "ttl_ms": ttl,
            }
        )
    markers.extend(
        [
        {
            "name": "grasp_traj_nominal",
            "frame": "world",
            "pos": [float(end[0]), float(end[1]), float(end[2])],
            "color": [1.0, 0.72, 0.12, 0.95],
            "radius": 0.015,
            "ttl_ms": ttl,
        },
        ]
    )
    if standoff_len > 1e-4:
        markers.append(
            {
                "name": "grasp_traj_standoff",
                "frame": "world",
                "pos": [float(end[0]), float(end[1]), float(end[2])],
                "dir": [float(v) for v in standoff_vec],
                "color": [1.0, 0.55, 0.05, 0.70],
                "radius": 0.005,
                "length": standoff_len,
                "ttl_ms": ttl,
            }
        )

    chain: list[np.ndarray] = [start]
    for wp in waypoints:
        chain.append(np.asarray(wp.position_world, dtype=float).reshape(3))

    for idx, wp in enumerate(waypoints):
        pos = np.asarray(wp.position_world, dtype=float).reshape(3)
        active = int(idx) == int(highlight_idx)
        markers.append(
            {
                "name": "grasp_traj_wp_%02d" % int(idx),
                "frame": "world",
                "pos": [float(pos[0]), float(pos[1]), float(pos[2])],
                "dir": [float(v) for v in wp.direction_world],
                "color": (
                    [1.0, 0.35, 1.0, 0.98]
                    if active
                    else [0.72, 0.38, 1.0, 0.88]
                ),
                "radius": 0.013 if active else 0.009,
                "length": 0.06,
                "ttl_ms": ttl,
            }
        )

    seg_color = [0.55, 0.42, 1.0, 0.62]
    for seg_idx in range(len(chain) - 1):
        seg = _segment_marker(
            name="grasp_traj_seg_%02d" % int(seg_idx),
            start=chain[seg_idx],
            end=chain[seg_idx + 1],
            color=seg_color,
            ttl_ms=ttl,
        )
        if seg is not None:
            markers.append(seg)

    return markers
