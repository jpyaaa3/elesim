"""Planned 3D grasp approach trajectory (centered_ready → nominal) for guided grasp."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class GraspWaypoint:
    position_world: tuple[float, float, float]
    direction_world: tuple[float, float, float]
    standoff_m: float


def _unit_vec3(vec: Sequence[float]) -> np.ndarray:
    v = np.asarray(vec, dtype=float).reshape(3)
    norm = float(np.linalg.norm(v))
    if norm <= 1e-9:
        raise ValueError("direction must be nonzero")
    return v / norm


def _slerp_unit(v0: np.ndarray, v1: np.ndarray, t: float) -> np.ndarray:
    t = float(np.clip(t, 0.0, 1.0))
    a = _unit_vec3(v0)
    b = _unit_vec3(v1)
    dot = float(np.clip(float(np.dot(a, b)), -1.0, 1.0))
    if dot > 0.9999:
        out = (1.0 - t) * a + t * b
        return _unit_vec3(out)
    omega = float(np.arccos(dot))
    sin_omega = float(np.sin(omega))
    if sin_omega <= 1e-9:
        return a
    scale0 = float(np.sin((1.0 - t) * omega) / sin_omega)
    scale1 = float(np.sin(t * omega) / sin_omega)
    return _unit_vec3(scale0 * a + scale1 * b)


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
    """Sample waypoints along a 3D lerp path with slerped approach directions.

    Returns forward waypoints only (excludes ``start_position``). Stops before the
    blind zone: axial distance from each point to ``end_position`` must exceed
    ``blind_start_m``.
    """
    _ = float(grasp_standoff_m)
    step = float(max(step_m, 0.005))
    blind = float(max(blind_start_m, 0.0))
    cap = max(1, int(max_waypoints))
    start = np.asarray(start_position, dtype=float).reshape(3)
    end = np.asarray(end_position, dtype=float).reshape(3)
    obj = np.asarray(object_world, dtype=float).reshape(3)
    start_dir = _unit_vec3(start_direction)
    end_dir = _unit_vec3(end_direction)

    chord = end - start
    chord_len = float(np.linalg.norm(chord))
    if chord_len <= 1e-6:
        return []

    delta_s = step / chord_len
    if delta_s <= 1e-9:
        delta_s = 1.0

    waypoints: list[GraspWaypoint] = []
    s = delta_s
    prev_standoff: float | None = None
    while s <= 1.0 + 1e-9 and len(waypoints) < cap:
        pos = (1.0 - s) * start + s * end
        direction = _slerp_unit(start_dir, end_dir, s)
        axial = _axial_dist_to_end(pos, end, direction)
        if axial <= blind + 1e-4:
            break
        standoff = _standoff_at(pos, obj, direction)
        if prev_standoff is not None and standoff > prev_standoff + 1e-4:
            break
        prev_standoff = standoff
        waypoints.append(
            GraspWaypoint(
                position_world=(float(pos[0]), float(pos[1]), float(pos[2])),
                direction_world=(
                    float(direction[0]),
                    float(direction[1]),
                    float(direction[2]),
                ),
                standoff_m=float(standoff),
            )
        )
        s += delta_s

    return waypoints


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


def trajectory_path_length_m(waypoints: Sequence[GraspWaypoint]) -> float:
    if len(waypoints) < 2:
        return 0.0
    total = 0.0
    prev = np.asarray(waypoints[0].position_world, dtype=float).reshape(3)
    for wp in waypoints[1:]:
        cur = np.asarray(wp.position_world, dtype=float).reshape(3)
        total += float(np.linalg.norm(cur - prev))
        prev = cur
    return total
