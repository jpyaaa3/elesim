"""Grasp trajectory types and debug markers for online guided grasp."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np


@dataclass(frozen=True)
class GraspWaypoint:
    position_world: tuple[float, float, float]
    direction_world: tuple[float, float, float]
    standoff_m: float
    q_seed: tuple[float, float, float, float] | None = None
    achieved_position_world: tuple[float, float, float] | None = None


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
    """Debug markers: executed polyline (start→waypoints) toward nominal pre-contact."""
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
            "color": [0.35, 0.85, 1.0, 0.85],
            "radius": 0.010,
            "ttl_ms": ttl,
        },
        {
            "name": "grasp_traj_nominal",
            "frame": "world",
            "pos": [float(end[0]), float(end[1]), float(end[2])],
            "color": [1.0, 0.55, 0.05, 0.90],
            "radius": 0.012,
            "ttl_ms": ttl,
        },
    ]
    if standoff_len > 1e-6:
        markers.append(
            {
                "name": "grasp_traj_standoff",
                "frame": "world",
                "pos": [float(end[0]), float(end[1]), float(end[2])],
                "dir": [float(standoff_vec[0]), float(standoff_vec[1]), float(standoff_vec[2])],
                "color": [1.0, 0.55, 0.05, 0.55],
                "radius": 0.005,
                "length": standoff_len,
                "ttl_ms": ttl,
            }
        )
    if look_anchor_position is not None:
        anchor = np.asarray(look_anchor_position, dtype=float).reshape(3)
        seg = _segment_marker(
            name="grasp_traj_look_anchor",
            start=anchor,
            end=start,
            color=[0.55, 0.75, 1.0, 0.45],
            ttl_ms=ttl,
        )
        if seg is not None:
            markers.append(seg)
        markers.append(
            {
                "name": "grasp_traj_look_pose",
                "frame": "world",
                "pos": [float(anchor[0]), float(anchor[1]), float(anchor[2])],
                "color": [0.55, 0.75, 1.0, 0.75],
                "radius": 0.008,
                "ttl_ms": ttl,
            }
        )

    chain: list[np.ndarray] = [start]
    for wp in waypoints:
        chain.append(np.asarray(wp.position_world, dtype=float).reshape(3))
    for idx in range(1, len(chain)):
        is_highlight = int(idx - 1) == int(highlight_idx)
        color = (
            [1.0, 0.25, 0.15, 0.95]
            if is_highlight
            else [0.20, 0.90, 0.45, 0.70]
        )
        seg = _segment_marker(
            name="grasp_traj_seg_%d" % int(idx - 1),
            start=chain[idx - 1],
            end=chain[idx],
            color=color,
            ttl_ms=ttl,
        )
        if seg is not None:
            markers.append(seg)

    for wp_idx, wp in enumerate(waypoints):
        is_highlight = int(wp_idx) == int(highlight_idx)
        color = (
            [1.0, 0.25, 0.15, 0.95]
            if is_highlight
            else [0.20, 0.90, 0.45, 0.80]
        )
        radius = 0.009 if is_highlight else 0.007
        markers.append(
            {
                "name": "grasp_traj_wp_%d" % int(wp_idx),
                "frame": "world",
                "pos": [float(wp.position_world[0]), float(wp.position_world[1]), float(wp.position_world[2])],
                "dir": [
                    float(wp.direction_world[0]),
                    float(wp.direction_world[1]),
                    float(wp.direction_world[2]),
                ],
                "color": color,
                "radius": radius,
                "ttl_ms": ttl,
            }
        )

    seg_nominal = _segment_marker(
        name="grasp_traj_to_nominal",
        start=chain[-1] if chain else start,
        end=end,
        color=[1.0, 0.55, 0.05, 0.35],
        ttl_ms=ttl,
    )
    if seg_nominal is not None:
        markers.append(seg_nominal)

    return markers
