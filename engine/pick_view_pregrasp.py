"""View-aware pregrasp candidate generation and camera-frame visibility scoring."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

import numpy as np


@dataclass(frozen=True)
class ViewPregraspLimits:
    z_min_m: float = 0.35
    z_max_m: float = 0.70
    x_abs_max_m: float = 0.15
    y_abs_max_m: float = 0.12
    z_target_m: float = 0.50


@dataclass(frozen=True)
class ViewPregraspCandidate:
    pregrasp_world: tuple[float, float, float]
    look_dir_world: tuple[float, float, float]
    tag: str


def _normalize(vec: np.ndarray) -> Optional[np.ndarray]:
    v = np.asarray(vec, dtype=float).reshape(3)
    n = float(np.linalg.norm(v))
    if n <= 1e-9:
        return None
    return v / n


def generate_view_pregrasp_candidates(
    object_world: Sequence[float],
    *,
    base_offset_m: Sequence[float],
    view_distance_m: float,
    lateral_offsets_m: Sequence[float] = (-0.05, 0.0, 0.05),
    height_offsets_m: Sequence[float] = (0.0, 0.05, 0.10),
) -> list[ViewPregraspCandidate]:
    """Build grasp pregrasp positions around the object with view-oriented offsets."""
    obj = np.asarray(object_world, dtype=float).reshape(3)
    base_off = np.asarray(base_offset_m, dtype=float).reshape(3)
    view_axis = _normalize(-base_off)
    if view_axis is None:
        view_axis = np.array([1.0, 0.0, 0.0], dtype=float)

    seen: set[tuple[float, float, float]] = set()
    out: list[ViewPregraspCandidate] = []

    def _add(pre: np.ndarray, tag: str) -> None:
        key = (round(float(pre[0]), 4), round(float(pre[1]), 4), round(float(pre[2]), 4))
        if key in seen:
            return
        seen.add(key)
        look = obj - pre
        look_n = _normalize(look)
        if look_n is None:
            return
        out.append(
            ViewPregraspCandidate(
                pregrasp_world=(float(pre[0]), float(pre[1]), float(pre[2])),
                look_dir_world=(float(look_n[0]), float(look_n[1]), float(look_n[2])),
                tag=str(tag),
            )
        )

    _add(obj + base_off, "base_offset")
    view_base = obj - view_axis * float(view_distance_m)
    _add(view_base, "view_distance")

    for dz in height_offsets_m:
        for dy in lateral_offsets_m:
            for dx in lateral_offsets_m:
                delta = np.array([float(dx), float(dy), float(dz)], dtype=float)
                _add(obj + base_off + delta, f"grid_{dx:+.2f}_{dy:+.2f}_{dz:+.2f}")
                _add(view_base + delta, f"view_{dx:+.2f}_{dy:+.2f}_{dz:+.2f}")

    return out


def camera_visibility_ok(p_camera: Sequence[float], limits: ViewPregraspLimits) -> bool:
    p = np.asarray(p_camera, dtype=float).reshape(3)
    z = float(p[2])
    if z <= 0.0:
        return False
    if z < float(limits.z_min_m) or z > float(limits.z_max_m):
        return False
    if abs(float(p[0])) > float(limits.x_abs_max_m):
        return False
    if abs(float(p[1])) > float(limits.y_abs_max_m):
        return False
    return True


def camera_visibility_score(p_camera: Sequence[float], limits: ViewPregraspLimits) -> float:
    """Higher is better. Returns -inf when object would be outside the camera view."""
    if not camera_visibility_ok(p_camera, limits):
        return float("-inf")
    p = np.asarray(p_camera, dtype=float).reshape(3)
    center_err = float(np.hypot(float(p[0]), float(p[1])))
    depth_err = abs(float(p[2]) - float(limits.z_target_m))
    return float(-(center_err + 0.5 * depth_err))


def pick_best_visible_candidate(
    scored: Iterable[tuple[ViewPregraspCandidate, float]],
) -> Optional[tuple[ViewPregraspCandidate, float]]:
    best: Optional[tuple[ViewPregraspCandidate, float]] = None
    for cand, score in scored:
        if not np.isfinite(float(score)):
            continue
        if best is None or float(score) > float(best[1]):
            best = (cand, float(score))
    return best
