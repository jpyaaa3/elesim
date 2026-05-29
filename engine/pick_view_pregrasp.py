"""View-aware pregrasp candidate generation and camera-frame visibility scoring."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Optional, Sequence

import numpy as np

from addons.perception_bridge.hand_eye import camera_axes_world, world_point_to_camera


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


@dataclass(frozen=True)
class ViewCandidateMetrics:
    p_camera: tuple[float, float, float]
    visible_pred: bool
    look_dot: float
    score: float
    camera_world: tuple[float, float, float]
    camera_look: tuple[float, float, float]
    object_dir: tuple[float, float, float]


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
    view_distance_m: float | None = None,
    view_distances_m: Sequence[float] | None = None,
    lateral_offsets_m: Sequence[float] = (-0.05, 0.0, 0.05),
    height_offsets_m: Sequence[float] = (0.0, 0.05, 0.10),
) -> list[ViewPregraspCandidate]:
    """Build grasp pregrasp positions around the object with view-oriented offsets."""
    obj = np.asarray(object_world, dtype=float).reshape(3)
    base_off = np.asarray(base_offset_m, dtype=float).reshape(3)
    view_axis = _normalize(-base_off)
    if view_axis is None:
        view_axis = np.array([1.0, 0.0, 0.0], dtype=float)

    distances: list[float] = []
    if view_distances_m is not None:
        for d in view_distances_m:
            distances.append(float(d))
    if view_distance_m is not None:
        distances.append(float(view_distance_m))
    if not distances:
        distances = [0.45]

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

    for dist in distances:
        view_base = obj - view_axis * float(dist)
        dist_tag = f"d{dist:.2f}".replace(".", "p")
        _add(view_base, f"view_{dist_tag}")

        for dz in height_offsets_m:
            for dy in lateral_offsets_m:
                for dx in lateral_offsets_m:
                    delta = np.array([float(dx), float(dy), float(dz)], dtype=float)
                    _add(obj + base_off + delta, f"grid_{dx:+.2f}_{dy:+.2f}_{dz:+.2f}")
                    _add(view_base + delta, f"view_{dist_tag}_{dx:+.2f}_{dy:+.2f}_{dz:+.2f}")

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


def evaluate_view_candidate(
    q4: Sequence[float],
    object_world: Sequence[float],
    *,
    ik_context: dict[str, Any],
    hand_eye_transform: np.ndarray,
    parent_frame: str,
    limits: ViewPregraspLimits,
    desired_xy: Sequence[float] = (0.0, 0.0),
) -> Optional[ViewCandidateMetrics]:
    """FK + hand-eye metrics for one IK solution q."""
    try:
        p_cam = world_point_to_camera(
            ik_context,
            q4,
            hand_eye_transform,
            object_world,
            parent_frame=parent_frame,
        )
        camera_world, camera_look, _camera_right = camera_axes_world(
            ik_context,
            q4,
            hand_eye_transform,
            parent_frame=parent_frame,
        )
    except Exception:
        return None

    p_cam_v = np.asarray(p_cam, dtype=float).reshape(3)
    cam_w = np.asarray(camera_world, dtype=float).reshape(3)
    cam_look_v = np.asarray(camera_look, dtype=float).reshape(3)
    obj_w = np.asarray(object_world, dtype=float).reshape(3)

    object_dir = obj_w - cam_w
    object_dir_n = _normalize(object_dir)
    look_dir_n = _normalize(cam_look_v)
    if object_dir_n is None or look_dir_n is None:
        look_dot = -1.0
        object_dir_unit = (0.0, 0.0, 0.0)
    else:
        look_dot = float(np.clip(np.dot(look_dir_n, object_dir_n), -1.0, 1.0))
        object_dir_unit = (float(object_dir_n[0]), float(object_dir_n[1]), float(object_dir_n[2]))

    visible_pred = camera_visibility_ok(p_cam_v, limits)
    score = view_candidate_score(
        p_cam_v,
        desired_xy=desired_xy,
        limits=limits,
        visible_pred=visible_pred,
    )

    return ViewCandidateMetrics(
        p_camera=(float(p_cam_v[0]), float(p_cam_v[1]), float(p_cam_v[2])),
        visible_pred=bool(visible_pred),
        look_dot=float(look_dot),
        score=float(score),
        camera_world=(float(cam_w[0]), float(cam_w[1]), float(cam_w[2])),
        camera_look=(float(cam_look_v[0]), float(cam_look_v[1]), float(cam_look_v[2])),
        object_dir=object_dir_unit,
    )


def view_candidate_passes(
    metrics: ViewCandidateMetrics,
    *,
    limits: ViewPregraspLimits,
    look_dot_min: float = 0.85,
) -> bool:
    if not bool(metrics.visible_pred):
        return False
    return float(metrics.look_dot) >= float(look_dot_min)


def view_candidate_score(
    p_camera: Sequence[float],
    *,
    desired_xy: Sequence[float],
    limits: ViewPregraspLimits,
    visible_pred: bool = True,
) -> float:
    """Higher is better. Returns -inf when not strictly visible."""
    if not visible_pred:
        return float("-inf")
    p = np.asarray(p_camera, dtype=float).reshape(3)
    desired = np.asarray(desired_xy, dtype=float).reshape(2)
    x_err = abs(float(p[0]) - float(desired[0]))
    y_err = abs(float(p[1]) - float(desired[1]))
    depth_err = abs(float(p[2]) - float(limits.z_target_m))
    return float(-(x_err + y_err + 0.5 * depth_err))


def camera_visibility_score(p_camera: Sequence[float], limits: ViewPregraspLimits) -> float:
    """Higher is better. Returns -inf when object would be outside the camera view."""
    visible = camera_visibility_ok(p_camera, limits)
    return view_candidate_score(p_camera, desired_xy=(0.0, 0.0), limits=limits, visible_pred=visible)


def camera_visibility_score_soft(p_camera: Sequence[float], limits: ViewPregraspLimits) -> float:
    """Finite ranking score; prefer in-FOV but still rank unreachable candidates."""
    p = np.asarray(p_camera, dtype=float).reshape(3)
    z = float(p[2])
    if z <= 0.01:
        return -1e6 + z
    center_err = float(np.hypot(float(p[0]), float(p[1])))
    depth_err = abs(z - float(limits.z_target_m))
    pen_x = max(0.0, abs(float(p[0])) - float(limits.x_abs_max_m))
    pen_y = max(0.0, abs(float(p[1])) - float(limits.y_abs_max_m))
    pen_z = 0.0
    if z < float(limits.z_min_m):
        pen_z = float(limits.z_min_m) - z
    elif z > float(limits.z_max_m):
        pen_z = z - float(limits.z_max_m)
    return float(-(center_err + 0.5 * depth_err + 5.0 * (pen_x + pen_y) + 3.0 * pen_z))


def pick_best_strict_candidate(
    scored: Iterable[tuple[ViewPregraspCandidate, np.ndarray, ViewCandidateMetrics]],
) -> Optional[tuple[ViewPregraspCandidate, np.ndarray, ViewCandidateMetrics]]:
    """Pick highest-score candidate among strict-passing rows only."""
    best: Optional[tuple[ViewPregraspCandidate, np.ndarray, ViewCandidateMetrics]] = None
    for cand, q, metrics in scored:
        if not np.isfinite(float(metrics.score)):
            continue
        if best is None or float(metrics.score) > float(best[2].score):
            best = (cand, q, metrics)
    return best


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


def format_view_candidate_log(
    cand: ViewPregraspCandidate,
    q4: Sequence[float],
    metrics: ViewCandidateMetrics,
    object_world: Sequence[float],
) -> str:
    q = np.asarray(q4, dtype=float).reshape(4)
    p = metrics.p_camera
    cw = metrics.camera_world
    cl = metrics.camera_look
    od = metrics.object_dir
    ow = np.asarray(object_world, dtype=float).reshape(3)
    return (
        f"tag={cand.tag} "
        f"q=[{q[0]:+.4f},{q[1]:+.4f},{q[2]:+.4f},{q[3]:+.4f}] "
        f"target_pos=[{cand.pregrasp_world[0]:+.4f},{cand.pregrasp_world[1]:+.4f},{cand.pregrasp_world[2]:+.4f}] "
        f"camera_world=[{cw[0]:+.4f},{cw[1]:+.4f},{cw[2]:+.4f}] "
        f"camera_look=[{cl[0]:+.4f},{cl[1]:+.4f},{cl[2]:+.4f}] "
        f"object_world=[{ow[0]:+.4f},{ow[1]:+.4f},{ow[2]:+.4f}] "
        f"object_dir=[{od[0]:+.4f},{od[1]:+.4f},{od[2]:+.4f}] "
        f"look_dot={metrics.look_dot:.3f} "
        f"p_camera_pred=[{p[0]:+.4f},{p[1]:+.4f},{p[2]:+.4f}] "
        f"visible_pred={str(metrics.visible_pred).lower()} "
        f"score={metrics.score:.4f}"
    )
