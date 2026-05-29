from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from .transforms import make_transform_from_pose, make_transform_from_world_pose, transform_point
from engine.iklib.kinematics import _forward_link_tf


def load_hand_eye_transform(path: str | Path) -> tuple[np.ndarray, dict[str, Any]]:
    p = Path(path)
    if not p.is_file():
        raise FileNotFoundError(f"hand-eye config not found: {p}")
    with open(p, "r", encoding="utf-8") as f:
        data = json.load(f)
    parent = str(data.get("parent_frame", "node9"))
    child = str(data.get("child_frame", "camera_color_optical_frame"))
    translation = data.get("translation_m", [0.0, 0.0, 0.0])
    quat = data.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
    T = make_transform_from_pose(translation, quat)
    meta = {
        "parent_frame": parent,
        "child_frame": child,
        "path": str(p.resolve()),
    }
    return T, meta


def camera_world_transform(
    context: dict[str, Any],
    q4: Sequence[float],
    T_parent_camera: np.ndarray,
    *,
    parent_frame: str,
) -> np.ndarray:
    link_tf = _forward_link_tf(context, q4)
    if parent_frame not in link_tf:
        raise RuntimeError(f"hand-eye parent frame '{parent_frame}' missing from FK result")
    p_parent, R_parent = link_tf[parent_frame]
    T_world_parent = make_transform_from_world_pose(p_parent, R_parent)
    return T_world_parent @ np.asarray(T_parent_camera, dtype=float).reshape(4, 4)


def world_point_to_camera(
    context: dict[str, Any],
    q4: Sequence[float],
    T_parent_camera: np.ndarray,
    point_world: np.ndarray | list[float],
    *,
    parent_frame: str,
) -> np.ndarray:
    T_world_camera = camera_world_transform(context, q4, T_parent_camera, parent_frame=parent_frame)
    T_camera_world = np.linalg.inv(np.asarray(T_world_camera, dtype=float).reshape(4, 4))
    return transform_point(T_camera_world, point_world)


def camera_point_to_world(
    context: dict[str, Any],
    q4: Sequence[float],
    T_parent_camera: np.ndarray,
    point_camera: np.ndarray | list[float],
    *,
    parent_frame: str,
) -> np.ndarray:
    T_world_camera = camera_world_transform(context, q4, T_parent_camera, parent_frame=parent_frame)
    return transform_point(T_world_camera, point_camera)


def camera_axes_world(
    context: dict[str, Any],
    q4: Sequence[float],
    T_parent_camera: np.ndarray,
    *,
    parent_frame: str,
    axis_len_m: float = 0.08,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    T_world_camera = camera_world_transform(context, q4, T_parent_camera, parent_frame=parent_frame)
    origin = transform_point(T_world_camera, [0.0, 0.0, 0.0])
    look = transform_point(T_world_camera, [0.0, 0.0, float(axis_len_m)]) - origin
    right = transform_point(T_world_camera, [float(axis_len_m), 0.0, 0.0]) - origin
    return origin, look, right
