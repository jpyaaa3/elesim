from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from engine.vision.perception_bridge.transforms import make_transform_from_world_pose, transform_point
from engine.simulation.genesis.utils import quat_wxyz_to_xyzw as _quat_wxyz_to_xyzw, to_numpy_1d as _to_numpy_1d
from engine.vision.sim_camera.mount import load_hand_eye_offset_T, _OPTICAL_FROM_GENESIS_CAMERA


def _link_world_transform(link) -> np.ndarray:
    pos = _to_numpy_1d(link.get_pos())[:3]
    quat_wxyz = _to_numpy_1d(link.get_quat())[:4]
    quat_xyzw = _quat_wxyz_to_xyzw(quat_wxyz)
    rot = Rot.from_quat(quat_xyzw).as_matrix()
    return make_transform_from_world_pose(pos, rot)


def camera_axes_from_genesis_camera_object(
    camera: Any,
    *,
    axis_len_m: float = 0.08,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """World optical origin / look / right from Genesis camera after attach (matches render)."""
    raw = camera.get_transform()
    if hasattr(raw, "detach"):
        raw = raw.detach()
    if hasattr(raw, "cpu"):
        raw = raw.cpu()
    if hasattr(raw, "numpy"):
        raw = raw.numpy()
    T_w_genesis = np.asarray(raw, dtype=float).reshape(4, 4)
    T_world_optical = T_w_genesis @ np.asarray(_OPTICAL_FROM_GENESIS_CAMERA, dtype=float).reshape(4, 4)
    origin = transform_point(T_world_optical, [0.0, 0.0, 0.0])
    look = transform_point(T_world_optical, [0.0, 0.0, float(axis_len_m)]) - origin
    right = transform_point(T_world_optical, [float(axis_len_m), 0.0, 0.0]) - origin
    return origin, look, right


def camera_axes_from_genesis_link(
    entity,
    *,
    hand_eye_path: str | Path,
    parent_link: str = "node9",
    axis_len_m: float = 0.08,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """World-frame camera optical origin / look / right from a live Genesis link pose."""
    link = entity.get_link(str(parent_link))
    T_world_parent = _link_world_transform(link)
    T_parent_camera = load_hand_eye_offset_T(hand_eye_path)
    T_world_camera = T_world_parent @ np.asarray(T_parent_camera, dtype=float).reshape(4, 4)
    origin = transform_point(T_world_camera, [0.0, 0.0, 0.0])
    look = transform_point(T_world_camera, [0.0, 0.0, float(axis_len_m)]) - origin
    right = transform_point(T_world_camera, [float(axis_len_m), 0.0, 0.0]) - origin
    return origin, look, right


def camera_point_to_world_from_axes(
    origin: np.ndarray | list[float],
    look: np.ndarray | list[float],
    right: np.ndarray | list[float],
    point_camera: np.ndarray | list[float],
) -> np.ndarray:
    """Transform optical-frame point to world using live camera axis vectors."""
    o = np.asarray(origin, dtype=float).reshape(3)
    look_v = np.asarray(look, dtype=float).reshape(3)
    right_v = np.asarray(right, dtype=float).reshape(3)
    look_n = float(np.linalg.norm(look_v))
    right_n = float(np.linalg.norm(right_v))
    if look_n <= 1e-9 or right_n <= 1e-9:
        raise ValueError("camera look/right vectors are degenerate")
    z_axis = look_v / look_n
    x_axis = right_v / right_n
    y_axis = np.cross(z_axis, x_axis)
    y_norm = float(np.linalg.norm(y_axis))
    if y_norm <= 1e-9:
        raise ValueError("camera axis triad is degenerate")
    y_axis = y_axis / y_norm
    x_axis = np.cross(y_axis, z_axis)
    rot = np.column_stack([x_axis, y_axis, z_axis])
    return transform_point(make_transform_from_world_pose(o, rot), point_camera)


def camera_point_to_world_from_genesis_link(
    entity,
    *,
    hand_eye_path: str | Path,
    point_camera: np.ndarray | list[float],
    parent_link: str = "node9",
) -> np.ndarray:
    link = entity.get_link(str(parent_link))
    T_world_parent = _link_world_transform(link)
    T_parent_camera = load_hand_eye_offset_T(hand_eye_path)
    T_world_camera = T_world_parent @ np.asarray(T_parent_camera, dtype=float).reshape(4, 4)
    return transform_point(T_world_camera, point_camera)
