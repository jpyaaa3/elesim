"""Geometry for converting sim camera metadata into world points."""

from __future__ import annotations

import numpy as np


def camera_point_to_world_from_axes(
    origin: np.ndarray | list[float],
    look: np.ndarray | list[float],
    right: np.ndarray | list[float],
    point_camera: np.ndarray | list[float],
) -> np.ndarray:
    camera_origin = np.asarray(origin, dtype=float).reshape(3)
    look_vector = np.asarray(look, dtype=float).reshape(3)
    right_vector = np.asarray(right, dtype=float).reshape(3)
    look_norm = float(np.linalg.norm(look_vector))
    right_norm = float(np.linalg.norm(right_vector))
    if look_norm <= 1e-9 or right_norm <= 1e-9:
        raise ValueError("camera look/right vectors are degenerate")
    z_axis = look_vector / look_norm
    x_axis = right_vector / right_norm
    y_axis = np.cross(z_axis, x_axis)
    y_norm = float(np.linalg.norm(y_axis))
    if y_norm <= 1e-9:
        raise ValueError("camera axis triad is degenerate")
    y_axis /= y_norm
    x_axis = np.cross(y_axis, z_axis)
    rotation = np.column_stack([x_axis, y_axis, z_axis])
    return camera_origin + rotation @ np.asarray(point_camera, dtype=float).reshape(3)
