"""Sim eye-in-hand camera projection for target in-frame visibility (matches MP4)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Sequence

import numpy as np

from elesim_controller.vision.sim_camera.mount import intrinsics_from_fov


@dataclass(frozen=True)
class SimTargetProjection:
    in_frame: bool
    u_norm: float
    v_norm: float
    z_cam_m: float
    u_px: float
    v_px: float


def _optical_axes_from_world(
    origin: np.ndarray,
    look: np.ndarray,
    right: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    o = np.asarray(origin, dtype=float).reshape(3)
    z_axis = np.asarray(look, dtype=float).reshape(3)
    x_axis = np.asarray(right, dtype=float).reshape(3)
    z_n = float(np.linalg.norm(z_axis))
    x_n = float(np.linalg.norm(x_axis))
    if z_n <= 1e-9 or x_n <= 1e-9:
        raise ValueError("degenerate camera axes")
    z_axis = z_axis / z_n
    x_axis = x_axis / x_n
    y_axis = np.cross(z_axis, x_axis)
    y_n = float(np.linalg.norm(y_axis))
    if y_n <= 1e-9:
        raise ValueError("degenerate camera axis triad")
    y_axis = y_axis / y_n
    x_axis = np.cross(y_axis, z_axis)
    return x_axis, y_axis, z_axis


def world_point_to_optical(
    origin: Sequence[float],
    look: Sequence[float],
    right: Sequence[float],
    point_world: Sequence[float],
) -> np.ndarray:
    o = np.asarray(origin, dtype=float).reshape(3)
    p = np.asarray(point_world, dtype=float).reshape(3)
    x_axis, y_axis, z_axis = _optical_axes_from_world(o, np.asarray(look), np.asarray(right))
    d = p - o
    return np.array(
        [
            float(np.dot(d, x_axis)),
            float(np.dot(d, y_axis)),
            float(np.dot(d, z_axis)),
        ],
        dtype=float,
    )


def project_world_point(
    *,
    object_world: Sequence[float],
    camera_origin: Sequence[float],
    camera_look: Sequence[float],
    camera_right: Sequence[float],
    width: int,
    height: int,
    fov_deg: float,
    margin_px: float = 0.0,
    min_z_m: float = 0.05,
) -> SimTargetProjection:
    p_cam = world_point_to_optical(camera_origin, camera_look, camera_right, object_world)
    z = float(p_cam[2])
    intr = intrinsics_from_fov(width=int(width), height=int(height), fov_deg=float(fov_deg))
    if z <= float(min_z_m):
        return SimTargetProjection(
            in_frame=False,
            u_norm=0.0,
            v_norm=0.0,
            z_cam_m=z,
            u_px=0.0,
            v_px=0.0,
        )
    u_px = float(intr.fx * float(p_cam[0]) / z + intr.cx)
    v_px = float(intr.fy * float(p_cam[1]) / z + intr.cy)
    m = float(max(0.0, margin_px))
    in_frame = bool(
        m <= u_px <= float(width) - m
        and m <= v_px <= float(height) - m
    )
    u_norm = float(2.0 * u_px / float(width) - 1.0)
    v_norm = float(2.0 * v_px / float(height) - 1.0)
    return SimTargetProjection(
        in_frame=in_frame,
        u_norm=u_norm,
        v_norm=v_norm,
        z_cam_m=z,
        u_px=u_px,
        v_px=v_px,
    )


def project_from_eye_camera(
    eye_camera: Any,
    object_world: Sequence[float],
    *,
    margin_px: float = 0.0,
) -> Optional[SimTargetProjection]:
    axes = eye_camera.camera_axes_world()
    if axes is None:
        return None
    origin, look, right = axes
    intr = eye_camera.intrinsics
    fov_deg = 60.0
    if intr.width > 0 and intr.fx > 0:
        fov_deg = float(np.degrees(2.0 * np.arctan(0.5 * intr.width / intr.fx)))
    return project_world_point(
        object_world=object_world,
        camera_origin=origin,
        camera_look=look,
        camera_right=right,
        width=int(intr.width),
        height=int(intr.height),
        fov_deg=fov_deg,
        margin_px=margin_px,
    )
