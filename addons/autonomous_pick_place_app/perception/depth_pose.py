"""Estimate object 3D position in the camera optical frame from depth + mask."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


def estimate_object_position_camera(
    mask: np.ndarray,
    depth_image: np.ndarray,
    intrinsics: CameraIntrinsics,
    depth_scale: float,
    *,
    z_min_m: float = 0.15,
    z_max_m: float = 2.5,
    outlier_sigma: float = 2.5,
) -> np.ndarray:
    """
    Returns p_camera_object = [x, y, z] in meters.

    RealSense color optical frame convention:
      x: right, y: down, z: forward
    """
    if mask.ndim != 2:
        raise ValueError("mask must be a 2D array")
    if depth_image.ndim != 2:
        raise ValueError("depth_image must be a 2D array")
    if mask.shape != depth_image.shape:
        raise ValueError(f"mask shape {mask.shape} != depth shape {depth_image.shape}")

    valid = mask.astype(bool) & (depth_image > 0)
    if not np.any(valid):
        raise RuntimeError("no valid depth samples inside mask")

    z_raw = depth_image[valid].astype(np.float64) * float(depth_scale)
    z_ok = z_raw[(z_raw >= z_min_m) & (z_raw <= z_max_m)]
    if z_ok.size == 0:
        raise RuntimeError("all masked depth values filtered by z_min/z_max")

    z_med = float(np.median(z_ok))
    if z_ok.size >= 4 and outlier_sigma > 0.0:
        mad = float(np.median(np.abs(z_ok - z_med)))
        if mad > 1e-9:
            sigma = 1.4826 * mad
            inlier_z = z_ok[np.abs(z_ok - z_med) <= outlier_sigma * sigma]
            if inlier_z.size > 0:
                z_ok = inlier_z

    ys, xs = np.where(valid)
    z_pix = depth_image[valid].astype(np.float64) * float(depth_scale)
    inlier = (z_pix >= z_min_m) & (z_pix <= z_max_m)
    if outlier_sigma > 0.0 and np.count_nonzero(inlier) >= 4:
        z_med_pix = float(np.median(z_pix[inlier]))
        mad = float(np.median(np.abs(z_pix[inlier] - z_med_pix)))
        if mad > 1e-9:
            sigma = 1.4826 * mad
            inlier = inlier & (np.abs(z_pix - z_med_pix) <= outlier_sigma * sigma)

    if not np.any(inlier):
        inlier = np.ones(z_pix.shape[0], dtype=bool)

    xs_i = xs[inlier]
    ys_i = ys[inlier]
    z_i = z_pix[inlier]

    fx, fy, cx, cy = (
        float(intrinsics.fx),
        float(intrinsics.fy),
        float(intrinsics.cx),
        float(intrinsics.cy),
    )

    X = (xs_i.astype(np.float64) - cx) * z_i / fx
    Y = (ys_i.astype(np.float64) - cy) * z_i / fy
    Z = z_i

    p = np.array([float(np.median(X)), float(np.median(Y)), float(np.median(Z))], dtype=float)
    if not np.all(np.isfinite(p)):
        raise RuntimeError("non-finite camera-frame position")
    return p
