"""
Pose-agnostic multi-view fusion primitives.

Transform-and-merge only -- deliberately NO ICP. The whole point of the
FK-as-metrology exercise is that registration quality must come from the pose
source alone, so any alignment refinement here would hide the quantity being
measured.

These functions are pure numpy and take poses as plain (R, t), so the same code
serves the live scan, an offline replay, and the synthetic self-test.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from .geometry import remove_dominant_plane


def transform_points(pts_cam: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Camera-frame points -> world frame, given a world<-camera pose."""
    pts = np.asarray(pts_cam, dtype=float)
    if pts.size == 0:
        return pts.reshape(0, 3)
    return pts @ np.asarray(R, dtype=float).T + np.asarray(t, dtype=float).reshape(1, 3)


_VOXEL_KEY_LIMIT = 1 << 20  # per-axis magnitude that fits the int64 packing below


def voxel_downsample(pts: np.ndarray, voxel: float) -> np.ndarray:
    """
    One point per voxel (mean of its members).

    Keys are packed into a single int64 and uniqued on that, which is ~16x
    faster than ``np.unique(keys, axis=0)`` (0.7 s vs 11.3 s on a full HD1080
    frame -- the row-wise version dominated the whole per-frame budget). Packing
    is only used when every key fits the 21-bit-per-axis field; out of range it
    falls back to the row-wise unique rather than silently colliding, which is
    how a hand-rolled packing goes wrong.
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) == 0:
        return pts.reshape(0, 3)
    if voxel <= 0.0:
        return pts
    keys = np.floor(pts / float(voxel)).astype(np.int64)
    if int(np.abs(keys).max()) < _VOXEL_KEY_LIMIT:
        off = _VOXEL_KEY_LIMIT
        packed = (
            ((keys[:, 0] + off) << 42) | ((keys[:, 1] + off) << 21) | (keys[:, 2] + off)
        )
        n_groups_arr, inverse, counts = np.unique(
            packed, return_inverse=True, return_counts=True
        )
        n_groups = len(n_groups_arr)
    else:
        uniq, inverse, counts = np.unique(
            keys, axis=0, return_inverse=True, return_counts=True
        )
        n_groups = len(uniq)
    inverse = np.asarray(inverse).reshape(-1)
    sums = np.zeros((n_groups, 3), dtype=float)
    np.add.at(sums, inverse, pts)
    return sums / counts[:, None]


def box_crop(pts: np.ndarray, center: np.ndarray, half: float) -> np.ndarray:
    pts = np.asarray(pts, dtype=float)
    if len(pts) == 0:
        return pts.reshape(0, 3)
    m = np.all(np.abs(pts - np.asarray(center, dtype=float).reshape(1, 3)) < float(half), axis=1)
    return pts[m]


def classify_surface_world(
    pts: np.ndarray,
    cam_positions: np.ndarray,
    axis: np.ndarray,
    axis_point: np.ndarray,
    radius: float,
    tol: float = 0.004,
) -> tuple[str, float]:
    """
    Exterior / interior for a WORLD-frame fit.

    A single-view classifier can assume the camera sits at the origin. After
    fusion every point was seen from a different camera pose, so the test needs
    per-point provenance: a visible exterior point has its outward radial
    direction facing the camera that actually observed it.
    """
    pts = np.asarray(pts, dtype=float)
    cam_positions = np.asarray(cam_positions, dtype=float)
    axis = np.asarray(axis, dtype=float)
    axis = axis / max(float(np.linalg.norm(axis)), 1e-12)
    X = pts - np.asarray(axis_point, dtype=float).reshape(1, 3)
    radial = X - (X @ axis)[:, None] * axis
    dist = np.linalg.norm(radial, axis=1)
    inl = np.abs(dist - float(radius)) < float(tol)
    if int(inl.sum()) < 50:
        return "unknown", float("nan")
    r_hat = radial[inl] / np.maximum(dist[inl], 1e-12)[:, None]
    to_cam = cam_positions[inl] - pts[inl]
    ext_frac = float(np.mean(np.einsum("ij,ij->i", r_hat, to_cam) > 0.0))
    if ext_frac >= 0.7:
        return "exterior", ext_frac
    if ext_frac <= 0.55:
        return "interior", ext_frac
    return "mixed", ext_frac


def clean_world_frame(pw: np.ndarray, max_planes: int = 2, min_points: int = 500) -> np.ndarray:
    """
    Strip planar structure from ONE frame's (already cropped) world points.

    Planes must go per frame: within a single frame the table is a clean plane,
    but across frames any pose error smears it into a thick slab that no single
    plane RANSAC can remove. Doing it after fusion is how the first capture
    failed.
    """
    pw = np.asarray(pw, dtype=float)
    for _ in range(int(max_planes)):
        if len(pw) < int(min_points):
            break
        kept = remove_dominant_plane(pw, min_fraction=0.20)
        if len(kept) == len(pw):
            break
        pw = kept
    return pw


def fuse(frames: Sequence[np.ndarray], voxel: float = 0.002) -> np.ndarray:
    """Transform-and-merge fusion. No ICP -- see the module docstring."""
    usable = [np.asarray(f, dtype=float) for f in frames if f is not None and len(f)]
    if not usable:
        return np.empty((0, 3))
    return voxel_downsample(np.vstack(usable), voxel)


def project_points(
    pts_w: np.ndarray,
    R: np.ndarray,
    t: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    *,
    min_z: float = 0.05,
) -> tuple[np.ndarray, np.ndarray]:
    """World points -> pixel (u, v) for a camera at world<-camera pose (R, t)."""
    pts_w = np.asarray(pts_w, dtype=float)
    if len(pts_w) == 0:
        return np.empty((0, 2)), np.zeros(0, dtype=bool)
    pc = (pts_w - np.asarray(t, dtype=float).reshape(1, 3)) @ np.asarray(R, dtype=float)
    z = pc[:, 2]
    valid = z > float(min_z)
    u = float(fx) * pc[:, 0] / np.maximum(z, 1e-9) + float(cx)
    v = float(fy) * pc[:, 1] / np.maximum(z, 1e-9) + float(cy)
    return np.stack([u, v], axis=1), valid


def anchor_from_frame(
    pts_cam: np.ndarray,
    R: np.ndarray,
    t: np.ndarray,
    *,
    box_half: float,
    coarse_voxel: float = 0.01,
    max_anchor_points: int = 120_000,
) -> Optional[np.ndarray]:
    """
    World-frame anchor from the first usable frame.

    Anchors on the non-planar remainder, not the raw median: the frame contains
    table around and behind the object, and a raw median lands there (the first
    field failure mode).

    The cloud is voxel-coarsened to ``coarse_voxel`` FIRST. Plane RANSAC costs
    ~5 s per pass at 800k points on a Jetson, and a full HD1080 frame is 2.1 M,
    so anchoring on the raw cloud stalled the sweep for ~25 s before the first
    capture. The anchor only needs a centroid, for which 1 cm cells are ample.
    """
    pts_cam = np.asarray(pts_cam, dtype=float)
    if len(pts_cam) == 0:
        return None
    # a centroid does not need every pixel; stride first so the one-off anchor
    # costs ~1 s instead of tens of seconds on a full-resolution frame
    if len(pts_cam) > max_anchor_points:
        pts_cam = pts_cam[:: (len(pts_cam) // int(max_anchor_points)) + 1]
    pw = transform_points(pts_cam, R, t)
    coarse = voxel_downsample(pw, coarse_voxel)
    coarse = clean_world_frame(coarse, min_points=100)
    if len(coarse) < 40:
        return None
    return np.median(coarse, axis=0)


def reanchor_from_view_rays(
    frames: Sequence[np.ndarray],
    cams: Sequence[np.ndarray],
    looks: Sequence[np.ndarray],
    half: float,
    *,
    cell: float = 0.03,
) -> tuple[list[np.ndarray], np.ndarray, list[int]]:
    """
    Offline salvage: recover the object centre and re-crop every frame.

    The hand-held version fitted a circle to the CAMERA TRAJECTORY, because an
    orbiting scan circles the object. A roll sweep does not orbit -- the optical
    centre barely moves -- so that cue is unavailable here. What a roll sweep
    does produce is a fan of OPTICAL AXES that all point at the object, so the
    least-squares intersection of those rays localises it, and table residue
    (which no ray is aimed at) cannot pull it. Density is the fallback.
    """
    cleaned = [clean_world_frame(np.asarray(f, dtype=float).copy()) for f in frames]
    stack_list = [c for c in cleaned if len(c)]
    if not stack_list:
        raise ValueError("re-anchor failed: nothing non-planar in the capture")
    stack = np.vstack(stack_list)
    if len(stack) < 500:
        raise ValueError("re-anchor failed: too few non-planar points")

    center: Optional[np.ndarray] = None
    C = np.asarray(cams, dtype=float).reshape(-1, 3)
    L = np.asarray(looks, dtype=float).reshape(-1, 3)
    if len(C) >= 4 and len(C) == len(L):
        L = L / np.maximum(np.linalg.norm(L, axis=1, keepdims=True), 1e-12)
        # least-squares point closest to all view rays: sum (I - dd^T) x = sum (I - dd^T) c
        A = np.zeros((3, 3))
        b = np.zeros(3)
        for c, d in zip(C, L):
            P = np.eye(3) - np.outer(d, d)
            A += P
            b += P @ c
        try:
            cand = np.linalg.lstsq(A, b, rcond=None)[0]
            # accept only if it lands inside the observed cloud's envelope --
            # a near-parallel ray fan makes the intersection ill-conditioned
            lo, hi = stack.min(axis=0) - 0.10, stack.max(axis=0) + 0.10
            if np.all(cand > lo) and np.all(cand < hi):
                cond = float(np.linalg.cond(A))
                if cond < 1.0e4:
                    center = cand
        except np.linalg.LinAlgError:
            center = None

    if center is None:
        keys = np.floor(stack / float(cell)).astype(np.int64)
        uniq, counts = np.unique(keys, axis=0, return_counts=True)
        center = (uniq[counts.argmax()].astype(float) + 0.5) * float(cell)

    out_frames: list[np.ndarray] = []
    out_idx: list[int] = []
    for i, c in enumerate(cleaned):
        if len(c) == 0:
            continue
        g = c[np.all(np.abs(c - center) < min(float(half), 0.12), axis=1)]
        if len(g) >= 300:
            out_frames.append(g)
            out_idx.append(i)
    return out_frames, center, out_idx


@dataclass
class FusionResult:
    """What a completed scan produced, before any reporting."""

    fused: np.ndarray
    stacked: np.ndarray  # pre-voxel union, for provenance-based classification
    cam_positions: np.ndarray  # per-point camera origin (same length as stacked)
    n_frames: int
    center: Optional[np.ndarray] = None
    roll_span_deg: float = 0.0
    view_span_deg: float = 0.0
    # angle subtended at the object by the camera positions; on a base-roll arm
    # this is the coverage that buys new surface, not view_span_deg
    observation_span_deg: float = 0.0
    baseline_span_m: float = 0.0
    sweeps: int = 0
    notes: list[str] = field(default_factory=list)


def build_provenance(
    frames: Sequence[np.ndarray], cam_positions: Sequence[np.ndarray]
) -> tuple[np.ndarray, np.ndarray]:
    """Stack frames and tile each frame's camera origin to per-point rows."""
    usable = [(np.asarray(f, dtype=float), np.asarray(c, dtype=float).reshape(3))
              for f, c in zip(frames, cam_positions) if f is not None and len(f)]
    if not usable:
        return np.empty((0, 3)), np.empty((0, 3))
    stacked = np.vstack([f for f, _ in usable])
    cams = np.vstack([np.tile(c.reshape(1, 3), (len(f), 1)) for f, c in usable])
    return stacked, cams
