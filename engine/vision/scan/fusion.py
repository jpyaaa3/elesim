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
    """
    Camera-frame points -> world frame, given a world<-camera pose.

    float32 input stays float32. The ZED delivers float32 and the heavy stages
    are memory-bound, so upcasting a 2 M-point frame to float64 doubled the
    traffic for nothing: at ~1 m magnitudes float32 resolves ~0.1 um, four
    orders below the 2 mm fusion voxel.
    """
    pts = np.asarray(pts_cam)
    if pts.size == 0:
        return pts.reshape(0, 3).astype(float, copy=False)
    dt = np.float32 if pts.dtype == np.float32 else np.float64
    pts = pts.astype(dt, copy=False)
    return pts @ np.asarray(R, dtype=dt).T + np.asarray(t, dtype=dt).reshape(1, 3)


def camera_depth_gate(
    pts_cam: np.ndarray,
    R: np.ndarray,
    t: np.ndarray,
    center: np.ndarray,
    box_half: float,
) -> np.ndarray:
    """
    Drop points that cannot land in the world-frame crop box, by depth alone.

    Every point of a cube of half-size h centred on the anchor lies within
    h*sqrt(3) of the anchor, so its camera-frame depth is within h*sqrt(3) of the
    anchor's depth. That makes this a conservative pre-filter -- it never removes
    a point ``box_crop`` would have kept -- and it is a 1-D mask, so it costs a
    fraction of transforming the full frame and then cropping in world space.
    """
    pts_cam = np.asarray(pts_cam)
    if len(pts_cam) == 0:
        return pts_cam
    R = np.asarray(R, dtype=float)
    rel = np.asarray(center, dtype=float).reshape(3) - np.asarray(t, dtype=float).reshape(3)
    d_anchor = float((R.T @ rel)[2])  # anchor depth along the optical axis
    reach = float(box_half) * 1.7320508
    z = pts_cam[:, 2]
    return pts_cam[np.abs(z - d_anchor) < reach]


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


def _inplane_aspect(pts: np.ndarray) -> float:
    """Aspect ratio (2nd/1st singular value) of a point set within its own plane."""
    X = pts - pts.mean(axis=0)
    svals = np.linalg.svd(X, compute_uv=False)
    return float(svals[1] / svals[0]) if svals[0] > 1e-12 else 0.0


def remove_dominant_plane_fast(
    points: np.ndarray,
    tol: float = 0.004,
    iters: int = 200,
    min_fraction: float = 0.25,
    rng: Optional[np.random.Generator] = None,
    fit_points: int = 8000,
) -> np.ndarray:
    """
    Same contract as the bench's ``remove_dominant_plane``, but the consensus
    search runs on a SUBSAMPLE and the winning plane is then applied once to
    every point.

    The bench version scores all N points on each of 200 iterations, which is
    1.08 s at 144k points on a Jetson -- the dominant per-frame cost of a sweep,
    ~350 s over a scan. A plane is 3 parameters, so consensus does not need more
    than a few thousand samples to be found; only the final masking needs the
    full cloud. The elongated-strip safeguard is kept, because without it a
    tangent band on a curved wall gets removed and biases the fitted radius.
    """
    points = np.asarray(points, dtype=float)
    n = len(points)
    if n < 50:
        return points
    rng = rng or np.random.default_rng(1)

    sample = points if n <= int(fit_points) else points[rng.choice(n, int(fit_points), replace=False)]
    m = len(sample)
    best_plane: Optional[tuple[np.ndarray, np.ndarray]] = None
    best_count = 0
    for _ in range(int(iters)):
        idx = rng.choice(m, size=3, replace=False)
        p0, p1, p2 = sample[idx]
        nrm = np.cross(p1 - p0, p2 - p0)
        norm = float(np.linalg.norm(nrm))
        if norm < 1e-9:
            continue
        nrm = nrm / norm
        count = int((np.abs((sample - p0) @ nrm) < tol).sum())
        if count > best_count:
            best_count, best_plane = count, (p0.copy(), nrm)
    if best_plane is None:
        return points
    # the subsample's inlier fraction estimates the full cloud's
    if best_count < float(min_fraction) * m:
        return points

    p0, nrm = best_plane
    mask = np.abs((points - p0) @ nrm) < tol
    if int(mask.sum()) < float(min_fraction) * n:
        return points
    if _inplane_aspect(points[mask]) < 0.25:
        # elongated strip, not an extended surface: almost certainly a tangent
        # band on a curved wall, and removing it would bias the object
        return points
    return points[~mask]


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
        kept = remove_dominant_plane_fast(pw, min_fraction=0.20)
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


def anchor_via_roi(
    xyz_organized: np.ndarray,
    R: np.ndarray,
    t: np.ndarray,
    *,
    min_depth: float,
    max_depth: float,
    depth_window: float = 0.12,
) -> tuple[Optional[np.ndarray], str]:
    """
    Anchor on a DETECTED object, using the bench's own extraction.

    ``anchor_from_frame`` takes the median of whatever survives plane removal
    across the entire frame, which lands on background: a live scan anchored that
    way produced a cloud that filled the crop box exactly in two axes (i.e. the
    box was slicing a wall, not enclosing an object) and whose frames shared 0%
    of their points.

    ``auto_roi`` finds the nearest significant non-planar cluster and
    ``extract_points`` reduces that ROI to object candidates -- which is what
    those functions exist for. Returns (anchor, description); anchor is None when
    no object cluster is found, which is worth reporting rather than falling back
    silently to a background anchor.
    """
    from . import geometry

    if not geometry.available():
        return None, f"object detection unavailable: {geometry.backend_error()}"
    roi = geometry.auto_roi(xyz_organized, min_depth=min_depth, max_depth=max_depth)
    if roi is None:
        return None, "auto_roi found no non-planar cluster (is the object in view?)"
    pts, valid_ratio, z_med = geometry.extract_points(
        xyz_organized, roi, float(depth_window), remove_plane=True
    )
    if pts is None or len(pts) < 200:
        return None, f"roi {roi} yielded only {0 if pts is None else len(pts)} object points"
    pw = transform_points(pts, R, t)
    extent = pw.max(axis=0) - pw.min(axis=0)
    return np.median(pw, axis=0), (
        f"roi={roi} n={len(pts)} valid={valid_ratio:.2f} depth={z_med:.3f}m "
        f"extent={np.round(extent, 3).tolist()}"
    )


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
