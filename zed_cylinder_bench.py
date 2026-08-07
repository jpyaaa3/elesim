#!/usr/bin/env python3
"""
ZED Mini bench test: can we estimate a cylindrical object's radius from a single view?

Runs on Jetson with ZED SDK / pyzed. Geometry code is pure numpy and can be
verified without hardware via --selftest.

Examples
--------
  # feasibility capture (ROI found automatically from geometry)
  python3 zed_cylinder_bench.py --label pvc20a_300mm --mode NEURAL_PLUS --gt-diameter 0.026

  # continuous live capture until Ctrl-C
  python3 zed_cylinder_bench.py --label live --roi 520,300,240,400 --trials 0 --interval 1.0

  # also solve the wrap grasp on the measured cross-section
  python3 zed_cylinder_bench.py --label pipe --roi 520,300,240,400 \
      --newarc-path ../newarc.py --segment-length 0.2075 --gripper-ratio 0.155

  # sweep confidence, non-interactive ROI, 5 repeats
  python3 zed_cylinder_bench.py --label pipe_d300 --roi 520,300,240,400 --trials 5 \
      --confidence 50 --texture-confidence 100

  # verify the fitting math with synthetic data (no camera needed)
  python3 zed_cylinder_bench.py --selftest

Outputs (under --outdir, default ./bench_out)
  results.csv          one row per capture
  <label>_NNN.json     per-capture detail
  <label>_NNN.ply      segmented object points
  <label>_NNN_cross_section.txt   polygon string for newarc.py --vertices
  <label>_NNN_left.png / _depth.png
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import time

import numpy as np

# --------------------------------------------------------------------------
# geometry: cylinder fitting (pure numpy, hardware-independent)
# --------------------------------------------------------------------------


def normalize(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        raise ValueError("zero-length vector")
    return np.asarray(v, dtype=float) / n


def orthonormal_basis(axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Two unit vectors spanning the plane perpendicular to `axis`."""
    axis = normalize(axis)
    seed = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(seed, axis))) > 0.9:
        seed = np.array([0.0, 1.0, 0.0])
    e1 = normalize(seed - np.dot(seed, axis) * axis)
    e2 = np.cross(axis, e1)
    return e1, e2


def _circle_from_three(p: np.ndarray) -> tuple[np.ndarray, float] | None:
    (x1, y1), (x2, y2), (x3, y3) = p
    d = 2.0 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    if abs(d) < 1e-12:
        return None
    s1, s2, s3 = x1 * x1 + y1 * y1, x2 * x2 + y2 * y2, x3 * x3 + y3 * y3
    cx = (s1 * (y2 - y3) + s2 * (y3 - y1) + s3 * (y1 - y2)) / d
    cy = (s1 * (x3 - x2) + s2 * (x1 - x3) + s3 * (x2 - x1)) / d
    c = np.array([cx, cy])
    return c, float(np.linalg.norm(p[0] - c))


def convex_hull_2d(pts: np.ndarray) -> np.ndarray:
    """Monotone-chain convex hull, counter-clockwise. No scipy dependency."""
    p = np.unique(np.asarray(pts, dtype=float).round(9), axis=0)
    if len(p) < 3:
        return p
    p = p[np.lexsort((p[:, 1], p[:, 0]))]

    def half(points):
        out: list[np.ndarray] = []
        for q in points:
            while len(out) >= 2:
                a, b = out[-2], out[-1]
                if (b[0] - a[0]) * (q[1] - a[1]) - (b[1] - a[1]) * (q[0] - a[0]) <= 0:
                    out.pop()
                else:
                    break
            out.append(q)
        return out

    lower = half(p)
    upper = half(p[::-1])
    return np.array(lower[:-1] + upper[:-1])


def circle_polygon(center: np.ndarray, radius: float, n: int = 24) -> np.ndarray:
    th = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    return np.stack([center[0] + radius * np.cos(th), center[1] + radius * np.sin(th)], axis=1)


def radial_resample(poly: np.ndarray, center: np.ndarray, n: int = 24) -> np.ndarray:
    """Resample a closed polygon at n uniform angles about `center`."""
    if len(poly) < 3:
        return poly
    out = []
    closed = np.vstack([poly, poly[0]])
    for th in np.linspace(0.0, 2 * math.pi, n, endpoint=False):
        d = np.array([math.cos(th), math.sin(th)])
        best_t = None
        for a, b in zip(closed[:-1], closed[1:]):
            e = b - a
            den = d[0] * e[1] - d[1] * e[0]
            if abs(den) < 1e-15:
                continue
            w = a - center
            t = (w[0] * e[1] - w[1] * e[0]) / den  # distance along d
            s = (d[0] * w[1] - d[1] * w[0]) / -den  # position along the edge
            if t > 0 and -1e-9 <= s <= 1 + 1e-9 and (best_t is None or t > best_t):
                best_t = t
        if best_t is not None:
            out.append(center + best_t * d)
    return np.array(out) if len(out) >= 3 else poly


def newarc_vertices_string(poly: np.ndarray) -> str:
    """Format a polygon for newarc.py --vertices."""
    return ";".join(f"{float(x):.5f},{float(y):.5f}" for x, y in poly)


def build_cross_section(
    pts2d: np.ndarray,
    center: np.ndarray,
    radius: float,
    span_deg: float,
    reliable: bool,
    n_vertices: int = 24,
) -> dict:
    """
    Produce a 2-D cross-section polygon of the object, for grasp-geometry solving.

    A single view only sees part of the circumference, so the measured hull is a
    partial outline. When the circle fit is trustworthy the full cross-section can
    be reconstructed from it; otherwise the polygon is flagged as incomplete and a
    fused multi-view scan is required.
    """
    hull = convex_hull_2d(pts2d)
    nearly_closed = span_deg >= 300.0

    if reliable:
        poly, mode = circle_polygon(center, radius, n_vertices), "circle_fit"
        complete = True
    elif nearly_closed and len(hull) >= 3:
        poly = radial_resample(hull, hull.mean(axis=0), n_vertices)
        mode, complete = "measured_hull", True
    else:
        poly, mode, complete = hull, "partial_hull", False

    return {
        "mode": mode,
        "complete": bool(complete),
        "coverage_deg": float(span_deg),
        "vertices_m": np.asarray(poly, dtype=float).tolist(),
        "hull_vertices_m": np.asarray(hull, dtype=float).tolist(),
        "newarc_vertices": newarc_vertices_string(poly),
    }


def _algebraic_circle(pts: np.ndarray) -> tuple[np.ndarray, float]:
    """Algebraic (Kasa) circle fit. Fast but biased on partial arcs -- init only."""
    x, y = pts[:, 0], pts[:, 1]
    A = np.stack([x, y, np.ones_like(x)], axis=1)
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = sol[0] / 2.0, sol[1] / 2.0
    r = math.sqrt(max(sol[2] + cx * cx + cy * cy, 0.0))
    return np.array([cx, cy]), float(r)


def _geometric_circle(
    pts: np.ndarray, c0: np.ndarray, r0: float, iters: int = 60, r_max: float = 2.0
) -> tuple[np.ndarray, float]:
    """
    Geometric least squares: minimize sum (|p_i - c| - r)^2 by Gauss-Newton.

    Unlike the algebraic fit this is unbiased for partial arcs, which matters
    because a single view only ever sees part of the circumference.
    """
    c, r = np.asarray(c0, dtype=float).copy(), float(r0)
    for _ in range(iters):
        d = pts - c
        dist = np.maximum(np.linalg.norm(d, axis=1), 1e-12)
        f = dist - r
        J = np.column_stack([-d[:, 0] / dist, -d[:, 1] / dist, -np.ones(len(pts))])
        try:
            delta, *_ = np.linalg.lstsq(J, -f, rcond=None)
        except np.linalg.LinAlgError:
            break
        c_new, r_new = c + delta[:2], r + delta[2]
        if not np.isfinite(r_new) or not (1e-4 < abs(r_new) < r_max):
            break  # diverging toward a straight line: keep the last valid estimate
        c, r = c_new, abs(r_new)
        if float(np.linalg.norm(delta)) < 1e-10:
            break
    return c, float(r)


def _lsq_circle(pts: np.ndarray) -> tuple[np.ndarray, float]:
    c, r = _algebraic_circle(pts)
    return _geometric_circle(pts, c, r)


def ransac_circle(
    pts: np.ndarray,
    inlier_tol: float = 0.003,
    iters: int = 300,
    rng: np.random.Generator | None = None,
    r_max: float = 0.25,
    min_span_deg: float = 30.0,
) -> dict:
    """
    Robust 2-D circle fit. `pts` is (N,2) in meters.

    Two gates protect against fitting a *plane* as a giant circle, which is the
    dominant failure mode in cluttered scenes: candidate radii are capped at
    `r_max`, and a candidate is only accepted if its inliers subtend at least
    `min_span_deg` about the fitted center (a planar strip subtends almost
    nothing on the huge circle it suggests). There is deliberately no
    fit-everything fallback -- if no circular consensus exists, we say so.
    """
    rng = rng or np.random.default_rng(0)
    n = len(pts)
    if n < 3:
        raise ValueError("need >= 3 points")

    best_inliers = None
    best_count = -1
    for _ in range(iters):
        idx = rng.choice(n, size=3, replace=False)
        got = _circle_from_three(pts[idx])
        if got is None:
            continue
        c, r = got
        if not (1e-4 < r < r_max):
            continue
        resid = np.abs(np.linalg.norm(pts - c, axis=1) - r)
        inliers = resid < inlier_tol
        count = int(inliers.sum())
        if count > best_count and count >= 3:
            if arc_span_deg(pts[inliers], c) < min_span_deg:
                continue
            best_count, best_inliers = count, inliers

    if best_inliers is None:
        raise ValueError("no circular consensus (structure may be planar)")

    c, r = _lsq_circle(pts[best_inliers])
    resid = np.linalg.norm(pts - c, axis=1) - r
    inliers = np.abs(resid) < inlier_tol
    if inliers.sum() >= 3:
        c, r = _lsq_circle(pts[inliers])
        resid = np.linalg.norm(pts - c, axis=1) - r
        inliers = np.abs(resid) < inlier_tol
    if not (1e-4 < r < r_max):
        raise ValueError("circle refinement diverged past r_max")

    return {
        "center": c,
        "radius": float(r),
        "inliers": inliers,
        "inlier_ratio": float(inliers.mean()),
        "residual_rms": float(np.sqrt(np.mean(resid[inliers] ** 2))) if inliers.any() else float("inf"),
    }


def arc_span_deg(pts2d: np.ndarray, center: np.ndarray) -> float:
    """Angular coverage of the visible arc: 360 minus the largest angular gap."""
    if len(pts2d) < 3:
        return 0.0
    ang = np.sort(np.arctan2(pts2d[:, 1] - center[1], pts2d[:, 0] - center[0]))
    gaps = np.diff(ang)
    wrap = (ang[0] + 2 * math.pi) - ang[-1]
    largest = max(float(gaps.max()) if len(gaps) else 0.0, float(wrap))
    return math.degrees(2 * math.pi - largest)


def _fibonacci_hemisphere(n: int) -> np.ndarray:
    """Roughly uniform directions on a hemisphere (axis sign is irrelevant)."""
    i = np.arange(n) + 0.5
    phi = np.arccos(np.clip(1.0 - i / n, -1.0, 1.0))
    theta = math.pi * (1.0 + math.sqrt(5.0)) * i
    return np.stack(
        [np.sin(phi) * np.cos(theta), np.sin(phi) * np.sin(theta), np.cos(phi)], axis=1
    )


def fit_cylinder(
    points: np.ndarray,
    axis_hint: np.ndarray | None = None,
    inlier_tol: float = 0.003,
    refine: bool = True,
    rng: np.random.Generator | None = None,
    cross_section_vertices: int = 24,
    coarse_directions: int = 96,
    coarse_points: int = 500,
    max_fit_points: int = 4000,
    prefer_exterior: bool = True,
    min_inlier_ratio: float = 0.3,
    _allow_retry: bool = True,
) -> dict:
    """
    Fit a cylinder to a partial surface patch.

    The axis is found by a coarse global direction search followed by local
    refinement. A PCA-only initialization is not reliable here: when the visible
    arc chord is comparable to the visible length along the axis -- which is exactly
    the case for short, wide objects -- PCA picks the arc direction instead of the
    axis and the radius estimate collapses.
    """
    points = np.asarray(points, dtype=float)
    if len(points) < 20:
        raise ValueError("too few points for a cylinder fit")
    rng = rng or np.random.default_rng(0)

    n_total = len(points)
    if n_total > max_fit_points:  # keep per-capture runtime bounded
        points = points[rng.choice(n_total, max_fit_points, replace=False)]

    centroid = points.mean(axis=0)
    X = points - centroid

    def evaluate(ax: np.ndarray, subset: np.ndarray, iters: int) -> dict:
        e1, e2 = orthonormal_basis(ax)
        p2 = np.stack([subset @ e1, subset @ e2], axis=1)
        out = ransac_circle(p2, inlier_tol=inlier_tol, iters=iters, rng=rng)
        # Penalize fits that only explain a small subset of the points.
        ratio = max(out["inlier_ratio"], 1e-6)
        out.update({"axis": ax, "e1": e1, "e2": e2, "pts2d": p2, "cost": out["residual_rms"] / ratio})
        return out

    # ---- coarse global search over axis directions -------------------------
    sub = X if len(X) <= coarse_points else X[rng.choice(len(X), coarse_points, replace=False)]
    _, _, vt = np.linalg.svd(X, full_matrices=False)
    candidates = [normalize(v) for v in vt]  # all three PCA axes, not just the first
    if axis_hint is not None:
        candidates.insert(0, normalize(axis_hint))
    candidates.extend(_fibonacci_hemisphere(coarse_directions))

    scored = []
    for ax in candidates:
        try:
            cand = evaluate(np.asarray(ax, dtype=float), sub, iters=40)
        except Exception:  # noqa: BLE001
            continue
        if cand["inlier_ratio"] >= min_inlier_ratio:
            scored.append(cand)
    if not scored:
        raise ValueError("no viable cylinder axis found")
    scored.sort(key=lambda c: c["cost"])

    # ---- local refinement from the best few coarse candidates -------------
    best = None
    for seed in scored[:3]:
        try:
            current = evaluate(seed["axis"], X, iters=80)
        except Exception:  # noqa: BLE001
            continue
        if refine:
            step = math.radians(12.0)
            for _ in range(5):
                improved = False
                e1, e2 = orthonormal_basis(current["axis"])
                for da in (-step, 0.0, step):
                    for db in (-step, 0.0, step):
                        if da == 0.0 and db == 0.0:
                            continue
                        try:
                            cand = evaluate(normalize(current["axis"] + da * e1 + db * e2), X, iters=80)
                        except Exception:  # noqa: BLE001
                            continue
                        if cand["cost"] < current["cost"]:
                            current, improved = cand, True
                if not improved:
                    step /= 2.0
        if best is None or current["cost"] < best["cost"]:
            best = current
    if best is None:
        raise ValueError("no viable cylinder axis found")

    inl = best["inliers"]
    span = arc_span_deg(best["pts2d"][inl], best["center"])
    axis_center_3d = centroid + best["center"][0] * best["e1"] + best["center"][1] * best["e2"]

    # Conditioning: a narrow arc is nearly a straight line, so the radius becomes
    # weakly observable. The sagitta (bulge height) is what actually carries the
    # radius information; compare it against the surface noise.
    sagitta = best["radius"] * (1.0 - math.cos(math.radians(min(span, 180.0)) / 2.0))
    radius_sigma = _bootstrap_radius_sigma(best["pts2d"][inl], best["center"], best["radius"], rng)
    conditioning = sagitta / best["residual_rms"] if best["residual_rms"] > 0 else float("inf")

    # Which side of the wall are we looking at? A visible surface patch must face
    # the camera (at the origin of the ZED point-cloud frame): exterior points have
    # their outward radial direction toward the camera, interior points the
    # opposite. A container seen into its opening yields a near-full interior ring,
    # so this matters for open objects like cups. Additionally, more than ~180 deg
    # of an *exterior* wall can never be visible from a single view (tangent
    # limit), which gives an independent consistency check.
    axis_center_3d_tmp = centroid + best["center"][0] * best["e1"] + best["center"][1] * best["e2"]
    inlier_pts_3d = (points - centroid)[inl] + centroid
    Xr = inlier_pts_3d - axis_center_3d_tmp
    radial = Xr - (Xr @ best["axis"])[:, None] * best["axis"]
    r_hat = radial / np.maximum(np.linalg.norm(radial, axis=1), 1e-12)[:, None]
    ext_frac = float(np.mean(np.einsum("ij,ij->i", r_hat, -inlier_pts_3d) > 0.0))
    if ext_frac >= 0.7 and span <= 185.0:
        surface = "exterior"
    elif ext_frac <= 0.55 or span > 240.0:
        surface = "interior"
    else:
        surface = "mixed"

    reliable = span >= 60.0 and conditioning >= 3.0 and radius_sigma * 1000.0 <= 5.0

    result = {
        "radius": best["radius"],
        "diameter": 2.0 * best["radius"],
        "axis": best["axis"].tolist(),
        "axis_point": axis_center_3d.tolist(),
        "residual_rms": best["residual_rms"],
        "inlier_ratio": best["inlier_ratio"],
        "arc_span_deg": span,
        "sagitta": float(sagitta),
        "radius_sigma": float(radius_sigma),
        "conditioning": float(conditioning),
        "radius_reliable": bool(reliable),
        "surface": surface,
        "exterior_fraction": ext_frac,
        "n_points": int(n_total),
        "n_fit_points": int(len(points)),
        "cross_section": build_cross_section(
            best["pts2d"][inl], best["center"], best["radius"], span, reliable, cross_section_vertices
        ),
    }

    # For grasping we want the OUTER wall. If a non-exterior structure won the
    # consensus (an open container's interior ring often does), strip its inliers
    # and give the remainder one chance to produce a valid exterior fit.
    if prefer_exterior and _allow_retry and surface != "exterior":
        Xall = points - axis_center_3d_tmp
        rad_all = np.linalg.norm(Xall - (Xall @ best["axis"])[:, None] * best["axis"], axis=1)
        complement = points[np.abs(rad_all - best["radius"]) > 2.0 * inlier_tol]
        if len(complement) >= max(500, int(0.1 * len(points))):
            try:
                retry = fit_cylinder(
                    complement,
                    axis_hint=best["axis"],
                    inlier_tol=inlier_tol,
                    refine=refine,
                    rng=rng,
                    cross_section_vertices=cross_section_vertices,
                    coarse_directions=coarse_directions,
                    coarse_points=coarse_points,
                    max_fit_points=max_fit_points,
                    prefer_exterior=False,
                    min_inlier_ratio=min_inlier_ratio,
                    _allow_retry=False,
                )
            except Exception:  # noqa: BLE001
                retry = None
            if (
                retry is not None
                and retry["surface"] == "exterior"
                and retry["conditioning"] >= 3.0
                and retry["arc_span_deg"] >= 45.0
            ):
                retry["secondary_interior_diameter"] = result["diameter"]
                retry["n_points"] = int(n_total)
                return retry

    return result


def _bootstrap_radius_sigma(
    pts2d: np.ndarray,
    center: np.ndarray,
    radius: float,
    rng: np.random.Generator,
    resamples: int = 24,
    max_points: int = 1500,
) -> float:
    """Empirical std of the fitted radius under resampling."""
    n = len(pts2d)
    if n < 30:
        return float("inf")
    radii = []
    for _ in range(resamples):
        idx = rng.integers(0, n, size=min(n, max_points))
        try:
            _, r = _geometric_circle(pts2d[idx], center, radius)
        except Exception:  # noqa: BLE001
            continue
        radii.append(r)
    return float(np.std(radii)) if len(radii) >= 5 else float("inf")


def remove_dominant_plane(
    points: np.ndarray,
    tol: float = 0.004,
    iters: int = 200,
    min_fraction: float = 0.25,
    rng: np.random.Generator | None = None,
) -> np.ndarray:
    """Drop points on the dominant plane (table/background) if one is large enough."""
    rng = rng or np.random.default_rng(1)
    n = len(points)
    if n < 50:
        return points
    best_mask, best_count = None, 0
    for _ in range(iters):
        idx = rng.choice(n, size=3, replace=False)
        p0, p1, p2 = points[idx]
        nrm = np.cross(p1 - p0, p2 - p0)
        norm = float(np.linalg.norm(nrm))
        if norm < 1e-9:
            continue
        nrm /= norm
        d = np.abs((points - p0) @ nrm)
        mask = d < tol
        count = int(mask.sum())
        if count > best_count:
            best_count, best_mask = count, mask
    if best_mask is None or best_count < min_fraction * n:
        return points
    if _inplane_aspect(points[best_mask]) < 0.25:
        # Elongated strip, not an extended surface: almost certainly a tangent
        # band on a curved wall. Removing it would bite a bias into the object.
        return points
    return points[~best_mask]


def _inplane_aspect(pts: np.ndarray) -> float:
    """Aspect ratio (2nd/1st singular value) of a point set within its own plane."""
    X = pts - pts.mean(axis=0)
    svals = np.linalg.svd(X, compute_uv=False)
    return float(svals[1] / svals[0]) if svals[0] > 1e-12 else 0.0


# --------------------------------------------------------------------------
# capture
# --------------------------------------------------------------------------


def open_camera(args):
    import pyzed.sl as sl

    init = sl.InitParameters()
    init.camera_resolution = getattr(sl.RESOLUTION, args.resolution)
    init.depth_mode = getattr(sl.DEPTH_MODE, args.mode)
    init.coordinate_units = sl.UNIT.METER
    init.depth_minimum_distance = args.min_depth
    init.depth_maximum_distance = args.max_depth
    init.camera_fps = args.fps

    cam = sl.Camera()
    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"camera open failed: {status}")

    rt = sl.RuntimeParameters()
    rt.confidence_threshold = args.confidence
    rt.texture_confidence_threshold = args.texture_confidence
    if hasattr(rt, "enable_fill_mode"):
        rt.enable_fill_mode = bool(args.fill)
    return sl, cam, rt


def grab(sl, cam, rt, warmup: int = 8):
    """Return (left_bgr uint8, xyz float32 HxWx3)."""
    left, cloud = sl.Mat(), sl.Mat()
    for _ in range(warmup):
        cam.grab(rt)
    if cam.grab(rt) != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError("grab failed")
    cam.retrieve_image(left, sl.VIEW.LEFT)
    cam.retrieve_measure(cloud, sl.MEASURE.XYZRGBA)
    img = np.array(left.get_data())[:, :, :3].copy()
    xyz = np.array(cloud.get_data())[:, :, :3].astype(np.float32).copy()
    return img, xyz


def _label_components(mask: np.ndarray) -> tuple[np.ndarray, int]:
    """Two-pass 4-connectivity labeling, pure numpy (no scipy/cv2 dependency)."""
    h, w = mask.shape
    labels = np.zeros((h, w), dtype=np.int32)
    parent = [0]

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    nxt = 1
    for i in range(h):
        for j in range(w):
            if not mask[i, j]:
                continue
            up = labels[i - 1, j] if i > 0 else 0
            left = labels[i, j - 1] if j > 0 else 0
            if up == 0 and left == 0:
                parent.append(nxt)
                labels[i, j] = nxt
                nxt += 1
            elif up and left:
                ru, rl = find(up), find(left)
                labels[i, j] = min(ru, rl)
                parent[max(ru, rl)] = min(ru, rl)
            else:
                labels[i, j] = up or left
    flat = labels.ravel()
    nz = flat > 0
    flat[nz] = np.array([find(v) for v in flat[nz]], dtype=np.int32)
    return labels, nxt


def auto_roi(
    xyz: np.ndarray,
    min_depth: float = 0.15,
    max_depth: float = 1.5,
    stride: int = 4,
    pad_frac: float = 0.10,
    min_cluster_px: int = 150,
) -> tuple[int, int, int, int] | None:
    """
    Propose an object ROI from geometry alone (no detector, no learned model):
    remove dominant planes (table/wall) on a downsampled cloud, take the nearest
    significant depth cluster among what remains, then the largest connected
    component of that cluster. Returns full-resolution (x, y, w, h) or None.
    """
    H, W = xyz.shape[:2]
    sub = xyz[::stride, ::stride, :]
    h, w = sub.shape[:2]
    pts = sub.reshape(-1, 3)
    z = np.abs(pts[:, 2])
    ok = np.isfinite(pts).all(axis=1) & (z > min_depth) & (z < max_depth)
    if ok.sum() < 500:
        return None

    # iterative plane removal with index tracking
    active = np.where(ok)[0]
    rng = np.random.default_rng(3)
    for _ in range(3):
        if len(active) < 500:
            break
        P = pts[active]
        best_mask, best = None, 0
        for _ in range(200):
            i3 = rng.choice(len(P), 3, replace=False)
            p0, p1, p2 = P[i3]
            nrm = np.cross(p1 - p0, p2 - p0)
            nn = float(np.linalg.norm(nrm))
            if nn < 1e-9:
                continue
            nrm /= nn
            m = np.abs((P - p0) @ nrm) < 0.005
            c = int(m.sum())
            if c > best:
                best, best_mask = c, m
        if best_mask is None or best < 0.15 * len(active):
            break
        if _inplane_aspect(P[best_mask]) < 0.25:
            break
        active = active[~best_mask]

    if len(active) < min_cluster_px:
        return None

    # nearest significant depth cluster
    za = np.abs(pts[active, 2])
    hist, edges = np.histogram(za, bins=int((max_depth - min_depth) / 0.02))
    signif = np.where(hist >= min_cluster_px // 3)[0]
    if len(signif) == 0:
        return None
    k = signif[0]
    zc = 0.5 * (edges[k] + edges[k + 1])
    sel = active[np.abs(za - zc) < 0.07]
    if len(sel) < min_cluster_px:
        return None

    mask = np.zeros(h * w, dtype=bool)
    mask[sel] = True
    labels, _ = _label_components(mask.reshape(h, w))
    if labels.max() == 0:
        return None
    counts = np.bincount(labels.ravel())
    counts[0] = 0
    biggest = int(counts.argmax())
    ys, xs = np.where(labels == biggest)
    if len(ys) < min_cluster_px:
        return None

    x0, x1 = int(xs.min()) * stride, int(xs.max()) * stride
    y0, y1 = int(ys.min()) * stride, int(ys.max()) * stride
    pw, ph = int((x1 - x0) * pad_frac) + stride, int((y1 - y0) * pad_frac) + stride
    x0, y0 = max(0, x0 - pw), max(0, y0 - ph)
    x1, y1 = min(W - 1, x1 + pw), min(H - 1, y1 + ph)
    if (x1 - x0) < 30 or (y1 - y0) < 30:
        return None
    return (x0, y0, x1 - x0, y1 - y0)


def select_roi(img):
    import cv2

    r = cv2.selectROI("select object ROI (ENTER to confirm)", img, showCrosshair=True)
    cv2.destroyAllWindows()
    if r[2] == 0 or r[3] == 0:
        raise RuntimeError("empty ROI")
    return tuple(int(v) for v in r)


def extract_points(
    xyz,
    roi,
    depth_window: float,
    remove_plane: bool = True,
    max_planes: int = 3,
    cluster_halfwidth: float = 0.06,
):
    """
    ROI -> object candidate points.

    Cleanup order matters: planes (table, monitors, walls) are removed
    iteratively first -- a single pass is not enough because an oblique noisy
    table splits into several planar consensi -- and only then is the densest
    depth cluster selected, so that the cluster is anchored on the object
    rather than on whatever plane dominated the ROI.
    """
    x, y, w, h = roi
    patch = xyz[y : y + h, x : x + w, :].reshape(-1, 3)
    finite = np.isfinite(patch).all(axis=1)
    valid_ratio = float(finite.mean())
    pts = patch[finite]
    if len(pts) == 0:
        return pts, valid_ratio, float("nan")

    z = np.abs(pts[:, 2])
    z_med = float(np.median(z))
    pts = pts[np.abs(z - z_med) < depth_window]

    if remove_plane:
        for _ in range(max_planes):
            if len(pts) < 500:
                break
            kept = remove_dominant_plane(pts, min_fraction=0.15)
            if len(kept) == len(pts):
                break
            pts = kept

    if len(pts) > 500:
        z = np.abs(pts[:, 2])
        hist, edges = np.histogram(z, bins=40)
        zc = 0.5 * (edges[hist.argmax()] + edges[hist.argmax() + 1])
        pts = pts[np.abs(z - zc) < cluster_halfwidth]
        z_med = float(zc)

    return pts, valid_ratio, z_med


def write_ply(path, pts):
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("end_header\n")
        for p in pts:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


def _render_wrap_png(
    module,
    verts,
    settings,
    solution,
    out_png: str,
    segment_length: float,
) -> bool:
    """
    Save a picture of HOW the arm wraps the measured cross-section.

    Uses newarc's own mesh construction (module._solution_mesh) so the chain pose
    is exactly what the solver decided -- we only draw, never re-derive geometry.
    Falls back gracefully if the module internals differ.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        tris = None
        try:
            target = module.make_target(verts, sdf_resolution=settings.sdf_resolution)
            if hasattr(module, "_solution_mesh"):
                tris = module._solution_mesh(target, solution)
        except Exception:  # noqa: BLE001
            tris = None

        fig, ax = plt.subplots(figsize=(7, 7))

        poly = np.asarray(verts, dtype=float)
        ax.fill(poly[:, 0], poly[:, 1], alpha=0.35, label="measured cross-section")
        ax.plot(np.r_[poly[:, 0], poly[0, 0]], np.r_[poly[:, 1], poly[0, 1]], lw=1.2)

        if tris:
            for t in tris:
                t = np.asarray(t, dtype=float)
                if t.ndim == 2 and t.shape[1] >= 2:
                    ax.fill(t[:, 0], t[:, 1], color="0.45", alpha=0.5, lw=0)
        for name, marker in (("p1", "o"), ("p2", "s")):
            pt = getattr(solution, name, None)
            if pt is not None:
                pt = np.asarray(pt, dtype=float).ravel()
                if pt.size >= 2:
                    ax.plot(pt[0], pt[1], marker, ms=9, mfc="none", mew=2, label=f"contact {name}")

        t1 = math.degrees(float(solution.gripper.turn1))
        t2 = math.degrees(float(solution.gripper.turn2))
        req = float(solution.length)
        fits = req <= segment_length
        ax.set_title(
            f"wrap solution: turns=({t1:+.1f}, {t2:+.1f}) deg\n"
            f"required arc {req*1000:.1f} mm vs segment {segment_length*1000:.1f} mm "
            f"-> {'FITS' if fits else 'TOO LARGE'}"
        )
        ax.set_aspect("equal")
        ax.legend(loc="lower right", fontsize=8)
        ax.set_xlabel("m")
        fig.tight_layout()
        fig.savefig(out_png, dpi=120)
        plt.close(fig)
        return True
    except Exception as exc:  # noqa: BLE001
        print(f"[newarc] wrap visualization skipped: {exc}")
        return False


def solve_with_newarc(cross_section: dict, args, out_png: str | None = None) -> dict | None:
    """
    Optionally hand the measured cross-section to newarc.py and report whether the
    real arm can wrap it.

    newarc.py solves for the arc length the pocket needs. Because the geometry only
    depends on (object size)/(arc length), a solution found for the measured object
    is realizable on hardware iff the required arc length does not exceed the
    physical segment length.
    """
    if not args.newarc_path:
        return None
    import importlib.util

    path = os.path.abspath(args.newarc_path)
    if not os.path.exists(path):
        print(f"[newarc] not found: {path}")
        return None

    spec = importlib.util.spec_from_file_location("newarc", path)
    if spec is None or spec.loader is None:
        print("[newarc] could not load module")
        return None
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception as exc:  # noqa: BLE001  (shapely/scipy may be missing)
        print(f"[newarc] import failed ({exc}); use --vertices manually")
        return None

    verts = cross_section["vertices_m"]
    if len(verts) < 3:
        return None
    try:
        settings = module.Settings(
            n=args.gripper_ratio,
            solution_count=3,
            time_limit=args.newarc_time_limit,
        ).checked()
        solutions, elapsed = module.solve_vertices(verts, settings)
    except Exception as exc:  # noqa: BLE001
        print(f"[newarc] solve failed: {exc}")
        return None

    if not solutions:
        print("[newarc] no feasible wrap found for this cross-section")
        return {"feasible": False, "elapsed": elapsed}

    best = solutions[0]
    required = float(best.length)
    fits = required <= args.segment_length
    viz_saved = False
    if out_png:
        viz_saved = _render_wrap_png(module, verts, settings, best, out_png, args.segment_length)
    out = {
        "feasible": True,
        "required_arc_length_m": required,
        "segment_length_m": args.segment_length,
        "fits_hardware": bool(fits),
        "turn1_deg": float(np.degrees(best.gripper.turn1)),
        "turn2_deg": float(np.degrees(best.gripper.turn2)),
        "score": float(best.score),
        "elapsed": float(elapsed),
        "n_solutions": len(solutions),
        "viz_png": out_png if viz_saved else None,
    }
    print(
        f"[newarc] required arc {required*1000:.1f} mm vs segment {args.segment_length*1000:.1f} mm"
        f" -> {'FITS' if fits else 'TOO LARGE'}   turns=({out['turn1_deg']:+.1f}, {out['turn2_deg']:+.1f}) deg"
    )
    return out


CSV_FIELDS = [
    "timestamp", "label", "mode", "resolution", "fill", "confidence", "texture_confidence",
    "distance_m", "valid_ratio", "n_points", "diameter_m", "gt_diameter_m", "diameter_err_mm",
    "residual_rms_mm", "radius_sigma_mm", "conditioning", "inlier_ratio", "arc_span_deg",
    "cs_mode", "cs_complete", "surface", "required_arc_mm", "fits_hardware", "turn1_deg", "turn2_deg",
    "verdict",
]


def verdict(valid_ratio, fit, gt_diameter):
    """Pass/fail on what actually decides feasibility of size estimation."""
    if fit is None:
        return "FAIL: no fit"
    reasons = []
    if valid_ratio < 0.5:
        reasons.append("sparse depth")
    if fit["arc_span_deg"] < 60:
        reasons.append("arc too narrow")
    if fit.get("surface") != "exterior":
        reasons.append(f"non-exterior surface ({fit.get('surface')}; open container interior?)")
    if fit["conditioning"] < 3.0:
        reasons.append("radius ill-conditioned (sagitta < 3x noise)")
    if fit["radius_sigma"] * 1000 > 5.0:
        reasons.append("radius sigma > 5mm")
    if gt_diameter is not None and abs(fit["diameter"] - gt_diameter) * 1000 > 10:
        reasons.append("size error > 10mm")
    return "PASS" if not reasons else "MARGINAL: " + ", ".join(reasons)


def run_capture(args):
    import cv2

    os.makedirs(args.outdir, exist_ok=True)
    csv_path = os.path.join(args.outdir, "results.csv")
    new_csv = not os.path.exists(csv_path)

    sl, cam, rt = open_camera(args)
    fixed_roi = tuple(int(v) for v in args.roi.split(",")) if args.roi else None
    unlimited = args.trials <= 0
    if unlimited:
        print("[info] continuous mode -- press Ctrl-C to stop")

    completed = 0
    try:
        with open(csv_path, "a", newline="") as fcsv:
            writer = csv.DictWriter(fcsv, fieldnames=CSV_FIELDS)
            if new_csv:
                writer.writeheader()

            trial = 0
            interactive_roi = None
            while unlimited or trial < args.trials:
                img, xyz = grab(sl, cam, rt)
                if fixed_roi is not None:
                    roi = fixed_roi
                elif args.roi_interactive:
                    if interactive_roi is None:
                        interactive_roi = select_roi(img)
                    roi = interactive_roi
                else:
                    roi = auto_roi(xyz, min_depth=args.min_depth, max_depth=args.max_depth)
                    if roi is None:
                        print("[warn] auto ROI failed (no non-planar cluster); falling back to interactive")
                        roi = select_roi(img)
                print(f"[roi] {roi[0]},{roi[1]},{roi[2]},{roi[3]}")

                pts, valid_ratio, dist = extract_points(
                    xyz, roi, args.depth_window, remove_plane=not args.keep_planes
                )

                fit = None
                if len(pts) >= 20:
                    hint = (
                        np.array([float(v) for v in args.axis_hint.split(",")])
                        if args.axis_hint
                        else None
                    )
                    try:
                        fit = fit_cylinder(
                            pts,
                            axis_hint=hint,
                            inlier_tol=args.inlier_tol,
                            cross_section_vertices=args.cs_vertices,
                        )
                    except Exception as exc:  # noqa: BLE001
                        print(f"[warn] fit failed: {exc}")

                grasp = (
                    solve_with_newarc(fit["cross_section"], args)
                    if (fit and fit.get("surface") == "exterior")
                    else None
                )

                gt = args.gt_diameter
                cs = fit["cross_section"] if fit else None
                row = {
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "label": args.label,
                    "mode": args.mode,
                    "resolution": args.resolution,
                    "fill": int(bool(args.fill)),
                    "confidence": args.confidence,
                    "texture_confidence": args.texture_confidence,
                    "distance_m": round(dist, 4) if dist == dist else "",
                    "valid_ratio": round(valid_ratio, 4),
                    "n_points": len(pts),
                    "diameter_m": round(fit["diameter"], 5) if fit else "",
                    "gt_diameter_m": gt if gt else "",
                    "diameter_err_mm": round((fit["diameter"] - gt) * 1000, 2) if (fit and gt) else "",
                    "residual_rms_mm": round(fit["residual_rms"] * 1000, 2) if fit else "",
                    "radius_sigma_mm": round(fit["radius_sigma"] * 1000, 2) if fit else "",
                    "conditioning": round(fit["conditioning"], 2) if fit else "",
                    "inlier_ratio": round(fit["inlier_ratio"], 3) if fit else "",
                    "arc_span_deg": round(fit["arc_span_deg"], 1) if fit else "",
                    "cs_mode": cs["mode"] if cs else "",
                    "cs_complete": int(cs["complete"]) if cs else "",
                    "surface": fit.get("surface", "") if fit else "",
                    "required_arc_mm": round(grasp["required_arc_length_m"] * 1000, 1)
                    if (grasp and grasp.get("feasible")) else "",
                    "fits_hardware": int(grasp["fits_hardware"])
                    if (grasp and grasp.get("feasible")) else "",
                    "turn1_deg": round(grasp["turn1_deg"], 1) if (grasp and grasp.get("feasible")) else "",
                    "turn2_deg": round(grasp["turn2_deg"], 1) if (grasp and grasp.get("feasible")) else "",
                    "verdict": verdict(valid_ratio, fit, gt),
                }
                writer.writerow(row)
                fcsv.flush()

                stem = os.path.join(args.outdir, f"{args.label}_{trial:03d}")
                x, y, w, h = roi
                marked = img.copy()
                cv2.rectangle(marked, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.imwrite(stem + "_left.png", marked)

                depth_vis = sl.Mat()
                cam.retrieve_image(depth_vis, sl.VIEW.DEPTH)
                cv2.imwrite(stem + "_depth.png", np.array(depth_vis.get_data())[:, :, :3])

                if len(pts):
                    write_ply(stem + ".ply", pts)
                if cs:
                    with open(stem + "_cross_section.txt", "w") as fcs:
                        fcs.write(cs["newarc_vertices"] + "\n")
                with open(stem + ".json", "w") as fj:
                    json.dump({"roi": roi, "row": row, "fit": fit, "grasp": grasp}, fj, indent=2)

                print(
                    f"[{trial:03d}] valid={valid_ratio:6.1%}  n={len(pts):6d}  "
                    f"d={row['diameter_m'] or '-'}  err={row['diameter_err_mm'] or '-'}mm  "
                    f"arc={row['arc_span_deg'] or '-'}deg  sigma_r={row['radius_sigma_mm'] or '-'}mm  "
                    f"surf={row['surface'] or '-'}  cs={row['cs_mode'] or '-'}  -> {row['verdict']}"
                )
                if cs and not args.newarc_path:
                    print(f'         newarc: --vertices "{cs["newarc_vertices"]}"')

                completed = trial + 1
                trial += 1
                if args.interval > 0 and (unlimited or trial < args.trials):
                    time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\n[info] interrupted by user -- stopping cleanly")
    finally:
        try:
            cam.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:  # noqa: BLE001
            pass

    print(f"\n{completed} capture(s) written -> {csv_path}")
    return 0


# --------------------------------------------------------------------------
# self-test (no hardware)
# --------------------------------------------------------------------------


def selftest() -> int:
    rng = np.random.default_rng(42)
    failures = 0

    def synth(radius, axis, arc_deg, length, n, noise):
        axis = normalize(axis)
        e1, e2 = orthonormal_basis(axis)
        th = rng.uniform(-math.radians(arc_deg) / 2, math.radians(arc_deg) / 2, n)
        t = rng.uniform(-length / 2, length / 2, n)
        pts = (
            radius * np.cos(th)[:, None] * e1
            + radius * np.sin(th)[:, None] * e2
            + t[:, None] * axis
        )
        return pts + rng.normal(0.0, noise, pts.shape) + np.array([0.1, -0.05, 0.4])

    # (name, radius, axis, arc_deg, length, n_points, noise, expect_reliable)
    # The "short and wide" cases matter most: for wrap-grasp targets the visible arc
    # chord is comparable to the visible length, which defeats a PCA-only axis guess.
    cases = [
        ("26mm pipe,   120deg arc, 1.0mm noise", 0.013, [0, 0, 1], 120, 0.08, 3000, 0.0010, True),
        ("100mm obj,    90deg arc, 2.0mm noise", 0.050, [0, 1, 0.2], 90, 0.10, 3000, 0.0020, True),
        ("200mm obj,    70deg arc, 2.0mm noise", 0.100, [1, 0, 0.1], 70, 0.12, 4000, 0.0020, True),
        ("100mm short,  115deg arc, 1.0mm noise", 0.050, [0, 0, 1], 115, 0.08, 2000, 0.0010, True),
        ("160mm short,  100deg arc, 2.0mm noise", 0.080, [0.3, 1, 0], 100, 0.09, 2500, 0.0020, True),
        ("40mm obj,     40deg arc, 1.5mm noise", 0.020, [0, 0, 1], 40, 0.08, 3000, 0.0015, False),
    ]
    for name, r, axis, arc, length, n, noise, expect_reliable in cases:
        pts = synth(r, axis, arc, length, n, noise)
        fit = fit_cylinder(pts, inlier_tol=max(3 * noise, 0.002), rng=rng)
        err_mm = (fit["radius"] - r) * 1000
        ang = math.degrees(
            math.acos(min(1.0, abs(float(np.dot(normalize(fit["axis"]), normalize(axis))))))
        )
        flagged = fit["conditioning"] < 3.0 or fit["radius_sigma"] * 1000 > 5.0

        if expect_reliable:
            ok = abs(err_mm) < 5.0 and ang < 10.0 and not flagged
            note = ""
        else:
            # A near-flat arc cannot determine the radius; the fit must SAY so.
            ok = flagged
            note = "  (expected to be flagged unreliable)"

        print(
            f"  {'ok ' if ok else 'BAD'} {name}{note}\n"
            f"      radius {r*1000:6.1f} -> {fit['radius']*1000:7.1f} mm (err {err_mm:+7.2f} mm), "
            f"axis err {ang:4.1f} deg\n"
            f"      arc {fit['arc_span_deg']:5.1f} deg, rms {fit['residual_rms']*1000:.2f} mm, "
            f"sagitta/noise {fit['conditioning']:5.1f}, sigma_r {fit['radius_sigma']*1000:6.2f} mm, "
            f"flagged={flagged}"
        )
        if not ok:
            failures += 1

    # ---- open-container scenes (regression for the cup failure) ----------
    def cup_scene(center, axis, r_out, r_in, with_exterior):
        axis = normalize(np.asarray(axis, dtype=float))
        e1, e2 = orthonormal_basis(axis)
        parts = []
        if with_exterior:
            # exterior wall arc facing the camera at the origin
            to_cam = -np.asarray(center)
            to_cam -= (to_cam @ axis) * axis
            phi0 = math.atan2(to_cam @ e2, to_cam @ e1)
            phi = phi0 + rng.uniform(-1.2, 1.2, 2500)
            t = rng.uniform(-0.03, 0.03, 2500)
            parts.append(
                center + r_out * np.cos(phi)[:, None] * e1
                + r_out * np.sin(phi)[:, None] * e2 + t[:, None] * axis
            )
        # interior: near-full inner ring, deeper along the axis
        phi = rng.uniform(-math.pi, math.pi, 2500)
        t = rng.uniform(0.01, 0.05, 2500)
        parts.append(
            center + r_in * np.cos(phi)[:, None] * e1
            + r_in * np.sin(phi)[:, None] * e2 + t[:, None] * axis
        )
        pts = np.vstack(parts) + rng.normal(0, 0.0012, (sum(len(x) for x in parts), 3))
        return pts

    center = np.array([0.05, -0.05, 0.32])
    axis = [0.0, 0.95, 0.31]

    scene = cup_scene(center, axis, r_out=0.067, r_in=0.042, with_exterior=True)
    fitA = fit_cylinder(scene, inlier_tol=0.003, rng=rng)
    okA = fitA["surface"] == "exterior" and abs(fitA["radius"] - 0.067) * 1000 < 4
    print(
        f"  {'ok ' if okA else 'BAD'} open cup, exterior intact -> "
        f"d={fitA['diameter']*1000:.1f} mm (true 134.0), surface={fitA['surface']}, "
        f"ext_frac={fitA['exterior_fraction']:.2f}"
    )
    failures += 0 if okA else 1

    scene = cup_scene(center, axis, r_out=0.067, r_in=0.042, with_exterior=False)
    fitB = fit_cylinder(scene, inlier_tol=0.003, rng=rng)
    okB = fitB["surface"] != "exterior" and abs(fitB["radius"] - 0.042) * 1000 < 4
    print(
        f"  {'ok ' if okB else 'BAD'} open cup, exterior lost (specular) -> "
        f"d={fitB['diameter']*1000:.1f} mm (true 84.0), surface={fitB['surface']} "
        f"(expected non-exterior flag)"
    )
    failures += 0 if okB else 1

    # a bare plane must FAIL loudly, never fit as a giant circle
    plane_pts = rng.uniform(-0.12, 0.12, (5000, 3))
    plane_pts[:, 2] = 0.35 + 0.3 * plane_pts[:, 1] + rng.normal(0, 0.001, 5000)
    try:
        bad = fit_cylinder(plane_pts, inlier_tol=0.003, rng=rng)
        okC = False
        print(f"  BAD bare plane fitted as d={bad['diameter']*1000:.0f} mm circle (must fail)")
    except ValueError:
        okC = True
        print("  ok  bare plane -> explicit failure (no circular consensus)")
    failures += 0 if okC else 1

    # plane removal
    plane = rng.uniform(-0.2, 0.2, (4000, 3))
    plane[:, 2] = 0.4 + rng.normal(0, 0.0005, 4000)
    obj = synth(0.03, [0, 0, 1], 140, 0.08, 1500, 0.001)
    kept = remove_dominant_plane(np.vstack([plane, obj]))
    frac = len(kept) / (len(plane) + len(obj))
    ok = 0.2 < frac < 0.45
    print(f"  {'ok ' if ok else 'BAD'} plane removal kept {frac:.1%} of points")
    failures += 0 if ok else 1

    print("\nselftest:", "PASS" if failures == 0 else f"{failures} FAILURE(S)")
    return 1 if failures else 0


# --------------------------------------------------------------------------


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--selftest", action="store_true", help="verify fitting math without a camera")
    p.add_argument("--label", default="capture", help="object/condition name used in filenames")
    p.add_argument("--mode", default="NEURAL_PLUS",
                   choices=["NEURAL_LIGHT", "NEURAL", "NEURAL_PLUS", "ULTRA", "QUALITY", "PERFORMANCE"])
    p.add_argument("--resolution", default="HD1080", choices=["VGA", "HD720", "HD1080", "HD2K"])
    p.add_argument("--fps", type=int, default=15)
    p.add_argument("--confidence", type=int, default=50, help="lower = stricter depth filtering")
    p.add_argument("--texture-confidence", type=int, default=100,
                   help="lower = reject low-texture pixels (key knob for matte objects)")
    p.add_argument("--fill", action="store_true", help="enable FILL mode (interpolates holes; test separately)")
    p.add_argument("--min-depth", type=float, default=0.15)
    p.add_argument("--max-depth", type=float, default=2.0)
    p.add_argument("--roi", default=None, help="x,y,w,h manual override (default: automatic)")
    p.add_argument("--roi-interactive", action="store_true", help="select ROI by mouse instead of auto")
    p.add_argument("--depth-window", type=float, default=0.12, help="keep points within +/- this of median depth")
    p.add_argument("--keep-planes", action="store_true",
                   help="disable plane removal / depth clustering (cleanup is ON by default)")
    p.add_argument("--axis-hint", default=None, help="x,y,z prior for the cylinder axis")
    p.add_argument("--inlier-tol", type=float, default=0.003)
    p.add_argument("--gt-diameter", type=float, default=None, help="ground-truth diameter [m] from calipers")
    p.add_argument("--trials", type=int, default=1, help="0 or less = run until Ctrl-C")
    p.add_argument("--interval", type=float, default=0.0, help="seconds to wait between captures")
    p.add_argument("--cs-vertices", type=int, default=24, help="vertex count of the exported cross-section")
    p.add_argument("--newarc-path", default=None,
                   help="path to newarc.py; if given, solve the wrap grasp directly")
    p.add_argument("--segment-length", type=float, default=0.2075,
                   help="physical arc length of one segment [m] (n_disk * h)")
    p.add_argument("--gripper-ratio", type=float, default=0.155,
                   help="newarc 'n': gripper protrusion / segment arc length")
    p.add_argument("--newarc-time-limit", type=float, default=10.0)
    p.add_argument("--outdir", default="./bench_out")
    args = p.parse_args()

    try:
        raise SystemExit(selftest() if args.selftest else run_capture(args))
    except KeyboardInterrupt:
        print("\n[info] interrupted")
        raise SystemExit(130)


if __name__ == "__main__":
    main()
