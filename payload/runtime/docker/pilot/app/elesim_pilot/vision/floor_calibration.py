"""Metric plane observations and the three camera-pose constraints they supply.

All fitting happens in the original optical frame. A confirmed reference is
fixed; fitting the reference anew after every correction would hide the very
kinematic error this observer is intended to measure. No motor commands live
at this boundary.
"""
from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass, replace
import math

import numpy as np
from scipy.spatial import ConvexHull, QhullError
from scipy.spatial.transform import Rotation, Slerp


def camera_transform(value) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (4, 4) or not np.all(np.isfinite(result)):
        raise ValueError("camera transform must be a finite 4x4 matrix")
    if (not np.allclose(result[3], [0, 0, 0, 1], atol=1e-7)
            or not np.allclose(result[:3, :3].T @ result[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(result[:3, :3]), 1, atol=1e-6)):
        raise ValueError("camera transform must contain a proper rigid rotation")
    return result.copy()


@dataclass(frozen=True)
class Plane:
    normal: tuple[float, float, float]
    offset_m: float
    rms_m: float = 0.0
    inliers: int = 0
    samples: int = 0
    area_m2: float = 0.0
    span_m: tuple[float, float] = (0.0, 0.0)
    outline: tuple[tuple[float, float, float], ...] = ()

    def __post_init__(self):
        normal = np.asarray(self.normal, dtype=float)
        length = float(np.linalg.norm(normal))
        if normal.shape != (3,) or not np.all(np.isfinite(normal)) or length < 1e-10:
            raise ValueError("plane normal must be a finite nonzero three-vector")
        if not math.isfinite(self.offset_m):
            raise ValueError("plane offset must be finite")
        object.__setattr__(self, "normal", tuple(float(v) for v in normal/length))
        object.__setattr__(self, "offset_m", float(self.offset_m)/length)

    def flipped(self):
        return replace(self, normal=tuple(-v for v in self.normal), offset_m=-self.offset_m)

    def payload(self):
        return {"normal": list(self.normal), "offset_m": self.offset_m,
                "rms_m": self.rms_m, "inliers": self.inliers, "samples": self.samples,
                "area_m2": self.area_m2, "span_m": list(self.span_m),
                "outline": [list(p) for p in self.outline]}


@dataclass(frozen=True)
class FloorSettings:
    max_points: int = 4096
    max_planes: int = 4
    iterations: int = 128
    distance_m: float = 0.006
    range_noise_m: float = 0.002
    min_inliers: int = 100
    min_area_m2: float = 0.08
    min_span_m: float = 0.15
    max_rms_m: float = 0.015
    normal_max_deg: float = 25.0
    match_angle_deg: float = 15.0
    match_distance_m: float = 0.18
    stable_frames: int = 3
    stale_after_s: float = 0.5
    future_tolerance_s: float = 0.03
    min_height_m: float = 0.10
    max_height_m: float = 2.5

    def __post_init__(self):
        for name in ("max_points", "max_planes", "iterations", "min_inliers", "stable_frames"):
            value = getattr(self, name)
            if type(value) is not int or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        if self.max_points > 100_000 or self.max_planes > 8 or self.iterations > 1024:
            raise ValueError("plane fitting budget exceeds its bounds")
        for name in ("distance_m", "min_area_m2", "min_span_m", "match_distance_m",
                     "stale_after_s", "min_height_m", "max_height_m", "max_rms_m"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not 0 < self.normal_max_deg < 90 or not 0 < self.match_angle_deg < 90:
            raise ValueError("plane angle limits must be within (0, 90)")
        if self.max_height_m <= self.min_height_m:
            raise ValueError("invalid camera height range")
        if (not math.isfinite(self.range_noise_m) or self.range_noise_m < 0
                or not math.isfinite(self.future_tolerance_s) or self.future_tolerance_s < 0):
            raise ValueError("invalid plane noise/time allowance")


def depth_points(depth, intrinsics, scale: float, *, max_points=4096, excluded=None):
    """Deproject a spatially distributed, bounded set of original depth pixels."""
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("depth scale must be finite and positive")
    image = np.asarray(depth)
    if image.ndim != 2 or image.shape != (intrinsics.height, intrinsics.width):
        raise ValueError("depth dimensions do not match camera intrinsics")
    values = np.array([intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy], dtype=float)
    if not np.all(np.isfinite(values)) or np.any(values[:2] <= 0):
        raise ValueError("invalid camera intrinsics")
    if type(max_points) is not int or not 3 <= max_points <= 100_000:
        raise ValueError("invalid depth point budget")
    if excluded is not None and np.shape(excluded) != image.shape:
        raise ValueError("depth exclusion mask dimensions do not match")
    stride = max(1, int(np.ceil(np.sqrt(image.size/max_points))))
    vv, uu = np.mgrid[0:image.shape[0]:stride, 0:image.shape[1]:stride]
    z = image[::stride, ::stride].astype(float)*scale
    valid = np.isfinite(z) & (z >= 0.10) & (z <= 5.0)
    if excluded is not None:
        valid &= ~np.asarray(excluded, dtype=bool)[::stride, ::stride]
    z, u, v = z[valid], uu[valid], vv[valid]
    points = np.column_stack(((u-intrinsics.cx)*z/intrinsics.fx,
                              (v-intrinsics.cy)*z/intrinsics.fy, z))
    return np.ascontiguousarray(points[:max_points])


def _fit(points, tolerances, samples):
    weights = 1/np.square(tolerances)
    center = np.average(points, axis=0, weights=weights)
    centered = points-center
    covariance = (centered*weights[:, None]).T @ centered/np.sum(weights)
    eigenvalues, basis = np.linalg.eigh(covariance)
    if eigenvalues[1] < 1e-8:
        return None
    normal = basis[:, 0]
    offset = -float(normal @ center)
    if offset < 0:
        normal, offset = -normal, -offset
    xy = centered @ basis[:, 1:]
    try:
        area = float(ConvexHull(xy).volume)
    except QhullError:
        return None
    lower, upper = np.quantile(xy, [0.01, 0.99], axis=0)
    outline = [center + basis[:, 1:3] @ np.array([x, y])
               for x, y in ((lower[0], lower[1]), (upper[0], lower[1]),
                            (upper[0], upper[1]), (lower[0], upper[1]))]
    residuals = points @ normal + offset
    return Plane(tuple(normal), offset, float(np.sqrt(np.mean(residuals**2))),
                 len(points), samples, area, tuple(upper-lower),
                 tuple(tuple(float(v) for v in p) for p in outline))


def detect_planes(points, settings: FloorSettings | None = None):
    cfg = settings or FloorSettings()
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("plane points must have shape Nx3")
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) > cfg.max_points:
        points = points[np.linspace(0, len(points)-1, cfg.max_points, dtype=int)]
    samples, result = len(points), []
    rng = np.random.default_rng(42)
    for _ in range(cfg.max_planes):
        if len(points) < cfg.min_inliers:
            break
        tolerances = cfg.distance_m + cfg.range_noise_m*np.square(points[:, 2])
        best, count = None, 0
        required_trials = cfg.iterations
        for trial in range(cfg.iterations):
            a, b, c = points[rng.choice(len(points), 3, replace=False)]
            normal = np.cross(b-a, c-a)
            length = np.linalg.norm(normal)
            if length < 1e-10:
                continue
            normal /= length
            inliers = np.abs(np.einsum("ij,j->i", points-a, normal)) < tolerances
            n = int(np.count_nonzero(inliers))
            if n > count:
                best, count = inliers, n
                # Standard RANSAC confidence bound: retain the configured cap
                # for difficult data, stop early on a well-supported plane.
                probability = (n/len(points))**3
                trials = (1 if probability >= 1-1e-12 else
                          math.ceil(math.log(0.001)/math.log1p(-probability)))
                required_trials = min(required_trials, max(12, trials))
            if trial+1 >= required_trials:
                break
        if best is None or count < cfg.min_inliers:
            break
        plane = _fit(points[best], tolerances[best], samples)
        if plane is None:
            points = points[~best]
            continue
        for _ in range(2):
            inliers = np.abs(np.einsum("ij,j->i", points, plane.normal)+plane.offset_m) < tolerances
            if np.count_nonzero(inliers) < cfg.min_inliers:
                break
            refined = _fit(points[inliers], tolerances[inliers], samples)
            if refined is None:
                break
            plane, best = refined, inliers
        if plane.area_m2 >= cfg.min_area_m2 and min(plane.span_m) >= cfg.min_span_m:
            result.append(plane)
        points = points[~best]
    return result


def plane_in_world(plane: Plane, transform):
    pose = camera_transform(transform)
    normal = pose[:3, :3] @ plane.normal
    outline = tuple(tuple(pose[:3, :3] @ p + pose[:3, 3]) for p in plane.outline)
    return replace(plane, normal=tuple(normal),
                   offset_m=plane.offset_m-float(normal @ pose[:3, 3]), outline=outline)


def plane_in_camera(plane: Plane, transform):
    pose = camera_transform(transform)
    inverse = np.linalg.inv(pose)
    return plane_in_world(plane, inverse)


def correct_camera_pose(predicted, observed: Plane, reference: Plane):
    """Apply only minimum tilt and normal translation; leave the plane's gauge free."""
    pose = camera_transform(predicted)
    current = pose[:3, :3] @ observed.normal
    target = np.asarray(reference.normal)
    if np.dot(current, target) < 0:
        observed = observed.flipped()
        current = -current
    cross = np.cross(current, target)
    sine, cosine = np.linalg.norm(cross), float(np.clip(current @ target, -1, 1))
    if sine > 1e-12:
        pose[:3, :3] = Rotation.from_rotvec(cross/sine * np.arctan2(sine, cosine)).as_matrix() @ pose[:3, :3]
    height = float(target @ pose[:3, 3]) + reference.offset_m
    pose[:3, 3] += target*(observed.offset_m-height)
    return pose


class PoseHistory:
    """Bounded source-clock pose history; never labels receipt time as capture time."""

    def __init__(self, *, max_gap_s=0.15, max_samples=256):
        self.max_gap_s = float(max_gap_s)
        self.max_samples = int(max_samples)
        if not math.isfinite(self.max_gap_s) or self.max_gap_s <= 0 or self.max_samples < 2:
            raise ValueError("invalid pose history bounds")
        self._identity = None
        self._samples = []

    def add(self, stamp, transform, *, identity):
        if not math.isfinite(stamp) or stamp <= 0:
            raise ValueError("pose timestamp must be finite and positive")
        pose = camera_transform(transform)
        if identity != self._identity:
            self._samples.clear()
            self._identity = identity
        times = [p[0] for p in self._samples]
        index = bisect_left(times, stamp)
        if index < len(times) and times[index] == stamp:
            self._samples[index] = (float(stamp), pose)
        else:
            self._samples.insert(index, (float(stamp), pose))
        self._samples = self._samples[-self.max_samples:]

    def at(self, stamp, *, identity):
        if identity != self._identity or not math.isfinite(stamp) or not self._samples:
            return None
        times = [p[0] for p in self._samples]
        index = bisect_left(times, stamp)
        if index < len(times) and abs(times[index]-stamp) < 1e-8:
            return self._samples[index][1].copy()
        if index == 0 or index == len(times):
            return None
        ta, a = self._samples[index-1]
        tb, b = self._samples[index]
        if tb-ta > self.max_gap_s:
            return None
        fraction = (stamp-ta)/(tb-ta)
        result = np.eye(4)
        result[:3, 3] = (1-fraction)*a[:3, 3] + fraction*b[:3, 3]
        result[:3, :3] = Slerp([ta, tb], Rotation.from_matrix([a[:3, :3], b[:3, :3]]))([stamp]).as_matrix()[0]
        return result


class FloorEstimator:
    """Track confirmed metric geometry without silently promoting tables to floors."""

    def __init__(self, settings=None, *, reference_height_m=None):
        self.settings = settings or FloorSettings()
        if reference_height_m is not None and not math.isfinite(reference_height_m):
            raise ValueError("floor reference height must be finite")
        self._configured_height = reference_height_m
        self._next_id = 0
        self.reset()

    def reset(self):
        self._identity = None
        self._last_stamp = 0.0
        self._last_valid_stamp = 0.0
        self._previous = []
        self._reference = (None if self._configured_height is None
                           else Plane((0, 0, 1), -self._configured_height))
        self._reference_kind = "configured" if self._reference is not None else ""
        self._corrected = None
        self._result = {"state": "searching", "valid": False, "reason": "waiting_for_depth",
                        "candidates": [], "normal_error_deg": None, "distance_error_m": None,
                        "required_stable_frames": self.settings.stable_frames,
                        "capture_time_s": 0.0, "observable": ["tilt_2d", "normal_distance"],
                        "unobservable": ["plane_translation_2d", "rotation_about_normal"]}

    def snapshot(self, *, now):
        result = dict(self._result)
        result["candidates"] = [dict(p) for p in result["candidates"]]
        result["reference"] = (None if self._reference is None else {
            **self._reference.payload(), "kind": self._reference_kind,
            "absolute": self._reference_kind == "configured"})
        if self._last_stamp and now-self._last_stamp > self.settings.stale_after_s:
            result.update(state="lost", valid=False, reason="stale_depth",
                          normal_error_deg=None, distance_error_m=None)
        return result

    def confirm(self, candidate_id, *, now):
        snapshot = self.snapshot(now=now)
        if snapshot["state"] == "lost":
            raise ValueError("cannot confirm a stale floor candidate")
        candidate = next((p for p in self._previous if p["id"] == candidate_id), None)
        if candidate is None or candidate["count"] < self.settings.stable_frames:
            raise ValueError("floor candidate is missing or not stable")
        self._reference = candidate["plane"]
        self._reference_kind = "initial_observation"
        self._result.update(valid=False, state="acquiring", reason="reference_confirmed")
        self._corrected = None

    def corrected_pose(self, *, now):
        if self.snapshot(now=now)["valid"] and self._corrected is not None:
            return self._corrected.copy()
        return None

    def update(self, planes, predicted, *, stamp, identity, now):
        cfg = self.settings
        identity = tuple(identity)
        if identity != self._identity:
            self.reset()
            self._identity = identity
        self._corrected = None
        self._result.update(valid=False, normal_error_deg=None, distance_error_m=None)
        if (not math.isfinite(stamp) or stamp <= 0 or stamp <= self._last_stamp
                or now-stamp > cfg.stale_after_s or stamp-now > cfg.future_tolerance_s):
            self._result.update(state="lost", reason="invalid_or_stale_capture_time")
            return self.snapshot(now=now)
        self._result["capture_time_s"] = float(stamp)
        self._last_stamp = float(stamp)
        if predicted is None:
            self._previous = []
            self._result.update(state="unavailable", reason="capture_pose_unavailable", candidates=[])
            return self.snapshot(now=now)
        pose = camera_transform(predicted)
        candidates, available = [], list(self._previous)
        for observed in planes[:cfg.max_planes]:
            world = plane_in_world(observed, pose)
            if world.normal[2] < 0:
                observed, world = observed.flipped(), world.flipped()
            height = float(np.dot(world.normal, pose[:3, 3]))+world.offset_m
            if (world.normal[2] < math.cos(math.radians(cfg.normal_max_deg))
                    or not cfg.min_height_m <= height <= cfg.max_height_m
                    or world.area_m2 < cfg.min_area_m2 or min(world.span_m) < cfg.min_span_m
                    or world.rms_m > cfg.max_rms_m):
                continue
            previous = next((p for p in available
                             if np.dot(p["plane"].normal, world.normal) > math.cos(math.radians(cfg.match_angle_deg))
                             and abs(p["plane"].offset_m-world.offset_m) < cfg.match_distance_m
                             and stamp-p["stamp"] <= cfg.stale_after_s), None)
            if previous is not None:
                available = [p for p in available if p is not previous]
                identifier, count = previous["id"], previous["count"]+1
            else:
                self._next_id += 1
                identifier, count = f"plane-{self._next_id}", 1
            candidates.append({"id": identifier, "count": count, "stamp": stamp,
                               "plane": world, "observed": observed, "height": height})
        self._previous = candidates
        self._result["candidates"] = [{"id": p["id"], "stable_frames": p["count"],
            "world_normal": list(p["plane"].normal), "world_offset_m": p["plane"].offset_m,
            "height_above_plane_m": p["height"], "rms_m": p["plane"].rms_m,
            "area_m2": p["plane"].area_m2, "outline_world": [list(v) for v in p["plane"].outline]}
            for p in candidates]
        if not candidates:
            self._result.update(state="lost", reason="no_supported_floor_plane")
            return self.snapshot(now=now)
        if self._reference is None:
            self._result.update(state="candidate", reason="confirm_reference_plane")
            return self.snapshot(now=now)
        matches = [p for p in candidates
                   if np.dot(p["plane"].normal, self._reference.normal) > math.cos(math.radians(cfg.match_angle_deg))
                   and abs(p["plane"].offset_m-self._reference.offset_m) < cfg.match_distance_m]
        if len(matches) != 1:
            self._result.update(state="lost", reason="ambiguous_reference" if matches else "reference_mismatch")
            return self.snapshot(now=now)
        selected = matches[0]
        if selected["count"] < cfg.stable_frames:
            self._result.update(state="acquiring", reason="waiting_for_stable_plane")
            return self.snapshot(now=now)
        observed, reference = selected["observed"], self._reference
        expected = plane_in_camera(reference, pose)
        self._corrected = correct_camera_pose(pose, observed, reference)
        self._last_valid_stamp = stamp
        self._result.update(state="tracking", valid=True, reason="",
            selected_id=selected["id"],
            observed=observed.payload(), predicted=expected.payload(),
            normal_error_deg=float(np.degrees(np.arccos(np.clip(np.dot(observed.normal, expected.normal), -1, 1)))),
            distance_error_m=float(observed.offset_m-expected.offset_m))
        return self.snapshot(now=now)
