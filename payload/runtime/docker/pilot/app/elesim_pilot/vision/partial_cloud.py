"""Sensor-only partial object clouds, with no simulator scene or oracle pose.

The replay packet is schema 1. It contains RGB-D, calibration and encoder
measurements; its strict allowlist intentionally rejects scene/catalog and
camera-truth fields. A caller supplies a pose derived from its robot model.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np

from .floor_calibration import camera_transform
from .perception.depth_pose import CameraIntrinsics


_FIELDS = frozenset({"schema_version", "color_bgr", "depth_raw", "depth_scale", "intrinsics",
                     "seq", "ts", "source_id", "source_boot_id", "calibration_id", "encoder_q"})


class ArmCameraModel:
    """Nominal camera FK from the installed four-DOF model and hand-eye file.

    A replay may declare its fixed arm-base transform. Camera/world truth is
    never taken from a sensor packet. This class does not command an arm.
    """

    def __init__(self, model_path, hand_eye_path, *, base_world_transform=None):
        from .perception_bridge.hand_eye import load_hand_eye_transform

        model_bytes = Path(model_path).read_bytes()
        self.model_digest = hashlib.sha256(model_bytes).hexdigest()
        raw = json.loads(model_bytes)
        if raw.get("schema_version") != 1:
            raise ValueError("unsupported arm model schema")
        self.context = dict(raw["context"])
        if base_world_transform is not None:
            self.context["base_world_T"] = camera_transform(base_world_transform)
        limits = self.context["limit"]["value"]
        bend = math.radians(limits["bend_deg"])
        self.lower = np.array([self.context["linear_min_m"], math.radians(limits["roll_min_deg"]), -bend, -bend])
        self.upper = np.array([self.context["linear_max_m"], math.radians(limits["roll_max_deg"]), bend, bend])
        if not np.all(np.isfinite([self.lower, self.upper])) or np.any(self.lower >= self.upper):
            raise ValueError("invalid arm joint limits")
        self.mount, meta = load_hand_eye_transform(hand_eye_path)
        self.parent_frame = meta["parent_frame"]

    def camera_pose(self, q):
        from .perception_bridge.hand_eye import camera_world_transform

        q = np.asarray(q, dtype=float)
        if (q.shape != (4,) or not np.all(np.isfinite(q))
                or np.any(q < self.lower-1e-9) or np.any(q > self.upper+1e-9)):
            raise ValueError("arm replay exceeds joint limits")
        return camera_world_transform(self.context, q, self.mount, parent_frame=self.parent_frame)


@dataclass(frozen=True)
class SensorObservation:
    color_bgr: np.ndarray
    depth_raw: np.ndarray
    depth_scale: float
    intrinsics: CameraIntrinsics
    seq: int
    ts: float
    source_id: str
    source_boot_id: str
    calibration_id: str
    encoder_q: tuple[float, ...]

    @classmethod
    def load(cls, path):
        path = Path(path)
        if path.stat().st_size > 64*1024*1024:
            raise ValueError("sensor packet exceeds compressed budget")
        with zipfile.ZipFile(path) as archive:
            if sum(info.file_size for info in archive.infolist()) > 64*1024*1024:
                raise ValueError("sensor packet exceeds decoded budget")
        with np.load(path, allow_pickle=False) as raw:
            missing, unexpected = _FIELDS-set(raw.files), set(raw.files)-_FIELDS
            if missing or unexpected:
                raise ValueError(f"sensor packet fields: missing={sorted(missing)}, unexpected={sorted(unexpected)}")
            if raw["schema_version"].shape != () or raw["schema_version"].item() != 1:
                raise ValueError("unsupported sensor observation schema")
            intr = np.asarray(raw["intrinsics"], dtype=float)
            if intr.shape != (6,) or not np.all(np.isfinite(intr)) or np.any(intr[:2] <= 0):
                raise ValueError("invalid sensor intrinsics")
            width, height = int(intr[4]), int(intr[5])
            if (width != intr[4] or height != intr[5] or min(width, height) < 1
                    or width*height > 4_194_304):
                raise ValueError("invalid sensor dimensions")
            color, depth = raw["color_bgr"], raw["depth_raw"]
            if color.dtype != np.uint8 or color.shape != (height, width, 3):
                raise ValueError("invalid sensor color")
            if depth.dtype not in (np.dtype(np.float32), np.dtype(np.uint16)) or depth.shape != (height, width):
                raise ValueError("invalid sensor depth")
            scale, stamp = float(raw["depth_scale"].item()), float(raw["ts"].item())
            sequence = raw["seq"].item()
            if (not math.isfinite(scale) or scale <= 0 or not math.isfinite(stamp) or stamp <= 0
                    or isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0):
                raise ValueError("invalid sensor scale/time/sequence")
            identifiers = [str(raw[key].item()) for key in ("source_id", "source_boot_id", "calibration_id")]
            if any(not value or len(value) > 128 for value in identifiers):
                raise ValueError("invalid sensor identity")
            encoder = np.asarray(raw["encoder_q"], dtype=float)
            if encoder.ndim != 1 or not 1 <= len(encoder) <= 32 or not np.all(np.isfinite(encoder)):
                raise ValueError("invalid encoder measurements")
            return cls(color.copy(), depth.copy(), scale,
                       CameraIntrinsics(*intr[:4], width, height), int(sequence), stamp,
                       *identifiers, tuple(float(q) for q in encoder))


@dataclass(frozen=True)
class PartialView:
    points_camera: np.ndarray
    pixels_uv: np.ndarray
    points_before: np.ndarray
    points_after: np.ndarray


def reconstruct_view(observation: SensorObservation, mask, predicted_pose, corrected_pose=None, *, max_points=20_000):
    if type(max_points) is not int or not 1 <= max_points <= 100_000:
        raise ValueError("invalid object point budget")
    mask = np.asarray(mask, dtype=bool)
    if mask.shape != observation.depth_raw.shape:
        raise ValueError("object mask dimensions do not match depth")
    depth = observation.depth_raw.astype(float)*observation.depth_scale
    valid = mask & np.isfinite(depth) & (depth >= 0.1) & (depth <= 5)
    v, u = np.nonzero(valid)
    if len(u) > max_points:
        take = np.linspace(0, len(u)-1, max_points, dtype=int)
        u, v = u[take], v[take]
    z = depth[v, u]
    intr = observation.intrinsics
    points = np.column_stack(((u-intr.cx)*z/intr.fx, (v-intr.cy)*z/intr.fy, z))
    predicted = camera_transform(predicted_pose)
    before = points @ predicted[:3, :3].T + predicted[:3, 3]
    if corrected_pose is not None:
        corrected = camera_transform(corrected_pose)
        after = points @ corrected[:3, :3].T + corrected[:3, 3]
    else:
        after = np.empty((0, 3))
    return PartialView(points, np.column_stack((u, v)).astype(np.int32), before, after)


class VoxelCloud:
    """Bounded weighted voxel means; deliberately performs no ICP registration."""

    def __init__(self, *, voxel_m=0.004, max_voxels=50_000):
        if not math.isfinite(voxel_m) or voxel_m <= 0 or type(max_voxels) is not int or max_voxels < 1:
            raise ValueError("invalid voxel cloud settings")
        self.voxel_m, self.max_voxels = float(voxel_m), max_voxels
        self._cells = {}

    def add(self, points):
        points = np.asarray(points, dtype=float)
        if (points.ndim != 2 or points.shape[1] != 3 or len(points) > 100_000
                or not np.all(np.isfinite(points)) or np.any(np.abs(points) > 100)):
            raise ValueError("invalid or excessive cloud points")
        if not len(points):
            return
        keys, inverse, counts = np.unique(np.floor(points/self.voxel_m).astype(np.int64),
                                          axis=0, return_inverse=True, return_counts=True)
        cells = [tuple(key) for key in keys]
        if len(set(self._cells).union(cells)) > self.max_voxels:
            raise ValueError("voxel cloud budget exceeded")
        sums = np.column_stack([np.bincount(inverse, weights=points[:, axis], minlength=len(keys))
                                for axis in range(3)])
        for key, total, count in zip(cells, sums, counts):
            previous, n = self._cells.get(key, (np.zeros(3), 0))
            self._cells[key] = (previous+total, n+int(count))

    def points(self):
        return np.array([self._cells[key][0]/self._cells[key][1] for key in sorted(self._cells)],
                        dtype=float).reshape(-1, 3)
