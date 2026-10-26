"""Latest-only Pilot worker for depth-plane calibration and bounded evidence.

The camera callback only samples/copies a bounded point set. Fitting, logging,
and operator snapshots do not run on a Robot control or DDS callback thread.
"""
from __future__ import annotations

from collections import deque
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace
import tempfile
import threading
import time

import numpy as np

from .floor_calibration import (
    FloorEstimator, FloorSettings, camera_transform, depth_points, detect_planes,
)


def frame_identity(frame):
    intr = frame.intrinsics
    calibration = (str(getattr(frame, "calibration_id", "")), float(frame.depth_scale),
                   intr.fx, intr.fy, intr.cx, intr.cy, intr.width, intr.height)
    digest = hashlib.sha256(repr(calibration).encode()).hexdigest()[:16]
    return (str(getattr(frame, "source_id", "")),
            str(getattr(frame, "source_boot_id", "")), digest)


def frame_camera_pose(frame):
    """Read the optical pose attached to this exact frame, never a later snapshot."""
    values = [getattr(frame, name, None) for name in
              ("camera_world_origin", "camera_world_look", "camera_world_right")]
    if any(v is None for v in values):
        return None
    origin, look, right = (np.asarray(v, dtype=float) for v in values)
    if any(v.shape != (3,) or not np.all(np.isfinite(v)) for v in (origin, look, right)):
        return None
    if min(np.linalg.norm(look), np.linalg.norm(right)) < 1e-8:
        return None
    look, right = look/np.linalg.norm(look), right/np.linalg.norm(right)
    down = np.cross(look, right)
    if np.linalg.norm(down) < 0.9 or abs(np.dot(look, right)) > 0.01:
        return None
    down /= np.linalg.norm(down)
    right = np.cross(down, look)
    result = np.eye(4)
    result[:3, :3] = np.column_stack((right, down, look))
    result[:3, 3] = origin
    return result


class FloorCalibrationService:
    def __init__(self, *, max_hz=6.0, reference_height_m=None, apply=False,
                 clock_verified=False, settings=None, pose_provider=None,
                 evidence_provider=None,
                 wall_clock=time.time, monotonic=time.monotonic):
        if not math.isfinite(max_hz) or not 0 < max_hz <= 30:
            raise ValueError("floor calibration rate must be within (0, 30] Hz")
        self.settings = settings or FloorSettings()
        self.estimator = FloorEstimator(self.settings, reference_height_m=reference_height_m)
        self.apply = bool(apply)
        self.clock_verified = bool(clock_verified)
        self.pose_provider = pose_provider
        self.evidence_provider = evidence_provider
        self.wall_clock, self.monotonic = wall_clock, monotonic
        self.period = 1.0/max_hz
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._pending = None
        self._thread = None
        self._stopping = False
        self._generation = 0
        self._last_submit = -math.inf
        self._last_pose = None
        self._last_correction = None
        self._last_identity = None
        self._records = deque(maxlen=2000)
        self._stats = {"submitted": 0, "processed": 0, "replaced": 0,
                       "processing_ms": 0.0, "error": "", "pose_source": "unavailable",
                       "evidence_overwrites": 0}

    def submit(self, frame):
        now = self.monotonic()
        with self._lock:
            if self._stopping or now-self._last_submit < self.period:
                return False
            self._last_submit = now
            generation = self._generation
        try:
            points = depth_points(frame.depth_raw, frame.intrinsics, float(frame.depth_scale),
                                  max_points=self.settings.max_points)
            identity = frame_identity(frame)
            pose = frame_camera_pose(frame)
            pose_source = "frame_camera_pose" if pose is not None else "unavailable"
            if self.pose_provider is not None:
                supplied = self.pose_provider(frame)
                if supplied is not None:
                    pose, pose_source = camera_transform(supplied), "source_pose_history"
            job = (points, pose, float(getattr(frame, "ts", 0.0)), identity,
                   int(getattr(frame, "seq", 0)), pose_source, generation,
                   SimpleNamespace(ts=float(getattr(frame, "ts", 0.0)),
                       source_id=identity[0], source_boot_id=identity[1],
                       calibration_id=str(getattr(frame, "calibration_id", ""))))
        except (ValueError, TypeError, AttributeError) as exc:
            with self._lock:
                self._stats["error"] = str(exc)[:256]
            return False
        with self._condition:
            if self._stopping or generation != self._generation:
                return False
            self._stats["submitted"] += 1
            if self._pending is not None:
                self._stats["replaced"] += 1
            self._pending = job
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name="pilot-floor-calibration", daemon=True)
                self._thread.start()
            self._condition.notify()
        return True

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._stopping or self._pending is not None)
                if self._stopping:
                    return
                job, self._pending = self._pending, None
            self._process(job)

    def _process(self, job):
        points, pose, stamp, identity, sequence, pose_source, generation, metadata = job
        started = self.monotonic()
        try:
            planes = detect_planes(points, self.settings)
            # Encoding and transport can deliver a frame just before the next
            # bracketing telemetry sample; try the source history again here.
            if self.pose_provider is not None:
                supplied = self.pose_provider(metadata)
                if supplied is not None:
                    pose, pose_source = camera_transform(supplied), "source_pose_history"
            evidence = {} if self.evidence_provider is None else self.evidence_provider(metadata)
            encoded_evidence = json.dumps(evidence, allow_nan=False)
            if len(encoded_evidence) > 32768:
                raise ValueError("floor calibration pose evidence exceeds 32 KiB")
            evidence = json.loads(encoded_evidence)
            with self._lock:
                if self._stopping or generation != self._generation:
                    return
                result = self.estimator.update(planes, pose, stamp=stamp,
                                               identity=identity, now=self.wall_clock())
                self._last_pose = None if pose is None else pose.copy()
                self._last_correction = self.estimator.corrected_pose(now=self.wall_clock())
                self._last_identity = identity
                self._stats.update(error="", processed=self._stats["processed"]+1,
                    processing_ms=(self.monotonic()-started)*1000, pose_source=pose_source,
                    sequence=sequence, source_id=identity[0], source_boot_id=identity[1])
                if len(self._records) == self._records.maxlen:
                    self._stats["evidence_overwrites"] += 1
                self._records.append({"capture_time_s": stamp, "sequence": sequence,
                    "identity": list(identity), "pose_source": pose_source,
                    "synchronization_verified": bool(all(identity[:2]) and (self.clock_verified or pose_source == "frame_camera_pose")),
                    "calibration_id": metadata.calibration_id, "pose_evidence": evidence,
                    "predicted_camera_pose": None if pose is None else pose.tolist(),
                    "observed_planes": [p.payload() for p in planes], "result": result})
        except Exception as exc:
            with self._lock:
                if self._stopping or generation != self._generation:
                    return
                self._last_correction = None
                self._stats["error"] = str(exc)[:256]

    def snapshot(self):
        with self._lock:
            result = self.estimator.snapshot(now=self.wall_clock())
            synchronized = bool(self._last_identity and all(self._last_identity[:2])
                                and (self.clock_verified or self._stats["pose_source"] == "frame_camera_pose"))
            result.update(self._stats, enabled=not self._stopping, apply_requested=self.apply,
                          synchronization_verified=synchronized, records=len(self._records))
            if self._stopping:
                result.update(state="stopped", valid=False, reason="perception_stopped")
            result["correction_available"] = bool(result["valid"] and synchronized and not result["error"] and not self._stopping)
            return result

    def confirm(self, candidate_id):
        if not isinstance(candidate_id, str) or not 1 <= len(candidate_id) <= 64:
            raise ValueError("invalid floor candidate identity")
        with self._lock:
            self.estimator.confirm(candidate_id, now=self.wall_clock())
            self._last_correction = None
            return self.snapshot()

    def reset(self):
        with self._lock:
            self._generation += 1
            self._pending = None
            self.estimator.reset()
            self._last_correction = self._last_pose = None
            return self.snapshot()

    def corrected_camera_pose(self, frame):
        """Use a fresh correction only near its measured pose and same source epoch."""
        with self._lock:
            status = self.snapshot()
            if (not self.apply or not status["correction_available"]
                    or self._last_correction is None or frame_identity(frame) != self._last_identity):
                return None
            current_pose = (self.pose_provider(frame) if self.pose_provider is not None else None)
            if current_pose is None:
                current_pose = frame_camera_pose(frame)
            stamp = float(getattr(frame, "ts", 0.0))
            if current_pose is None or not 0 <= stamp-status["capture_time_s"] <= 0.2:
                return None
            current_pose = camera_transform(current_pose)
            difference = current_pose[:3, :3] @ self._last_pose[:3, :3].T
            angle = np.arccos(np.clip((np.trace(difference)-1)/2, -1, 1))
            if angle > np.radians(3) or np.linalg.norm(current_pose[:3, 3]-self._last_pose[:3, 3]) > 0.02:
                return None
            # Reuse only the measured orientation delta. Applying a rigid
            # delta to the full pose would rotate a newer camera origin about
            # the old one and silently introduce translation again.
            correction = self._last_correction[:3, :3] @ self._last_pose[:3, :3].T
            result = current_pose.copy()
            result[:3, :3] = correction @ result[:3, :3]
            return result

    def corrected_world_point(self, frame, point):
        pose = self.corrected_camera_pose(frame)
        value = np.asarray(point, dtype=float)
        if pose is None or value.shape != (3,) or not np.all(np.isfinite(value)):
            return None
        return tuple(float(v) for v in pose[:3, :3] @ value + pose[:3, 3])

    def overlay(self, frame, image):
        """Annotate a separate preview; never draw into the depth/RGB source."""
        import cv2
        with self._lock:
            status = self.snapshot()
        result = np.asarray(image).copy()
        try:
            pose = self.pose_provider(frame) if self.pose_provider is not None else None
        except (ValueError, TypeError, KeyError):
            return result
        if pose is None:
            pose = frame_camera_pose(frame)
        if pose is None or status["state"] in {"lost", "unavailable"}:
            return result
        inverse = np.linalg.inv(camera_transform(pose))
        intr = frame.intrinsics
        for candidate in status.get("candidates", []):
            points = np.asarray(candidate["outline_world"], dtype=float)
            if points.shape != (4, 3):
                continue
            points = points @ inverse[:3, :3].T + inverse[:3, 3]
            if np.any(points[:, 2] <= 0.05):
                continue
            pixels = np.column_stack((intr.fx*points[:, 0]/points[:, 2]+intr.cx,
                                      intr.fy*points[:, 1]/points[:, 2]+intr.cy))
            pixels = np.clip(np.rint(pixels), -10000, 10000).astype(np.int32)
            selected = candidate["id"] == status.get("selected_id")
            color = (90, 230, 180) if selected else (0, 190, 255)
            cv2.polylines(result, [pixels], True, color, 2)
            cv2.putText(result, candidate["id"], tuple(pixels[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        return result

    def export(self):
        with self._lock:
            records = list(self._records)
        if not records:
            raise ValueError("no floor calibration observations to export")
        directory = Path(tempfile.mkdtemp(prefix="elesim-floor-calibration-"))
        target = directory / "observations.jsonl"
        with target.open("w", encoding="utf-8") as handle:
            for record in records:
                handle.write(json.dumps(record, allow_nan=False, separators=(",", ":"))+"\n")
        return str(target)

    def close(self):
        with self._condition:
            self._stopping = True
            self._generation += 1
            self._pending = None
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
