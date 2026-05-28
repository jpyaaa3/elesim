"""ROI depth tracker: YOLO lock once, then track 3D centroid inside scaled bbox ROI."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Any, Deque, Optional

import numpy as np

from perception.depth_pose import CameraIntrinsics, estimate_object_position_camera
from perception.detector import DetectionResult


class TrackState(str, Enum):
    SEARCH = "SEARCH"
    TRACKING_3D = "TRACKING_3D"
    LOST = "LOST"


@dataclass
class TrackPacket:
    track_state: str
    track_confidence: float
    bbox_xyxy: tuple[int, int, int, int]
    center_uv: tuple[float, float]
    depth_valid_ratio: float
    lost_count: int
    p_camera: Optional[np.ndarray] = None
    mu_camera: Optional[np.ndarray] = None
    sigma_camera: Optional[np.ndarray] = None
    label: str = ""

    @property
    def publishable(self) -> bool:
        return self.p_camera is not None and np.all(np.isfinite(self.p_camera))

    @property
    def should_publish_to_host(self) -> bool:
        """Publish TRACKING_3D/LOST telemetry; SEARCH has nothing to send yet."""
        return str(self.track_state) != TrackState.SEARCH.value


def _clamp_bbox_xyxy(
    bbox: tuple[int, int, int, int],
    *,
    width: int,
    height: int,
) -> tuple[int, int, int, int]:
    x0, y0, x1, y1 = (int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3]))
    x0 = max(0, min(width - 1, x0))
    x1 = max(0, min(width - 1, x1))
    y0 = max(0, min(height - 1, y0))
    y1 = max(0, min(height - 1, y1))
    if x1 < x0:
        x0, x1 = x1, x0
    if y1 < y0:
        y0, y1 = y1, y0
    return x0, y0, x1, y1


def _scale_bbox_xyxy(
    bbox: tuple[int, int, int, int],
    *,
    scale: float,
    width: int,
    height: int,
) -> tuple[int, int, int, int]:
    x0, y0, x1, y1 = bbox
    cx = 0.5 * (float(x0) + float(x1))
    cy = 0.5 * (float(y0) + float(y1))
    bw = max(2.0, float(x1 - x0) * float(scale))
    bh = max(2.0, float(y1 - y0) * float(scale))
    nx0 = int(round(cx - 0.5 * bw))
    ny0 = int(round(cy - 0.5 * bh))
    nx1 = int(round(cx + 0.5 * bw))
    ny1 = int(round(cy + 0.5 * bh))
    return _clamp_bbox_xyxy((nx0, ny0, nx1, ny1), width=width, height=height)


def _roi_mask_from_bbox(
    bbox: tuple[int, int, int, int],
    *,
    width: int,
    height: int,
) -> np.ndarray:
    x0, y0, x1, y1 = _clamp_bbox_xyxy(bbox, width=width, height=height)
    mask = np.zeros((int(height), int(width)), dtype=np.uint8)
    if x1 > x0 and y1 > y0:
        mask[y0 : y1 + 1, x0 : x1 + 1] = 255
    return mask


def _bbox_center_uv(bbox: tuple[int, int, int, int]) -> tuple[float, float]:
    x0, y0, x1, y1 = bbox
    return (0.5 * (float(x0) + float(x1)), 0.5 * (float(y0) + float(y1)))


def _depth_valid_ratio_in_mask(
    mask: np.ndarray,
    depth_image: np.ndarray,
    depth_scale: float,
    *,
    z_min_m: float,
    z_max_m: float,
) -> float:
    roi_pixels = int(np.count_nonzero(mask > 0))
    if roi_pixels <= 0:
        return 0.0
    valid = (mask.astype(bool)) & (depth_image > 0)
    if not np.any(valid):
        return 0.0
    z_m = depth_image[valid].astype(np.float64) * float(depth_scale)
    z_ok = np.count_nonzero((z_m >= float(z_min_m)) & (z_m <= float(z_max_m)))
    return float(z_ok) / float(roi_pixels)


def _window_median_mad(samples: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    arr = np.asarray(samples, dtype=float)
    if arr.ndim != 2 or arr.shape[0] == 0 or arr.shape[1] != 3:
        raise ValueError("samples must be Nx3")
    mu = np.median(arr, axis=0)
    mad = np.median(np.abs(arr - mu), axis=0)
    sigma = 1.4826 * mad
    sigma = np.maximum(sigma, 1e-6)
    return mu.reshape(3), sigma.reshape(3)


def _compute_confidence(
    *,
    depth_valid_ratio: float,
    sigma_camera: Optional[np.ndarray],
    n_samples: int,
    window_size: int,
    sigma_ref: float = 0.05,
) -> float:
    w1, w2, w3 = 0.45, 0.35, 0.20
    fill = float(n_samples) / float(max(1, window_size))
    sigma_term = 0.0
    if sigma_camera is not None:
        sigma_norm = float(np.linalg.norm(np.asarray(sigma_camera, dtype=float).reshape(3)))
        sigma_term = float(np.exp(-sigma_norm / max(float(sigma_ref), 1e-6)))
    conf = w1 * float(depth_valid_ratio) + w2 * sigma_term + w3 * fill
    return float(max(0.0, min(1.0, conf)))


@dataclass
class TargetTracker:
    """YOLO-seg lock + ROI depth centroid tracking with sliding-window mu/sigma."""

    roi_scale: float = 1.5
    window_size: int = 20
    min_samples: int = 3
    min_depth_valid_ratio: float = 0.15
    lost_frames_threshold: int = 5
    redetect_interval_frames: int = 3
    z_min_m: float = 0.15
    z_max_m: float = 2.5
    outlier_sigma: float = 2.5
    sigma_ref: float = 0.05

    state: TrackState = TrackState.SEARCH
    lost_count: int = 0
    _lock_bbox: Optional[tuple[int, int, int, int]] = None
    _roi_bbox: Optional[tuple[int, int, int, int]] = None
    _label: str = ""
    _samples: Deque[np.ndarray] = None  # type: ignore[assignment]
    _consecutive_fail: int = 0
    _frame_idx: int = 0
    _last_depth_valid_ratio: float = 0.0

    def __post_init__(self) -> None:
        self._samples = deque(maxlen=max(3, int(self.window_size)))

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> TargetTracker:
        tracker_cfg = dict(cfg.get("tracker", {}) or {})
        if not tracker_cfg and any(k in cfg for k in ("roi_scale", "window_size")):
            tracker_cfg = cfg
        return cls(
            roi_scale=float(tracker_cfg.get("roi_scale", 1.5)),
            window_size=int(tracker_cfg.get("window_size", 20)),
            min_samples=int(tracker_cfg.get("min_samples", 3)),
            min_depth_valid_ratio=float(tracker_cfg.get("min_depth_valid_ratio", 0.15)),
            lost_frames_threshold=int(tracker_cfg.get("lost_frames_threshold", 5)),
            redetect_interval_frames=int(tracker_cfg.get("redetect_interval_frames", 3)),
            z_min_m=float(tracker_cfg.get("z_min_m", cfg.get("z_min_m", 0.15))),
            z_max_m=float(tracker_cfg.get("z_max_m", cfg.get("z_max_m", 2.5))),
            outlier_sigma=float(tracker_cfg.get("outlier_sigma", 2.5)),
            sigma_ref=float(tracker_cfg.get("sigma_ref", 0.05)),
        )

    def reset(self) -> None:
        self.state = TrackState.SEARCH
        self.lost_count = 0
        self._lock_bbox = None
        self._roi_bbox = None
        self._label = ""
        self._samples.clear()
        self._consecutive_fail = 0
        self._frame_idx = 0
        self._last_depth_valid_ratio = 0.0

    def needs_yolo(self) -> bool:
        if self.state == TrackState.SEARCH:
            return True
        if self.state == TrackState.LOST:
            interval = max(1, int(self.redetect_interval_frames))
            return (int(self._frame_idx) % interval) == 0
        return False

    def try_lock(self, det: Optional[DetectionResult], *, width: int, height: int) -> bool:
        if det is None:
            return False
        bbox = _clamp_bbox_xyxy(det.bbox_xyxy, width=width, height=height)
        if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
            return False
        self._lock_bbox = bbox
        self._roi_bbox = _scale_bbox_xyxy(bbox, scale=float(self.roi_scale), width=width, height=height)
        self._label = str(det.label)
        self.state = TrackState.TRACKING_3D
        self._consecutive_fail = 0
        self._samples.clear()
        return True

    def update(
        self,
        *,
        depth_raw: np.ndarray,
        intrinsics: CameraIntrinsics,
        depth_scale: float,
    ) -> TrackPacket:
        self._frame_idx += 1
        w = int(intrinsics.width)
        h = int(intrinsics.height)

        if self.state == TrackState.SEARCH or self._roi_bbox is None:
            return TrackPacket(
                track_state=TrackState.SEARCH.value,
                track_confidence=0.0,
                bbox_xyxy=(0, 0, 0, 0),
                center_uv=(float(intrinsics.cx), float(intrinsics.cy)),
                depth_valid_ratio=0.0,
                lost_count=int(self.lost_count),
                label=str(self._label),
            )

        roi_bbox = self._roi_bbox
        assert roi_bbox is not None
        lock_bbox = self._lock_bbox if self._lock_bbox is not None else roi_bbox
        mask = _roi_mask_from_bbox(roi_bbox, width=w, height=h)
        depth_valid_ratio = _depth_valid_ratio_in_mask(
            mask,
            depth_raw,
            depth_scale,
            z_min_m=float(self.z_min_m),
            z_max_m=float(self.z_max_m),
        )
        self._last_depth_valid_ratio = float(depth_valid_ratio)

        p_camera: Optional[np.ndarray] = None
        frame_ok = False
        try:
            p_camera = estimate_object_position_camera(
                mask,
                depth_raw,
                intrinsics,
                depth_scale,
                z_min_m=float(self.z_min_m),
                z_max_m=float(self.z_max_m),
                outlier_sigma=float(self.outlier_sigma),
            )
            frame_ok = True
        except RuntimeError:
            frame_ok = False

        if (
            frame_ok
            and p_camera is not None
            and depth_valid_ratio >= float(self.min_depth_valid_ratio)
        ):
            self._samples.append(np.asarray(p_camera, dtype=float).reshape(3))
            self._consecutive_fail = 0
            if self.state == TrackState.LOST:
                self.state = TrackState.TRACKING_3D
        else:
            self._consecutive_fail += 1
            if self._consecutive_fail >= int(self.lost_frames_threshold):
                if self.state == TrackState.TRACKING_3D:
                    self.lost_count += 1
                self.state = TrackState.LOST
                self._samples.clear()

        mu_camera: Optional[np.ndarray] = None
        sigma_camera: Optional[np.ndarray] = None
        if len(self._samples) >= int(self.min_samples):
            mu_camera, sigma_camera = _window_median_mad(list(self._samples))

        conf = _compute_confidence(
            depth_valid_ratio=float(depth_valid_ratio),
            sigma_camera=sigma_camera,
            n_samples=len(self._samples),
            window_size=int(self.window_size),
            sigma_ref=float(self.sigma_ref),
        )
        if self.state == TrackState.LOST:
            conf = float(min(conf, 0.25))

        return TrackPacket(
            track_state=self.state.value,
            track_confidence=float(conf),
            bbox_xyxy=tuple(int(v) for v in lock_bbox),
            center_uv=_bbox_center_uv(roi_bbox),
            depth_valid_ratio=float(depth_valid_ratio),
            lost_count=int(self.lost_count),
            p_camera=None if p_camera is None else np.asarray(p_camera, dtype=float).reshape(3),
            mu_camera=mu_camera,
            sigma_camera=sigma_camera,
            label=str(self._label),
        )
