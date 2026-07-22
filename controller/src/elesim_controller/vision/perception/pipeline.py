"""Shared helpers for local visual perception capture."""

from __future__ import annotations

import time
from typing import Any, Optional

import numpy as np
from elesim_controller.observability.tracing import sampled_traced

from elesim_controller.vision.perception.depth_pose import CameraIntrinsics, estimate_object_position_camera
from elesim_controller.vision.perception.detector import DetectionResult, ObjectDetector
from elesim_controller.vision.perception.observation import CameraObservation


def list_frame_detections(detector: ObjectDetector, color_bgr: np.ndarray) -> list[DetectionResult]:
    list_fn = getattr(detector, "list_detections", None)
    if callable(list_fn):
        return list(list_fn(color_bgr))
    det = detector.detect(color_bgr)
    return [det] if det is not None else []


def pick_target_detection(dets: list[DetectionResult], target_label: str) -> Optional[DetectionResult]:
    if not dets:
        return None
    key = target_label.strip().lower()
    if not key:
        return max(dets, key=lambda d: float(d.confidence))
    matches = [d for d in dets if d.label.strip().lower() == key]
    if not matches:
        return None
    return max(matches, key=lambda d: float(d.confidence))


def model_class_names(detector: ObjectDetector) -> list[str]:
    names = getattr(detector, "class_names", None)
    if names is None:
        return []
    return [str(x) for x in names]


def build_camera_observation(
    *,
    detection_label: str,
    confidence: float,
    p_camera_object: np.ndarray,
) -> CameraObservation:
    return CameraObservation(
        label=detection_label,
        confidence=float(confidence),
        p_camera_object=np.asarray(p_camera_object, dtype=float).reshape(3),
        timestamp=time.time(),
    )


def normalized_detection_center_uv(det: DetectionResult, *, image_width: int, image_height: int) -> tuple[float, float]:
    from elesim_controller.vision.perception.detection_utils import detection_center_pixel

    w = max(int(image_width), 1)
    h = max(int(image_height), 1)
    cx, cy = detection_center_pixel(det, image_width=w, image_height=h)
    return (float(2.0 * (cx / float(w)) - 1.0), float(2.0 * (cy / float(h)) - 1.0))


def detection_scale(det: DetectionResult, *, image_width: int, image_height: int) -> float:
    w = max(int(image_width), 1)
    h = max(int(image_height), 1)
    img_area = float(w * h)
    if isinstance(det.mask, np.ndarray) and det.mask.size > 0:
        area = float(np.count_nonzero(det.mask))
    else:
        x0, y0, x1, y1 = det.bbox_xyxy
        area = float(max(0, x1 - x0) * max(0, y1 - y0))
    return float(max(0.0, min(1.0, area / img_area)))


def resolve_detector_cfg(
    file_cfg: dict[str, Any],
    *,
    detector_cli: str,
    target_label_cli: str | None,
    yolo_device_cli: str | None,
) -> dict[str, Any]:
    cfg = dict(file_cfg)
    det = str(detector_cli).strip().lower()
    if det == "yolo":
        cfg["type"] = "yolo"
    elif det not in ("", "config", "external"):
        cfg["type"] = det
    if target_label_cli:
        cfg["target_label"] = str(target_label_cli).strip()
    if yolo_device_cli is not None and str(yolo_device_cli).strip() != "":
        cfg["device"] = str(yolo_device_cli).strip()
    return cfg


def run_mock_frame(detector_cfg: dict) -> tuple[np.ndarray, np.ndarray, CameraIntrinsics, float]:
    import cv2

    w, h = 640, 480
    color = np.zeros((h, w, 3), dtype=np.uint8)
    color[:, :] = (40, 120, 40)
    cv2.circle(color, (w // 2, h // 2), 40, (0, 0, 220), -1)
    intrinsics = CameraIntrinsics(fx=615.0, fy=615.0, cx=320.0, cy=240.0, width=w, height=h)
    depth_scale = 0.001
    z_m = 0.65
    depth_raw = np.zeros((h, w), dtype=np.uint16)
    depth_raw[:, :] = int(round(z_m / depth_scale))
    return color, depth_raw, intrinsics, depth_scale


@sampled_traced("perception.detect", sample_key="perception.detect", every=60)
def measure_detection(
    det: DetectionResult,
    *,
    depth_raw: np.ndarray,
    intrinsics: CameraIntrinsics,
    depth_scale: float,
    detector_cfg: dict[str, Any],
) -> Optional[np.ndarray]:
    mask = getattr(det, "mask", None)
    if mask is None:
        try:
            x0, y0, x1, y1 = [int(v) for v in det.bbox_xyxy]
            depth_shape = getattr(depth_raw, "shape", None)
            if depth_shape is None or len(depth_shape) < 2:
                return None
            h, w = int(depth_shape[0]), int(depth_shape[1])
            x0 = max(0, min(w, x0))
            x1 = max(0, min(w, x1))
            y0 = max(0, min(h, y0))
            y1 = max(0, min(h, y1))
            if x1 <= x0 or y1 <= y0:
                return None
            mask = np.zeros((h, w), dtype=np.uint8)
            mask[y0:y1, x0:x1] = 255
        except Exception:
            return None
    if depth_raw is None:
        return None
    try:
        return estimate_object_position_camera(
            mask,
            depth_raw,
            intrinsics,
            depth_scale,
            z_min_m=float(detector_cfg.get("z_min_m", 0.15)),
            z_max_m=float(detector_cfg.get("z_max_m", 2.5)),
        )
    except (AttributeError, RuntimeError, TypeError, ValueError):
        return None
