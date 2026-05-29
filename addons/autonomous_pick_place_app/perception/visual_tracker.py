"""Lightweight bbox tracker (CSRT with KCF fallback) for post-YOLO tracking."""

from __future__ import annotations

from typing import Optional

import cv2
import numpy as np

from perception.detector import DetectionResult


def _create_opencv_tracker(prefer: str = "csrt"):
    kind = str(prefer).strip().lower()
    creators = []
    if kind in ("csrt", "auto"):
        creators.append(("csrt", getattr(cv2, "TrackerCSRT_create", None)))
    if kind in ("kcf", "auto", "csrt"):
        creators.append(("kcf", getattr(cv2, "TrackerKCF_create", None)))
    if kind == "mosse":
        creators.append(("mosse", getattr(cv2, "legacy_TrackerMOSSE_create", None)))
    for name, factory in creators:
        if factory is None:
            continue
        try:
            tracker = factory()
            if tracker is not None:
                return tracker, name
        except Exception:
            continue
    raise RuntimeError(
        "no OpenCV tracker available (install opencv-contrib-python for CSRT, or use KCF)"
    )


def bbox_xyxy_to_xywh(bbox_xyxy: tuple[int, int, int, int]) -> tuple[int, int, int, int]:
    x0, y0, x1, y1 = (int(bbox_xyxy[0]), int(bbox_xyxy[1]), int(bbox_xyxy[2]), int(bbox_xyxy[3]))
    return x0, y0, max(1, x1 - x0), max(1, y1 - y0)


def clamp_bbox_xyxy(
    bbox_xyxy: tuple[int, int, int, int],
    *,
    image_width: int,
    image_height: int,
) -> tuple[int, int, int, int]:
    w = max(int(image_width), 1)
    h = max(int(image_height), 1)
    x0, y0, x1, y1 = bbox_xyxy
    x0 = max(0, min(w - 2, int(x0)))
    y0 = max(0, min(h - 2, int(y0)))
    x1 = max(x0 + 1, min(w - 1, int(x1)))
    y1 = max(y0 + 1, min(h - 1, int(y1)))
    return x0, y0, x1, y1


def detection_from_bbox(
    bbox_xyxy: tuple[int, int, int, int],
    *,
    image_width: int,
    image_height: int,
    label: str = "tracked",
    confidence: float = 1.0,
) -> DetectionResult:
    x0, y0, x1, y1 = clamp_bbox_xyxy(bbox_xyxy, image_width=image_width, image_height=image_height)
    h = max(int(image_height), 1)
    w = max(int(image_width), 1)
    mask = np.zeros((h, w), dtype=np.uint8)
    mask[y0 : y1 + 1, x0 : x1 + 1] = 255
    return DetectionResult(
        mask=mask,
        bbox_xyxy=(x0, y0, x1, y1),
        label=str(label),
        confidence=float(confidence),
    )


class BboxTracker:
    """OpenCV single-object bbox tracker."""

    def __init__(self, *, tracker_type: str = "csrt") -> None:
        self._prefer = str(tracker_type).strip().lower() or "csrt"
        self._tracker = None
        self._backend_name = ""
        self._initialized = False

    @property
    def backend_name(self) -> str:
        return str(self._backend_name)

    @property
    def initialized(self) -> bool:
        return bool(self._initialized)

    def reset(self) -> None:
        self._tracker = None
        self._backend_name = ""
        self._initialized = False

    def init(self, frame_bgr: np.ndarray, bbox_xyxy: tuple[int, int, int, int]) -> bool:
        self.reset()
        h, w = frame_bgr.shape[:2]
        bbox = clamp_bbox_xyxy(bbox_xyxy, image_width=w, image_height=h)
        rect = bbox_xyxy_to_xywh(bbox)
        try:
            tracker, name = _create_opencv_tracker(self._prefer)
            ok = bool(tracker.init(frame_bgr, rect))
            if not ok:
                return False
            self._tracker = tracker
            self._backend_name = str(name)
            self._initialized = True
            return True
        except Exception:
            return False

    def update(self, frame_bgr: np.ndarray) -> Optional[tuple[int, int, int, int]]:
        if not self._initialized or self._tracker is None:
            return None
        try:
            ok, box = self._tracker.update(frame_bgr)
        except Exception:
            self._initialized = False
            return None
        if not bool(ok):
            self._initialized = False
            return None
        x, y, bw, bh = [int(round(v)) for v in box]
        if bw <= 0 or bh <= 0:
            self._initialized = False
            return None
        h, w = frame_bgr.shape[:2]
        return clamp_bbox_xyxy((x, y, x + bw, y + bh), image_width=w, image_height=h)
