"""Simple object detectors: HSV threshold, ROI mask, or center mock."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import cv2
import numpy as np


@dataclass
class DetectionResult:
    mask: np.ndarray
    bbox_xyxy: tuple[int, int, int, int]
    label: str
    confidence: float


class ObjectDetector(Protocol):
    def detect(self, color_bgr: np.ndarray) -> DetectionResult | None: ...


def _bbox_from_mask(mask: np.ndarray) -> tuple[int, int, int, int] | None:
    ys, xs = np.where(mask > 0)
    if len(xs) == 0 or len(ys) == 0:
        return None
    return int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())


class HsvDetector:
    def __init__(self, cfg: dict[str, Any]) -> None:
        hsv = cfg.get("hsv", {}) or {}
        self._lower = np.array(hsv.get("lower", [0, 80, 80]), dtype=np.uint8)
        self._upper = np.array(hsv.get("upper", [20, 255, 255]), dtype=np.uint8)
        self._label = str(cfg.get("label", "object"))
        self._min_area = int(cfg.get("min_area_px", 200))

    def detect(self, color_bgr: np.ndarray) -> DetectionResult | None:
        hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._lower, self._upper)
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        if int(np.count_nonzero(mask)) < self._min_area:
            return None
        bbox = _bbox_from_mask(mask)
        if bbox is None:
            return None
        return DetectionResult(
            mask=mask,
            bbox_xyxy=bbox,
            label=self._label,
            confidence=1.0,
        )


class RoiDetector:
    """Fixed ROI rectangle as binary mask."""

    def __init__(self, cfg: dict[str, Any]) -> None:
        roi = cfg.get("roi_xyxy", [0, 0, 100, 100])
        self._roi = tuple(int(x) for x in roi)
        self._label = str(cfg.get("label", "roi_object"))

    def detect(self, color_bgr: np.ndarray) -> DetectionResult | None:
        h, w = color_bgr.shape[:2]
        x0, y0, x1, y1 = self._roi
        x0 = max(0, min(w - 1, x0))
        x1 = max(0, min(w - 1, x1))
        y0 = max(0, min(h - 1, y0))
        y1 = max(0, min(h - 1, y1))
        if x1 <= x0 or y1 <= y0:
            return None
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[y0 : y1 + 1, x0 : x1 + 1] = 255
        return DetectionResult(
            mask=mask,
            bbox_xyxy=(x0, y0, x1, y1),
            label=self._label,
            confidence=1.0,
        )


class MockCenterDetector:
    """Center patch detector for mock / camera-less runs."""

    def __init__(self, cfg: dict[str, Any]) -> None:
        self._frac = float(cfg.get("center_fraction", 0.25))
        self._label = str(cfg.get("label", "mock_object"))

    def detect(self, color_bgr: np.ndarray) -> DetectionResult | None:
        h, w = color_bgr.shape[:2]
        fw = max(8, int(w * self._frac))
        fh = max(8, int(h * self._frac))
        cx, cy = w // 2, h // 2
        x0 = max(0, cx - fw // 2)
        y0 = max(0, cy - fh // 2)
        x1 = min(w - 1, x0 + fw)
        y1 = min(h - 1, y0 + fh)
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[y0 : y1 + 1, x0 : x1 + 1] = 255
        return DetectionResult(
            mask=mask,
            bbox_xyxy=(x0, y0, x1, y1),
            label=self._label,
            confidence=1.0,
        )


def load_detector_config(path: str | Path) -> dict[str, Any]:
    p = Path(path)
    if not p.is_file():
        raise FileNotFoundError(f"detector config not found: {p}")
    with open(p, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    if isinstance(cfg, dict):
        cfg["_config_dir"] = str(p.parent.resolve())
    return cfg


def create_detector(cfg: dict[str, Any]) -> ObjectDetector:
    kind = str(cfg.get("type", "mock_center")).strip().lower()
    if kind == "hsv":
        return HsvDetector(cfg)
    if kind == "roi":
        return RoiDetector(cfg)
    if kind in ("mock", "mock_center", "center"):
        return MockCenterDetector(cfg)
    if kind == "yolo":
        from perception.yolo_detector import YoloDetector

        return YoloDetector(cfg)
    raise ValueError(f"unknown detector type: {kind}")
