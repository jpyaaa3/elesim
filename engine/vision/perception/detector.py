"""Object detectors for visual servoing."""

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
        ranges_raw = hsv.get("ranges", None)
        self._ranges: list[tuple[np.ndarray, np.ndarray]] = []
        if isinstance(ranges_raw, list) and len(ranges_raw) > 0:
            for item in ranges_raw:
                if not isinstance(item, dict):
                    continue
                lower = np.array(item.get("lower", [0, 80, 80]), dtype=np.uint8)
                upper = np.array(item.get("upper", [20, 255, 255]), dtype=np.uint8)
                self._ranges.append((lower, upper))
        if not self._ranges:
            lower = np.array(hsv.get("lower", [0, 80, 80]), dtype=np.uint8)
            upper = np.array(hsv.get("upper", [20, 255, 255]), dtype=np.uint8)
            self._ranges.append((lower, upper))
        self._label = str(cfg.get("target_label", "object"))
        self._min_area = int(cfg.get("min_area_px", 200))

    def detect(self, color_bgr: np.ndarray) -> DetectionResult | None:
        if color_bgr is None or color_bgr.size == 0:
            return None
        img = np.ascontiguousarray(color_bgr, dtype=np.uint8)
        if img.ndim != 3 or img.shape[2] != 3:
            return None
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in self._ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        if int(np.count_nonzero(mask)) < self._min_area:
            return None
        bbox = _bbox_from_mask(mask)
        if bbox is None:
            return None
        return DetectionResult(mask=mask, bbox_xyxy=bbox, label=self._label, confidence=1.0)


class RoiDetector:
    def __init__(self, cfg: dict[str, Any]) -> None:
        self._roi = tuple(int(x) for x in cfg.get("roi_xyxy", [0, 0, 100, 100]))
        self._label = str(cfg.get("target_label", "roi_object"))

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
        return DetectionResult(mask=mask, bbox_xyxy=(x0, y0, x1, y1), label=self._label, confidence=1.0)


def load_detector_config(path: str | Path) -> dict[str, Any]:
    p = Path(path)
    if not p.is_file():
        raise FileNotFoundError(f"detector config not found: {p}")
    with open(p, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    if not isinstance(cfg, dict):
        raise ValueError(f"detector config must be a JSON object: {p}")
    cfg["_config_dir"] = str(p.parent.resolve())
    return cfg


def create_detector(cfg: dict[str, Any]) -> ObjectDetector:
    kind = str(cfg.get("type", "")).strip().lower()
    if not kind:
        raise ValueError("detector config missing required 'type'")
    if kind == "hsv":
        return HsvDetector(cfg)
    if kind == "roi":
        return RoiDetector(cfg)
    if kind == "yolo":
        from engine.vision.perception.yolo_detector import YoloDetector

        return YoloDetector(cfg)
    raise ValueError(f"unknown detector type: {kind}")
