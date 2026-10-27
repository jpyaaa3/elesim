from __future__ import annotations

from types import SimpleNamespace
from typing import Any

import numpy as np


def _bbox_from_mask(mask: np.ndarray) -> tuple[int, int, int, int] | None:
    ys, xs = np.nonzero(np.asarray(mask) > 0)
    if xs.size == 0 or ys.size == 0:
        return None
    return int(xs.min()), int(ys.min()), int(xs.max() + 1), int(ys.max() + 1)


def _clamp_bbox(
    bbox: Any,
    *,
    image_width: int,
    image_height: int,
) -> tuple[int, int, int, int]:
    x0, y0, x1, y1 = [int(round(float(v))) for v in bbox]
    x0 = max(0, min(int(image_width), x0))
    x1 = max(0, min(int(image_width), x1))
    y0 = max(0, min(int(image_height), y0))
    y1 = max(0, min(int(image_height), y1))
    if x1 < x0:
        x0, x1 = x1, x0
    if y1 < y0:
        y0, y1 = y1, y0
    return x0, y0, x1, y1


def _clone_detection(det: Any, **updates: Any) -> SimpleNamespace:
    data = dict(getattr(det, "__dict__", {}) or {})
    data.update(updates)
    return SimpleNamespace(**data)


def bbox_xyxy_area(bbox: Any) -> int:
    x0, y0, x1, y1 = [float(v) for v in bbox]
    return int(max(0.0, x1 - x0) * max(0.0, y1 - y0))


def detection_center_pixel(
    det: Any,
    *,
    image_width: int,
    image_height: int,
) -> tuple[float, float]:
    mask = getattr(det, "mask", None)
    if mask is not None:
        bbox = _bbox_from_mask(np.asarray(mask))
        if bbox is not None:
            x0, y0, x1, y1 = bbox
            return 0.5 * float(x0 + x1), 0.5 * float(y0 + y1)
    bbox = detection_init_bbox(det, image_width=image_width, image_height=image_height, padding=0.0)
    x0, y0, x1, y1 = bbox
    return 0.5 * float(x0 + x1), 0.5 * float(y0 + y1)


def detection_init_bbox(
    det: Any,
    *,
    image_width: int,
    image_height: int,
    padding: float = 0.0,
) -> tuple[int, int, int, int]:
    bbox = getattr(det, "bbox_xyxy", None)
    if bbox is None and getattr(det, "mask", None) is not None:
        bbox = _bbox_from_mask(np.asarray(det.mask))
    if bbox is None:
        bbox = (0, 0, int(image_width), int(image_height))
    x0, y0, x1, y1 = [float(v) for v in bbox]
    pad = max(0.0, float(padding))
    dx = (x1 - x0) * pad
    dy = (y1 - y0) * pad
    return _clamp_bbox(
        (x0 - dx, y0 - dy, x1 + dx, y1 + dy),
        image_width=int(image_width),
        image_height=int(image_height),
    )


def detection_from_bbox(
    bbox: Any,
    *,
    image_width: int,
    image_height: int,
    label: str = "",
    confidence: float = 1.0,
) -> SimpleNamespace:
    bbox_xyxy = _clamp_bbox(bbox, image_width=int(image_width), image_height=int(image_height))
    return SimpleNamespace(
        bbox_xyxy=bbox_xyxy,
        label=str(label),
        confidence=float(confidence),
        mask=None,
    )


def _translate_mask(mask: np.ndarray, *, dx: int, dy: int, image_width: int, image_height: int) -> np.ndarray:
    src = np.asarray(mask)
    out = np.zeros((int(image_height), int(image_width)), dtype=src.dtype)
    src_h, src_w = src.shape[:2]
    src_x0 = max(0, -int(dx))
    src_y0 = max(0, -int(dy))
    dst_x0 = max(0, int(dx))
    dst_y0 = max(0, int(dy))
    width = min(src_w - src_x0, int(image_width) - dst_x0)
    height = min(src_h - src_y0, int(image_height) - dst_y0)
    if width > 0 and height > 0:
        out[dst_y0 : dst_y0 + height, dst_x0 : dst_x0 + width] = src[
            src_y0 : src_y0 + height,
            src_x0 : src_x0 + width,
        ]
    return out


def detection_mask_translated(
    det: Any,
    *,
    dx: int,
    dy: int,
    image_width: int,
    image_height: int,
) -> SimpleNamespace | None:
    bbox = detection_init_bbox(det, image_width=image_width, image_height=image_height, padding=0.0)
    shifted_bbox = _clamp_bbox(
        (bbox[0] + int(dx), bbox[1] + int(dy), bbox[2] + int(dx), bbox[3] + int(dy)),
        image_width=int(image_width),
        image_height=int(image_height),
    )
    mask = getattr(det, "mask", None)
    shifted_mask = None
    if mask is not None:
        shifted_mask = _translate_mask(
            np.asarray(mask),
            dx=int(dx),
            dy=int(dy),
            image_width=int(image_width),
            image_height=int(image_height),
        )
        if int(np.count_nonzero(shifted_mask)) <= 0:
            return None
    return _clone_detection(det, bbox_xyxy=shifted_bbox, mask=shifted_mask)


def detection_mask_coast_aligned(
    det: Any,
    *,
    csrt_bbox: Any,
    anchor_center: tuple[float, float],
    image_width: int,
    image_height: int,
) -> SimpleNamespace | None:
    x0, y0, x1, y1 = [float(v) for v in csrt_bbox]
    csrt_center = (0.5 * (x0 + x1), 0.5 * (y0 + y1))
    dx = int(round(csrt_center[0] - float(anchor_center[0])))
    dy = int(round(csrt_center[1] - float(anchor_center[1])))
    return detection_mask_translated(
        det,
        dx=dx,
        dy=dy,
        image_width=int(image_width),
        image_height=int(image_height),
    )


def refine_detection_mask(det: Any, *, erode_px: int = 0) -> Any:
    mask = getattr(det, "mask", None)
    if mask is None or int(erode_px) <= 0:
        return det
    try:
        import cv2

        kernel = np.ones((int(erode_px) * 2 + 1, int(erode_px) * 2 + 1), dtype=np.uint8)
        eroded = cv2.erode(np.asarray(mask), kernel, iterations=1)
    except Exception:
        eroded = np.asarray(mask)
    return _clone_detection(det, mask=eroded)
