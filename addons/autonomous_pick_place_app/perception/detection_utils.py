"""Helpers for bbox init, padding, and UV from detections."""

from __future__ import annotations

from perception.detector import DetectionResult, _bbox_from_mask
from perception.visual_tracker import clamp_bbox_xyxy


def bbox_xyxy_area(bbox_xyxy: tuple[int, int, int, int]) -> int:
    x0, y0, x1, y1 = bbox_xyxy
    return int(max(0, x1 - x0) * max(0, y1 - y0))


def pad_bbox_xyxy(
    bbox_xyxy: tuple[int, int, int, int],
    *,
    padding: float,
    image_width: int,
    image_height: int,
) -> tuple[int, int, int, int]:
    """Expand bbox about its center by ``padding`` (e.g. 1.25 = +25% per axis)."""
    factor = max(float(padding), 1.0)
    x0, y0, x1, y1 = (int(bbox_xyxy[0]), int(bbox_xyxy[1]), int(bbox_xyxy[2]), int(bbox_xyxy[3]))
    cx = 0.5 * (float(x0) + float(x1))
    cy = 0.5 * (float(y0) + float(y1))
    half_w = 0.5 * float(x1 - x0) * factor
    half_h = 0.5 * float(y1 - y0) * factor
    raw = (
        int(round(cx - half_w)),
        int(round(cy - half_h)),
        int(round(cx + half_w)),
        int(round(cy + half_h)),
    )
    return clamp_bbox_xyxy(raw, image_width=image_width, image_height=image_height)


def detection_init_bbox(
    det: DetectionResult,
    *,
    image_width: int,
    image_height: int,
    padding: float = 1.25,
) -> tuple[int, int, int, int]:
    """Bbox for tracker init: prefer mask extent, then pad."""
    w = max(int(image_width), 1)
    h = max(int(image_height), 1)
    mask = det.mask
    if mask is not None and getattr(mask, "size", 0) > 0:
        from_mask = _bbox_from_mask(mask)
        if from_mask is not None:
            base = from_mask
        else:
            base = det.bbox_xyxy
    else:
        base = det.bbox_xyxy
    return pad_bbox_xyxy(base, padding=padding, image_width=w, image_height=h)


def detection_center_pixel(
    det: DetectionResult,
    *,
    image_width: int,
    image_height: int,
) -> tuple[float, float]:
    """Mask centroid when available, else bbox center."""
    mask = det.mask
    if mask is not None and getattr(mask, "size", 0) > 0:
        import numpy as np

        ys, xs = np.where(mask > 0)
        if len(xs) > 0 and len(ys) > 0:
            return float(xs.mean()), float(ys.mean())
    x0, y0, x1, y1 = det.bbox_xyxy
    return 0.5 * (float(x0) + float(x1)), 0.5 * (float(y0) + float(y1))
