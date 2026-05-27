"""OpenCV preview overlay for live detection."""

from __future__ import annotations

from typing import Optional, Sequence

import cv2
import numpy as np

from perception.detector import DetectionResult


def _draw_class_panel(
    vis: np.ndarray,
    *,
    all_detections: Sequence[DetectionResult],
    target_label: str,
    model_classes: Sequence[str],
) -> None:
    lines: list[str] = ["frame detections:"]
    if all_detections:
        for det in all_detections[:12]:
            mark = "*" if target_label and det.label.strip().lower() == target_label.strip().lower() else " "
            lines.append(f"{mark}{det.label} {det.confidence:.2f}")
        if len(all_detections) > 12:
            lines.append(f"  ... +{len(all_detections) - 12} more")
    else:
        lines.append("  (none above conf)")

    if target_label:
        lines.append(f"target: {target_label}")
    if model_classes:
        lines.append(f"model has {len(model_classes)} classes")

    y = 52
    for line in lines:
        cv2.putText(
            vis,
            line,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 0),
            1,
            cv2.LINE_AA,
        )
        y += 18


def draw_detection_overlay(
    color_bgr: np.ndarray,
    det: Optional[DetectionResult],
    *,
    status: str,
    target_label: str = "",
    frame_idx: int = 0,
    p_camera: Optional[np.ndarray] = None,
    p_world: Optional[np.ndarray] = None,
    all_detections: Optional[Sequence[DetectionResult]] = None,
    model_classes: Optional[Sequence[str]] = None,
) -> np.ndarray:
    vis = np.asarray(color_bgr, dtype=np.uint8).copy()
    h, w = vis.shape[:2]

    dets = list(all_detections or [])
    target_key = target_label.strip().lower()
    for other in dets:
        if det is not None and other.label == det.label and other.bbox_xyxy == det.bbox_xyxy:
            continue
        x0, y0, x1, y1 = other.bbox_xyxy
        is_target = bool(target_key) and other.label.strip().lower() == target_key
        color = (0, 255, 0) if is_target else (0, 220, 255)
        cv2.rectangle(vis, (x0, y0), (x1, y1), color, 1)
        cv2.putText(
            vis,
            f"{other.label} {other.confidence:.2f}",
            (max(0, x0), max(12, y1 + 14)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            color,
            1,
            cv2.LINE_AA,
        )

    _draw_class_panel(
        vis,
        all_detections=dets,
        target_label=target_label,
        model_classes=model_classes or [],
    )

    if det is not None and det.mask is not None and det.mask.shape[:2] == (h, w):
        mask = det.mask > 0
        tint = vis.copy()
        tint[mask] = (0, 200, 0)
        vis = cv2.addWeighted(vis, 0.65, tint, 0.35, 0.0)
        x0, y0, x1, y1 = det.bbox_xyxy
        cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 2)
        label_text = f"{det.label} {det.confidence:.2f}"
        cv2.putText(
            vis,
            label_text,
            (max(0, x0), max(20, y0 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
    else:
        msg = "searching"
        if target_label:
            msg += f" ({target_label})"
        if dets:
            msg += f" | saw {len(dets)} det"
        cv2.putText(
            vis,
            msg,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 180, 255),
            2,
            cv2.LINE_AA,
        )

    line2 = f"status={status} frame={frame_idx}"
    if p_camera is not None:
        p = np.asarray(p_camera, dtype=float).reshape(3)
        line2 += f" | cam=[{p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}]m"
    if p_world is not None:
        p = np.asarray(p_world, dtype=float).reshape(3)
        line2 += f" | world=[{p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}]m"
    cv2.putText(
        vis,
        line2,
        (12, h - 14),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (240, 240, 240),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        vis,
        "q/ESC=quit",
        (12, h - 36),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (200, 200, 200),
        1,
        cv2.LINE_AA,
    )
    return vis


def show_preview(window_name: str, image_bgr: np.ndarray) -> int:
    """Show frame; returns waitKey code (lower 8 bits)."""
    cv2.imshow(window_name, image_bgr)
    return int(cv2.waitKey(1)) & 0xFF


def close_preview(window_name: str) -> None:
    try:
        cv2.destroyWindow(window_name)
    except Exception:
        pass
