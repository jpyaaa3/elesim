"""OpenCV preview overlay for live detection."""

from __future__ import annotations

import os
import sys
import threading
import time
from pathlib import Path
from typing import Optional, Sequence

import cv2
import numpy as np

from elesim_pilot.vision.perception.detector import DetectionResult

_cv2_gui_env_ready = False
_open_preview_windows: set[str] = set()
_preview_disabled_reason: Optional[str] = None
_preview_cond = threading.Condition()
_preview_thread: Optional[threading.Thread] = None
_preview_frames: dict[str, np.ndarray] = {}
_preview_close_requests: set[str] = set()
_preview_keys: dict[str, int] = {}


def ensure_cv2_gui_env() -> None:
    global _cv2_gui_env_ready
    if _cv2_gui_env_ready:
        return
    _cv2_gui_env_ready = True

    font_dir: Optional[str] = None
    for candidate in (
        "/usr/share/fonts/truetype/dejavu",
        "/usr/share/fonts/truetype/liberation",
        "/usr/share/fonts/truetype",
        "/usr/share/fonts",
        "/usr/local/share/fonts",
        "/System/Library/Fonts",
        "/Library/Fonts",
    ):
        if os.path.isdir(candidate):
            font_dir = candidate
            break
    if font_dir:
        os.environ.setdefault("QT_QPA_FONTDIR", font_dir)
    if not sys.platform.startswith("linux") or not font_dir:
        return
    try:
        cv2_dir = Path(cv2.__file__).resolve().parent
        qt_fonts = cv2_dir / "qt" / "fonts"
        if not qt_fonts.exists():
            qt_fonts.parent.mkdir(parents=True, exist_ok=True)
            qt_fonts.symlink_to(font_dir, target_is_directory=True)
    except OSError:
        pass


def preview_disabled_reason() -> Optional[str]:
    return _preview_disabled_reason


def _disable_preview(reason: str) -> None:
    global _preview_disabled_reason
    if _preview_disabled_reason is None:
        _preview_disabled_reason = str(reason)
        print(f"[preview] disabled: {reason}")


def open_preview_window(window_name: str) -> None:
    ensure_cv2_gui_env()
    if _preview_disabled_reason is not None:
        return
    name = str(window_name)
    if name in _open_preview_windows and _window_visible(name):
        return
    _open_preview_windows.discard(name)
    try:
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        _open_preview_windows.add(name)
    except cv2.error as exc:
        _disable_preview(f"namedWindow failed: {exc}")
    except Exception as exc:
        _disable_preview(f"namedWindow failed: {exc}")


def _ensure_preview_thread() -> None:
    global _preview_thread
    if _preview_disabled_reason is not None:
        return
    with _preview_cond:
        if _preview_thread is not None and _preview_thread.is_alive():
            return
        _preview_thread = threading.Thread(target=_preview_loop, name="opencv-preview", daemon=True)
        _preview_thread.start()


def _preview_loop() -> None:
    while True:
        with _preview_cond:
            if not _preview_frames and not _preview_close_requests:
                _preview_cond.wait(timeout=0.05)
            frames = dict(_preview_frames)
            _preview_frames.clear()
            close_requests = set(_preview_close_requests)
            _preview_close_requests.clear()

        for name in close_requests:
            _destroy_preview_window(name)

        for name, frame in frames.items():
            _show_preview_frame(name, frame)

        _pump_preview_events()


def _show_preview_frame(name: str, image_bgr: np.ndarray) -> None:
    if _preview_disabled_reason is not None:
        return
    try:
        if name not in _open_preview_windows or not _window_visible(name):
            open_preview_window(name)
        if _preview_disabled_reason is not None:
            return
        cv2.imshow(name, image_bgr)
    except cv2.error as exc:
        _handle_preview_error(name, exc)
    except Exception as exc:
        _handle_preview_error(name, exc)


def _destroy_preview_window(name: str) -> None:
    try:
        cv2.destroyWindow(str(name))
    except Exception:
        pass
    try:
        cv2.waitKey(1)
    except Exception:
        pass
    _open_preview_windows.discard(str(name))
    with _preview_cond:
        _preview_cond.notify_all()


def _pump_preview_events() -> None:
    if _preview_disabled_reason is not None or not _open_preview_windows:
        return
    try:
        key = int(cv2.waitKey(1)) & 0xFF
    except cv2.error as exc:
        _handle_preview_error("", exc)
        return
    except Exception as exc:
        _handle_preview_error("", exc)
        return
    closed: list[str] = []
    for name in list(_open_preview_windows):
        if not _window_visible(name):
            closed.append(name)
    if key != 255:
        with _preview_cond:
            for name in list(_open_preview_windows):
                _preview_keys[name] = key
            _preview_cond.notify_all()
    if closed:
        for name in closed:
            _open_preview_windows.discard(name)
        with _preview_cond:
            _preview_cond.notify_all()


def _handle_preview_error(name: str, exc: Exception) -> None:
    if name:
        _open_preview_windows.discard(str(name))
    if "The function is not implemented" in str(exc):
        _disable_preview(str(exc))


def _window_visible(window_name: str) -> bool:
    try:
        return float(cv2.getWindowProperty(str(window_name), cv2.WND_PROP_VISIBLE)) >= 1.0
    except Exception:
        return False


def normalized_uv_to_pixel(u: float, v: float, *, image_width: int, image_height: int) -> tuple[int, int]:
    w = max(int(image_width), 1)
    h = max(int(image_height), 1)
    px = int(round((float(u) + 1.0) * 0.5 * float(w)))
    py = int(round((float(v) + 1.0) * 0.5 * float(h)))
    return max(0, min(w - 1, px)), max(0, min(h - 1, py))


def _draw_uv_crosshair(
    vis: np.ndarray,
    u: float,
    v: float,
    *,
    color: tuple[int, int, int],
    size: int = 14,
    thickness: int = 2,
) -> None:
    h, w = vis.shape[:2]
    px, py = normalized_uv_to_pixel(u, v, image_width=w, image_height=h)
    arm = max(4, int(size))
    cv2.line(vis, (px - arm, py), (px + arm, py), color, thickness, cv2.LINE_AA)
    cv2.line(vis, (px, py - arm), (px, py + arm), color, thickness, cv2.LINE_AA)
    cv2.circle(vis, (px, py), 4, color, 1, cv2.LINE_AA)


def _draw_uv_targets(
    vis: np.ndarray,
    *,
    target_uv: Optional[tuple[float, float]] = None,
    center_uv: Optional[tuple[float, float]] = None,
) -> None:
    h, w = vis.shape[:2]
    if target_uv is not None:
        tu, tv = float(target_uv[0]), float(target_uv[1])
        _draw_uv_crosshair(vis, tu, tv, color=(255, 180, 0), size=16, thickness=2)
        cv2.putText(vis, f"target u,v=({tu:+.3f},{tv:+.3f})", (12, h - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 180, 0), 1, cv2.LINE_AA)
    if center_uv is not None:
        cu, cv = float(center_uv[0]), float(center_uv[1])
        _draw_uv_crosshair(vis, cu, cv, color=(0, 255, 255), size=10, thickness=1)
        cv2.putText(vis, f"center u,v=({cu:+.3f},{cv:+.3f})", (12, h - 68), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
        if target_uv is not None:
            p0 = normalized_uv_to_pixel(cu, cv, image_width=w, image_height=h)
            p1 = normalized_uv_to_pixel(tu, tv, image_width=w, image_height=h)
            cv2.line(vis, p0, p1, (200, 200, 200), 1, cv2.LINE_AA)


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
        cv2.putText(vis, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 0), 1, cv2.LINE_AA)
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
    image_scale: Optional[float] = None,
    bbox_wh: Optional[tuple[int, int]] = None,
    tracker_phase: str = "",
    tracker_backend: str = "",
    target_uv: Optional[tuple[float, float]] = None,
    center_uv: Optional[tuple[float, float]] = None,
) -> np.ndarray:
    vis = np.asarray(color_bgr, dtype=np.uint8).copy()
    h, w = vis.shape[:2]
    _draw_uv_targets(vis, target_uv=target_uv, center_uv=center_uv)

    dets = list(all_detections or [])
    target_key = target_label.strip().lower()
    for other in dets:
        if det is not None and other.label == det.label and other.bbox_xyxy == det.bbox_xyxy:
            continue
        x0, y0, x1, y1 = other.bbox_xyxy
        is_target = bool(target_key) and other.label.strip().lower() == target_key
        color = (0, 255, 0) if is_target else (0, 220, 255)
        cv2.rectangle(vis, (x0, y0), (x1, y1), color, 1)
        cv2.putText(vis, f"{other.label} {other.confidence:.2f}", (max(0, x0), max(12, y1 + 14)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)

    _draw_class_panel(vis, all_detections=dets, target_label=target_label, model_classes=model_classes or [])
    if det is not None and det.mask is not None and det.mask.shape[:2] == (h, w):
        mask = det.mask > 0
        tint = vis.copy()
        tint[mask] = (0, 200, 0)
        vis = cv2.addWeighted(vis, 0.65, tint, 0.35, 0.0)
        x0, y0, x1, y1 = det.bbox_xyxy
        cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2.putText(vis, f"{det.label} {det.confidence:.2f}", (max(0, x0), max(20, y0 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
    else:
        msg = "searching"
        if target_label:
            msg += f" ({target_label})"
        if dets:
            msg += f" | saw {len(dets)} det"
        cv2.putText(vis, msg, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 180, 255), 2, cv2.LINE_AA)

    diag_parts: list[str] = []
    if str(tracker_phase).strip():
        diag_parts.append(f"phase={tracker_phase}")
    if str(tracker_backend).strip():
        diag_parts.append(f"trk={tracker_backend}")
    if image_scale is not None:
        diag_parts.append(f"scale={float(image_scale):.3f}")
    if bbox_wh is not None:
        diag_parts.append(f"bbox={int(bbox_wh[0])}x{int(bbox_wh[1])}px")
    line2 = f"status={status} frame={frame_idx}"
    if diag_parts:
        line2 += " | " + " ".join(diag_parts)
    if p_camera is not None:
        p = np.asarray(p_camera, dtype=float).reshape(3)
        line2 += f" | cam=[{p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}]m"
    if p_world is not None:
        p = np.asarray(p_world, dtype=float).reshape(3)
        line2 += f" | world=[{p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}]m"
    status_y = h - 86 if (target_uv is not None or center_uv is not None) else h - 14
    cv2.putText(vis, line2, (12, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (240, 240, 240), 1, cv2.LINE_AA)
    quit_y = h - 32 if (target_uv is not None or center_uv is not None) else h - 50
    cv2.putText(vis, "q/ESC=quit", (12, quit_y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
    return vis


def show_preview(window_name: str, image_bgr: np.ndarray) -> int:
    if _preview_disabled_reason is not None:
        return -1
    name = str(window_name)
    try:
        vis = np.ascontiguousarray(image_bgr, dtype=np.uint8)
        if vis.ndim != 3 or vis.shape[2] != 3 or vis.size == 0:
            return -1
    except Exception as exc:
        _handle_preview_error(name, exc)
        return -1
    _ensure_preview_thread()
    with _preview_cond:
        _preview_close_requests.discard(name)
        _preview_frames[name] = vis.copy()
        key = int(_preview_keys.pop(name, -1))
        _preview_cond.notify_all()
    return key


def close_preview(window_name: str) -> None:
    name = str(window_name)
    _ensure_preview_thread()
    deadline = time.monotonic() + 1.0
    with _preview_cond:
        _preview_frames.pop(name, None)
        _preview_close_requests.add(name)
        _preview_cond.notify_all()
        while name in _open_preview_windows and time.monotonic() < deadline:
            _preview_cond.wait(timeout=0.05)
