"""YOLO-based object detector (segmentation mask preferred, bbox fallback)."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import cv2
import numpy as np

_APP_ROOT = Path(__file__).resolve().parents[1]

from perception.detector import DetectionResult, _bbox_from_mask

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None  # type: ignore


class YoloUnavailableError(RuntimeError):
    """Raised when ultralytics is not installed or the model cannot be loaded."""


def resolve_yolo_device(raw: Any) -> str | int:
    """
    Normalize device for Ultralytics predict().

    Accepts:
      - int: 0, 1, 2 -> cuda device index
      - str: "0", "1", "cuda:2", "cpu"
      - None / "auto": 0 (first visible CUDA GPU)
    """
    if raw is None:
        return 0
    if isinstance(raw, int):
        return int(raw)
    text = str(raw).strip()
    if not text or text.lower() == "auto":
        return 0
    lowered = text.lower()
    if lowered == "cpu":
        return "cpu"
    if lowered.startswith("cuda:"):
        return text
    if text.isdigit():
        return int(text)
    return text


def resolve_model_path(raw: str, *, config_dir: Path | None = None) -> Path:
    """Resolve YOLO .pt path: absolute, cwd, app root, or relative to config file."""
    text = str(raw).strip()
    if not text:
        raise ValueError("empty YOLO model path")
    p = Path(text).expanduser()
    if p.is_file():
        return p.resolve()
    candidates: list[Path] = []
    if not p.is_absolute():
        candidates.append(Path.cwd() / p)
        candidates.append(_APP_ROOT / p)
        if config_dir is not None:
            candidates.append(Path(config_dir) / p)
    for cand in candidates:
        if cand.is_file():
            return cand.resolve()
    raise FileNotFoundError(f"YOLO weights not found: {text!r} (cwd={Path.cwd()})")


def _format_model_load_error(model_path: Path, exc: Exception) -> str:
    size = -1
    try:
        size = int(model_path.stat().st_size)
    except OSError:
        pass
    lines = [
        f"failed to load YOLO model: {model_path}",
        f"  error: {exc}",
        f"  file_size_bytes: {size}",
    ]
    if size >= 0 and size < 500_000:
        lines.append("  hint: file looks too small; re-download or copy a valid .pt checkpoint")
    lines.append("  hint: use absolute path in detector JSON, e.g. /home/.../yolov8n-seg.pt")
    if os.environ.get("CUDA_VISIBLE_DEVICES") is not None:
        lines.append(f"  CUDA_VISIBLE_DEVICES={os.environ.get('CUDA_VISIBLE_DEVICES')}")
    return "\n".join(lines)


def _bbox_mask(h: int, w: int, bbox_xyxy: tuple[int, int, int, int]) -> np.ndarray:
    x0, y0, x1, y1 = bbox_xyxy
    mask = np.zeros((h, w), dtype=np.uint8)
    mask[y0 : y1 + 1, x0 : x1 + 1] = 255
    return mask


def _extract_seg_mask(result: Any, index: int, h: int, w: int) -> np.ndarray | None:
    masks = getattr(result, "masks", None)
    if masks is None:
        return None
    data = getattr(masks, "data", None)
    if data is None or len(data) <= index:
        return None
    raw = data[index]
    try:
        arr = raw.cpu().numpy()
    except AttributeError:
        arr = np.asarray(raw)
    if arr.ndim == 3:
        arr = arr[0]
    arr = cv2.resize(arr.astype(np.float32), (w, h), interpolation=cv2.INTER_LINEAR)
    return ((arr > 0.5).astype(np.uint8)) * 255


class YoloDetector:
    def __init__(self, cfg: dict[str, Any]) -> None:
        if YOLO is None:
            raise YoloUnavailableError(
                "ultralytics is not installed. Install with: pip install ultralytics"
            )
        config_dir = cfg.get("_config_dir")
        config_dir_path = Path(str(config_dir)) if config_dir else None
        try:
            model_file = resolve_model_path(str(cfg.get("model", "yolov8n-seg.pt")), config_dir=config_dir_path)
        except FileNotFoundError as exc:
            raise YoloUnavailableError(str(exc)) from exc
        model_path = str(model_file)
        self._target_label = str(cfg.get("target_label", "") or "").strip().lower()
        self._conf = float(cfg.get("confidence_threshold", 0.25))
        self._iou = float(cfg.get("iou_threshold", 0.45))
        self._min_area = int(cfg.get("min_area_px", 100))
        self._imgsz = int(cfg.get("imgsz", 640))
        self._device = resolve_yolo_device(cfg.get("device", cfg.get("gpu", 0)))
        try:
            self._model = YOLO(model_path)
        except Exception as exc:
            raise YoloUnavailableError(_format_model_load_error(model_file, exc)) from exc
        self._class_names = self._read_class_names()
        print(f"[YOLO] model={model_path} device={self._device}")
        if self._target_label:
            print(f"[YOLO] target_label={self._target_label!r}")
        n_cls = len(self._class_names)
        if n_cls > 0:
            preview = ", ".join(self._class_names[:12])
            if n_cls > 12:
                preview += f", ... (+{n_cls - 12} more)"
            print(f"[YOLO] model classes ({n_cls}): {preview}")

    def _read_class_names(self) -> list[str]:
        names = getattr(self._model, "names", None) or {}
        if isinstance(names, dict):
            ordered = [str(names[k]) for k in sorted(names.keys(), key=lambda x: int(x))]
            return ordered
        return [str(x) for x in names]

    @property
    def device(self) -> str | int:
        return self._device

    @property
    def class_names(self) -> list[str]:
        return list(self._class_names)

    def _predict_raw(self, color_bgr: np.ndarray):
        return self._model.predict(
            color_bgr,
            verbose=False,
            conf=self._conf,
            iou=self._iou,
            imgsz=self._imgsz,
            device=self._device,
        )

    def _detections_from_result(self, result: Any, *, h: int, w: int) -> list[DetectionResult]:
        names = result.names or {}
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return []

        out: list[DetectionResult] = []
        for i in range(len(boxes)):
            cls_id = int(boxes.cls[i].item())
            label = str(names.get(cls_id, str(cls_id)))
            conf = float(boxes.conf[i].item())
            if conf < self._conf:
                continue
            x1, y1, x2, y2 = boxes.xyxy[i].tolist()
            bbox = (
                max(0, int(x1)),
                max(0, int(y1)),
                min(w - 1, int(x2)),
                min(h - 1, int(y2)),
            )
            if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
                continue

            mask = _extract_seg_mask(result, i, h, w)
            if mask is None:
                mask = _bbox_mask(h, w, bbox)
            if int(np.count_nonzero(mask)) < self._min_area:
                continue

            out.append(
                DetectionResult(
                    mask=mask,
                    bbox_xyxy=bbox,
                    label=label,
                    confidence=conf,
                )
            )
        out.sort(key=lambda d: float(d.confidence), reverse=True)
        return out

    def list_detections(self, color_bgr: np.ndarray) -> list[DetectionResult]:
        """All detections in frame above confidence threshold (no target_label filter)."""
        h, w = color_bgr.shape[:2]
        results = self._predict_raw(color_bgr)
        if not results:
            return []
        return self._detections_from_result(results[0], h=h, w=w)

    def detect(self, color_bgr: np.ndarray) -> DetectionResult | None:
        dets = self.list_detections(color_bgr)
        if not dets:
            return None
        if not self._target_label:
            return dets[0]

        best: DetectionResult | None = None
        best_conf = -1.0
        for det in dets:
            if det.label.strip().lower() != self._target_label:
                continue
            if float(det.confidence) > best_conf:
                best = det
                best_conf = float(det.confidence)
        return best
