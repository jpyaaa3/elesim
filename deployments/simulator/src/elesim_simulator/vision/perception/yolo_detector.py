"""YOLO object detector, segmentation mask preferred with bbox fallback."""

from __future__ import annotations

import os
import threading
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from elesim_simulator.vision.perception.detector import DetectionResult


class YoloUnavailableError(RuntimeError):
    """Raised when ultralytics is unavailable or the model cannot be loaded."""


_YOLO_MODEL_CACHE: dict[tuple[str, str], Any] = {}
_YOLO_MODEL_CACHE_LOCK = threading.Lock()


def _load_yolo_class() -> Any:
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise YoloUnavailableError(
            f"failed to import ultralytics.YOLO: {type(exc).__name__}: {exc}"
        ) from exc
    return YOLO


def resolve_yolo_device(raw: Any) -> str | int:
    import torch

    def auto_device() -> str | int:
        if torch.cuda.is_available():
            return 0
        if getattr(torch.backends, "mps", None) is not None and torch.backends.mps.is_available():
            return "mps"
        return "cpu"

    if raw is None:
        return auto_device()
    if isinstance(raw, int):
        if int(raw) == 0 and not torch.cuda.is_available():
            return auto_device()
        return int(raw)
    text = str(raw).strip()
    if not text or text.lower() == "auto":
        return auto_device()
    lowered = text.lower()
    if lowered in ("cpu", "mps"):
        return lowered
    if lowered.startswith("cuda:"):
        return text
    if text.isdigit():
        idx = int(text)
        if idx == 0 and not torch.cuda.is_available():
            return auto_device()
        return idx
    return text


def resolve_model_path(raw: str, *, config_dir: Path | None = None) -> Path:
    text = str(raw).strip()
    if not text:
        raise ValueError("empty YOLO model path")
    p = Path(text).expanduser()
    if p.is_file():
        return p.resolve()
    candidates: list[Path] = []
    if not p.is_absolute():
        candidates.append(Path.cwd() / p)
        if config_dir is not None:
            candidates.append(Path(config_dir) / p)
    for cand in candidates:
        if cand.is_file():
            return cand.resolve()
    if p.suffix.lower() == ".pt" and not p.is_absolute() and "/" not in text and "\\" not in text:
        return p
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
        lines.append("  hint: file looks too small; copy a valid .pt checkpoint")
    lines.append("  hint: use an absolute model path in detector JSON")
    if os.environ.get("CUDA_VISIBLE_DEVICES") is not None:
        lines.append(f"  CUDA_VISIBLE_DEVICES={os.environ.get('CUDA_VISIBLE_DEVICES')}")
    return "\n".join(lines)


def _cached_yolo_model(model_file: Path, device: str | int) -> tuple[Any, bool]:
    key = (str(Path(model_file).resolve()), str(device))
    with _YOLO_MODEL_CACHE_LOCK:
        cached = _YOLO_MODEL_CACHE.get(key)
        if cached is not None:
            return cached, True
    yolo_class = _load_yolo_class()
    try:
        model = yolo_class(str(model_file))
    except Exception as exc:
        raise YoloUnavailableError(_format_model_load_error(model_file, exc)) from exc
    with _YOLO_MODEL_CACHE_LOCK:
        existing = _YOLO_MODEL_CACHE.get(key)
        if existing is not None:
            return existing, True
        _YOLO_MODEL_CACHE[key] = model
    return model, False


def _bbox_mask(h: int, w: int, bbox_xyxy: tuple[int, int, int, int]) -> np.ndarray:
    x0, y0, x1, y1 = bbox_xyxy
    mask = np.zeros((h, w), dtype=np.uint8)
    mask[y0 : y1 + 1, x0 : x1 + 1] = 255
    return mask


def _extract_seg_mask(result: Any, index: int, h: int, w: int, *, mask_threshold: float = 0.5) -> np.ndarray | None:
    masks = getattr(result, "masks", None)
    if masks is None:
        return None
    data = getattr(masks, "data", None)
    if data is None or len(data) <= index:
        return None
    raw = data[index]
    if raw is None:
        return None
    try:
        arr = raw.cpu().numpy()
    except AttributeError:
        arr = np.asarray(raw)
    if arr is None:
        return None
    arr = np.asarray(arr)
    if arr.size <= 0:
        return None
    if arr.ndim == 3:
        arr = arr[0]
    if arr.ndim != 2:
        return None
    arr = cv2.resize(arr.astype(np.float32), (w, h), interpolation=cv2.INTER_LINEAR)
    thr = float(max(0.0, min(1.0, mask_threshold)))
    return ((arr > thr).astype(np.uint8)) * 255


class YoloDetector:
    def __init__(self, cfg: dict[str, Any]) -> None:
        config_dir = cfg.get("_config_dir")
        config_dir_path = Path(str(config_dir)) if config_dir else None
        try:
            model_file = resolve_model_path(str(cfg.get("model", "yolov8n-seg.pt")), config_dir=config_dir_path)
        except FileNotFoundError as exc:
            raise YoloUnavailableError(str(exc)) from exc
        self._target_label = str(cfg.get("target_label", "") or "").strip().lower()
        self._conf = float(cfg.get("confidence_threshold", 0.25))
        self._iou = float(cfg.get("iou_threshold", 0.45))
        self._min_area = int(cfg.get("min_area_px", 100))
        self._mask_threshold = float(cfg.get("mask_threshold", 0.5))
        self._imgsz = int(cfg.get("imgsz", 640))
        self._device = resolve_yolo_device(cfg.get("device", cfg.get("gpu", 0)))
        self._model, cache_hit = _cached_yolo_model(model_file, self._device)
        self._class_names = self._read_class_names()
        cache_tag = "cached" if cache_hit else "loaded"
        print(f"[YOLO] {cache_tag} model={model_file} device={self._device}")

    def _read_class_names(self) -> list[str]:
        names = getattr(self._model, "names", None) or {}
        if isinstance(names, dict):
            return [str(names[k]) for k in sorted(names.keys(), key=lambda x: int(x))]
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
            bbox = (max(0, int(x1)), max(0, int(y1)), min(w - 1, int(x2)), min(h - 1, int(y2)))
            if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
                continue
            mask = _extract_seg_mask(result, i, h, w, mask_threshold=float(self._mask_threshold))
            if mask is None:
                mask = _bbox_mask(h, w, bbox)
            if int(np.count_nonzero(mask)) < self._min_area:
                continue
            out.append(DetectionResult(mask=mask, bbox_xyxy=bbox, label=label, confidence=conf))
        out.sort(key=lambda d: float(d.confidence), reverse=True)
        return out

    def list_detections(self, color_bgr: np.ndarray) -> list[DetectionResult]:
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
        matches = [d for d in dets if d.label.strip().lower() == self._target_label]
        if not matches:
            return None
        return max(matches, key=lambda d: float(d.confidence))
