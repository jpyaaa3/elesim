from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .detection_utils import detection_from_bbox


@dataclass(frozen=True)
class CsrtTrackerTuning:
    psr_threshold: float = 0.0
    scale_lr: float = 0.0
    histogram_lr: float = 0.0
    padding: float = 2.0
    scale_step: float = 1.02
    bbox_smooth_alpha: float = 0.0


class BboxTracker:
    def __init__(self, tracker_type: str = "csrt", csrt_tuning: CsrtTrackerTuning | None = None) -> None:
        self.backend_name = str(tracker_type or "bbox").lower()
        self.csrt_tuning = csrt_tuning or CsrtTrackerTuning()
        self.initialized = False
        self.last_init_error = ""
        self._tracker: Any = None

    def init(self, frame, bbox) -> bool:
        self.reset()
        try:
            import cv2

            legacy = getattr(cv2, "legacy", None)
            if self.backend_name == "csrt" and hasattr(cv2, "TrackerCSRT_create"):
                self._tracker = cv2.TrackerCSRT_create()
            elif self.backend_name == "csrt" and legacy is not None and hasattr(legacy, "TrackerCSRT_create"):
                self._tracker = legacy.TrackerCSRT_create()
            elif hasattr(cv2, "TrackerKCF_create"):
                self._tracker = cv2.TrackerKCF_create()
                self.backend_name = "kcf"
            elif legacy is not None and hasattr(legacy, "TrackerKCF_create"):
                self._tracker = legacy.TrackerKCF_create()
                self.backend_name = "kcf"
            else:
                self._tracker = None
            if self._tracker is None:
                self.last_init_error = "opencv tracker unavailable; install opencv-contrib-python"
                return False
            x0, y0, x1, y1 = [int(v) for v in bbox]
            result = self._tracker.init(frame, (x0, y0, max(1, x1 - x0), max(1, y1 - y0)))
            # Some OpenCV Python builds return None on successful tracker init.
            ok = True if result is None else bool(result)
            self.initialized = ok
            if not ok:
                self.last_init_error = "opencv tracker init returned false"
            return ok
        except Exception as exc:
            self.last_init_error = str(exc)
            self._tracker = None
            self.initialized = False
            return False

    def update(self, frame):
        if self._tracker is None or not self.initialized:
            return None
        try:
            ok, bbox = self._tracker.update(frame)
        except Exception as exc:
            self.last_init_error = str(exc)
            self.initialized = False
            return None
        if not ok:
            return None
        x, y, w, h = [int(round(float(v))) for v in bbox]
        return (x, y, x + max(1, w), y + max(1, h))

    def reset(self) -> None:
        self._tracker = None
        self.initialized = False
