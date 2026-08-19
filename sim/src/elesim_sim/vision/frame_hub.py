"""Latest-frame exchange shared by Genesis capture and media transports."""

from __future__ import annotations

import threading
from typing import Any, Iterable, Optional

import numpy as np


class FrameHub:
    def __init__(self, streams: Iterable[str]) -> None:
        names = tuple(str(name).strip() for name in streams)
        if not names or any(not name for name in names) or len(set(names)) != len(names):
            raise ValueError("FrameHub stream names must be non-empty and unique")
        self._lock = threading.Lock()
        self._frames: dict[str, Optional[Any]] = {name: None for name in names}
        self._versions: dict[str, int] = {name: 0 for name in names}

    def publish(self, stream: str, frame: Any) -> int:
        name = str(stream)
        with self._lock:
            if name not in self._frames:
                raise KeyError(f"unknown frame stream: {name}")
            self._frames[name] = frame
            self._versions[name] += 1
            return self._versions[name]

    def latest(self, stream: str) -> Optional[Any]:
        with self._lock:
            if stream not in self._frames:
                raise KeyError(f"unknown frame stream: {stream}")
            return self._frames[stream]

    def latest_bgr(self, stream: str) -> Optional[np.ndarray]:
        frame = self.latest(stream)
        if frame is None:
            return None
        image = getattr(frame, "color_bgr", None)
        return image if isinstance(image, np.ndarray) else None

    def version(self, stream: str) -> int:
        with self._lock:
            if stream not in self._versions:
                raise KeyError(f"unknown frame stream: {stream}")
            return self._versions[stream]


__all__ = ["FrameHub"]
