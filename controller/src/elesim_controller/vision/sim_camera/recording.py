from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np


class SimCameraVideoRecorder:
    """Record a DDS RGB-D topic to MP4 with wall-clock pacing."""

    def __init__(
        self,
        endpoint: str,
        *,
        out_path: str | Path,
        fps: float = 30.0,
        use_jpeg: bool = True,
    ) -> None:
        self.endpoint = str(endpoint)
        self.out_path = Path(out_path)
        self.fps = float(max(1.0, fps))
        self.use_jpeg = bool(use_jpeg)
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._frame_count = 0
        self._unique_count = 0
        self._last_error = ""

    @property
    def last_error(self) -> str:
        with self._lock:
            return str(self._last_error)

    @property
    def frame_count(self) -> int:
        with self._lock:
            return int(self._frame_count)

    @property
    def unique_count(self) -> int:
        with self._lock:
            return int(self._unique_count)

    def start(self) -> bool:
        if self._thread is not None and self._thread.is_alive():
            self._set_error("already active")
            return False
        try:
            import cv2  # noqa: F401
            from elesim_controller.vision.sim_camera.subscriber import SimCameraSubscriber  # noqa: F401
        except Exception as exc:
            self._set_error(f"dependency unavailable: {exc}")
            return False
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        self._stop_event.clear()
        with self._lock:
            self._frame_count = 0
            self._unique_count = 0
            self._last_error = ""
        self._thread = threading.Thread(
            target=self._run,
            name="sim-observer-camera-recorder",
            daemon=True,
        )
        self._thread.start()
        return True

    def stop(self, *, timeout_s: float = 5.0) -> tuple[bool, str, int, int, str]:
        thread = self._thread
        if thread is None:
            return False, str(self.out_path.resolve()), self.frame_count, self.unique_count, "not active"
        self._stop_event.set()
        thread.join(timeout=max(float(timeout_s), 0.1))
        alive = bool(thread.is_alive())
        if alive:
            self._set_error("stop timeout")
        else:
            self._thread = None
        err = self.last_error
        frame_count = self.frame_count
        unique_count = self.unique_count
        if not alive and frame_count <= 0 and not err:
            err = "no frames recorded"
        return (
            (not alive) and frame_count > 0,
            str(self.out_path.resolve()),
            frame_count,
            unique_count,
            err,
        )

    def _set_error(self, msg: str) -> None:
        with self._lock:
            self._last_error = str(msg)

    def _bump_frame(self) -> None:
        with self._lock:
            self._frame_count += 1

    def _bump_unique(self) -> None:
        with self._lock:
            self._unique_count += 1

    def _run(self) -> None:
        import cv2

        from elesim_controller.vision.sim_camera.subscriber import SimCameraSubscriber

        sub = None
        writer: Any = None
        writer_size: Optional[tuple[int, int]] = None
        last_image: Optional[np.ndarray] = None
        last_seq: Optional[int] = None
        period = 1.0 / max(float(self.fps), 1.0)
        max_writes_per_tick = max(1, int(float(self.fps) * 2.0))
        next_write_t = time.monotonic()
        try:
            sub = SimCameraSubscriber(self.endpoint)
            while not self._stop_event.is_set():
                now = time.monotonic()
                wait_s = max(0.0, min(0.05, next_write_t - now))
                try:
                    frame = sub.recv_latest(timeout_ms=max(1, int(wait_s * 1000.0)))
                except Exception as exc:
                    self._set_error(f"receive failed: {exc}")
                    frame = None
                if frame is not None:
                    image = np.asarray(frame.color_bgr)
                    if image.ndim == 3 and image.shape[-1] >= 3:
                        last_image = np.ascontiguousarray(image[..., :3], dtype=np.uint8)
                        seq = int(getattr(frame, "seq", 0))
                        if last_seq is None or seq != last_seq:
                            self._bump_unique()
                        last_seq = seq

                now = time.monotonic()
                writes_this_tick = 0
                while last_image is not None and now >= next_write_t and writes_this_tick < max_writes_per_tick:
                    writer, writer_size = self._ensure_writer(cv2, writer, writer_size, last_image)
                    if writer is None:
                        return
                    out_image = last_image
                    if writer_size is not None and (
                        int(out_image.shape[1]) != int(writer_size[0])
                        or int(out_image.shape[0]) != int(writer_size[1])
                    ):
                        out_image = cv2.resize(out_image, writer_size, interpolation=cv2.INTER_LINEAR)
                    writer.write(out_image)
                    self._bump_frame()
                    next_write_t += period
                    writes_this_tick += 1
                if writes_this_tick >= max_writes_per_tick:
                    next_write_t = time.monotonic() + period
        finally:
            if writer is not None:
                try:
                    writer.release()
                except Exception:
                    pass
            if sub is not None:
                try:
                    sub.close()
                except Exception:
                    pass

    def _ensure_writer(
        self,
        cv2: Any,
        writer: Any,
        writer_size: Optional[tuple[int, int]],
        image: np.ndarray,
    ) -> tuple[Any, Optional[tuple[int, int]]]:
        if writer is not None:
            return writer, writer_size
        h, w = int(image.shape[0]), int(image.shape[1])
        if h <= 0 or w <= 0:
            self._set_error("empty frame")
            return None, None
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(self.out_path), fourcc, float(self.fps), (w, h))
        if not writer.isOpened():
            self._set_error(f"could not open writer: {self.out_path}")
            return None, None
        return writer, (w, h)


def capture_sim_camera_snapshot(
    endpoint: str,
    *,
    use_jpeg: bool = True,
    timeout_s: float = 1.5,
):
    """Return one latest sim-camera frame, allowing PUB/SUB warmup."""
    from elesim_controller.vision.sim_camera.subscriber import SimCameraSubscriber

    deadline = time.monotonic() + max(float(timeout_s), 0.1)
    sub = SimCameraSubscriber(str(endpoint))
    try:
        while time.monotonic() < deadline:
            frame = sub.recv_latest(timeout_ms=100)
            if frame is not None:
                return frame
    finally:
        sub.close()
    return None


def save_sim_camera_snapshot(
    *,
    frame,
    out_dir: str | Path,
    stem: str,
    meta: Optional[dict[str, Any]] = None,
) -> Path:
    import cv2

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    image = np.ascontiguousarray(np.asarray(frame.color_bgr)[..., :3], dtype=np.uint8)
    path = out / f"{stem}_color.jpg"
    cv2.imwrite(str(path), image)
    meta_out = {
        "mode": "sim_observer_camera",
        "seq": int(getattr(frame, "seq", 0)),
        "ts": float(getattr(frame, "ts", 0.0)),
        "width": int(getattr(frame.intrinsics, "width", image.shape[1])),
        "height": int(getattr(frame.intrinsics, "height", image.shape[0])),
        "camera_world_origin": getattr(frame, "camera_world_origin", None),
        "camera_world_look": getattr(frame, "camera_world_look", None),
        "camera_world_right": getattr(frame, "camera_world_right", None),
    }
    if meta:
        meta_out.update(meta)
    with open(out / f"{stem}_meta.json", "w", encoding="utf-8") as f:
        json.dump(meta_out, f, indent=2)
    return path
