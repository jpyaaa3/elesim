"""Walking baseline trial helpers: standoff stop geometry and eye-camera video."""

from __future__ import annotations

import math
from pathlib import Path
from typing import TYPE_CHECKING, Optional, Sequence

if TYPE_CHECKING:
    from engine.config_loader import AppConfigBundle
    from engine.controller.state import HostState


def sim_object_world_xyz(bundle: AppConfigBundle) -> tuple[float, float, float]:
    spawn = bundle.spawn_config
    xyz = getattr(spawn, "sim_target_xyz", (1.2, 0.0, 0.08))
    return (float(xyz[0]), float(xyz[1]), float(xyz[2]))


def horizontal_base_object_distance_m(
    base_pos: Optional[Sequence[float]],
    object_xyz: Sequence[float],
) -> Optional[float]:
    """Horizontal (xy) distance from GO2 base to object center; z ignored."""
    if base_pos is None:
        return None
    dx = float(base_pos[0]) - float(object_xyz[0])
    dy = float(base_pos[1]) - float(object_xyz[1])
    return float(math.hypot(dx, dy))


def standoff_base_pos(host: HostState) -> Optional[tuple[float, float, float]]:
    """Prefer sim-mirrored GO2 pose for standoff when hardware odom is active."""
    sim_pos = getattr(host, "go2_sim_base_pos", None)
    if sim_pos is not None:
        return sim_pos
    return host.go2_base_pos


def host_horizontal_object_distance_m(
    host: HostState,
    object_xyz: Sequence[float],
) -> Optional[float]:
    return horizontal_base_object_distance_m(standoff_base_pos(host), object_xyz)


class TrialEyeCameraVideoRecorder:
    """Record eye-in-hand sim camera frames to an MP4 file (one file per trial)."""

    def __init__(
        self,
        *,
        endpoint: str,
        use_jpeg: bool,
        out_path: Path,
        fps: float = 20.0,
    ) -> None:
        self._endpoint = str(endpoint)
        self._use_jpeg = bool(use_jpeg)
        self._out_path = Path(out_path)
        self._fps = max(1.0, float(fps))
        self._writer = None
        self._subscriber = None
        self._frame_count = 0
        self._unique_frame_count = 0
        self._last_bgr = None
        self._size: Optional[tuple[int, int]] = None

    @property
    def frame_count(self) -> int:
        return int(self._frame_count)

    @property
    def unique_frame_count(self) -> int:
        return int(self._unique_frame_count)

    @property
    def out_path(self) -> Path:
        return self._out_path

    def _ensure_subscriber(self):
        if self._subscriber is None:
            from engine.vision.sim_camera.subscriber import SimCameraSubscriber

            self._subscriber = SimCameraSubscriber(self._endpoint, use_jpeg=self._use_jpeg)
            self._subscriber.connect()

    def flush_stale(self, *, max_frames: int = 120) -> int:
        """Drop buffered ZMQ frames so the next write is fresh."""
        self._ensure_subscriber()
        assert self._subscriber is not None
        drained = 0
        for _ in range(int(max_frames)):
            if self._subscriber.recv_latest(timeout_ms=0) is None:
                break
            drained += 1
        return drained

    def _write_bgr(self, img) -> None:
        try:
            import cv2
        except ImportError:
            return
        h, w = int(img.shape[0]), int(img.shape[1])
        if self._writer is None:
            self._out_path.parent.mkdir(parents=True, exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._writer = cv2.VideoWriter(str(self._out_path), fourcc, self._fps, (w, h))
            self._size = (w, h)
        out = img
        if self._size is not None and (w, h) != self._size:
            out = cv2.resize(img, self._size, interpolation=cv2.INTER_LINEAR)
        self._writer.write(out)
        self._frame_count += 1

    def poll_and_write(self, *, timeout_ms: int = 0, hold_last: bool = True) -> bool:
        """Poll sim camera; when *hold_last*, repeat the last frame for wall-clock 1:1 pacing."""
        try:
            import cv2  # noqa: F401
        except ImportError:
            return False
        self._ensure_subscriber()
        assert self._subscriber is not None
        got_new = False
        frame = self._subscriber.recv_latest(timeout_ms=int(timeout_ms))
        if frame is not None and frame.color_bgr is not None:
            self._last_bgr = frame.color_bgr
            self._unique_frame_count += 1
            got_new = True
        if hold_last:
            if self._last_bgr is None:
                return False
            self._write_bgr(self._last_bgr)
            return True
        if not got_new or self._last_bgr is None:
            return False
        self._write_bgr(self._last_bgr)
        return True

    def close(self) -> Optional[Path]:
        if self._writer is not None:
            self._writer.release()
            self._writer = None
        if self._subscriber is not None:
            self._subscriber.close()
            self._subscriber = None
        if self._frame_count > 0 and self._out_path.is_file():
            return self._out_path
        if self._frame_count == 0 and self._out_path.is_file():
            try:
                self._out_path.unlink()
            except OSError:
                pass
        return None
