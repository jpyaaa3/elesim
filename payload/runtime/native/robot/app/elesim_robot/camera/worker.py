"""Lifecycle-managed RGB-D publisher worker."""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Optional

import numpy as np

from elesim_protocol import (
    DdsEncodedRgbdPublisher,
    DdsRgbdPublisher,
    DdsRuntimeSettings,
    encoded_frame_from_rgbd,
)

from elesim_protocol.rgbd import RgbdFrame, RgbdIntrinsics


class CameraPublisherThread:
    def __init__(
        self,
        topic: str,
        *,
        endpoint_id: str,
        boot_id: str,
        dds_settings: DdsRuntimeSettings,
        width: int,
        height: int,
        fps: int,
        right_topic: str = "",
        rotate_180: bool = False,
        wire_format: str = "raw-rgbd-v1",
        color_codec: str = "jpeg",
        depth_codec: str = "zlib",
        jpeg_quality: int = 85,
        camera_factory: Optional[Callable[..., Any]] = None,
        publisher_factory: Callable[..., Any] = DdsRgbdPublisher,
    ) -> None:
        self.topic = str(topic)
        self.endpoint_id = str(endpoint_id)
        self.boot_id = str(boot_id)
        self.dds_settings = dds_settings
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.right_topic = str(right_topic).strip()
        self.rotate_180 = bool(rotate_180)
        self.wire_format = str(wire_format).strip().lower() or "raw-rgbd-v1"
        if self.wire_format not in {"raw-rgbd-v1", "encoded-rgbd-v1"}:
            raise ValueError(f"unsupported RGB-D wire format: {wire_format!r}")
        self._encoded = self.wire_format == "encoded-rgbd-v1"
        if self.right_topic and not self._encoded:
            raise ValueError("right stereo camera topic requires encoded-rgbd-v1")
        if self._encoded and publisher_factory is DdsRgbdPublisher:
            publisher_factory = DdsEncodedRgbdPublisher
        self._color_codec = str(color_codec).strip().lower()
        self._depth_codec = str(depth_codec).strip().lower()
        self._jpeg_quality = int(jpeg_quality)
        if camera_factory is None:
            from elesim_robot.camera.realsense import RealSenseCamera
            camera_factory = RealSenseCamera
        self._camera_factory = camera_factory
        self._publisher_factory = publisher_factory
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._error = ""
        self._frames_published = 0
        self._last_frame_at = 0.0

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        with self._lock:
            self._running = True
            self._error = ""
            self._frames_published = 0
            self._last_frame_at = 0.0
        self._thread = threading.Thread(target=self._run, name="robot-rgbd", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def status(self) -> dict[str, object]:
        with self._lock:
            return {
                "running": self._running,
                "healthy": self._running and not self._error,
                "error": self._error,
                "frames_published": self._frames_published,
                "last_frame_at": self._last_frame_at,
            }

    def _run(self) -> None:
        camera: Any = None
        publisher: Any = None
        right_publisher: Any = None
        try:
            camera = self._camera_factory(
                color_width=self.width,
                color_height=self.height,
                fps=self.fps,
            )
            publisher = self._publisher_factory(
                self.topic,
                endpoint_id=self.endpoint_id,
                boot_id=self.boot_id,
                settings=self.dds_settings,
                **({"send_depth": True} if not self._encoded else {}),
            )
            if self.right_topic:
                right_publisher = self._publisher_factory(
                    self.right_topic,
                    endpoint_id=f"{self.endpoint_id}-rgbd-right",
                    boot_id=self.boot_id,
                    settings=self.dds_settings,
                )
            camera.start()
            seq = 0
            while not self._stop_event.is_set():
                source = camera.capture(timeout_ms=1000)
                seq += 1
                color = self._orient(source.color_bgr)
                depth = (
                    self._orient(source.depth_raw)
                    if source.depth_raw is not None
                    else None
                )
                raw_frame = RgbdFrame(
                    color_bgr=color,
                    depth_raw=depth,
                    depth_scale=source.depth_scale,
                    intrinsics=self._orient_intrinsics(source.intrinsics),
                    seq=seq,
                    ts=time.time(),
                )
                frame = (
                    encoded_frame_from_rgbd(
                        raw_frame,
                        source_id=self.endpoint_id,
                        source_boot_id=self.boot_id,
                        color_codec=self._color_codec,
                        depth_codec=self._depth_codec,
                        jpeg_quality=self._jpeg_quality,
                    )
                    if self._encoded
                    else raw_frame
                )
                published = publisher.publish(frame)
                if right_publisher is not None:
                    right_color = getattr(source, "right_color_bgr", None)
                    if right_color is None:
                        raise RuntimeError("configured ZED right topic has no right image")
                    right_intrinsics = getattr(source, "right_intrinsics", None)
                    if right_intrinsics is None:
                        raise RuntimeError("configured ZED right topic has no right intrinsics")
                    right_frame = RgbdFrame(
                        color_bgr=self._orient(right_color),
                        depth_raw=None,
                        depth_scale=0.0,
                        intrinsics=self._orient_intrinsics(right_intrinsics),
                        seq=seq,
                        ts=raw_frame.ts,
                    )
                    right_publisher.publish(
                        encoded_frame_from_rgbd(
                            right_frame,
                            source_id=self.endpoint_id,
                            source_boot_id=self.boot_id,
                            color_codec=self._color_codec,
                            depth_codec=self._depth_codec,
                            jpeg_quality=self._jpeg_quality,
                        )
                    )
                if published:
                    with self._lock:
                        self._frames_published += 1
                        self._last_frame_at = time.time()
        except Exception as exc:
            with self._lock:
                self._error = repr(exc)
        finally:
            if camera is not None:
                try:
                    camera.stop()
                except Exception as exc:
                    self._record_cleanup_error("camera stop", exc)
            if publisher is not None:
                try:
                    publisher.close()
                except Exception as exc:
                    self._record_cleanup_error("publisher close", exc)
            if right_publisher is not None:
                try:
                    right_publisher.close()
                except Exception as exc:
                    self._record_cleanup_error("right publisher close", exc)
            with self._lock:
                self._running = False

    def _orient(self, image: Any) -> Any:
        if not self.rotate_180:
            return image
        return np.ascontiguousarray(image[::-1, ::-1])

    def _orient_intrinsics(self, intrinsics: Any) -> RgbdIntrinsics:
        cx = float(intrinsics.cx)
        cy = float(intrinsics.cy)
        if self.rotate_180:
            cx = int(intrinsics.width) - 1 - cx
            cy = int(intrinsics.height) - 1 - cy
        return RgbdIntrinsics(
            float(intrinsics.fx),
            float(intrinsics.fy),
            cx,
            cy,
            int(intrinsics.width),
            int(intrinsics.height),
        )

    def _record_cleanup_error(self, operation: str, exc: Exception) -> None:
        with self._lock:
            suffix = f"{operation} failed: {exc!r}"
            self._error = f"{self._error}; {suffix}" if self._error else suffix


__all__ = ["CameraPublisherThread"]
