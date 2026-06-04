"""RealSense D435i color + aligned depth capture."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

import numpy as np

from perception.depth_pose import CameraIntrinsics

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None  # type: ignore


class RealSenseUnavailableError(RuntimeError):
    """Raised when pyrealsense2 or a physical device is not available."""


@dataclass
class RealSenseFrame:
    color_bgr: np.ndarray
    depth_raw: np.ndarray
    depth_scale: float
    intrinsics: CameraIntrinsics


class RealSenseCamera:
    def __init__(
        self,
        *,
        color_width: int = 640,
        color_height: int = 480,
        fps: int = 30,
        color_auto_warmup_s: float = 1.0,
    ) -> None:
        if rs is None:
            raise RealSenseUnavailableError(
                "pyrealsense2 is not installed. Install with: pip install pyrealsense2"
            )
        self._pipeline = rs.pipeline()
        self._config = rs.config()
        self._config.enable_stream(
            rs.stream.color,
            int(color_width),
            int(color_height),
            rs.format.bgr8,
            int(fps),
        )
        self._config.enable_stream(
            rs.stream.depth,
            int(color_width),
            int(color_height),
            rs.format.z16,
            int(fps),
        )
        self._align = rs.align(rs.stream.color)
        self._profile: Any = None
        self._depth_scale = 0.001
        self._color_auto_warmup_s = float(max(color_auto_warmup_s, 0.0))
        self._color_options_locked = False

    @staticmethod
    def _set_sensor_option(sensor: Any, option: Any, value: float) -> None:
        if sensor.supports(option):
            sensor.set_option(option, float(value))

    def _iter_color_sensors(self, device: Any):
        query = getattr(device, "query_sensors", None)
        sensors = query() if callable(query) else getattr(device, "sensors", ())
        for sensor in sensors:
            try:
                if sensor.is_color_sensor():
                    yield sensor
            except Exception:
                continue

    def _set_color_auto_enabled(self, *, enabled: bool) -> None:
        if self._profile is None:
            return
        value = 1.0 if bool(enabled) else 0.0
        device = self._profile.get_device()
        for sensor in self._iter_color_sensors(device):
            self._set_sensor_option(sensor, rs.option.enable_auto_exposure, value)
            self._set_sensor_option(sensor, rs.option.enable_auto_white_balance, value)
            return

    def _lock_color_options(self) -> None:
        """Freeze AE/AWB at values converged during warmup."""
        if self._profile is None or self._color_options_locked:
            return
        self._set_color_auto_enabled(enabled=False)
        self._color_options_locked = True

    def _warmup_color_auto_then_lock(self) -> None:
        """Let auto exposure/WB settle, then lock for stable tracking."""
        if self._profile is None:
            return
        self._set_color_auto_enabled(enabled=True)
        warmup_s = float(self._color_auto_warmup_s)
        if warmup_s <= 1e-6:
            self._lock_color_options()
            return
        deadline = time.monotonic() + warmup_s
        while time.monotonic() < deadline:
            frames = self._pipeline.wait_for_frames()
            self._align.process(frames)
        self._lock_color_options()

    def start(self) -> None:
        try:
            self._profile = self._pipeline.start(self._config)
        except Exception as exc:
            raise RealSenseUnavailableError(
                f"failed to start RealSense pipeline (is D435i connected?): {exc}"
            ) from exc
        self._color_options_locked = False
        depth_sensor = self._profile.get_device().first_depth_sensor()
        self._depth_scale = float(depth_sensor.get_depth_scale())
        self._warmup_color_auto_then_lock()

    def stop(self) -> None:
        try:
            self._pipeline.stop()
        except Exception:
            pass
        self._profile = None
        self._color_options_locked = False

    def __enter__(self) -> RealSenseCamera:
        self.start()
        return self

    def __exit__(self, *args: object) -> None:
        self.stop()

    def capture(self) -> RealSenseFrame:
        if self._profile is None:
            raise RuntimeError("camera not started; call start() first")
        if not self._color_options_locked:
            self._lock_color_options()
        frames = self._pipeline.wait_for_frames()
        aligned = self._align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            raise RuntimeError("RealSense returned empty color or depth frame")

        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())
        intr = color_frame.profile.as_video_stream_profile().intrinsics
        intrinsics = CameraIntrinsics(
            fx=float(intr.fx),
            fy=float(intr.fy),
            cx=float(intr.ppx),
            cy=float(intr.ppy),
            width=int(intr.width),
            height=int(intr.height),
        )
        return RealSenseFrame(
            color_bgr=color,
            depth_raw=depth,
            depth_scale=float(self._depth_scale),
            intrinsics=intrinsics,
        )
