"""RealSense D435i color + aligned depth capture."""

from __future__ import annotations

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

    def start(self) -> None:
        try:
            self._profile = self._pipeline.start(self._config)
        except Exception as exc:
            raise RealSenseUnavailableError(
                f"failed to start RealSense pipeline (is D435i connected?): {exc}"
            ) from exc
        depth_sensor = self._profile.get_device().first_depth_sensor()
        self._depth_scale = float(depth_sensor.get_depth_scale())

    def stop(self) -> None:
        try:
            self._pipeline.stop()
        except Exception:
            pass

    def __enter__(self) -> RealSenseCamera:
        self.start()
        return self

    def __exit__(self, *args: object) -> None:
        self.stop()

    def capture(self) -> RealSenseFrame:
        if self._profile is None:
            raise RuntimeError("camera not started; call start() first")
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
