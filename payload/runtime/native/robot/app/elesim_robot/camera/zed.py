"""ZED Mini stereo color images and registered metric depth."""
from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any

import numpy as np

from elesim_protocol.rgbd import RgbdIntrinsics


@dataclass
class ZedFrame:
    color_bgr: np.ndarray
    depth_raw: np.ndarray
    depth_scale: float
    intrinsics: RgbdIntrinsics
    right_color_bgr: np.ndarray | None = None
    right_intrinsics: RgbdIntrinsics | None = None
    ts: float = 0.0


class ZedMiniCamera:
    def __init__(self, *, color_width=640, color_height=480, fps=30) -> None:
        try:
            import pyzed.sl as sl
        except ImportError as exc:
            raise RuntimeError("ZED SDK Python binding pyzed.sl is unavailable") from exc
        self._sl = sl
        self._width, self._height, self._fps = int(color_width), int(color_height), int(fps)
        self._camera: Any = None

    def start(self) -> None:
        if self._camera is not None:
            return
        sl = self._sl
        devices = [d for d in sl.Camera.get_device_list()
                   if d.camera_model == sl.MODEL.ZED_M
                   and d.camera_state == sl.CAMERA_STATE.AVAILABLE
                   and int(d.serial_number) > 0]
        if not devices:
            raise RuntimeError("No ZED Mini video device detected; check the USB 3 connection")
        params = sl.InitParameters()
        params.set_from_serial_number(devices[0].serial_number)
        params.camera_resolution = sl.RESOLUTION.HD720
        params.camera_fps = self._fps
        params.coordinate_units = sl.UNIT.MILLIMETER
        params.depth_mode = sl.DEPTH_MODE.QUALITY
        camera = sl.Camera()
        error = camera.open(params)
        if error != sl.ERROR_CODE.SUCCESS:
            camera.close()
            raise RuntimeError(f"ZED Mini open failed: {error}")
        self._camera = camera
        self._resolution = sl.Resolution(self._width, self._height)
        self._image, self._right_image, self._depth = sl.Mat(), sl.Mat(), sl.Mat()
        self._runtime = sl.RuntimeParameters()
        calibration = camera.get_camera_information(
            self._resolution
        ).camera_configuration.calibration_parameters
        self._intrinsics = self._intrinsics_from_calibration(calibration.left_cam)
        self._right_intrinsics = self._intrinsics_from_calibration(calibration.right_cam)

    def _intrinsics_from_calibration(self, calibration: Any) -> RgbdIntrinsics:
        return RgbdIntrinsics(
            float(calibration.fx),
            float(calibration.fy),
            float(calibration.cx),
            float(calibration.cy),
            self._width,
            self._height,
        )

    def capture(self, *, timeout_ms=1000) -> ZedFrame:
        if self._camera is None:
            raise RuntimeError("ZED Mini is not started")
        sl = self._sl
        error = self._camera.grab(self._runtime)
        if error != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED Mini grab failed: {error}")
        capture_time = time.time()
        if hasattr(self._camera, "get_timestamp") and hasattr(sl, "TIME_REFERENCE"):
            sensor_time = self._camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()/1e9
            if np.isfinite(sensor_time) and abs(sensor_time-capture_time) < 5:
                capture_time = float(sensor_time)
        for error in (
            self._camera.retrieve_image(self._image, sl.VIEW.LEFT, sl.MEM.CPU, self._resolution),
            self._camera.retrieve_image(self._right_image, sl.VIEW.RIGHT, sl.MEM.CPU, self._resolution),
            self._camera.retrieve_measure(self._depth, sl.MEASURE.DEPTH, sl.MEM.CPU, self._resolution),
        ):
            if error != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"ZED Mini stereo RGB-D retrieval failed: {error}")
        color = np.ascontiguousarray(self._image.get_data()[..., :3]).copy()
        right_color = np.ascontiguousarray(self._right_image.get_data()[..., :3]).copy()
        millimeters = np.asarray(self._depth.get_data())
        valid = np.isfinite(millimeters) & (millimeters > 0) & (millimeters <= 65535)
        depth = np.where(valid, np.rint(millimeters), 0).astype(np.uint16)
        return ZedFrame(
            color,
            depth,
            0.001,
            self._intrinsics,
            right_color_bgr=right_color,
            right_intrinsics=self._right_intrinsics,
            ts=capture_time,
        )

    def stop(self) -> None:
        if self._camera is not None:
            self._camera.close()
            self._camera = None
