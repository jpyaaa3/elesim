"""Optional ZED Mini RGB-D driver.

The SDK is imported only when this driver module is selected.  A missing SDK or
camera is a hard, actionable profile error; it is never converted into a
silent RealSense fallback.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from elesim_pilot.vision.perception.depth_pose import CameraIntrinsics
from elesim_pilot.vision.perception.realsense_camera import RealSenseFrame

try:  # The vendor SDK is intentionally optional for all non-ZED installs.
    import pyzed.sl as sl
except ImportError:  # pragma: no cover - exercised by profile error tests
    sl = None  # type: ignore[assignment]


class ZedUnavailableError(RuntimeError):
    """Raised when the selected ZED profile cannot access its SDK/device."""


class ZedMiniCamera:
    def __init__(
        self,
        *,
        color_width: int = 640,
        color_height: int = 480,
        fps: int = 30,
        **_: Any,
    ) -> None:
        if sl is None:
            raise ZedUnavailableError(
                "camera profile 'zed_mini' requires the optional ZED SDK (pyzed.sl)"
            )
        self._camera = sl.Camera()
        self._params = sl.InitParameters()
        self._params.camera_resolution = self._resolution(int(color_width), int(color_height))
        self._params.camera_fps = int(fps)
        units = getattr(sl, "UNIT", None)
        if units is not None and hasattr(units, "METER"):
            # Keep the frame contract in metres, matching the RealSense
            # depth-scale conversion used by the perception pipeline.
            self._params.coordinate_units = units.METER
        depth_modes = sl.DEPTH_MODE
        self._params.depth_mode = getattr(
            depth_modes,
            "PERFORMANCE",
            getattr(depth_modes, "QUALITY", None),
        )
        self._runtime = sl.RuntimeParameters()
        self._image = sl.Mat()
        self._depth = sl.Mat()
        self._intrinsics: CameraIntrinsics | None = None

    @staticmethod
    def _resolution(width: int, height: int) -> Any:
        resolutions = sl.RESOLUTION
        if width >= 1280 or height >= 720:
            return getattr(resolutions, "HD720", getattr(resolutions, "HD1080", None))
        return getattr(resolutions, "VGA", getattr(resolutions, "HD720", None))

    @staticmethod
    def _success(status: Any) -> bool:
        success = getattr(getattr(sl, "ERROR_CODE", None), "SUCCESS", None)
        return status == success or str(status).upper().endswith("SUCCESS")

    def start(self) -> None:
        status = self._camera.open(self._params)
        if not self._success(status):
            raise ZedUnavailableError(f"failed to open ZED Mini: {status}")
        try:
            info = self._camera.get_camera_information()
            calibration = info.camera_configuration.calibration_parameters.left_cam
            resolution = info.camera_configuration.resolution
            self._intrinsics = CameraIntrinsics(
                fx=float(calibration.fx),
                fy=float(calibration.fy),
                cx=float(calibration.cx),
                cy=float(calibration.cy),
                width=int(getattr(resolution, "width", 640)),
                height=int(getattr(resolution, "height", 480)),
            )
        except Exception as exc:
            self.stop()
            raise ZedUnavailableError(f"ZED Mini calibration unavailable: {exc}") from exc

    def stop(self) -> None:
        try:
            self._camera.close()
        except Exception:
            pass
        self._intrinsics = None

    def __enter__(self) -> "ZedMiniCamera":
        self.start()
        return self

    def __exit__(self, *args: object) -> None:
        self.stop()

    def capture(self, *, timeout_ms: int = 5000) -> RealSenseFrame:
        del timeout_ms  # ZED's grab timeout is configured by the SDK runtime.
        if self._intrinsics is None:
            raise RuntimeError("ZED Mini not started; call start() first")
        status = self._camera.grab(self._runtime)
        if not self._success(status):
            raise RuntimeError(f"ZED Mini grab failed: {status}")
        view = getattr(sl, "VIEW").LEFT
        measure = getattr(sl, "MEASURE").DEPTH
        self._camera.retrieve_image(self._image, view)
        self._camera.retrieve_measure(self._depth, measure)
        color = np.asarray(self._image.get_data())
        depth = np.asarray(self._depth.get_data())
        if color.ndim == 3 and color.shape[2] > 3:
            color = color[..., :3]
        if depth.ndim == 3:
            depth = depth[..., 0]
        return RealSenseFrame(
            color_bgr=np.ascontiguousarray(color),
            depth_raw=np.ascontiguousarray(depth, dtype=np.float32),
            depth_scale=1.0,
            intrinsics=self._intrinsics,
        )


__all__ = ["ZedMiniCamera", "ZedUnavailableError"]
