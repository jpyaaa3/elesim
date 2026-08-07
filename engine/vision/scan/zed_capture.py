"""
ZED point-cloud capture for the roll-sweep scan.

Separate from ``engine/vision/perception/realsense_camera.py`` on purpose: that
path serves the detect-and-track pipeline and only needs colour + aligned
depth, while the scan needs the full per-pixel XYZ measure. Positional tracking
is NOT enabled -- the scan takes its poses from FK, and leaving VIO running
would only add a second, unused pose estimate.

Coordinate system is pinned to ``COORDINATE_SYSTEM.IMAGE`` (+X right, +Y down,
+Z forward), which is the convention ``hand_eye.camera.json`` is written in, so
a retrieved XYZ point needs no axis permutation before the FK transform.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

import numpy as np

try:  # pragma: no cover - hardware dependency
    import pyzed.sl as sl
except Exception:  # noqa: BLE001
    sl = None  # type: ignore[assignment]


class ZedUnavailableError(RuntimeError):
    """Raised when the ZED SDK bindings or a physical camera are missing."""


@dataclass(frozen=True)
class ZedIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass
class ZedScanFrame:
    xyz: np.ndarray  # (H, W, 3) metres, camera optical frame; NaN where invalid
    color_bgr: Optional[np.ndarray] = None
    intrinsics: Optional[ZedIntrinsics] = None
    ts_s: float = 0.0


class ZedScanCamera:
    """Minimal XYZ grabber. Context-manager safe; close() is idempotent."""

    def __init__(
        self,
        *,
        resolution: str = "HD1080",
        depth_mode: str = "NEURAL",
        fps: int = 15,
        min_depth: float = 0.15,
        max_depth: float = 1.5,
        confidence: int = 50,
        texture_confidence: int = 100,
        want_color: bool = True,
    ) -> None:
        if sl is None:
            raise ZedUnavailableError("pyzed is not installed")
        init = sl.InitParameters()
        init.camera_resolution = self._enum(sl.RESOLUTION, resolution, "resolution")
        init.depth_mode = self._enum(sl.DEPTH_MODE, depth_mode, "depth_mode")
        init.coordinate_units = sl.UNIT.METER
        init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
        init.depth_minimum_distance = float(min_depth)
        init.depth_maximum_distance = float(max_depth)
        init.camera_fps = int(fps)

        self._cam = sl.Camera()
        status = self._cam.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise ZedUnavailableError(f"ZED open failed: {status}")

        self._rt = sl.RuntimeParameters()
        self._rt.confidence_threshold = int(confidence)
        self._rt.texture_confidence_threshold = int(texture_confidence)
        self._cloud = sl.Mat()
        self._left = sl.Mat()
        self._want_color = bool(want_color)
        self._closed = False
        self._intr = self._read_intrinsics()

    @staticmethod
    def _enum(enum_cls: Any, name: str, what: str) -> Any:
        value = getattr(enum_cls, str(name).strip().upper(), None)
        if value is None:
            options = [a for a in dir(enum_cls) if a.isupper()]
            raise ZedUnavailableError(f"unknown {what} '{name}'; available: {options}")
        return value

    def _read_intrinsics(self) -> Optional[ZedIntrinsics]:
        try:
            info = self._cam.get_camera_information()
            try:
                lc = info.camera_configuration.calibration_parameters.left_cam
                res = info.camera_configuration.resolution
            except AttributeError:  # older pyzed layout
                lc = info.calibration_parameters.left_cam
                res = info.camera_resolution
            return ZedIntrinsics(
                fx=float(lc.fx), fy=float(lc.fy), cx=float(lc.cx), cy=float(lc.cy),
                width=int(res.width), height=int(res.height),
            )
        except Exception:  # noqa: BLE001
            return None

    @property
    def intrinsics(self) -> Optional[ZedIntrinsics]:
        return self._intr

    def warmup(self, frames: int = 10) -> None:
        for _ in range(max(int(frames), 0)):
            self._cam.grab(self._rt)

    def grab(self) -> Optional[ZedScanFrame]:
        if self._closed:
            return None
        if self._cam.grab(self._rt) != sl.ERROR_CODE.SUCCESS:
            return None
        self._cam.retrieve_measure(self._cloud, sl.MEASURE.XYZRGBA)
        xyz = np.array(self._cloud.get_data())[:, :, :3].astype(np.float32)
        color = None
        if self._want_color:
            self._cam.retrieve_image(self._left, sl.VIEW.LEFT)
            color = np.array(self._left.get_data())[:, :, :3].copy()
        ts = 0.0
        try:
            ts = float(self._cam.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_seconds())
        except Exception:  # noqa: BLE001
            pass
        return ZedScanFrame(xyz=xyz, color_bgr=color, intrinsics=self._intr, ts_s=ts)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._cam.close()
        except Exception:  # noqa: BLE001
            pass

    def __enter__(self) -> "ZedScanCamera":
        return self

    def __exit__(self, *_exc: Any) -> None:
        self.close()


def valid_points(xyz: np.ndarray, *, min_depth: float, max_depth: float) -> np.ndarray:
    """(H, W, 3) -> (N, 3) finite points inside the depth window."""
    flat = np.asarray(xyz, dtype=float).reshape(-1, 3)
    ok = np.isfinite(flat).all(axis=1)
    z = flat[:, 2]
    ok &= (z > float(min_depth)) & (z < float(max_depth))
    return flat[ok]
