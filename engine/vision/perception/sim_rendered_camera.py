"""RealSense-compatible adapter over the Genesis sim camera relay."""

from __future__ import annotations

from typing import Optional

from engine.vision.perception.depth_pose import CameraIntrinsics
from engine.vision.perception.realsense_camera import RealSenseFrame
from engine.vision.sim_camera.subscriber import SimCameraSubscriber


class SimRenderedCamera:
    def __init__(
        self,
        *,
        endpoint: str = "tcp://127.0.0.1:5568",
        use_jpeg: bool = True,
        timeout_ms: int = 500,
    ) -> None:
        self._endpoint = str(endpoint)
        self._use_jpeg = bool(use_jpeg)
        self._timeout_ms = int(timeout_ms)
        self._sub: SimCameraSubscriber | None = None

    def start(self) -> None:
        self._sub = SimCameraSubscriber(self._endpoint, use_jpeg=self._use_jpeg)
        self._sub.connect()

    def stop(self) -> None:
        if self._sub is not None:
            self._sub.close()
            self._sub = None

    def __enter__(self) -> "SimRenderedCamera":
        self.start()
        return self

    def __exit__(self, *args: object) -> None:
        self.stop()

    def capture(self, *, retries: int = 40, retry_sleep_s: float = 0.05) -> RealSenseFrame:
        if self._sub is None:
            raise RuntimeError("SimRenderedCamera not started")
        last_exc: Optional[Exception] = None
        for _ in range(max(1, int(retries))):
            frame = self._sub.recv_latest(timeout_ms=self._timeout_ms)
            if frame is not None:
                intr = frame.intrinsics
                return RealSenseFrame(
                    color_bgr=frame.color_bgr,
                    depth_raw=frame.depth_raw,
                    depth_scale=float(frame.depth_scale),
                    intrinsics=CameraIntrinsics(
                        fx=float(intr.fx),
                        fy=float(intr.fy),
                        cx=float(intr.cx),
                        cy=float(intr.cy),
                        width=int(intr.width),
                        height=int(intr.height),
                    ),
                    camera_world_origin=frame.camera_world_origin,
                    camera_world_look=frame.camera_world_look,
                    camera_world_right=frame.camera_world_right,
                )
            last_exc = RuntimeError(f"no sim camera frame received from {self._endpoint}")
            if retry_sleep_s > 0:
                import time

                time.sleep(float(retry_sleep_s))
        raise last_exc or RuntimeError(f"no sim camera frame received from {self._endpoint}")
