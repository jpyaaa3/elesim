"""RealSense-compatible adapter over the Genesis sim camera relay."""

from __future__ import annotations

from typing import Optional

from elesim_protocol import DdsRuntimeSettings

from elesim_pilot.vision.perception.depth_pose import CameraIntrinsics
from elesim_pilot.vision.perception.realsense_camera import RealSenseFrame
from elesim_pilot.vision.sim_camera.subscriber import SimCameraSubscriber


class SimRenderedCamera:
    def __init__(
        self,
        *,
        topic: str = "/elesim/sim/rgbd/frame",
        timeout_ms: int = 500,
        endpoint_id: str = "pilot-rgbd",
        dds_settings: DdsRuntimeSettings | None = None,
        expected_source_id: str = "",
        expected_boot_id: str = "",
        wire_format: str = "raw-rgbd-v1",
    ) -> None:
        self._topic = str(topic)
        self._timeout_ms = int(timeout_ms)
        self._endpoint_id = str(endpoint_id)
        self._dds_settings = dds_settings
        self._expected_source_id = str(expected_source_id)
        self._expected_boot_id = str(expected_boot_id)
        self._wire_format = str(wire_format).strip().lower() or "raw-rgbd-v1"
        self._sub: SimCameraSubscriber | None = None

    def start(self) -> None:
        self._sub = SimCameraSubscriber(
            self._topic,
            endpoint_id=self._endpoint_id,
            dds_settings=self._dds_settings,
            expected_source_id=self._expected_source_id,
            expected_boot_id=self._expected_boot_id,
            wire_format=self._wire_format,
        )
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
            last_exc = RuntimeError(
                f"no DDS RGB-D sample received from {self._topic}"
            )
            if retry_sleep_s > 0:
                import time

                time.sleep(float(retry_sleep_s))
        raise last_exc or RuntimeError(
            f"no DDS RGB-D sample received from {self._topic}"
        )
