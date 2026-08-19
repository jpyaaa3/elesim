"""Pilot adapter for the typed latest-only DDS RGB-D stream."""

from __future__ import annotations

from typing import Optional

from elesim_protocol import DdsRgbdSubscriber, DdsRuntimeSettings

from elesim_pilot.observability.tracing import sampled_traced
from elesim_pilot.vision.sim_camera.types import (
    SimCameraFrame,
    SimCameraIntrinsics,
)


class SimCameraSubscriber:
    """Receive a target's coherent color/depth sample from one DDS topic."""

    def __init__(
        self,
        topic: str,
        *,
        endpoint_id: str = "pilot-rgbd",
        dds_settings: DdsRuntimeSettings | None = None,
        expected_source_id: str = "",
        expected_boot_id: str = "",
    ) -> None:
        self.topic = str(topic)
        self.endpoint_id = str(endpoint_id)
        self.dds_settings = dds_settings
        self._subscriber = DdsRgbdSubscriber(
            self.topic,
            endpoint_id=self.endpoint_id,
            settings=self.dds_settings,
            expected_source_id=str(expected_source_id),
            expected_boot_id=str(expected_boot_id),
        )
        self._connected = False

    def connect(self) -> None:
        if self._connected:
            return
        self._subscriber.connect()
        self._connected = True

    def close(self) -> None:
        self._subscriber.close()
        self._connected = False

    @sampled_traced(
        "camera.sim.receive",
        sample_key="camera.sim.receive",
        every=60,
        kind="consumer",
    )
    def recv_latest(self, *, timeout_ms: int = 500) -> Optional[SimCameraFrame]:
        if not self._connected:
            self.connect()
        sample = self._subscriber.recv_latest(timeout_ms=timeout_ms)
        if sample is None:
            return None
        intrinsics = sample.intrinsics
        return SimCameraFrame(
            color_bgr=sample.color_bgr,
            depth_raw=sample.depth_raw,
            depth_scale=float(sample.depth_scale),
            intrinsics=SimCameraIntrinsics(
                fx=float(intrinsics.fx),
                fy=float(intrinsics.fy),
                cx=float(intrinsics.cx),
                cy=float(intrinsics.cy),
                width=int(intrinsics.width),
                height=int(intrinsics.height),
            ),
            seq=int(sample.seq),
            ts=float(sample.ts),
            arm_q=sample.arm_q,
            camera_world_origin=sample.camera_world_origin,
            camera_world_look=sample.camera_world_look,
            camera_world_right=sample.camera_world_right,
        )
