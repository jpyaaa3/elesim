from __future__ import annotations

from typing import Any, Callable, Optional

from elesim_protocol import (
    DdsEncodedRgbdSubscriber,
    DdsRgbdSubscriber,
    DdsRuntimeSettings,
    RgbdSample,
)
from elesim_sim.observability.tracing import sampled_traced
from elesim_sim.vision.sim_camera.types import (
    SimCameraFrame,
    SimCameraIntrinsics,
)


def _sim_frame(sample: RgbdSample) -> SimCameraFrame:
    intrinsics = sample.intrinsics
    return SimCameraFrame(
        color_bgr=sample.color_bgr,
        depth_raw=sample.depth_raw,
        depth_scale=sample.depth_scale,
        intrinsics=SimCameraIntrinsics(
            fx=intrinsics.fx,
            fy=intrinsics.fy,
            cx=intrinsics.cx,
            cy=intrinsics.cy,
            width=intrinsics.width,
            height=intrinsics.height,
        ),
        seq=sample.seq,
        ts=sample.ts,
        arm_q=sample.arm_q,
        camera_world_origin=sample.camera_world_origin,
        camera_world_look=sample.camera_world_look,
        camera_world_right=sample.camera_world_right,
    )


def _sim_encoded_frame(sample: object) -> SimCameraFrame:
    from elesim_protocol import rgbd_from_encoded_frame

    return _sim_frame(rgbd_from_encoded_frame(sample))  # type: ignore[arg-type]


class SimCameraSubscriber:
    """Receive the newest simulated RGB-D sample from DDS."""

    def __init__(
        self,
        topic: str,
        *,
        endpoint_id: str,
        settings: Optional[DdsRuntimeSettings] = None,
        expected_source_id: str = "",
        expected_boot_id: str = "",
        wire_format: str = "raw-rgbd-v1",
        subscriber_factory: Callable[..., Any] = DdsRgbdSubscriber,
    ) -> None:
        self.topic = str(topic)
        self.wire_format = str(wire_format).strip().lower() or "raw-rgbd-v1"
        if self.wire_format not in {"raw-rgbd-v1", "encoded-rgbd-v1"}:
            raise ValueError(f"unsupported RGB-D wire format: {wire_format!r}")
        if self.wire_format == "encoded-rgbd-v1" and subscriber_factory is DdsRgbdSubscriber:
            subscriber_factory = DdsEncodedRgbdSubscriber
        self._encoded = self.wire_format == "encoded-rgbd-v1"
        self._subscriber = subscriber_factory(
            self.topic,
            endpoint_id=str(endpoint_id),
            settings=settings,
            expected_source_id=str(expected_source_id),
            expected_boot_id=str(expected_boot_id),
        )

    def connect(self) -> None:
        self._subscriber.connect()

    def close(self) -> None:
        self._subscriber.close()

    @sampled_traced(
        "camera.sim.receive",
        sample_key="camera.sim.receive",
        every=60,
        kind="consumer",
    )
    def recv_latest(
        self,
        *,
        timeout_ms: int = 500,
    ) -> Optional[SimCameraFrame]:
        sample = self._subscriber.recv_latest(timeout_ms=timeout_ms)
        return None if sample is None else (
            _sim_encoded_frame(sample) if self._encoded else _sim_frame(sample)
        )


__all__ = ["SimCameraSubscriber", "_sim_frame", "_sim_encoded_frame"]
