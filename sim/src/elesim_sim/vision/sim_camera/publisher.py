from __future__ import annotations

from typing import Any, Callable, Optional

from elesim_protocol import (
    DdsEncodedRgbdPublisher,
    DdsRgbdPublisher,
    DdsRuntimeSettings,
    encoded_frame_from_rgbd,
)
from elesim_sim.observability.tracing import sampled_traced
from elesim_sim.vision.sim_camera.types import SimCameraFrame


class SimCameraPublisher:
    """Publish simulated eye-in-hand RGB-D frames on a typed DDS topic."""

    def __init__(
        self,
        topic: str,
        *,
        endpoint_id: str,
        boot_id: str = "",
        settings: Optional[DdsRuntimeSettings] = None,
        send_depth: bool = True,
        wire_format: str = "raw-rgbd-v1",
        color_codec: str = "jpeg",
        depth_codec: str = "zlib",
        jpeg_quality: int = 85,
        publisher_factory: Callable[..., Any] = DdsRgbdPublisher,
    ) -> None:
        self.topic = str(topic)
        self.wire_format = str(wire_format).strip().lower() or "raw-rgbd-v1"
        if self.wire_format not in {"raw-rgbd-v1", "encoded-rgbd-v1"}:
            raise ValueError(f"unsupported RGB-D wire format: {wire_format!r}")
        self._encoded = self.wire_format == "encoded-rgbd-v1"
        if self._encoded and publisher_factory is DdsRgbdPublisher:
            publisher_factory = DdsEncodedRgbdPublisher
        self._color_codec = str(color_codec).strip().lower()
        self._depth_codec = str(depth_codec).strip().lower()
        self._jpeg_quality = int(jpeg_quality)
        self._send_depth = bool(send_depth)
        self._endpoint_id = str(endpoint_id)
        self._boot_id = str(boot_id)
        self._publisher = publisher_factory(
            self.topic,
            endpoint_id=str(endpoint_id),
            boot_id=str(boot_id),
            settings=settings,
            **({"send_depth": bool(send_depth)} if not self._encoded else {}),
        )
        print(
            f"[sim_camera] DDS publisher topic={self.bound_endpoint} "
            f"depth={bool(send_depth)} qos=sensor_data_keep_last_1"
        )

    @property
    def bound_endpoint(self) -> str:
        return str(self._publisher.bound_endpoint)

    @property
    def published(self) -> int:
        return int(self._publisher.published)

    @property
    def dropped(self) -> int:
        return int(self._publisher.dropped)

    @sampled_traced(
        "camera.sim.publish",
        sample_key="camera.sim.publish",
        every=60,
        kind="producer",
    )
    def publish(self, frame: SimCameraFrame) -> bool:
        if not self._encoded:
            return bool(self._publisher.publish(frame))
        encoded = encoded_frame_from_rgbd(
            frame,
            source_id=self._endpoint_id,
            source_boot_id=self._boot_id,
            color_codec=self._color_codec,
            depth_codec=self._depth_codec if bool(frame.depth_raw is not None and self._send_depth) else "raw",
            jpeg_quality=self._jpeg_quality,
        )
        if not self._send_depth:
            from dataclasses import replace

            encoded = replace(
                encoded,
                has_depth=False,
                depth_codec="",
                depth_encoding="",
                depth_payload=b"",
                depth_scale=0.0,
            )
        return bool(self._publisher.publish(encoded))

    def close(self) -> None:
        self._publisher.close()
