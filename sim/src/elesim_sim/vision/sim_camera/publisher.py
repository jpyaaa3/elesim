from __future__ import annotations

from typing import Any, Callable, Optional

from elesim_protocol import DdsRgbdPublisher, DdsRuntimeSettings
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
        publisher_factory: Callable[..., Any] = DdsRgbdPublisher,
    ) -> None:
        self.topic = str(topic)
        self._publisher = publisher_factory(
            self.topic,
            endpoint_id=str(endpoint_id),
            boot_id=str(boot_id),
            settings=settings,
            send_depth=bool(send_depth),
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
        return bool(self._publisher.publish(frame))

    def close(self) -> None:
        self._publisher.close()
