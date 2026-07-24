"""Controller composition helper for experiment and debug tools.

Tools that execute Controller-owned workflows run a real Controller DDS
participant in-process.  They do not create a second transport shim or connect
to a removed TCP endpoint.
"""

from __future__ import annotations

import time
from dataclasses import replace
from pathlib import Path
from typing import Any

from elesim_protocol import (
    MEDIA_KIND_RGBD,
    MEDIA_TRANSPORT_DDS,
    MediaStreamDescriptor,
)

from elesim_controller.config import load_app_config, load_runtime_role_config
from elesim_controller.connection import ControllerConnection
from elesim_controller.pick import ControlClient
from elesim_controller.runtime import build_control_runtime
from elesim_controller.simulation_sync import SimulationWorkflowSync


class ToolControlService:
    """Delegate workflow calls while owning the in-process DDS lifecycle."""

    def __init__(
        self,
        *,
        config_path: str,
        runtime_config_path: str | Path,
        target_id: str = "",
    ) -> None:
        role = load_runtime_role_config(runtime_config_path)
        if role.role != "controller":
            raise ValueError(
                f"runtime role must be controller, got {role.role!r}"
            )
        bundle = load_app_config(config_path)
        link = ControlClient(cfg=bundle.mapping_config)
        runtime = build_control_runtime(config_path, link)
        connection = ControllerConnection(
            controller_id=role.endpoint_id,
            initial_target=str(target_id).strip() or role.active_target,
            mapping=runtime.bundle.mapping_config,
            state_sink=link,
            dds_settings=role.dds,
        )
        link.attach_sender(connection.submit)

        self._runtime = runtime
        self._link = link
        self._connection = connection
        self._simulation_sync = SimulationWorkflowSync(runtime.service)
        self._closed = False

        def configure_target_stream(descriptor: dict[str, Any]) -> None:
            streams = descriptor.get("streams", {})
            raw_stream = (
                streams.get("rgbd") if isinstance(streams, dict) else None
            )
            if not isinstance(raw_stream, dict):
                return
            stream = MediaStreamDescriptor.from_dict(raw_stream)
            if (
                stream.transport != MEDIA_TRANSPORT_DDS
                or stream.media_kind != MEDIA_KIND_RGBD
            ):
                raise ValueError("target rgbd stream must be a DDS RGB-D topic")
            current = runtime.service._perception_cfg
            runtime.service.update_perception_config(
                replace(
                    current,
                    mode="sim",
                    provider="local",
                    run_local=True,
                    sim_camera_topic=stream.endpoint,
                    sim_camera_dds_settings=role.dds,
                    sim_camera_source_id=str(
                        descriptor.get("endpoint_id", "")
                    ),
                    sim_camera_source_boot_id=str(
                        descriptor.get("instance_id", "")
                    ),
                )
            )

        connection.on_target_selected = configure_target_stream
        connection.simulation_status_handler = self._simulation_sync.accept
        connection.start()

    @property
    def state(self) -> Any:
        return self._runtime.state

    @property
    def bundle(self) -> Any:
        return self._runtime.bundle

    @property
    def client(self) -> ControlClient:
        return self._link

    @property
    def _gaze_cfg(self) -> Any:
        return self._runtime.service._gaze_cfg

    @property
    def _last_pick_profile(self) -> Any:
        return self._runtime.service._last_pick_profile

    def wait_until_connected(self, timeout_s: float = 10.0) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            host = self._link.refresh_state()
            if host.connected:
                return True
            time.sleep(0.05)
        return bool(self._link.refresh_state().connected)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._simulation_sync.close()
        self._runtime.service.close()
        self._connection.close()

    def __getattr__(self, name: str) -> Any:
        return getattr(self._runtime.service, name)


def start_tool_controller(
    config_path: str,
    *,
    runtime_config_path: str | Path,
    target_id: str = "",
    connect_timeout_s: float = 10.0,
) -> ToolControlService:
    service = ToolControlService(
        config_path=str(config_path),
        runtime_config_path=runtime_config_path,
        target_id=target_id,
    )
    if service.wait_until_connected(connect_timeout_s):
        return service
    service.close()
    raise RuntimeError(
        "target not reachable - start the target on the same DDS graph first"
    )


__all__ = ["ToolControlService", "start_tool_controller"]
