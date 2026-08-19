#!/usr/bin/env python3
"""Laptop-side owner of IK, perception and Pick/Gaze workflows."""

from __future__ import annotations

import argparse
import time
from dataclasses import replace
from pathlib import Path

from elesim_protocol import (
    DdsRuntimeSettings,
    MEDIA_KIND_RGBD,
    MEDIA_TRANSPORT_DDS,
    MediaStreamDescriptor,
)

from elesim_pilot.connection import PilotConnection
from elesim_pilot.operator import OperatorDispatcher
from elesim_pilot.runtime import build_control_runtime
from elesim_pilot.config import load_app_config, load_runtime_role_config
from elesim_pilot.pick import ControlClient
from elesim_pilot.observability.tracing import configure_tracing, shutdown_tracing, span
from elesim_pilot.simulation_sync import SimulationWorkflowSync


_ROOT = Path(__file__).resolve().parents[2]


class _ControlFacade:
    def __init__(
        self,
        service,
        connection: PilotConnection,
        *,
        dds_settings: DdsRuntimeSettings,
    ) -> None:
        self._service = service
        self._connection = connection
        self._dds_settings = dds_settings

    def __getattr__(self, name):
        return getattr(self._service, name)

    def select_endpoint(self, target_id: str) -> None:
        self._connection.select_target(target_id)

    def configure_target_stream(self, descriptor: dict) -> None:
        streams = descriptor.get("streams", {})
        raw_stream = streams.get("rgbd") if isinstance(streams, dict) else None
        if not isinstance(raw_stream, dict):
            return
        stream = MediaStreamDescriptor.from_dict(raw_stream)
        if stream.transport != MEDIA_TRANSPORT_DDS or stream.media_kind != MEDIA_KIND_RGBD:
            raise ValueError("target rgbd stream must be a DDS RGB-D topic")
        current = self._service._perception_cfg
        updated = replace(
            current,
            mode="sim",
            provider="local",
            run_local=True,
            sim_camera_topic=stream.endpoint,
            sim_camera_dds_settings=self._dds_settings,
            sim_camera_source_id=str(descriptor.get("endpoint_id", "")),
            sim_camera_source_boot_id=str(descriptor.get("instance_id", "")),
        )
        self._service.update_perception_config(updated)
        print(
            f"[pilot_agent] RGBD source={stream.endpoint} "
            f"target={descriptor.get('endpoint_id', '')} transport=DDS"
        )

    @property
    def available_endpoints(self):
        return list(self._connection.endpoints)

    @property
    def active_endpoint(self) -> str:
        return str(self._connection.active_target)


def _run() -> None:
    parser = argparse.ArgumentParser(description="EleSim control computation agent")
    parser.add_argument("--config", default=str(_ROOT / "config/config.yaml"))
    parser.add_argument("--mode", default=None, help="select a profile from the application YAML")
    parser.add_argument("--runtime-config", default=str(_ROOT / "config/runtime.yaml"))
    parser.add_argument("--id", default="")
    parser.add_argument("--target", default="")
    args = parser.parse_args()
    bundle = load_app_config(args.config, mode=args.mode)
    role = load_runtime_role_config(args.runtime_config)
    if role.role != "pilot":
        raise ValueError(f"runtime role must be pilot, got {role.role!r}")
    pilot_id = str(args.id).strip() or role.endpoint_id
    target_id = str(args.target).strip() or role.active_target
    link = ControlClient(cfg=bundle.mapping_config)
    connection = PilotConnection(
        pilot_id=pilot_id,
        mapping=bundle.mapping_config,
        initial_target=target_id,
        state_sink=link,
        dds_settings=role.dds,
    )
    link.attach_sender(connection.submit)
    runtime = build_control_runtime(args.config, link, mode=args.mode)
    facade = _ControlFacade(
        runtime.service,
        connection,
        dds_settings=role.dds,
    )
    simulation_sync = SimulationWorkflowSync(runtime.service)
    dispatcher = OperatorDispatcher(runtime.state, facade)
    connection.on_target_selected = facade.configure_target_stream
    connection.operator_handler = dispatcher.handle
    connection.simulation_status_handler = simulation_sync.accept
    connection.start()
    print(
        f"[pilot_agent] DDS system={role.dds.system_id} "
        f"domain={role.dds.domain_id} target={target_id}"
    )
    try:
        while True:
            time.sleep(0.25)
    except KeyboardInterrupt:
        pass
    finally:
        simulation_sync.close()
        runtime.service.close()
        connection.close()


def main() -> None:
    configure_tracing("elesim-pilot-agent")
    try:
        with span("pilot_agent.process.run"):
            _run()
    finally:
        shutdown_tracing()


if __name__ == "__main__":
    main()
