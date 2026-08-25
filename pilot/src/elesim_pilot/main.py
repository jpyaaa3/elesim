#!/usr/bin/env python3
"""Laptop-side owner of IK, perception and Pick/Gaze workflows."""

from __future__ import annotations

import argparse
import time
from dataclasses import replace
from pathlib import Path

from elesim_protocol import (
    CAPABILITY_SIM_MOCK_HUG,
    DdsRuntimeSettings,
    MEDIA_KIND_RGBD,
    MEDIA_TRANSPORT_DDS,
    MediaStreamDescriptor,
    SimQ,
)

from elesim_pilot.connection import PilotConnection
from elesim_pilot.operator import OperatorDispatcher
from elesim_pilot.runtime import build_control_runtime
from elesim_pilot.config import load_app_config, load_runtime_role_config
from elesim_pilot.pick import ControlClient
from elesim_pilot.pick.mock_hug import MockHugCoordinator
from elesim_pilot.observability.tracing import configure_tracing, shutdown_tracing, span
from elesim_pilot.simulation_sync import SimulationWorkflowSync
from elesim_pilot.vision.rgbd import DdsRgbdRelay


_ROOT = Path(__file__).resolve().parents[2]


class _ControlFacade:
    def __init__(
        self,
        service,
        connection: PilotConnection,
        *,
        dds_settings: DdsRuntimeSettings,
        rgbd_broker_topic: str = "",
    ) -> None:
        self._service = service
        self._connection = connection
        self._dds_settings = dds_settings
        self._rgbd_broker_topic = str(rgbd_broker_topic).strip()
        self._rgbd_relay: DdsRgbdRelay | None = None
        self._mock_hug: MockHugCoordinator | None = None

    def attach_mock_hug(self, coordinator: MockHugCoordinator) -> None:
        self._mock_hug = coordinator

    def compute_mock_hug(self) -> dict[str, object]:
        if self._mock_hug is None:
            raise RuntimeError("mock hug coordinator is not initialized")
        self._require_sim_target()
        return self._mock_hug.compute()

    def execute_mock_hug(self, solution_id: str) -> dict[str, object]:
        if self._mock_hug is None:
            raise RuntimeError("mock hug coordinator is not initialized")
        self._require_sim_target()
        return self._mock_hug.execute(solution_id)

    def _require_sim_target(self) -> None:
        self.mock_hug_execution_context()

    def mock_hug_execution_context(self) -> tuple[str, str, str]:
        target = str(self._connection.active_target)
        descriptor = next(
            (
                value
                for value in self._connection.endpoints
                if str(value.get("endpoint_id", "")) == target
            ),
            None,
        )
        capabilities = () if descriptor is None else tuple(descriptor.get("capabilities", ()))
        boot_id = "" if descriptor is None else str(descriptor.get("instance_id", ""))
        lease_id = str(self._connection.lease_id)
        if (
            not target
            or descriptor is None
            or str(descriptor.get("role", "")) != "sim"
            or CAPABILITY_SIM_MOCK_HUG not in capabilities
            or not boot_id
            or not lease_id
        ):
            raise RuntimeError("mock hug requires a capable exact Sim boot and motion lease")
        return target, boot_id, lease_id

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
        source_topic = str(stream.endpoint)
        broker_topic = self._rgbd_broker_topic or source_topic
        source_format = str(stream.format or "raw-rgbd-v1")
        if self._rgbd_relay is not None:
            self._rgbd_relay.close()
            self._rgbd_relay = None
        consumer_format = source_format
        if source_topic != broker_topic:
            self._rgbd_relay = DdsRgbdRelay(
                source_topic,
                broker_topic,
                source_format=source_format,
                endpoint_id=self._connection.pilot_id,
                settings=self._dds_settings,
                expected_source_id=str(descriptor.get("endpoint_id", "")),
                expected_boot_id=str(descriptor.get("instance_id", "")),
            )
            self._rgbd_relay.start()
            consumer_format = "encoded-rgbd-v1"
            print(
                f"[pilot_agent] RGBD relay source={source_topic} broker={broker_topic} "
                f"format={source_format}"
            )
        current = self._service._perception_cfg
        updated = replace(
            current,
            mode="sim",
            provider="local",
            run_local=True,
            sim_camera_topic=broker_topic,
            sim_camera_dds_settings=self._dds_settings,
            sim_camera_source_id=str(descriptor.get("endpoint_id", "")),
            sim_camera_source_boot_id=str(descriptor.get("instance_id", "")),
            sim_camera_wire_format=consumer_format,
        )
        self._service.update_perception_config(updated)
        print(
            f"[pilot_agent] RGBD source={stream.endpoint} "
            f"target={descriptor.get('endpoint_id', '')} transport=DDS"
        )

    def close(self) -> None:
        if self._rgbd_relay is not None:
            self._rgbd_relay.close()
            self._rgbd_relay = None

    def target_lost(self) -> None:
        if self._rgbd_relay is not None:
            self._rgbd_relay.close()
            self._rgbd_relay = None

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
        rgbd_topic=str(
            ((role.rgbd or {}).get("wire") or {}).get("topic", "")
        ),
    )
    link.attach_sender(connection.submit)
    runtime = build_control_runtime(args.config, link, mode=args.mode)
    facade = _ControlFacade(
        runtime.service,
        connection,
        dds_settings=role.dds,
        rgbd_broker_topic=str(
            ((role.rgbd or {}).get("wire") or {}).get("topic", "")
        ),
    )
    simulation_sync = SimulationWorkflowSync(runtime.service)
    mock_hug = MockHugCoordinator(
        link,
        lambda: simulation_sync.latest,
        lambda: SimQ(
            float(runtime.state.linear),
            float(runtime.state.roll),
            float(runtime.state.theta1),
            float(runtime.state.theta2),
        ),
        mapping=bundle.mapping_config,
        execution_context=facade.mock_hug_execution_context,
    )
    facade.attach_mock_hug(mock_hug)
    simulation_sync.add_cancel_callback(mock_hug.cancel)
    def _target_changed(target: str) -> None:
        simulation_sync.clear("simulation target changed")
        if not str(target).strip():
            facade.target_lost()

    connection.on_target_changed = _target_changed
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
        mock_hug.close()
        simulation_sync.close()
        runtime.service.close()
        facade.close()
        connection.close()


def main() -> None:
    configure_tracing("elesim-pilot-agent")
    try:
        with span(
            "elesim_pilot.main.main",
            attributes={
                "code.function.name": "elesim_pilot.main.main",
                "elesim.flow.id": "pilot.lifecycle",
            },
        ):
            _run()
    finally:
        shutdown_tracing()


if __name__ == "__main__":
    main()
