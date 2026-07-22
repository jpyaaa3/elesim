#!/usr/bin/env python3
"""Laptop-side owner of IK, perception and Pick/Gaze workflows."""

from __future__ import annotations

import argparse
import time
from dataclasses import replace
from pathlib import Path

from elesim_protocol import (
    CurveClientConfig,
    MEDIA_KIND_RGBD,
    MEDIA_SECURITY_CURVE,
    MEDIA_TRANSPORT_ZMQ,
    MediaStreamDescriptor,
)

from elesim_controller.connection import ControllerConnection
from elesim_controller.operator import OperatorDispatcher
from elesim_controller.runtime import build_control_runtime
from elesim_controller.config import load_app_config, load_runtime_role_config
from elesim_controller.pick import ControlClient
from elesim_controller.observability.tracing import configure_tracing, shutdown_tracing, span
from elesim_controller.simulation_sync import SimulationWorkflowSync


_ROOT = Path(__file__).resolve().parents[2]


class _ControlFacade:
    def __init__(
        self,
        service,
        connection: ControllerConnection,
        *,
        media_client_secret_file: str = "",
        allow_insecure_remote: bool = False,
    ) -> None:
        self._service = service
        self._connection = connection
        self._media_client_secret_file = str(media_client_secret_file)
        self._allow_insecure_remote = bool(allow_insecure_remote)

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
        if stream.transport != MEDIA_TRANSPORT_ZMQ or stream.media_kind != MEDIA_KIND_RGBD:
            raise ValueError("target rgbd stream must be ZMQ RGB-D media")
        if stream.security == MEDIA_SECURITY_CURVE and not self._media_client_secret_file:
            raise ValueError("target rgbd stream requires a media CURVE client certificate")
        current = self._service._perception_cfg
        updated = replace(
            current,
            mode="sim",
            provider="local",
            run_local=True,
            sim_camera_port=stream.endpoint,
            sim_camera_curve_client_secret_file=(
                self._media_client_secret_file
                if stream.security == MEDIA_SECURITY_CURVE
                else ""
            ),
            sim_camera_curve_server_key=stream.curve_server_key,
            sim_camera_allow_insecure_remote=self._allow_insecure_remote,
        )
        self._service.update_perception_config(updated)
        print(
            f"[control_agent] RGBD source={stream.endpoint} "
            f"target={descriptor.get('endpoint_id', '')} security={stream.security}"
        )

    @property
    def available_endpoints(self):
        return list(self._connection.endpoints)

    @property
    def active_endpoint(self) -> str:
        return str(self._connection.active_target)


def _run() -> None:
    parser = argparse.ArgumentParser(description="Elesim control computation agent")
    parser.add_argument("--config", default=str(_ROOT / "config/default.yaml"))
    parser.add_argument("--runtime-config", default=str(_ROOT / "config/runtime.yaml"))
    parser.add_argument("--server", default="")
    parser.add_argument("--id", default="")
    parser.add_argument("--target", default="")
    args = parser.parse_args()
    bundle = load_app_config(args.config)
    role = load_runtime_role_config(args.runtime_config)
    if role.role != "controller":
        raise ValueError(f"runtime role must be controller, got {role.role!r}")
    server_endpoint = str(args.server).strip() or role.server_endpoint
    controller_id = str(args.id).strip() or role.endpoint_id
    target_id = str(args.target).strip() or role.active_target
    router_curve = None
    if bool(role.router_client_secret_file) != bool(role.router_server_public_file):
        raise ValueError("router CURVE client and server certificate paths must be configured together")
    if role.router_client_secret_file:
        router_curve = CurveClientConfig.from_files(
            client_secret_file=role.router_client_secret_file,
            server_public_file=role.router_server_public_file,
        )
    link = ControlClient(cfg=bundle.mapping_config)
    connection = ControllerConnection(
        server_endpoint=server_endpoint,
        controller_id=controller_id,
        mapping=bundle.mapping_config,
        initial_target=target_id,
        state_sink=link,
        curve=router_curve,
        allow_insecure_remote=role.allow_insecure_remote,
    )
    link.attach_sender(connection.submit)
    runtime = build_control_runtime(args.config, link)
    facade = _ControlFacade(
        runtime.service,
        connection,
        media_client_secret_file=(
            role.media_client_secret_file or role.router_client_secret_file
        ),
        allow_insecure_remote=role.allow_insecure_remote,
    )
    simulation_sync = SimulationWorkflowSync(runtime.service)
    dispatcher = OperatorDispatcher(runtime.state, facade)
    connection.on_target_selected = facade.configure_target_stream
    connection.operator_handler = dispatcher.handle
    connection.simulation_status_handler = simulation_sync.accept
    connection.start()
    print(f"[control_agent] server={server_endpoint} target={target_id}")
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
    configure_tracing("elesim-control-agent")
    try:
        with span("control_agent.process.run"):
            _run()
    finally:
        shutdown_tracing()


if __name__ == "__main__":
    main()
