#!/usr/bin/env python3
"""Laptop-side owner of IK, perception and Pick/Gaze workflows."""

from __future__ import annotations

import argparse
import time
from dataclasses import replace
from pathlib import Path

from elesim_controller.connection import ControllerConnection
from elesim_controller.operator import OperatorDispatcher
from elesim_controller.runtime import build_control_runtime
from elesim_controller.config import load_app_config, load_runtime_role_config
from elesim_controller.pick import ControlClient
from elesim_controller.observability.tracing import configure_tracing, shutdown_tracing, span


_ROOT = Path(__file__).resolve().parents[2]


class _ControlFacade:
    def __init__(self, service, connection: ControllerConnection) -> None:
        self._service = service
        self._connection = connection

    def __getattr__(self, name):
        return getattr(self._service, name)

    def send_sim_camera_input(self, command: str, values=()) -> None:
        self._connection.send_camera_input(command, tuple(float(value) for value in values))

    def select_endpoint(self, target_id: str) -> None:
        self._connection.select_target(target_id)

    def configure_target_stream(self, descriptor: dict) -> None:
        streams = descriptor.get("streams", {})
        endpoint = str(streams.get("rgbd", "")) if isinstance(streams, dict) else ""
        if not endpoint:
            return
        current = self._service._perception_cfg
        updated = replace(
            current,
            mode="sim",
            provider="local",
            run_local=True,
            sim_camera_port=endpoint,
        )
        self._service.update_perception_config(updated)
        print(f"[control_agent] RGBD source={endpoint} target={descriptor.get('endpoint_id', '')}")

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
    link = ControlClient(cfg=bundle.mapping_config)
    connection = ControllerConnection(
        server_endpoint=server_endpoint,
        controller_id=controller_id,
        mapping=bundle.mapping_config,
        initial_target=target_id,
        state_sink=link,
    )
    link.attach_sender(connection.submit)
    runtime = build_control_runtime(args.config, link)
    facade = _ControlFacade(runtime.service, connection)
    dispatcher = OperatorDispatcher(runtime.state, facade)
    connection.on_target_selected = facade.configure_target_stream
    connection.operator_handler = dispatcher.handle
    connection.start()
    print(f"[control_agent] server={server_endpoint} target={target_id}")
    try:
        while True:
            time.sleep(0.25)
    except KeyboardInterrupt:
        pass
    finally:
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
