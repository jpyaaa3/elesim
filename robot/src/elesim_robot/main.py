#!/usr/bin/env python3
"""Jetson-side motor, GO2, sensor and RGB-D endpoint process."""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

from elesim_protocol import (
    CAPABILITY_MOTION_ARM,
    CAPABILITY_MOTION_GO2,
    CAPABILITY_STREAM_RGBD,
    CurveClientConfig,
    CurveServerConfig,
    EndpointClient,
    EndpointDescriptor,
    MEDIA_KIND_RGBD,
    MEDIA_SECURITY_CURVE,
    MEDIA_SECURITY_NONE,
    MEDIA_TRANSPORT_ZMQ,
    MediaStreamDescriptor,
)
from elesim_robot.camera.worker import CameraPublisherThread
from elesim_robot.config import load_config
from elesim_robot.go2 import create_go2_bridge_if_enabled
from elesim_robot.runtime import RobotRuntime
from elesim_robot.tracing import configure_tracing, shutdown_tracing, span


PROJECT_CONFIG = Path(__file__).resolve().parents[2] / "config/default.yaml"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Elesim physical robot agent")
    parser.add_argument(
        "--config",
        default=os.environ.get("ELESIM_ROBOT_CONFIG", str(PROJECT_CONFIG)),
    )
    parser.add_argument("--server", default="")
    parser.add_argument("--id", default="")
    parser.add_argument("--device", default="")
    parser.add_argument("--rgbd-bind", default="")
    parser.add_argument("--rgbd-advertise", default="")
    parser.add_argument("--camera", action=argparse.BooleanOptionalAction, default=None)
    return parser


def _run() -> None:
    args = _parser().parse_args()
    config = load_config(args.config)
    server_endpoint = str(args.server).strip() or config.server_endpoint
    endpoint_id = str(args.id).strip() or config.endpoint_id
    camera_enabled = config.camera.enabled if args.camera is None else bool(args.camera)
    rgbd_bind = str(args.rgbd_bind).strip() or config.camera.bind
    advertised = str(args.rgbd_advertise).strip() or config.camera.advertise or rgbd_bind
    router_curve = None
    if bool(config.security.router_client_secret_file) != bool(
        config.security.router_server_public_file
    ):
        raise ValueError("router CURVE client and server certificate paths must be configured together")
    if config.security.router_client_secret_file:
        router_curve = CurveClientConfig.from_files(
            client_secret_file=config.security.router_client_secret_file,
            server_public_file=config.security.router_server_public_file,
        )
    if bool(config.security.media_server_secret_file) != bool(
        config.security.media_client_public_keys_dir
    ):
        raise ValueError(
            "media CURVE server certificate and client-key directory must be configured together"
        )
    media_curve = (
        None
        if not config.security.media_server_secret_file
        else CurveServerConfig.from_file(config.security.media_server_secret_file)
    )

    streams = {}
    if camera_enabled:
        streams["rgbd"] = MediaStreamDescriptor(
            transport=MEDIA_TRANSPORT_ZMQ,
            media_kind=MEDIA_KIND_RGBD,
            endpoint=advertised,
            security=(MEDIA_SECURITY_CURVE if media_curve is not None else MEDIA_SECURITY_NONE),
            curve_server_key=(
                "" if media_curve is None else media_curve.public_key.decode("ascii")
            ),
        )

    capabilities = [CAPABILITY_MOTION_ARM]
    if camera_enabled:
        capabilities.append(CAPABILITY_STREAM_RGBD)
    if config.use_go2:
        capabilities.append(CAPABILITY_MOTION_GO2)
    client = EndpointClient(
        server_endpoint,
        EndpointDescriptor(
            endpoint_id,
            "robot",
            tuple(capabilities),
            streams=streams,
        ),
        curve=router_curve,
        allow_insecure_remote=config.security.allow_insecure_remote,
    )

    runtime = RobotRuntime(
        mapping=config.mapping,
        hardware_config=config.arm,
        safety_config=config.safety,
        device=str(args.device).strip() or config.device,
        go2_bridge=create_go2_bridge_if_enabled(config.go2, use_go2=config.use_go2),
    )
    camera = (
        CameraPublisherThread(
            rgbd_bind,
            width=config.camera.width,
            height=config.camera.height,
            fps=config.camera.fps,
            curve=media_curve,
            curve_client_keys_dir=config.security.media_client_public_keys_dir,
            allow_insecure_remote=config.security.allow_insecure_remote,
        )
        if camera_enabled
        else None
    )

    runtime.open()
    if camera is not None:
        camera.start()
    last_state = 0.0
    try:
        while True:
            client.heartbeat()
            messages = tuple(client.receive(timeout_ms=20))
            if messages:
                runtime.mark_router_alive()
            for message in messages:
                if message.message_type == "lease_granted":
                    runtime.grant_lease(
                        str((message.payload or {}).get("controller_id", "")),
                        message.lease_id,
                    )
                elif message.message_type == "lease_revoked":
                    runtime.revoke_lease()
                elif message.message_type == "motion_command":
                    ok, reason = runtime.apply(message)
                    client.send(
                        "ack",
                        target_id=message.source_id,
                        payload={"reply_to": message.message_id, "ok": ok, "reason": reason},
                        lease_id=message.lease_id,
                    )
            runtime.tick()
            now = time.monotonic()
            if runtime.controller_id and now - last_state >= config.safety.telemetry_period_s:
                last_state = now
                state = runtime.state()
                if camera is not None:
                    state["camera"] = camera.status()
                client.send(
                    "telemetry",
                    target_id=runtime.controller_id,
                    payload=state,
                    lease_id=runtime.active_lease,
                )
    except KeyboardInterrupt:
        pass
    finally:
        if camera is not None:
            camera.stop()
        runtime.close()
        client.close()


def main() -> None:
    configure_tracing("elesim-robot-agent")
    try:
        with span("robot_agent.process.run"):
            _run()
    finally:
        shutdown_tracing()


if __name__ == "__main__":
    main()
