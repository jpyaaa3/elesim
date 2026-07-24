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
    EndpointDescriptor,
    MEDIA_KIND_RGBD,
    MEDIA_SECURITY_DDS,
    MEDIA_SECURITY_NONE,
    MEDIA_TRANSPORT_DDS,
    MediaStreamDescriptor,
    PeerClient,
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
    parser.add_argument("--id", default="")
    parser.add_argument("--device", default="")
    parser.add_argument("--rgbd-topic", default="")
    parser.add_argument("--camera", action=argparse.BooleanOptionalAction, default=None)
    return parser


def _run() -> None:
    args = _parser().parse_args()
    config = load_config(args.config)
    endpoint_id = str(args.id).strip() or config.endpoint_id
    camera_enabled = config.camera.enabled if args.camera is None else bool(args.camera)
    rgbd_topic = str(args.rgbd_topic).strip() or config.camera.topic

    streams = {}
    if camera_enabled:
        streams["rgbd"] = MediaStreamDescriptor(
            transport=MEDIA_TRANSPORT_DDS,
            media_kind=MEDIA_KIND_RGBD,
            endpoint=rgbd_topic,
            security=(
                MEDIA_SECURITY_DDS
                if config.dds.security_profile == "sros2"
                else MEDIA_SECURITY_NONE
            ),
        )

    capabilities = [CAPABILITY_MOTION_ARM]
    if camera_enabled:
        capabilities.append(CAPABILITY_STREAM_RGBD)
    if config.use_go2:
        capabilities.append(CAPABILITY_MOTION_GO2)
    client = PeerClient(
        EndpointDescriptor(
            endpoint_id,
            "robot",
            tuple(capabilities),
            streams=streams,
        ),
        settings=config.dds,
    )

    runtime = RobotRuntime(
        mapping=config.mapping,
        hardware_config=config.arm,
        safety_config=config.safety,
        device=str(args.device).strip() or config.device,
        go2_bridge=create_go2_bridge_if_enabled(
            config.go2,
            use_go2=config.use_go2,
            ros_args=config.dds.apply_environment(),
        ),
    )
    camera = (
        CameraPublisherThread(
            rgbd_topic,
            endpoint_id=endpoint_id,
            boot_id=client.node.identity.boot_id,
            dds_settings=config.dds,
            width=config.camera.width,
            height=config.camera.height,
            fps=config.camera.fps,
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
