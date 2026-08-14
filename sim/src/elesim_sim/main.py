#!/usr/bin/env python3
"""Genesis Sim with direct ROS 2/DDS control and WebRTC video."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from elesim_protocol import (
    MEDIA_KIND_RGB,
    MEDIA_KIND_RGBD,
    MEDIA_SECURITY_DDS,
    MEDIA_SECURITY_DTLS_SRTP,
    MEDIA_SECURITY_NONE,
    MEDIA_TRANSPORT_DDS,
    MEDIA_TRANSPORT_WEBRTC,
    MediaStreamDescriptor,
)
from elesim_sim.config import load_app_config, load_runtime_role_config
from elesim_sim.control_state import SimulationStateSource
from elesim_sim.endpoint import SimEndpoint
from elesim_sim.model_bundle import resolve_model_bundle
from elesim_sim.observability.tracing import configure_tracing, shutdown_tracing, span
from elesim_sim.simulation.operator_control import SimulationOperatorMailbox
from elesim_sim.telemetry import RuntimeTelemetry
from elesim_sim.turn import load_turn_credential_provider
from elesim_sim.vision.frame_hub import FrameHub
from elesim_sim.vision.webrtc import NamedWebRtcVideoSender, available as webrtc_available


_ROOT = Path(__file__).resolve().parents[2]


def _rgbd_descriptor(
    *,
    topic: str,
    secure: bool,
) -> MediaStreamDescriptor:
    return MediaStreamDescriptor(
        transport=MEDIA_TRANSPORT_DDS,
        media_kind=MEDIA_KIND_RGBD,
        endpoint=str(topic),
        security=MEDIA_SECURITY_DDS if secure else MEDIA_SECURITY_NONE,
    )


def _webrtc_descriptor(endpoint_id: str, stream: str) -> MediaStreamDescriptor:
    return MediaStreamDescriptor(
        transport=MEDIA_TRANSPORT_WEBRTC,
        media_kind=MEDIA_KIND_RGB,
        endpoint=f"webrtc://{endpoint_id}/{stream}",
        security=MEDIA_SECURITY_DTLS_SRTP,
    )


def _run() -> None:
    parser = argparse.ArgumentParser(description="EleSim distributed Genesis agent")
    parser.add_argument("--config", default=str(_ROOT / "config/default.yaml"))
    parser.add_argument("--runtime-config", default=str(_ROOT / "config/runtime.yaml"))
    parser.add_argument("--model-bundle", default="")
    parser.add_argument("--id", default="")
    args, sim_args = parser.parse_known_args()

    bundle = load_app_config(args.config)
    role = load_runtime_role_config(args.runtime_config)
    if role.role != "sim":
        raise ValueError(f"runtime role must be sim, got {role.role!r}")
    endpoint_id = str(args.id).strip() or role.endpoint_id
    turn_provider = load_turn_credential_provider(role.turn)

    development_rebuild = os.environ.get("ELESIM_SIM_DEV_REBUILD", "").strip() == "1"
    model_bundle = ""
    if not development_rebuild:
        model_bundle = str(resolve_model_bundle(args.model_bundle or None))

    frame_hub = FrameHub(("rgbd", "observer", "hand_eye_preview"))
    operator_mailbox = SimulationOperatorMailbox(max_pending=128)
    providers = {}
    rates = {}
    frame_sizes = {}
    if bool(bundle.sim_config.sim_observer_camera_enable):
        providers["observer"] = lambda: frame_hub.latest_bgr("observer")
        rates["observer"] = float(bundle.sim_config.sim_observer_camera_max_hz)
        frame_sizes["observer"] = (
            int(bundle.sim_config.sim_observer_camera_width),
            int(bundle.sim_config.sim_observer_camera_height),
        )
    if bool(bundle.sim_config.sim_camera_enable):
        providers["hand_eye_preview"] = lambda: frame_hub.latest_bgr("hand_eye_preview")
        rates["hand_eye_preview"] = float(bundle.sim_config.sim_camera_max_hz)
        frame_sizes["hand_eye_preview"] = (
            int(bundle.sim_config.sim_camera_width),
            int(bundle.sim_config.sim_camera_height),
        )
    webrtc = (
        NamedWebRtcVideoSender(providers, fps=rates, frame_sizes=frame_sizes)
        if providers and webrtc_available()
        else None
    )
    if webrtc is None:
        print("[sim_agent] WebRTC unavailable; install aiortc and av")

    streams: dict[str, MediaStreamDescriptor] = {}
    if bool(bundle.sim_config.sim_camera_enable):
        rgbd_topic = str(role.streams.get("rgbd_topic", "")).strip()
        if not rgbd_topic:
            raise ValueError(
                "runtime.streams.rgbd_topic is required when the hand-eye camera is enabled"
            )
        streams["rgbd"] = _rgbd_descriptor(
            topic=rgbd_topic,
            secure=role.dds.security_profile == "sros2",
        )
    if webrtc is not None:
        for stream in providers:
            streams[stream] = _webrtc_descriptor(endpoint_id, stream)

    state = SimulationStateSource(bundle.mapping_config)
    endpoint = SimEndpoint(
        endpoint_id=endpoint_id,
        state=state,
        streams=streams,
        settings=role.dds,
        operator_mailbox=operator_mailbox,
        webrtc_offer_handler=None if webrtc is None else webrtc.accept_offer,
        webrtc_session_close_handler=None if webrtc is None else webrtc.close_session,
        turn_credential_provider=turn_provider,
    )
    telemetry = RuntimeTelemetry(endpoint.publish_telemetry)
    endpoint.start()
    if endpoint.peer_identity is None:
        raise RuntimeError("sim DDS endpoint did not establish a boot identity")
    try:
        from elesim_sim.runtime import run_runtime

        run_runtime(
            config_path=args.config,
            argv=sim_args,
            model_bundle=model_bundle,
            state_source=state,
            feedback_publisher=telemetry,
            frame_hub=frame_hub,
            operator_mailbox=operator_mailbox,
            simulation_status_publisher=endpoint.publish_simulation_status,
            dds_settings=role.dds,
            rgbd_topic=role.streams.get("rgbd_topic", ""),
            rgbd_endpoint_id=endpoint_id,
            rgbd_boot_id=endpoint.peer_identity.boot_id,
        )
    finally:
        endpoint.close()
        if webrtc is not None:
            webrtc.close()


def main() -> None:
    configure_tracing("elesim-sim-agent")
    try:
        with span("sim_agent.process.run"):
            _run()
    finally:
        shutdown_tracing()


if __name__ == "__main__":
    main()
