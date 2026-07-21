#!/usr/bin/env python3
"""Genesis simulation endpoint with protocol-v4 control and direct media."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from elesim_protocol import (
    CurveClientConfig,
    CurveServerConfig,
    MEDIA_KIND_RGB,
    MEDIA_KIND_RGBD,
    MEDIA_SECURITY_CURVE,
    MEDIA_SECURITY_DTLS_SRTP,
    MEDIA_SECURITY_NONE,
    MEDIA_TRANSPORT_WEBRTC,
    MEDIA_TRANSPORT_ZMQ,
    MediaStreamDescriptor,
)
from elesim_simulator.config import load_app_config, load_runtime_role_config
from elesim_simulator.control_state import SimulationStateSource
from elesim_simulator.endpoint import SimulatorEndpoint
from elesim_simulator.model_bundle import resolve_model_bundle
from elesim_simulator.observability.tracing import configure_tracing, shutdown_tracing, span
from elesim_simulator.simulation.operator_control import SimulationOperatorMailbox
from elesim_simulator.telemetry import RuntimeTelemetry
from elesim_simulator.vision.frame_hub import FrameHub
from elesim_simulator.vision.webrtc import NamedWebRtcVideoSender, available as webrtc_available


_ROOT = Path(__file__).resolve().parents[2]


def _router_curve(role) -> CurveClientConfig | None:
    client = str(role.router_client_secret_file).strip()
    server = str(role.router_server_public_file).strip()
    if bool(client) != bool(server):
        raise ValueError("router CURVE client and server certificate paths must be configured together")
    if not client:
        return None
    return CurveClientConfig.from_files(
        client_secret_file=client,
        server_public_file=server,
    )


def _media_curve(role) -> CurveServerConfig | None:
    secret = str(role.media_server_secret_file).strip()
    authorized = str(role.media_client_public_keys_dir).strip()
    if bool(secret) != bool(authorized):
        raise ValueError(
            "media CURVE server certificate and client-key directory must be configured together"
        )
    return None if not secret else CurveServerConfig.from_file(secret)


def _media_descriptor(
    *,
    endpoint: str,
    media_kind: str,
    curve: CurveServerConfig | None,
) -> MediaStreamDescriptor:
    return MediaStreamDescriptor(
        transport=MEDIA_TRANSPORT_ZMQ,
        media_kind=media_kind,
        endpoint=str(endpoint),
        security=MEDIA_SECURITY_CURVE if curve is not None else MEDIA_SECURITY_NONE,
        curve_server_key="" if curve is None else curve.public_key.decode("ascii"),
    )


def _webrtc_descriptor(endpoint_id: str, stream: str) -> MediaStreamDescriptor:
    return MediaStreamDescriptor(
        transport=MEDIA_TRANSPORT_WEBRTC,
        media_kind=MEDIA_KIND_RGB,
        endpoint=f"webrtc://{endpoint_id}/{stream}",
        security=MEDIA_SECURITY_DTLS_SRTP,
    )


def _run() -> None:
    parser = argparse.ArgumentParser(description="Elesim distributed Genesis agent")
    parser.add_argument("--config", default=str(_ROOT / "config/default.yaml"))
    parser.add_argument("--runtime-config", default=str(_ROOT / "config/runtime.yaml"))
    parser.add_argument("--model-bundle", default="")
    parser.add_argument("--server", default="")
    parser.add_argument("--id", default="")
    args, sim_args = parser.parse_known_args()

    bundle = load_app_config(args.config)
    role = load_runtime_role_config(args.runtime_config)
    if role.role != "simulator":
        raise ValueError(f"runtime role must be simulator, got {role.role!r}")
    server_endpoint = str(args.server).strip() or role.server_endpoint
    endpoint_id = str(args.id).strip() or role.endpoint_id
    router_curve = _router_curve(role)
    media_curve = _media_curve(role)
    media_client_keys = str(role.media_client_public_keys_dir).strip()

    development_rebuild = os.environ.get("ELESIM_SIM_DEV_REBUILD", "").strip() == "1"
    model_bundle = ""
    if not development_rebuild:
        model_bundle = str(resolve_model_bundle(args.model_bundle or None))

    frame_hub = FrameHub(("rgbd", "observer", "hand_eye_preview"))
    operator_mailbox = SimulationOperatorMailbox(max_pending=128)
    providers = {}
    rates = {}
    if bool(bundle.sim_config.sim_observer_camera_enable):
        providers["observer"] = lambda: frame_hub.latest_bgr("observer")
        rates["observer"] = float(bundle.sim_config.sim_observer_camera_max_hz)
    if bool(bundle.sim_config.sim_camera_enable):
        providers["hand_eye_preview"] = lambda: frame_hub.latest_bgr("hand_eye_preview")
        rates["hand_eye_preview"] = float(bundle.sim_config.sim_camera_max_hz)
    webrtc = (
        NamedWebRtcVideoSender(providers, fps=rates)
        if providers and webrtc_available()
        else None
    )
    if webrtc is None:
        print("[sim_agent] WebRTC unavailable; install aiortc and av")

    streams: dict[str, MediaStreamDescriptor] = {}
    if bool(bundle.sim_config.sim_camera_enable):
        rgbd_endpoint = role.streams.get("rgbd_advertise", "") or str(bundle.sim_config.sim_camera_port)
        streams["rgbd"] = _media_descriptor(
            endpoint=rgbd_endpoint,
            media_kind=MEDIA_KIND_RGBD,
            curve=media_curve,
        )
    if bool(bundle.sim_config.sim_observer_camera_enable):
        observer_endpoint = role.streams.get("observer_advertise", "") or str(
            bundle.sim_config.sim_observer_camera_port
        )
        streams["observer_rgb"] = _media_descriptor(
            endpoint=observer_endpoint,
            media_kind=MEDIA_KIND_RGB,
            curve=media_curve,
        )
    if webrtc is not None:
        for stream in providers:
            streams[stream] = _webrtc_descriptor(endpoint_id, stream)

    state = SimulationStateSource(bundle.mapping_config)
    endpoint = SimulatorEndpoint(
        server_endpoint=server_endpoint,
        endpoint_id=endpoint_id,
        state=state,
        streams=streams,
        operator_mailbox=operator_mailbox,
        webrtc_offer_handler=None if webrtc is None else webrtc.accept_offer,
        webrtc_session_close_handler=None if webrtc is None else webrtc.close_session,
        curve=router_curve,
        allow_insecure_remote=role.allow_insecure_remote,
    )
    telemetry = RuntimeTelemetry(endpoint.publish_telemetry)
    endpoint.start()
    try:
        from elesim_simulator.runtime import run_runtime

        run_runtime(
            config_path=args.config,
            argv=sim_args,
            model_bundle=model_bundle,
            state_source=state,
            feedback_publisher=telemetry,
            frame_hub=frame_hub,
            operator_mailbox=operator_mailbox,
            simulation_status_publisher=endpoint.publish_simulation_status,
            media_curve=media_curve,
            media_curve_client_keys_dir=media_client_keys,
            media_bind_endpoints={
                "rgbd": role.streams.get("rgbd_bind", ""),
                "observer": role.streams.get("observer_bind", ""),
            },
            allow_insecure_remote_media=role.allow_insecure_remote,
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
