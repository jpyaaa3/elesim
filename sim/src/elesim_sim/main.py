#!/usr/bin/env python3
"""Genesis Sim with direct ROS 2/DDS control and WebRTC video."""

from __future__ import annotations

import argparse
import os
import threading
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
from elesim_sim.model_bundle import resolve_camera_profile_bundle
from elesim_sim.media import (
    MediaWorkerClient,
    MediaWorkerUnavailable,
    VideoStreamSpec,
)
from elesim_sim.observability.tracing import configure_tracing, shutdown_tracing, span
from elesim_sim.simulation.operator_control import SimulationOperatorMailbox
from elesim_sim.telemetry import RuntimeTelemetry
from elesim_sim.turn import load_turn_credential_provider
from elesim_sim.vision.frame_hub import FrameHub
from elesim_sim.vision.webrtc import available as webrtc_available


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
    parser.add_argument("--config", default=str(_ROOT / "config/config.yaml"))
    parser.add_argument("--mode", default=None, help="select a profile from the application YAML")
    parser.add_argument("--runtime-config", default=str(_ROOT / "config/runtime.yaml"))
    parser.add_argument("--model-bundle", default="")
    parser.add_argument("--id", default="")
    args, sim_args = parser.parse_known_args()

    bundle = load_app_config(args.config, mode=args.mode)
    role = load_runtime_role_config(args.runtime_config)
    if role.role != "sim":
        raise ValueError(f"runtime role must be sim, got {role.role!r}")
    endpoint_id = str(args.id).strip() or role.endpoint_id
    turn_provider = load_turn_credential_provider(role.turn)

    # aiortc defaults to software libx264.  Tie the media-worker default to
    # the same Sim backend policy while still allowing an explicit operator
    # override through ELESIM_WEBRTC_ENCODER.
    if "ELESIM_WEBRTC_ENCODER" not in os.environ:
        os.environ["ELESIM_WEBRTC_ENCODER"] = (
            "cpu"
            if "--cpu" in sim_args or not bool(bundle.sim_config.use_gpu)
            else "auto"
        )

    development_rebuild = os.environ.get("ELESIM_SIM_DEV_REBUILD", "").strip() == "1"
    model_bundle = ""
    if not development_rebuild:
        model_bundle = str(
            resolve_camera_profile_bundle(
                bundle.sim_config.camera_profile,
                args.model_bundle or None,
            )
        )

    frame_hub = FrameHub(("rgbd", "observer", "hand_eye_preview"))
    operator_mailbox = SimulationOperatorMailbox(max_pending=128)
    video_specs: dict[str, VideoStreamSpec] = {}
    if bool(bundle.sim_config.sim_observer_camera_enable):
        video_specs["observer"] = VideoStreamSpec(
            name="observer",
            fps=float(bundle.sim_config.sim_observer_camera_max_hz),
            width=int(bundle.sim_config.sim_observer_camera_width),
            height=int(bundle.sim_config.sim_observer_camera_height),
        )
    if bool(bundle.sim_config.sim_camera_enable):
        video_specs["hand_eye_preview"] = VideoStreamSpec(
            name="hand_eye_preview",
            fps=float(bundle.sim_config.sim_camera_max_hz),
            width=int(bundle.sim_config.sim_camera_width),
            height=int(bundle.sim_config.sim_camera_height),
        )

    media: MediaWorkerClient | None = None
    if video_specs and webrtc_available():
        try:
            media = MediaWorkerClient(video_specs)
            media.start()
        except MediaWorkerUnavailable as exc:
            print(f"[sim-media] WebRTC worker unavailable; RGB-D remains enabled: {exc}")
            media = None
    elif video_specs:
        print("[sim-media] WebRTC unavailable; install aiortc and av")

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
    if media is not None:
        for stream in video_specs:
            streams[stream] = _webrtc_descriptor(endpoint_id, stream)

    state = SimulationStateSource(bundle.mapping_config)
    runtime_ready_event = threading.Event()

    def simulation_session_ready() -> tuple[bool, str]:
        if not runtime_ready_event.is_set():
            return False, "scene is still building"
        if media is not None and not media.ready:
            return False, media.failure or "media worker is not ready"
        return True, "ready"

    endpoint = SimEndpoint(
        endpoint_id=endpoint_id,
        state=state,
        streams=streams,
        settings=role.dds,
        operator_mailbox=operator_mailbox,
        webrtc_offer_handler=None if media is None else media.accept_offer,
        webrtc_session_close_handler=None if media is None else media.close_session,
        simulation_session_ready_provider=simulation_session_ready,
        turn_credential_provider=turn_provider,
    )
    telemetry = RuntimeTelemetry(endpoint.publish_telemetry)
    try:
        endpoint.start()
        if endpoint.peer_identity is None:
            raise RuntimeError("sim DDS endpoint did not establish a boot identity")
        from elesim_sim.runtime import run_runtime

        run_runtime(
            config_path=args.config,
            config_mode=args.mode,
            argv=sim_args,
            model_bundle=model_bundle,
            state_source=state,
            feedback_publisher=telemetry,
            frame_hub=frame_hub,
            video_mailboxes=None if media is None else media.mailboxes,
            operator_mailbox=operator_mailbox,
            simulation_status_publisher=endpoint.publish_simulation_status,
            dds_settings=role.dds,
            rgbd_topic=role.streams.get("rgbd_topic", ""),
            rgbd_endpoint_id=endpoint_id,
            rgbd_boot_id=endpoint.peer_identity.boot_id,
            runtime_ready_event=runtime_ready_event,
        )
    finally:
        endpoint.close()
        if media is not None:
            media.close()


def main() -> None:
    configure_tracing("elesim-sim-agent")
    try:
        with span("sim_agent.process.run"):
            _run()
    finally:
        shutdown_tracing()


if __name__ == "__main__":
    main()
