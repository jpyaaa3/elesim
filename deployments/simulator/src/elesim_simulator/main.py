#!/usr/bin/env python3
"""Genesis simulation endpoint with direct protocol-v3 transport."""

from __future__ import annotations

import argparse
import os
import threading
from pathlib import Path

from elesim_simulator.config import load_app_config, load_runtime_role_config
from elesim_simulator.control_state import SimulationStateSource
from elesim_simulator.endpoint import SimulatorEndpoint
from elesim_simulator.model_bundle import resolve_model_bundle
from elesim_simulator.observability.tracing import configure_tracing, shutdown_tracing, span
from elesim_simulator.telemetry import RuntimeTelemetry
from elesim_simulator.vision.sim_camera.subscriber import SimCameraSubscriber
from elesim_simulator.vision.webrtc import WebRtcVideoSender, available as webrtc_available


_ROOT = Path(__file__).resolve().parents[2]


class _RenderedFrameSource:
    def __init__(self, endpoint: str, *, use_jpeg: bool) -> None:
        self.subscriber = SimCameraSubscriber(endpoint, use_jpeg=use_jpeg)
        self.latest = None
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name="sim-rendered-frame", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def _run(self) -> None:
        while not self.stop_event.is_set():
            frame = self.subscriber.recv_latest(timeout_ms=250)
            if frame is not None:
                self.latest = frame.color_bgr

    def get(self):
        return self.latest

    def close(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        self.subscriber.close()


def _camera_input(command: str, values: object) -> None:
    from elesim_simulator.vision.sim_camera.remote_control import enqueue

    enqueue(command, values)


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
    development_rebuild = os.environ.get("ELESIM_SIM_DEV_REBUILD", "").strip() == "1"
    model_bundle = ""
    if not development_rebuild:
        model_bundle = str(resolve_model_bundle(args.model_bundle or None))

    rendered_source = _RenderedFrameSource(
        str(bundle.sim_config.sim_side_camera_port),
        use_jpeg=bool(bundle.sim_config.sim_side_camera_jpeg),
    )
    rendered_source.start()
    webrtc = (
        WebRtcVideoSender(
            rendered_source.get,
            fps=float(bundle.sim_config.sim_side_camera_max_hz),
        )
        if webrtc_available()
        else None
    )
    if webrtc is None:
        print("[sim_agent] WebRTC unavailable; install aiortc and av")

    state = SimulationStateSource(bundle.mapping_config)
    endpoint = SimulatorEndpoint(
        server_endpoint=server_endpoint,
        endpoint_id=endpoint_id,
        state=state,
        streams={
            "rgbd": role.streams.get("rgbd_advertise", "") or str(bundle.sim_config.sim_camera_port),
            "rendered_view": role.streams.get("rendered_view", "webrtc"),
        },
        camera_input_handler=_camera_input,
        webrtc_offer_handler=None if webrtc is None else webrtc.accept_offer,
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
        )
    finally:
        endpoint.close()
        if webrtc is not None:
            webrtc.close()
        rendered_source.close()


def main() -> None:
    configure_tracing("elesim-sim-agent")
    try:
        with span("sim_agent.process.run"):
            _run()
    finally:
        shutdown_tracing()


if __name__ == "__main__":
    main()
