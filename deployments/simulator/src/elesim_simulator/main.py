#!/usr/bin/env python3
"""Genesis simulation endpoint with protocol-v3 transport."""

from __future__ import annotations

import argparse
import os
import sys
import tempfile
import threading
import time
from pathlib import Path

import yaml

from elesim_simulator.bridge import SimProtocolBridge
from elesim_simulator.config import load_app_config, load_runtime_role_config
from elesim_simulator.observability.tracing import configure_tracing, shutdown_tracing, span
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


def _runtime_config(source: str, state_endpoint: str, feedback_endpoint: str, bundle_dir: str) -> str:
    document = {
        "schema_version": 1,
        "extends": os.path.abspath(source),
        "transport": {
            "host": {
                "simulation_endpoint": state_endpoint,
                "feedback_endpoint": feedback_endpoint,
            }
        },
        "simulation": {
            "assembly": {
                "build_dir": os.path.abspath(bundle_dir),
                "rebuild_assembly": False,
            }
        },
    }
    handle = tempfile.NamedTemporaryFile("w", suffix=".yaml", prefix="elesim-sim-agent-", delete=False)
    with handle:
        yaml.safe_dump(document, handle, sort_keys=False)
    return handle.name


def _run() -> None:
    parser = argparse.ArgumentParser(description="Elesim distributed Genesis agent")
    parser.add_argument("--config", default=str(_ROOT / "config/default.yaml"))
    parser.add_argument("--runtime-config", default=str(_ROOT / "config/runtime.yaml"))
    parser.add_argument("--model-bundle", default=str(_ROOT.parents[1] / "model/bundles/default"))
    parser.add_argument("--server", default="")
    parser.add_argument("--id", default="")
    parser.add_argument("--legacy-state", default="tcp://127.0.0.1:5586")
    parser.add_argument("--legacy-feedback", default="tcp://127.0.0.1:5587")
    args, sim_args = parser.parse_known_args()
    bundle = load_app_config(args.config)
    role = load_runtime_role_config(args.runtime_config)
    if role.role != "simulator":
        raise ValueError(f"runtime role must be simulator, got {role.role!r}")
    server_endpoint = str(args.server).strip() or role.server_endpoint
    endpoint_id = str(args.id).strip() or role.endpoint_id
    rendered_source = _RenderedFrameSource(
        str(bundle.sim_config.sim_side_camera_port),
        use_jpeg=bool(bundle.sim_config.sim_side_camera_jpeg),
    )
    rendered_source.start()
    webrtc = WebRtcVideoSender(rendered_source.get, fps=float(bundle.sim_config.sim_side_camera_max_hz)) if webrtc_available() else None
    if webrtc is None:
        print("[sim_agent] WebRTC unavailable; install aiortc and av")
    bridge = SimProtocolBridge(
        server_endpoint=server_endpoint,
        endpoint_id=endpoint_id,
        legacy_state_bind=args.legacy_state,
        legacy_feedback_bind=args.legacy_feedback,
        mapping=bundle.mapping_config,
        streams={
            "rgbd": role.streams.get("rgbd_advertise", "") or str(bundle.sim_config.sim_camera_port),
            "rendered_view": role.streams.get("rendered_view", "webrtc"),
        },
        webrtc_offer_handler=None if webrtc is None else webrtc.accept_offer,
    )
    bridge.start()
    generated = _runtime_config(args.config, args.legacy_state, args.legacy_feedback, args.model_bundle)
    try:
        from elesim_simulator.runtime import main as run_genesis

        sys.argv = ["sim_agent.py", "--config", generated, *sim_args]
        run_genesis()
    finally:
        bridge.close()
        if webrtc is not None:
            webrtc.close()
        rendered_source.close()
        try:
            os.unlink(generated)
        except OSError:
            pass


def main() -> None:
    configure_tracing("elesim-sim-agent")
    try:
        with span("sim_agent.process.run"):
            _run()
    finally:
        shutdown_tracing()


if __name__ == "__main__":
    main()
