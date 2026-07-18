#!/usr/bin/env python3
"""Desktop UI client for the independent control-agent process."""

from __future__ import annotations

import argparse
from pathlib import Path

from apps.controller.rpc import ControlRpcClient, RemoteControlService, RemotePanelState
from apps.control.webrtc_session import UiWebRtcSession
from engine.config import load_app_config, load_runtime_role_config
from engine.observability.tracing import configure_tracing, shutdown_tracing, span
from ui.control_panel import ControlPanel


_ROOT = Path(__file__).resolve().parents[2]


def _run() -> None:
    parser = argparse.ArgumentParser(description="Elesim desktop control UI")
    parser.add_argument("--config", default=str(_ROOT / "configs/config.pc.yaml"))
    parser.add_argument("--runtime-config", default=str(_ROOT / "configs/runtime/ui.yaml"))
    parser.add_argument("--control-agent", default="")
    parser.add_argument("--server", default="")
    parser.add_argument("--sim-id", default="")
    parser.add_argument("--no-webrtc", action="store_true")
    args = parser.parse_args()
    bundle = load_app_config(args.config)
    role = load_runtime_role_config(args.runtime_config)
    if role.role != "ui":
        raise ValueError(f"runtime role must be ui, got {role.role!r}")
    control_endpoint = str(args.control_agent).strip() or role.rpc_endpoint
    server_endpoint = str(args.server).strip() or role.server_endpoint
    sim_id = str(args.sim_id).strip() or role.active_target
    rpc = ControlRpcClient(control_endpoint)
    state = RemotePanelState(rpc)
    service = RemoteControlService(rpc, state)
    video = None
    if not args.no_webrtc:
        try:
            video = UiWebRtcSession(server_endpoint, ui_id=role.endpoint_id, sim_id=sim_id)
            video.start()
        except Exception as exc:
            print(f"[ctrl] WebRTC unavailable: {exc}")
            video = None

    def select_endpoint(endpoint_id: str, endpoint_role: str) -> None:
        service.select_endpoint(endpoint_id)
        if video is not None and endpoint_role == "sim":
            video.switch_target(endpoint_id)

    panel = ControlPanel(
        state,
        service,
        use_hardware=bool(bundle.sim_config.use_hardware),
        use_go2=bool(bundle.sim_config.use_go2),
        go2_teleop_vx_mps=float(getattr(bundle.spawn_config, "go2_teleop_vx_mps", 0.35)),
        go2_teleop_vy_mps=float(getattr(bundle.spawn_config, "go2_teleop_vy_mps", 0.25)),
        go2_teleop_wz_radps=float(getattr(bundle.spawn_config, "go2_teleop_wz_radps", 0.80)),
        hardware_cfg=bundle.hardware_config,
        perception_cfg=bundle.perception_config,
        pick_cfg=bundle.pick_config,
        gaze_cfg=bundle.gaze_stabilizer_config,
        video_source=None if video is None else video.frame,
        camera_input=lambda command, values: service.send_sim_camera_input(command, values),
        endpoint_select=select_endpoint,
    )
    try:
        service.refresh_host_state()
        panel.run()
    finally:
        if video is not None:
            video.close()
        service.close()


def main() -> None:
    configure_tracing("elesim-control-ui")
    try:
        with span("control_ui.process.run"):
            _run()
    finally:
        shutdown_tracing()


if __name__ == "__main__":
    main()
