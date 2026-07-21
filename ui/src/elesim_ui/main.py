#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from elesim_ui.config import load_config
from elesim_ui.control_panel import ControlPanel
from elesim_ui.operator import RemoteControlService, RemotePanelState
from elesim_ui.operator_session import OperatorSession
from elesim_ui.webrtc_session import UiWebRtcSession


_ROOT = Path(__file__).resolve().parents[2]


def main() -> None:
    parser = argparse.ArgumentParser(description="Elesim desktop operator UI")
    parser.add_argument("--config", default=str(_ROOT / "config/default.yaml"))
    parser.add_argument("--server", default="")
    parser.add_argument("--controller-id", default="")
    parser.add_argument("--sim-id", default="")
    parser.add_argument("--no-webrtc", action="store_true")
    args = parser.parse_args()
    config = load_config(args.config)
    server = str(args.server).strip() or config.server_endpoint
    controller_id = str(args.controller_id).strip() or config.controller_id
    sim_id = str(args.sim_id).strip() or config.simulator_id

    session = OperatorSession(
        server,
        ui_id=config.endpoint_id,
        controller_id=controller_id,
    )
    state = RemotePanelState(
        session,
        initial_state={
            "visual_target_label": str(config.perception.target_label).strip(),
            "visual_target_scale": float(config.pick.target_scale),
            "visual_center_tol": float(config.pick.center_tol),
            "visual_target_uv_u": float(config.pick.target_uv_u),
            "visual_target_uv_v": float(config.pick.target_uv_v),
            "visual_scale_tol": float(config.pick.scale_tol),
            "visual_ready_distance_m": float(config.pick.ready_pose_standoff_m),
            "visual_look_distance_m": float(config.pick.look_pose_standoff_m),
        },
    )
    service = RemoteControlService(session, state)
    video = None
    if not args.no_webrtc:
        try:
            video = UiWebRtcSession(server, ui_id=config.endpoint_id, sim_id=sim_id)
            video.start()
        except Exception as exc:
            print(f"[ui] WebRTC unavailable: {exc}")

    def select_endpoint(endpoint_id: str, endpoint_role: str) -> None:
        service.select_endpoint(endpoint_id)
        if video is not None and endpoint_role == "simulator":
            video.switch_target(endpoint_id)

    panel = ControlPanel(
        state,
        service,
        use_hardware=config.use_hardware,
        use_go2=config.use_go2,
        go2_teleop_vx_mps=config.go2_vx,
        go2_teleop_vy_mps=config.go2_vy,
        go2_teleop_wz_radps=config.go2_wz,
        hardware_cfg=config.hardware,
        perception_cfg=config.perception,
        pick_cfg=config.pick,
        gaze_cfg=config.gaze,
        video_source=None if video is None else video.frame,
        camera_input=lambda command, values: service.send_sim_camera_input(command, values),
        endpoint_select=select_endpoint,
    )
    try:
        panel.run()
    finally:
        if video is not None:
            video.close()
        service.close()


if __name__ == "__main__":
    main()
