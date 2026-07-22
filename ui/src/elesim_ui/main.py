#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from elesim_ui.config import load_config
from elesim_ui.control_panel import ControlPanel
from elesim_ui.operator import RemoteControlService, RemotePanelState
from elesim_ui.operator_session import OperatorSession
from elesim_protocol import CurveClientConfig
from elesim_ui.simulator_session import UiSimulatorSession


_ROOT = Path(__file__).resolve().parents[2]


def _router_curve(config) -> CurveClientConfig | None:
    client = str(config.router_client_secret_file).strip()
    server = str(config.router_server_public_file).strip()
    if bool(client) != bool(server):
        raise ValueError(
            "router CURVE client and server certificate paths must be configured together"
        )
    if not client:
        return None
    return CurveClientConfig.from_files(
        client_secret_file=client,
        server_public_file=server,
    )


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
    router_curve = _router_curve(config)

    session = OperatorSession(
        server,
        ui_id=config.endpoint_id,
        controller_id=controller_id,
        curve=router_curve,
        allow_insecure_remote=config.allow_insecure_remote,
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
    simulator_session = None
    if not args.no_webrtc:
        try:
            simulator_session = UiSimulatorSession(
                server,
                ui_id=config.endpoint_id,
                sim_id=sim_id,
                curve=router_curve,
                allow_insecure_remote=config.allow_insecure_remote,
            )
        except Exception as exc:
            print(f"[ui] WebRTC unavailable: {exc}")

    def select_endpoint(endpoint_id: str, endpoint_role: str) -> None:
        service.select_endpoint(endpoint_id)
        if simulator_session is not None and endpoint_role == "simulator":
            simulator_session.switch_target(endpoint_id)

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
        simulator_session=simulator_session,
        endpoint_select=select_endpoint,
    )
    try:
        panel.run()
    finally:
        if simulator_session is not None:
            simulator_session.close()
        service.close()


if __name__ == "__main__":
    main()
