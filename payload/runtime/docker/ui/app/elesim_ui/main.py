#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from elesim_ui.config import load_config
from elesim_ui.control_panel import ControlPanel
from elesim_ui.operator import RemoteControlService, RemotePanelState
from elesim_ui.operator_session import OperatorSession
from elesim_ui.peer_runtime import UiPeerHub
from elesim_ui.sim_session import UiSimSession
from elesim_ui.zed_preview import ZedCameraPreview
from elesim_protocol import (
    configure_protocol_tracing,
    shutdown_protocol_tracing,
)
from elesim_protocol.tracing import span


def _default_config() -> Path:
    for root in Path(__file__).resolve().parents:
        candidate = root / "payload/config/ui/default.yaml"
        if candidate.is_file():
            return candidate
    return Path("/opt/elesim/config/installed.yaml")


_DEFAULT_CONFIG = _default_config()


def main() -> None:
    parser = argparse.ArgumentParser(description="EleSim desktop operator UI")
    parser.add_argument("--config", default=str(_DEFAULT_CONFIG))
    parser.add_argument("--pilot-id", default="")
    parser.add_argument("--sim-id", default="")
    parser.add_argument("--no-webrtc", action="store_true")
    args = parser.parse_args()
    configure_protocol_tracing("elesim-ui")
    try:
        with span(
            "elesim_ui.main.main",
            attributes={
                "code.function.name": "elesim_ui.main.main",
                "elesim.flow.id": "ui.lifecycle",
            },
        ):
            _run(args)
    finally:
        shutdown_protocol_tracing()


def _run(args: argparse.Namespace) -> None:
    config = load_config(args.config)
    pilot_id = str(args.pilot_id).strip() or config.pilot_id
    sim_id = str(args.sim_id).strip() or config.sim_id
    peer_hub = UiPeerHub(endpoint_id=config.endpoint_id, settings=config.dds)
    zed_preview = None
    zed_right_preview = None
    if config.show_zed_preview and config.rgbd_topic:
        zed_preview = ZedCameraPreview(
            config.rgbd_topic,
            endpoint_id=config.endpoint_id,
            settings=config.dds,
            stream_id="zed-left-preview",
        )
        zed_preview.start()
        if config.rgbd_right_topic:
            zed_right_preview = ZedCameraPreview(
                config.rgbd_right_topic,
                endpoint_id=config.endpoint_id,
                settings=config.dds,
                stream_id="zed-right-preview",
            )
            zed_right_preview.start()

    session = OperatorSession(
        ui_id=config.endpoint_id,
        pilot_id=pilot_id,
        peer=peer_hub.channel("operator"),
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
    sim_session = None
    if not args.no_webrtc:
        try:
            sim_session = UiSimSession(
                ui_id=config.endpoint_id,
                sim_id=sim_id,
                peer=peer_hub.channel("sim"),
            )
        except Exception as exc:
            print(f"[ui] WebRTC unavailable: {exc}")

    def select_endpoint(endpoint_id: str, endpoint_role: str) -> None:
        # Only the explicit CONTROL TARGET selector changes the motion route.
        # Camera source selection belongs to SimView and is independent.
        role = str(endpoint_role).strip().lower()
        if role not in {"sim", "robot"}:
            return
        service.select_endpoint(endpoint_id)

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
        sim_session=sim_session,
        zed_preview=zed_preview,
        zed_right_preview=zed_right_preview,
        endpoint_select=select_endpoint,
    )
    try:
        panel.run()
    finally:
        if zed_preview is not None:
            zed_preview.close()
        if zed_right_preview is not None:
            zed_right_preview.close()
        if sim_session is not None:
            sim_session.close()
        service.close()
        peer_hub.close()


if __name__ == "__main__":
    main()
