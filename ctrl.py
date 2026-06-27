#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import os

from engine.vision.perception_bridge.hand_eye import load_hand_eye_transform
from engine import ik as ik_pipeline
from engine.coordinates.go2_arm_frame import Go2ArmFrameConfig
from engine.controller import ControlClient, ControlService, PanelState
from ui.control_panel import ControlPanel


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(__file__), "config.ini"),
        help="path to ini config file",
    )
    args = ap.parse_args()

    bundle, ik_context = ik_pipeline.load_solver_context(args.config)
    hand_eye_transform = None
    hand_eye_parent_frame = "node9"
    hand_eye_path = str(bundle.sim_config.hand_eye_config).strip()
    if hand_eye_path:
        try:
            hand_eye_transform, hand_eye_meta = load_hand_eye_transform(hand_eye_path)
            hand_eye_parent_frame = str(hand_eye_meta.get("parent_frame", "node9"))
        except Exception as exc:
            print(f"[ctrl] hand-eye config unavailable: {exc}")
            hand_eye_transform = None
    link = ControlClient(str(bundle.sim_config.host_ctrl_port), cfg=bundle.mapping_config)
    state = PanelState(
        sag_model_path="",
        raw_sag_model=None,
    )
    perception_cfg = bundle.perception_config
    pick_cfg = bundle.pick_config
    state.visual_target_label = str(perception_cfg.target_label).strip()
    state.visual_target_scale = float(pick_cfg.target_scale)
    state.visual_center_tol = float(pick_cfg.center_tol)
    state.visual_target_uv_u = float(pick_cfg.target_uv_u)
    state.visual_target_uv_v = float(pick_cfg.target_uv_v)
    state.visual_scale_tol = float(pick_cfg.scale_tol)
    state.visual_ready_distance_m = float(pick_cfg.ready_pose_standoff_m)
    state.visual_look_distance_m = float(pick_cfg.look_pose_standoff_m)

    go2_arm_frame = Go2ArmFrameConfig.from_context(
        use_go2=bool(bundle.sim_config.use_go2),
        spawn_xyz=bundle.spawn_config.spawn_xyz,
        spawn_euler_deg=bundle.spawn_config.spawn_euler_deg,
        go2_spawn_height=float(bundle.spawn_config.go2_spawn_height),
        go2_spawn_euler_deg=bundle.spawn_config.go2_spawn_euler_deg,
        mount_offset_body_m=bundle.spawn_config.go2_mount_offset_m,
        ik_context=ik_context,
    )

    service = ControlService(
        state,
        client=link,
        mapping_cfg=bundle.mapping_config,
        ik_cfg=bundle.ik_config,
        ik_context=ik_context,
        config_path=args.config,
        perception_cfg=perception_cfg,
        pick_cfg=pick_cfg,
        gaze_cfg=bundle.gaze_stabilizer_config,
        hand_eye_transform=hand_eye_transform,
        hand_eye_parent_frame=hand_eye_parent_frame,
        go2_arm_frame=go2_arm_frame,
        use_hardware=bool(bundle.sim_config.use_hardware),
    )
    gui = ControlPanel(
        state,
        service,
        use_hardware=bool(bundle.sim_config.use_hardware),
        use_go2=bool(bundle.sim_config.use_go2),
        go2_teleop_vx_mps=float(getattr(bundle.spawn_config, "go2_teleop_vx_mps", 0.35)),
        go2_teleop_vy_mps=float(getattr(bundle.spawn_config, "go2_teleop_vy_mps", 0.25)),
        go2_teleop_wz_radps=float(getattr(bundle.spawn_config, "go2_teleop_wz_radps", 0.80)),
        hardware_cfg=bundle.hardware_config,
        perception_cfg=perception_cfg,
        pick_cfg=pick_cfg,
    )
    try:
        service.refresh_host_state()
        service.send_current_target_meta(source="target")
        gui.run()
    finally:
        service.close()


if __name__ == "__main__":
    main()
