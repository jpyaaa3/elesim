"""Composition root for laptop-side control algorithms."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from elesim_protocol import default_start_sim_q
from elesim_controller.pick import ControlService, PanelState
from elesim_controller.robot.arm import ik as ik_pipeline
from elesim_controller.robot.arm.mounts.go2_mount import Go2ArmMount
from elesim_controller.vision.perception_bridge.hand_eye import load_hand_eye_transform


@dataclass
class ControlRuntime:
    bundle: Any
    state: PanelState
    service: ControlService


def build_control_runtime(config_path: str, client: Any) -> ControlRuntime:
    bundle, ik_context = ik_pipeline.load_solver_context(config_path)
    hand_eye_transform = None
    hand_eye_parent_frame = "node9"
    hand_eye_path = str(bundle.sim_config.hand_eye_config).strip()
    if hand_eye_path:
        try:
            hand_eye_transform, hand_eye_meta = load_hand_eye_transform(hand_eye_path)
            hand_eye_parent_frame = str(hand_eye_meta.get("parent_frame", "node9"))
        except Exception as exc:
            print(f"[control_agent] hand-eye config unavailable: {exc}")
    state = PanelState(sag_model_path="", raw_sag_model=None)
    state.set_u_offsets(
        linear=float(bundle.hardware_config.u_offset_linear),
        roll=float(bundle.hardware_config.u_offset_roll),
        s1=float(bundle.hardware_config.u_offset_s1),
        s2=float(bundle.hardware_config.u_offset_s2),
    )
    spawn_q = default_start_sim_q(bundle.mapping_config)
    state.set_q(spawn_q.linear_m, spawn_q.roll_rad, spawn_q.theta1_rad, spawn_q.theta2_rad)
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
    try:
        state.set_mock_object_world_xyz(*tuple(float(value) for value in bundle.spawn_config.sim_target_xyz))
    except Exception:
        pass
    mount = Go2ArmMount.from_context(
        use_go2=bool(bundle.sim_config.use_go2),
        spawn_xyz=bundle.spawn_config.spawn_xyz,
        go2_spawn_height=float(bundle.spawn_config.go2_spawn_height),
        go2_spawn_euler_deg=bundle.spawn_config.go2_spawn_euler_deg,
        mount_offset_body_m=bundle.spawn_config.go2_mount_offset_m,
        ik_context=ik_context,
    )
    service = ControlService(
        state,
        client=client,
        mapping_cfg=bundle.mapping_config,
        ik_cfg=bundle.ik_config,
        ik_context=ik_context,
        config_path=config_path,
        perception_cfg=perception_cfg,
        pick_cfg=pick_cfg,
        gaze_cfg=bundle.gaze_stabilizer_config,
        hand_eye_transform=hand_eye_transform,
        hand_eye_parent_frame=hand_eye_parent_frame,
        go2_arm_mount=mount,
        use_hardware=bool(bundle.sim_config.use_hardware),
    )
    return ControlRuntime(bundle, state, service)

