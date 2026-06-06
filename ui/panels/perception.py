from __future__ import annotations

from pathlib import Path

import imgui

from engine.config_loader import PerceptionConfig, PROJECT_ROOT
from engine.controller.perception_capture import load_mock_world_xyz_from_detector_path


def _draw_mock_object_editor(panel) -> None:
    if str(panel._perception_mode_draft).strip().lower() != "mock":
        return

    imgui.separator()
    imgui.text("Mock Object Control (world [m])")
    mx, my, mz = panel.state.mock_object_world_xyz()
    changed = False

    ch_x, val_x = imgui.input_float("mock x", float(mx), step=0.01, step_fast=0.05, format="%.3f")
    if ch_x:
        mx = float(val_x)
        changed = True

    ch_y, val_y = imgui.input_float("mock y", float(my), step=0.01, step_fast=0.05, format="%.3f")
    if ch_y:
        my = float(val_y)
        changed = True

    ch_z, val_z = imgui.input_float("mock z", float(mz), step=0.01, step_fast=0.05, format="%.3f")
    if ch_z:
        mz = float(val_z)
        changed = True

    if changed:
        panel.service.set_mock_object_world(mx, my, mz)
        panel.service.publish_mock_object_world()

    if imgui.button("Publish Mock Object"):
        panel.service.publish_mock_object_world()


def _draw_ready_pose_dir_editor(panel) -> None:
    changed_look, look_dist = imgui.input_float(
        "look distance [m]",
        float(panel.state.visual_look_distance_m),
        step=0.01,
        step_fast=0.05,
        format="%.3f",
    )
    if changed_look:
        panel.state.visual_look_distance_m = max(0.0, float(look_dist))

    changed_dist, ready_dist = imgui.input_float(
        "ready distance [m]",
        float(panel.state.visual_ready_distance_m),
        step=0.01,
        step_fast=0.05,
        format="%.3f",
    )
    if changed_dist:
        panel.state.visual_ready_distance_m = max(0.0, float(ready_dist))
        panel.service.send_ready_pose_meta(source="target")


def _build_perception_config(panel) -> PerceptionConfig:
    return PerceptionConfig(
        enabled=True,
        detector_config=str(panel._perception_config_path_draft),
        mode=str(panel._perception_mode_draft),
        detector=str(panel._perception_detector_draft),
        target_label=str(panel._perception_target_label_draft),
        yolo_device=str(panel._perception_yolo_device_draft),
        publish_hz=float(panel._perception_publish_hz_draft),
        show_preview=bool(panel._perception_show_preview_draft),
        pipeline=str(panel._perception_pipeline_draft),
        tracker=str(panel._perception_tracker_draft),
    )


def draw_perception_panel(panel) -> None:
    if not panel._perception_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._perception_header_init_open = True

    if not imgui.collapsing_header("Visual Servoing", visible=True)[0]:
        return

    changed_path, path_draft = imgui.input_text(
        "detector config",
        str(panel._perception_config_path_draft),
        256,
    )
    if changed_path:
        panel._perception_config_path_draft = str(path_draft).strip()
        cfg_path = Path(panel._perception_config_path_draft)
        if not cfg_path.is_absolute():
            cfg_path = PROJECT_ROOT / cfg_path
        mock_xyz = load_mock_world_xyz_from_detector_path(cfg_path)
        if mock_xyz is not None:
            panel.state.set_mock_object_world_xyz(*mock_xyz)

    changed_mode, mode_idx = imgui.combo(
        "mode",
        0 if str(panel._perception_mode_draft).strip().lower() == "camera" else 1,
        ["camera", "mock"],
    )
    if changed_mode:
        panel._perception_mode_draft = "camera" if int(mode_idx) == 0 else "mock"

    _draw_mock_object_editor(panel)

    changed_label, label_draft = imgui.input_text(
        "target label",
        str(panel._perception_target_label_draft),
        64,
    )
    if changed_label:
        panel._perception_target_label_draft = str(label_draft).strip()
        panel.state.visual_target_label = str(label_draft).strip()

    changed_preview, show_preview = imgui.checkbox(
        "show preview",
        bool(panel._perception_show_preview_draft),
    )
    if changed_preview:
        panel._perception_show_preview_draft = bool(show_preview)

    changed_hz, publish_hz = imgui.input_float(
        "publish hz",
        float(panel._perception_publish_hz_draft),
        step=1.0,
        step_fast=5.0,
        format="%.1f",
    )
    if changed_hz:
        panel._perception_publish_hz_draft = max(0.1, float(publish_hz))

    pipeline_options = ["search_track", "yolo_only"]
    pipeline_idx = 0 if str(panel._perception_pipeline_draft).strip().lower() != "yolo_only" else 1
    changed_pipe, pipe_idx = imgui.combo("pipeline", pipeline_idx, pipeline_options)
    if changed_pipe:
        panel._perception_pipeline_draft = pipeline_options[int(pipe_idx)]

    tracker_options = ["csrt", "kcf"]
    tracker_idx = 0 if str(panel._perception_tracker_draft).strip().lower() != "kcf" else 1
    changed_tr, tr_idx = imgui.combo("tracker", tracker_idx, tracker_options)
    if changed_tr:
        panel._perception_tracker_draft = tracker_options[int(tr_idx)]

    running = bool(panel.state.perception_running)
    if running:
        if imgui.button("Stop Perception"):
            panel.service.stop_perception_capture()
        imgui.same_line()
        if imgui.button("Refresh"):
            panel.service.refresh_perception_capture()
    else:
        if imgui.button("Start Perception"):
            cfg = _build_perception_config(panel)
            panel.service.update_perception_config(cfg)
            panel.service.start_perception_capture(config=cfg)
        imgui.same_line()
        if imgui.button("Refresh"):
            panel.service.refresh_perception_capture()

    imgui.separator()
    imgui.text("Look / Aim / Grasp (UV centering + equal-sag + object IK)")

    changed_scale, target_scale = imgui.input_float(
        "pick target scale",
        float(panel.state.visual_target_scale),
        step=0.01,
        step_fast=0.05,
        format="%.3f",
    )
    if changed_scale:
        panel.state.visual_target_scale = max(0.001, float(target_scale))

    changed_tu, target_uv_u = imgui.input_float(
        "gripper target u",
        float(panel.state.visual_target_uv_u),
        step=0.05,
        step_fast=0.1,
        format="%.3f",
    )
    if changed_tu:
        panel.state.visual_target_uv_u = max(-1.0, min(1.0, float(target_uv_u)))

    changed_tv, target_uv_v = imgui.input_float(
        "gripper target v",
        float(panel.state.visual_target_uv_v),
        step=0.05,
        step_fast=0.1,
        format="%.3f",
    )
    if changed_tv:
        panel.state.visual_target_uv_v = max(-1.0, min(1.0, float(target_uv_v)))

    imgui.text_wrapped(
        "Look -> Aim -> Grasp (E2E) runs all three steps. "
        "Look: view pose + equal-sag baseline. "
        "Aim: UV center + drift estimate (stops when centered). "
        "Grasp: IK to object - approach_dir * grasp_standoff_m, then close gripper "
        "(works after Look in sim; Aim optional for equal-sag correction)."
    )
    imgui.separator()
    _draw_ready_pose_dir_editor(panel)

    pick_running = bool(panel.state.pick_running) or bool(panel.service.pick_e2e_running())
    if pick_running:
        if imgui.button("Stop"):
            panel.service.stop_pick_e2e()
    else:
        cfg = _build_perception_config(panel)
        if imgui.button("Look -> Aim -> Grasp"):
            panel.service.update_perception_config(cfg)
            panel.service.start_look_aim_grasp_e2e()
        imgui.same_line()
        if imgui.button("Look"):
            panel.service.update_perception_config(cfg)
            panel.service.start_look()
        imgui.same_line()
        if imgui.button("Aim"):
            panel.service.update_perception_config(cfg)
            panel.service.start_aim()
        imgui.same_line()
        if imgui.button("Grasp"):
            panel.service.start_grasp()
        if imgui.tree_node("Advanced (debug)"):
            if imgui.button("Ready Pose"):
                panel.service.update_perception_config(cfg)
                panel.service.start_ready_pose()
            imgui.same_line()
            if imgui.button("Pick forward"):
                panel.service.start_pick_forward(distance_m=0.15)
            imgui.tree_pop()

    pick_phase = str(panel.state.pick_phase) or "idle"
    pick_status = "running" if pick_running else "idle"
    if panel.state.pick_failed:
        pick_status = "failed"
    imgui.text(f"Pick: {pick_status} | phase: {pick_phase}")
    if str(panel.state.pick_status_msg).strip():
        imgui.text_wrapped(str(panel.state.pick_status_msg))

    imgui.separator()
    status = "idle"
    if panel.state.perception_running:
        status = "running"
    if panel.state.perception_failed:
        status = "failed"
    imgui.text(
        "Status: %s | frame: %d | tracker: %s | track_ok: %d"
        % (
            status,
            int(panel.state.perception_frame_idx),
            str(panel.state.perception_tracker_phase),
            int(panel.state.perception_track_ok_frames),
        )
    )
    bw = panel.state.perception_bbox_wh
    imgui.text(
        "Scale: %.3f | bbox: %dx%d px | backend: %s"
        % (
            float(panel.state.perception_image_scale),
            int(bw[0]),
            int(bw[1]),
            str(panel.state.perception_tracker_backend) or "—",
        )
    )
    if str(panel.state.perception_status_msg).strip():
        imgui.text_wrapped(str(panel.state.perception_status_msg))

    label = str(panel.state.perception_label) or "(none)"
    imgui.text(f"Detection: {label} | conf: {float(panel.state.perception_confidence):.2f}")
    if panel.state.perception_camera_xyz is not None:
        p = panel.state.perception_camera_xyz
        imgui.text(f"Camera XYZ [m]: ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})")
    else:
        imgui.text("Camera XYZ [m]: —")
    if panel.state.perception_world_xyz is not None:
        p = panel.state.perception_world_xyz
        imgui.text(f"World XYZ [m]: ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})")
    else:
        imgui.text("World XYZ [m]: —")
