from __future__ import annotations

import imgui

from engine.config_loader import PerceptionConfig


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

    if not imgui.collapsing_header("Perception", visible=True)[0]:
        return

    changed_path, path_draft = imgui.input_text(
        "detector config",
        str(panel._perception_config_path_draft),
        256,
    )
    if changed_path:
        panel._perception_config_path_draft = str(path_draft).strip()

    changed_mode, mode_idx = imgui.combo(
        "mode",
        0 if str(panel._perception_mode_draft).strip().lower() == "camera" else 1,
        ["camera", "mock"],
    )
    if changed_mode:
        panel._perception_mode_draft = "camera" if int(mode_idx) == 0 else "mock"

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
    else:
        if imgui.button("Start Perception"):
            cfg = _build_perception_config(panel)
            panel.service.update_perception_config(cfg)
            panel.service.start_perception_capture(config=cfg)

    imgui.separator()
    imgui.text("Object Pick (YOLO once + CSRT + linear approach)")

    changed_scale, target_scale = imgui.input_float(
        "pick target scale",
        float(panel.state.visual_target_scale),
        step=0.01,
        step_fast=0.05,
        format="%.3f",
    )
    if changed_scale:
        panel.state.visual_target_scale = max(0.001, float(target_scale))

    pick_running = bool(panel.state.pick_running)
    if pick_running:
        if imgui.button("Stop Object Pick"):
            panel.service.stop_object_pick()
    else:
        if imgui.button("Start Object Pick"):
            panel.service.start_object_pick()

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
