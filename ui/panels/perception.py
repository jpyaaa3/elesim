from __future__ import annotations

import imgui

from engine.config_loader import PerceptionConfig
from ui.helpers import panel_header, scaled


_CAPTURE_SOURCES = (("camera", "Camera"), ("sim", "Sim"))
_PERCEPTION_LABEL_W = 88.0


def _field_width(panel) -> float:
    width_getter = getattr(imgui, "get_content_region_available_width", None)
    available = float(width_getter()) if callable(width_getter) else scaled(panel, 180.0)
    return max(1.0, available)


def _control_label(panel, text: str) -> None:
    imgui.text(str(text))
    imgui.same_line(scaled(panel, _PERCEPTION_LABEL_W))


def _input_text(panel, label: str, identifier: str, value: str, buffer_size: int) -> tuple[bool, str]:
    _control_label(panel, label)
    imgui.push_item_width(_field_width(panel))
    try:
        return imgui.input_text(f"##{identifier}", str(value), int(buffer_size))
    finally:
        imgui.pop_item_width()


def _input_float(
    panel,
    label: str,
    identifier: str,
    value: float,
    *,
    step: float,
    step_fast: float,
    format: str,
) -> tuple[bool, float]:
    _control_label(panel, label)
    imgui.push_item_width(_field_width(panel))
    try:
        return imgui.input_float(
            f"##{identifier}",
            float(value),
            0.0,
            0.0,
            format=format,
        )
    finally:
        imgui.pop_item_width()


def _combo(panel, label: str, identifier: str, index: int, items: list[str]) -> tuple[bool, int]:
    _control_label(panel, label)
    imgui.push_item_width(_field_width(panel))
    try:
        return imgui.combo(f"##{identifier}", int(index), items)
    finally:
        imgui.pop_item_width()


def _checkbox(panel, label: str, identifier: str, value: bool) -> tuple[bool, bool]:
    _control_label(panel, label)
    return imgui.checkbox(f"##{identifier}", bool(value))


def _capture_source_index(mode: str) -> int:
    key = str(mode).strip().lower()
    for idx, (value, _label) in enumerate(_CAPTURE_SOURCES):
        if key == value:
            return idx
    return 0


def _local_detector_mode(detector: str) -> str:
    key = str(detector).strip().lower()
    return "config" if key in ("", "external") else key


def _draw_ready_pose_dir_editor(panel) -> None:
    changed_look, look_dist = _input_float(
        panel,
        "Look dist",
        "visual_look_distance",
        float(panel.state.visual_look_distance_m),
        step=0.01,
        step_fast=0.05,
        format="%.3f",
    )
    if changed_look:
        panel.state.visual_look_distance_m = max(0.0, float(look_dist))

    changed_dist, ready_dist = _input_float(
        panel,
        "Ready dist",
        "visual_ready_distance",
        float(panel.state.visual_ready_distance_m),
        step=0.01,
        step_fast=0.05,
        format="%.3f",
    )
    if changed_dist:
        panel.state.visual_ready_distance_m = max(0.0, float(ready_dist))
        panel.service.send_ready_pose_meta(source="target")


def _build_perception_config(panel) -> PerceptionConfig:
    run_local = bool(getattr(panel, "_perception_run_local", True))
    return PerceptionConfig(
        enabled=True,
        detector_config=str(panel._perception_config_path_draft),
        mode=str(panel._perception_mode_draft),
        detector=(
            _local_detector_mode(str(panel._perception_detector_draft))
            if run_local
            else str(panel._perception_detector_draft)
        ),
        target_label=str(panel._perception_target_label_draft),
        yolo_device=str(panel._perception_yolo_device_draft),
        publish_hz=float(panel._perception_publish_hz_draft),
        show_preview=bool(panel._perception_show_preview_draft),
        pipeline=str(panel._perception_pipeline_draft),
        tracker=str(panel._perception_tracker_draft),
        run_local=run_local,
    )


def draw_perception_panel(panel) -> None:
    if not panel._perception_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._perception_header_init_open = True

    if not panel_header("Visual Servoing", visible=True)[0]:
        return

    run_local = bool(getattr(panel, "_perception_run_local", True))
    if not run_local:
        imgui.text_wrapped(
            "Remote perception: capture runs on Jetson (perception_worker.py). "
            "Start/Stop here are disabled; use host relay below."
        )
        imgui.separator()
        imgui.text_wrapped("source: Remote host relay")
    else:
        changed_path, path_draft = _input_text(
            panel,
            "Config",
            "detector_config",
            str(panel._perception_config_path_draft),
            256,
        )
        if changed_path:
            panel._perception_config_path_draft = str(path_draft).strip()

        source_idx = _capture_source_index(panel._perception_mode_draft)
        panel._perception_mode_draft = _CAPTURE_SOURCES[source_idx][0]
        changed_source, source_idx = _combo(
            panel,
            "Source",
            "capture_source",
            source_idx,
            [label for _value, label in _CAPTURE_SOURCES],
        )
        if changed_source:
            panel._perception_mode_draft = _CAPTURE_SOURCES[int(source_idx)][0]

    changed_label, label_draft = _input_text(
        panel,
        "Label",
        "target_label",
        str(panel._perception_target_label_draft),
        64,
    )
    if changed_label:
        panel._perception_target_label_draft = str(label_draft).strip()
        panel.state.visual_target_label = str(label_draft).strip()

    if run_local:
        changed_preview, show_preview = _checkbox(
            panel,
            "Preview",
            "show_preview",
            bool(panel._perception_show_preview_draft),
        )
        if changed_preview:
            panel._perception_show_preview_draft = bool(show_preview)

        changed_hz, publish_hz = _input_float(
            panel,
            "Rate",
            "publish_hz",
            float(panel._perception_publish_hz_draft),
            step=1.0,
            step_fast=5.0,
            format="%.1f",
        )
        if changed_hz:
            panel._perception_publish_hz_draft = max(0.1, float(publish_hz))

        pipeline_options = ["yolo_seg", "search_track", "yolo_only"]
        pipe_draft = str(panel._perception_pipeline_draft).strip().lower().replace("-", "_")
        pipeline_idx = 0
        if pipe_draft in ("search_track", "track"):
            pipeline_idx = 1
        elif pipe_draft == "yolo_only":
            pipeline_idx = 2
        changed_pipe, pipe_idx = _combo(panel, "Pipeline", "pipeline", pipeline_idx, pipeline_options)
        if changed_pipe:
            panel._perception_pipeline_draft = pipeline_options[int(pipe_idx)]

        tracker_options = ["csrt", "kcf"]
        tracker_idx = 0 if str(panel._perception_tracker_draft).strip().lower() != "kcf" else 1
        changed_tr, tr_idx = _combo(panel, "Tracker", "tracker", tracker_idx, tracker_options)
        if changed_tr:
            panel._perception_tracker_draft = tracker_options[int(tr_idx)]

    running = bool(panel.state.perception_running)
    if run_local:
        if imgui.button("Save Frame"):
            panel.service.capture_perception_frame()
        if running:
            if imgui.button("Stop Perception"):
                panel.service.stop_perception_capture()
            if imgui.button("Refresh"):
                panel.service.refresh_perception_capture()
        else:
            if imgui.button("Start Perception"):
                cfg = _build_perception_config(panel)
                panel.service.update_perception_config(cfg)
                panel.service.start_perception_capture(config=cfg)
            if imgui.button("Refresh"):
                panel.service.refresh_perception_capture()
    else:
        imgui.text_wrapped("Perception capture: Jetson worker (see Status host relay)")

    imgui.separator()
    imgui.text_wrapped("Look / Aim / Grasp (UV centering + equal-sag + object IK)")

    changed_scale, target_scale = _input_float(
        panel,
        "Scale",
        "pick_target_scale",
        float(panel.state.visual_target_scale),
        step=0.01,
        step_fast=0.05,
        format="%.3f",
    )
    if changed_scale:
        panel.state.visual_target_scale = max(0.001, float(target_scale))

    changed_tu, target_uv_u = _input_float(
        panel,
        "Target u",
        "gripper_target_u",
        float(panel.state.visual_target_uv_u),
        step=0.05,
        step_fast=0.1,
        format="%.3f",
    )
    if changed_tu:
        panel.state.visual_target_uv_u = max(-1.0, min(1.0, float(target_uv_u)))

    changed_tv, target_uv_v = _input_float(
        panel,
        "Target v",
        "gripper_target_v",
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
        "Grasp: guided waypoints (UV center + online sag) toward nominal pre-contact, "
        "then blind approach and close gripper (Aim optional for initial equal-sag)."
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
        if imgui.button("Look"):
            panel.service.update_perception_config(cfg)
            panel.service.start_look()
        if imgui.button("Aim"):
            panel.service.update_perception_config(cfg)
            panel.service.start_aim()
        if imgui.button("Grasp"):
            panel.service.update_perception_config(cfg)
            panel.service.start_grasp()
        if imgui.tree_node("Advanced (debug)"):
            if imgui.button("Ready Pose"):
                panel.service.update_perception_config(cfg)
                panel.service.start_ready_pose()
            if imgui.button("Pick forward"):
                panel.service.start_pick_forward(distance_m=0.15)
            imgui.tree_pop()
