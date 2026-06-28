from __future__ import annotations

import math
from pathlib import Path

import imgui

from engine.config_loader import PerceptionConfig
from ui.helpers import (
    begin_collapsible_section,
    begin_disabled_ui,
    end_collapsible_section,
    end_disabled_ui,
    panel_header,
    scaled,
    toggle_switch,
)


_CAPTURE_SOURCES = (("camera", "Real"), ("sim", "Virtual"))
_PERCEPTION_LABEL_W = 88.0
_TRACK_BUTTON_W = 124.0
_TRACK_DEMO_BUTTON_W = 132.0
_BUTTON_H = 26.0
_REFRESH_BUTTON_W = 30.0
_PREVIEW_ALIGN_W = 92.0
_CONFIG_BROWSE_W = 72.0
_MODE_BUTTON_W = 76.0
_DETECTION_EVERY_BUTTON_W = 112.0
_DETECTION_TRACK_BUTTON_W = 132.0
_BALL_MOVE_W = 58.0
_MODEL_CONFIGS = {
    "yolo": "model_presets/visual_servoing/detector.yolo.example.json",
    "hsv": "model_presets/visual_servoing/detector.sim_hsv.json",
}


def _project_root() -> Path:
    return Path(__file__).resolve().parents[2]


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


def _checkbox(panel, label: str, identifier: str, value: bool) -> tuple[bool, bool]:
    _control_label(panel, label)
    return imgui.checkbox(f"##{identifier}", bool(value))


def _button(panel, label: str, width: float, height: float = _BUTTON_H) -> bool:
    return bool(imgui.button(label, scaled(panel, width), scaled(panel, height)))


def _mode_button(
    panel,
    label: str,
    widget_id: str,
    selected: bool,
    *,
    width: float = _MODE_BUTTON_W,
    disabled: bool = False,
) -> bool:
    pushed = 0
    if disabled:
        colors = (
            (imgui.COLOR_BUTTON, 0.72, 0.75, 0.80, 1.0),
            (imgui.COLOR_BUTTON_HOVERED, 0.72, 0.75, 0.80, 1.0),
            (imgui.COLOR_BUTTON_ACTIVE, 0.72, 0.75, 0.80, 1.0),
            (imgui.COLOR_TEXT, 0.45, 0.48, 0.53, 1.0),
        )
    elif selected:
        colors = (
            (imgui.COLOR_BUTTON, 0.13, 0.62, 0.30, 1.0),
            (imgui.COLOR_BUTTON_HOVERED, 0.16, 0.68, 0.34, 1.0),
            (imgui.COLOR_BUTTON_ACTIVE, 0.10, 0.52, 0.25, 1.0),
            (imgui.COLOR_TEXT, 1.0, 1.0, 1.0, 1.0),
        )
    else:
        colors = (
            (imgui.COLOR_BUTTON, 0.82, 0.84, 0.87, 1.0),
            (imgui.COLOR_BUTTON_HOVERED, 0.76, 0.79, 0.84, 1.0),
            (imgui.COLOR_BUTTON_ACTIVE, 0.66, 0.70, 0.76, 1.0),
            (imgui.COLOR_TEXT, 0.12, 0.14, 0.17, 1.0),
        )
    for color in colors:
        try:
            imgui.push_style_color(*color)
            pushed += 1
        except Exception:
            break
    disabled_token = begin_disabled_ui(disabled)
    try:
        clicked = bool(imgui.button(f"{label}##{widget_id}", scaled(panel, width), 0.0))
        return (not disabled) and clicked
    finally:
        end_disabled_ui(disabled_token)
        if pushed:
            imgui.pop_style_color(pushed)


def _draw_camera_mode_row(panel) -> None:
    mode = str(panel._perception_mode_draft).strip().lower()
    is_real = mode != "sim"
    _control_label(panel, "Mode")
    if _mode_button(panel, "Real", "camera_mode_real", is_real):
        panel._perception_mode_draft = "camera"
        provider = str(getattr(panel, "_perception_real_provider_draft", "")).strip().lower()
        panel._perception_provider_draft = provider if provider in ("local", "host") else "local"
    imgui.same_line()
    if _mode_button(panel, "Virtual", "camera_mode_virtual", not is_real):
        if is_real:
            provider = str(getattr(panel, "_perception_provider_draft", "local")).strip().lower()
            if provider in ("local", "host"):
                panel._perception_real_provider_draft = provider
        panel._perception_mode_draft = "sim"


def _detector_model_from_draft(panel) -> str:
    detector = str(getattr(panel, "_perception_detector_draft", "")).strip().lower()
    if detector in ("yolo", "hsv"):
        return detector
    path = str(getattr(panel, "_perception_config_path_draft", "")).strip().lower()
    if "yolo" in path:
        return "yolo"
    if "hsv" in path:
        return "hsv"
    return ""


def _set_detector_model(panel, model: str) -> None:
    key = str(model).strip().lower()
    path = _MODEL_CONFIGS.get(key)
    if not path:
        return
    panel._perception_detector_draft = "config"
    panel._perception_config_path_draft = path


def _draw_model_row(panel) -> None:
    model = _detector_model_from_draft(panel)
    _control_label(panel, "Model")
    if _mode_button(panel, "YOLO", "model_yolo", model == "yolo"):
        _set_detector_model(panel, "yolo")
    imgui.same_line()
    if _mode_button(panel, "HSV", "model_hsv", model == "hsv"):
        _set_detector_model(panel, "hsv")


def _draw_detection_row(panel) -> int:
    pipe_draft = str(panel._perception_pipeline_draft).strip().lower().replace("-", "_")
    pipeline_idx = 1 if pipe_draft in ("search_track", "track") else 0
    _control_label(panel, "Detection")
    if _mode_button(
        panel,
        "Every frame",
        "detection_every_frame",
        pipeline_idx == 0,
        width=_DETECTION_EVERY_BUTTON_W,
    ):
        pipeline_idx = 0
        panel._perception_pipeline_draft = "yolo_seg"
    imgui.same_line()
    if _mode_button(
        panel,
        "Once + Tracker",
        "detection_once_tracker",
        pipeline_idx == 1,
        width=_DETECTION_TRACK_BUTTON_W,
    ):
        pipeline_idx = 1
        panel._perception_pipeline_draft = "search_track"
    return int(pipeline_idx)


def _draw_tracker_row(panel, *, disabled: bool) -> None:
    tracker = str(panel._perception_tracker_draft).strip().lower()
    tracker_idx = 1 if tracker == "kcf" else 0
    _control_label(panel, "Tracker")
    if _mode_button(panel, "CSRT", "tracker_csrt", tracker_idx == 0, disabled=disabled):
        panel._perception_tracker_draft = "csrt"
    imgui.same_line()
    if _mode_button(panel, "KCF", "tracker_kcf", tracker_idx == 1, disabled=disabled):
        panel._perception_tracker_draft = "kcf"


def _browse_detector_config_path(initial_path: str) -> str | None:
    try:
        import tkinter as tk
        from tkinter import filedialog

        root_path = _project_root()
        default_dir = root_path / "model_presets" / "visual_servoing"
        initial = str(initial_path or "").strip()
        initial_path_obj = Path(initial).expanduser() if initial else default_dir
        if not initial_path_obj.is_absolute():
            initial_path_obj = root_path / initial_path_obj
        initial_dir = initial_path_obj.parent if initial_path_obj.suffix else initial_path_obj
        if not initial_dir.is_dir():
            initial_dir = default_dir if default_dir.is_dir() else root_path

        root = tk.Tk()
        root.withdraw()
        root.update_idletasks()
        selected = filedialog.askopenfilename(
            title="Select detector config JSON",
            initialdir=str(initial_dir),
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
        )
        root.destroy()
        selected_path = Path(str(selected or "").strip())
        if not selected_path:
            return None
        try:
            return str(selected_path.resolve().relative_to(root_path.resolve()))
        except ValueError:
            return str(selected_path)
    except Exception:
        return None


def _draw_config_path_row(panel) -> None:
    _control_label(panel, "Config")
    browse_w = scaled(panel, _CONFIG_BROWSE_W)
    spacing_x = float(getattr(imgui.get_style().item_spacing, "x", scaled(panel, 8.0)))
    input_w = max(1.0, _field_width(panel) - browse_w - spacing_x)
    imgui.push_item_width(input_w)
    try:
        changed_path, path_draft = imgui.input_text(
            "##detector_config",
            str(panel._perception_config_path_draft),
            256,
        )
    finally:
        imgui.pop_item_width()
    if changed_path:
        panel._perception_config_path_draft = str(path_draft).strip()
    imgui.same_line()
    if imgui.button("Browse", browse_w, 0.0):
        selected = _browse_detector_config_path(panel._perception_config_path_draft)
        if selected:
            panel._perception_config_path_draft = str(selected).strip()


def _draw_ball_xyz_row(panel) -> None:
    _control_label(panel, "Ball xyz")
    x, y, z = panel.state.mock_object_world_xyz()
    spacing_x = float(getattr(imgui.get_style().item_spacing, "x", scaled(panel, 8.0)))
    move_w = scaled(panel, _BALL_MOVE_W)
    input_w = max(scaled(panel, 36.0), (_field_width(panel) - move_w - spacing_x * 3.0) / 3.0)
    changed = False
    values = []
    for idx, value in enumerate((x, y, z)):
        if idx > 0:
            imgui.same_line()
        imgui.push_item_width(input_w)
        try:
            changed_i, value_i = imgui.input_float(
                f"##ball_xyz_{idx}",
                float(value),
                0.0,
                0.0,
                format="%.3f",
            )
        finally:
            imgui.pop_item_width()
        changed = bool(changed or changed_i)
        values.append(float(value_i))
    imgui.same_line()
    if imgui.button("Move", move_w, 0.0):
        panel.service.send_sim_target_xyz(float(values[0]), float(values[1]), float(values[2]))
    elif changed:
        panel.state.set_mock_object_world_xyz(float(values[0]), float(values[1]), float(values[2]))


def _xy(value) -> tuple[float, float]:
    if hasattr(value, "x") and hasattr(value, "y"):
        return float(value.x), float(value.y)
    return float(value[0]), float(value[1])


def _color_u32(r: float, g: float, b: float, a: float = 1.0) -> int:
    getter = getattr(imgui, "get_color_u32_rgba", None)
    if callable(getter):
        return int(getter(float(r), float(g), float(b), float(a)))
    ri = max(0, min(255, int(float(r) * 255.0)))
    gi = max(0, min(255, int(float(g) * 255.0)))
    bi = max(0, min(255, int(float(b) * 255.0)))
    ai = max(0, min(255, int(float(a) * 255.0)))
    return (ai << 24) | (bi << 16) | (gi << 8) | ri


def _draw_rect_filled(draw_list, x1: float, y1: float, x2: float, y2: float, color: int, rounding: float = 0.0) -> None:
    for args in (
        ((x1, y1), (x2, y2), color, float(rounding)),
        (x1, y1, x2, y2, color, float(rounding)),
        ((x1, y1), (x2, y2), color),
        (x1, y1, x2, y2, color),
    ):
        try:
            draw_list.add_rect_filled(*args)
            return
        except TypeError:
            continue


def _draw_line(draw_list, x1: float, y1: float, x2: float, y2: float, color: int, thickness: float = 1.0) -> None:
    for args in (
        ((x1, y1), (x2, y2), color, float(thickness)),
        (x1, y1, x2, y2, color, float(thickness)),
        ((x1, y1), (x2, y2), color),
        (x1, y1, x2, y2, color),
    ):
        try:
            draw_list.add_line(*args)
            return
        except TypeError:
            continue


def _draw_triangle_filled(draw_list, points: tuple[tuple[float, float], tuple[float, float], tuple[float, float]], color: int) -> None:
    p1, p2, p3 = points
    for args in (
        (p1, p2, p3, color),
        (p1[0], p1[1], p2[0], p2[1], p3[0], p3[1], color),
    ):
        try:
            draw_list.add_triangle_filled(*args)
            return
        except TypeError:
            continue


def _draw_circle_filled(draw_list, x: float, y: float, radius: float, color: int) -> None:
    for args in (
        ((x, y), float(radius), color, 32),
        (x, y, float(radius), color, 32),
        ((x, y), float(radius), color),
        (x, y, float(radius), color),
    ):
        try:
            draw_list.add_circle_filled(*args)
            return
        except TypeError:
            continue


def _draw_refresh_icon(draw_list, x: float, y: float, w: float, h: float, color: int) -> None:
    size = min(float(w), float(h))
    cx = float(x) + float(w) * 0.5
    cy = float(y) + float(h) * 0.52
    radius = size * 0.27
    thickness = max(1.5, size * 0.075)
    points: list[tuple[float, float]] = []
    start_deg = 35.0
    end_deg = 315.0
    steps = 28
    for i in range(steps + 1):
        t = i / float(steps)
        a = math.radians(start_deg + (end_deg - start_deg) * t)
        points.append((cx + math.cos(a) * radius, cy + math.sin(a) * radius))
    for p1, p2 in zip(points, points[1:]):
        _draw_line(draw_list, p1[0], p1[1], p2[0], p2[1], color, thickness)

    anchor = points[-1]
    prev = points[-2]
    dx = anchor[0] - prev[0]
    dy = anchor[1] - prev[1]
    mag = max((dx * dx + dy * dy) ** 0.5, 1e-6)
    ux = dx / mag
    uy = dy / mag
    px = -uy
    py = ux
    head_len = size * 0.28
    head_half = size * 0.15
    head_y_nudge = size * 0.045
    tip = (anchor[0] + ux * head_len * 0.55, anchor[1] + uy * head_len * 0.55 + head_y_nudge)
    base_center = (tip[0] - ux * head_len, tip[1] - uy * head_len)
    _draw_triangle_filled(
        draw_list,
        (
            tip,
            (base_center[0] + px * head_half, base_center[1] + py * head_half),
            (base_center[0] - px * head_half, base_center[1] - py * head_half),
        ),
        color,
    )


def _refresh_button(panel, label: str) -> bool:
    width = scaled(panel, _REFRESH_BUTTON_W)
    frame_h = getattr(imgui, "get_frame_height", None)
    height = float(frame_h()) if callable(frame_h) else scaled(panel, _BUTTON_H)
    if not callable(getattr(imgui, "invisible_button", None)) or not callable(getattr(imgui, "get_window_draw_list", None)):
        return bool(imgui.button(f"R##{label}", width, 0.0))

    x, y = _xy(imgui.get_cursor_screen_pos())
    clicked = bool(imgui.invisible_button(f"##{label}", float(width), float(height)))
    active = bool(imgui.is_item_active())
    hovered = bool(getattr(imgui, "is_item_hovered", lambda: False)())
    draw_list = imgui.get_window_draw_list()

    fill = (0.66, 0.70, 0.76) if active else (0.76, 0.79, 0.84) if hovered else (0.82, 0.84, 0.87)
    bg = _color_u32(*fill, 1.0)
    fg = _color_u32(0.12, 0.14, 0.17, 1.0)
    _draw_rect_filled(draw_list, x, y, x + width, y + height, bg, scaled(panel, 4.0))
    _draw_refresh_icon(draw_list, x, y, width, height, fg)
    return clicked


def _draw_preview_checkbox_right(panel) -> None:
    row_w = max(1.0, float(imgui.get_content_region_available_width()))
    start_x = float(getattr(imgui, "get_cursor_pos_x", lambda: 0.0)())
    set_x = getattr(imgui, "set_cursor_pos_x", None)
    checkbox_w = float(getattr(imgui, "get_frame_height", lambda: scaled(panel, _BUTTON_H))())
    text_w = float(imgui.calc_text_size("Preview")[0]) if callable(getattr(imgui, "calc_text_size", None)) else scaled(panel, 52.0)
    spacing_x = float(getattr(imgui.get_style().item_spacing, "x", scaled(panel, 8.0)))
    group_w = checkbox_w + spacing_x + text_w
    target_x = start_x + max(0.0, row_w - group_w)
    if callable(set_x):
        set_x(target_x)
    changed_preview, show_preview = imgui.checkbox(
        "##show_preview",
        bool(panel._perception_show_preview_draft),
    )
    if changed_preview:
        panel._perception_show_preview_draft = bool(show_preview)
    imgui.same_line()
    imgui.text("Preview")


def _draw_camera_controls(panel, *, running: bool) -> None:
    _control_label(panel, "Camera")
    if toggle_switch(
        panel,
        "vision_camera_switch",
        bool(running),
    ):
        if running:
            panel.service.stop_perception_capture()
        else:
            cfg = _build_perception_config(panel)
            panel.service.update_perception_config(cfg)
            panel.service.start_perception_capture(config=cfg)
    imgui.same_line()
    if _refresh_button(panel, "vision_refresh"):
        panel.service.refresh_perception_capture()
    imgui.same_line()
    _draw_preview_checkbox_right(panel)


def _draw_tracking_controls(panel) -> None:
    if _button(panel, "Track Target##visual_gaze_stand", _TRACK_BUTTON_W):
        panel.service.start_gaze_stabilizer_standing()
    imgui.same_line()
    if _button(panel, "Walk + Track##visual_gaze_walk", _TRACK_BUTTON_W):
        panel.service.start_gaze_stabilizer_walking()
    imgui.same_line()
    if _button(panel, "Stop Tracking##visual_stop_gaze", _TRACK_BUTTON_W):
        panel.service.stop_gaze_stabilizer()
    if _button(panel, "Stop + Grasp##visual_demo4", _TRACK_DEMO_BUTTON_W):
        panel.service.start_demo4_stop_and_grasp()


def _pick_action_button(panel, label: str, item_id: str, width: float) -> bool:
    return bool(imgui.button(f"{label}##pick_{item_id}", float(width), scaled(panel, _BUTTON_H)))


def _draw_pick_play_button(panel, *, disabled: bool) -> bool:
    size = scaled(panel, _BUTTON_H)
    if not callable(getattr(imgui, "invisible_button", None)) or not callable(getattr(imgui, "get_window_draw_list", None)):
        return (not disabled) and bool(imgui.button("Play##pick_full", size, size))

    x, y = _xy(imgui.get_cursor_screen_pos())
    clicked = bool(imgui.invisible_button("##pick_full_play", float(size), float(size)))
    active = bool(imgui.is_item_active())
    hovered = bool(getattr(imgui, "is_item_hovered", lambda: False)())
    draw_list = imgui.get_window_draw_list()

    if disabled:
        bg_fill = (0.72, 0.75, 0.80)
        fg_fill = (0.48, 0.51, 0.56)
    else:
        bg_fill = (0.66, 0.70, 0.76) if active else (0.76, 0.79, 0.84) if hovered else (0.82, 0.84, 0.87)
        fg_fill = (0.12, 0.14, 0.17)
    _draw_rect_filled(
        draw_list,
        x,
        y,
        x + size,
        y + size,
        _color_u32(*bg_fill, 1.0),
        scaled(panel, 4.0),
    )
    cx = x + size * 0.52
    cy = y + size * 0.50
    tri_w = size * 0.38
    tri_h = size * 0.48
    _draw_triangle_filled(
        draw_list,
        (
            (cx - tri_w * 0.42, cy - tri_h * 0.50),
            (cx - tri_w * 0.42, cy + tri_h * 0.50),
            (cx + tri_w * 0.58, cy),
        ),
        _color_u32(*fg_fill, 1.0),
    )
    return (not disabled) and clicked


def _draw_pick_stop_button(panel, *, disabled: bool) -> bool:
    size = scaled(panel, _BUTTON_H)
    if not callable(getattr(imgui, "invisible_button", None)) or not callable(getattr(imgui, "get_window_draw_list", None)):
        return (not disabled) and bool(imgui.button("Stop##pick_stop", size, size))

    x, y = _xy(imgui.get_cursor_screen_pos())
    clicked = bool(imgui.invisible_button("##pick_stop_button", float(size), float(size)))
    active = bool(imgui.is_item_active())
    hovered = bool(getattr(imgui, "is_item_hovered", lambda: False)())
    draw_list = imgui.get_window_draw_list()

    if disabled:
        bg_fill = (0.72, 0.75, 0.80)
        fg_fill = (0.48, 0.51, 0.56)
    else:
        bg_fill = (0.66, 0.70, 0.76) if active else (0.76, 0.79, 0.84) if hovered else (0.82, 0.84, 0.87)
        fg_fill = (0.12, 0.14, 0.17)
    _draw_rect_filled(
        draw_list,
        x,
        y,
        x + size,
        y + size,
        _color_u32(*bg_fill, 1.0),
        scaled(panel, 4.0),
    )
    mark = size * 0.34
    cx = x + size * 0.50
    cy = y + size * 0.50
    _draw_rect_filled(
        draw_list,
        cx - mark * 0.50,
        cy - mark * 0.50,
        cx + mark * 0.50,
        cy + mark * 0.50,
        _color_u32(*fg_fill, 1.0),
        scaled(panel, 1.0),
    )
    return (not disabled) and clicked


def _pick_progress_done_flags(panel) -> tuple[bool, bool, bool]:
    phase = str(getattr(panel.state, "pick_phase", "")).strip().lower()
    msg = str(getattr(panel.state, "pick_status_msg", "")).strip().lower()

    if phase in ("failed", "idle") or "stopped" in msg:
        return (False, False, False)
    if "e2e done" in msg or "grasp done" in msg:
        return (True, True, True)
    if "aim done" in msg:
        return (True, True, False)
    if "look done" in msg:
        return (True, False, False)
    if "e2e: grasp" in msg or phase in ("grasp", "grasp_approach", "approach", "extend"):
        return (True, True, False)
    if "e2e: aim" in msg or phase in ("acquire", "center"):
        return (True, False, False)
    if phase == "done":
        return (True, True, True)
    return (False, False, False)


def _draw_pick_progress_bar(panel, *, width: float, height: float, stage_w: float, spacing_x: float) -> None:
    done = _pick_progress_done_flags(panel)
    x, y = _xy(imgui.get_cursor_screen_pos())
    if callable(getattr(imgui, "dummy", None)):
        imgui.dummy(float(width), float(height))
    else:
        imgui.invisible_button("##pick_progress_spacer", float(width), float(height))
    draw_list = imgui.get_window_draw_list() if callable(getattr(imgui, "get_window_draw_list", None)) else None
    if draw_list is None:
        return

    center_y = y + float(height) * 0.5
    centers = [
        x + stage_w * 0.5,
        x + stage_w * 1.5 + spacing_x,
        x + stage_w * 2.5 + spacing_x * 2.0,
    ]
    radius = max(scaled(panel, 4.8), min(float(height) * 0.22, scaled(panel, 7.0)))
    line_color = _color_u32(0.64, 0.67, 0.72, 1.0)
    done_color = _color_u32(0.13, 0.62, 0.30, 1.0)
    pending_fill = _color_u32(0.95, 0.96, 0.98, 1.0)
    pending_border = _color_u32(0.54, 0.57, 0.62, 1.0)
    line_thickness = max(scaled(panel, 2.0), 1.5)

    for idx in range(2):
        color = done_color if done[idx] and done[idx + 1] else line_color
        _draw_line(
            draw_list,
            centers[idx] + radius,
            center_y,
            centers[idx + 1] - radius,
            center_y,
            color,
            line_thickness,
        )

    for idx, center_x in enumerate(centers):
        if done[idx]:
            _draw_circle_filled(draw_list, center_x, center_y, radius + scaled(panel, 1.0), done_color)
            _draw_circle_filled(draw_list, center_x, center_y, radius * 0.72, done_color)
        else:
            _draw_circle_filled(draw_list, center_x, center_y, radius + scaled(panel, 1.0), pending_border)
            _draw_circle_filled(draw_list, center_x, center_y, radius, pending_fill)


def _draw_pick_actions(panel, cfg: PerceptionConfig, *, pick_running: bool) -> None:
    spacing_x = float(getattr(imgui.get_style().item_spacing, "x", scaled(panel, 8.0)))
    available = max(1.0, float(imgui.get_content_region_available_width()))
    play_size = scaled(panel, _BUTTON_H)
    stop_size = play_size
    stage_area_w = max(1.0, available - play_size - stop_size - spacing_x * 2.0)
    stage_w = max(1.0, (stage_area_w - spacing_x * 2.0) / 3.0)
    stage_area_w = stage_w * 3.0 + spacing_x * 2.0
    stage_button_w = max(1.0, stage_w * 0.78)
    button_pad = max(0.0, (stage_w - stage_button_w) * 0.5)

    start_x = float(getattr(imgui, "get_cursor_pos_x", lambda: 0.0)())
    if _draw_pick_play_button(panel, disabled=pick_running):
        panel.service.update_perception_config(cfg)
        panel.service.start_look_aim_grasp_e2e()
    imgui.same_line()
    if _draw_pick_stop_button(panel, disabled=not pick_running):
        panel.service.stop_pick_e2e()
    imgui.same_line()
    _draw_pick_progress_bar(
        panel,
        width=stage_area_w,
        height=play_size,
        stage_w=stage_w,
        spacing_x=spacing_x,
    )

    set_cursor_x = getattr(imgui, "set_cursor_pos_x", None)
    row_x = start_x + play_size + stop_size + spacing_x * 2.0
    if callable(set_cursor_x):
        set_cursor_x(row_x + button_pad)

    disabled_token = begin_disabled_ui(pick_running)
    try:
        if _pick_action_button(panel, "Look", "look", stage_button_w) and not pick_running:
            panel.service.update_perception_config(cfg)
            panel.service.start_look()
        imgui.same_line()
        if callable(set_cursor_x):
            set_cursor_x(row_x + stage_w + spacing_x + button_pad)
        if _pick_action_button(panel, "Aim", "aim", stage_button_w) and not pick_running:
            panel.service.update_perception_config(cfg)
            panel.service.start_aim()
        imgui.same_line()
        if callable(set_cursor_x):
            set_cursor_x(row_x + stage_w * 2.0 + spacing_x * 2.0 + button_pad)
        if _pick_action_button(panel, "Grasp", "grasp", stage_button_w) and not pick_running:
            panel.service.update_perception_config(cfg)
            panel.service.start_grasp()
    finally:
        end_disabled_ui(disabled_token)


def _begin_section(label: str, item_id: str) -> bool:
    token = begin_collapsible_section(label, item_id, namespace="visual")
    setattr(_begin_section, "_last_token", token)
    return token is not None


def _end_section() -> None:
    token = getattr(_begin_section, "_last_token", None)
    end_collapsible_section(token)
    setattr(_begin_section, "_last_token", None)


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
    mode = str(panel._perception_mode_draft).strip().lower()
    provider = str(getattr(panel, "_perception_provider_draft", "local")).strip().lower() or "local"
    if mode == "sim":
        effective_provider = "local"
    elif provider not in ("local", "host"):
        effective_provider = "local"
    else:
        effective_provider = provider
        panel._perception_real_provider_draft = effective_provider
    run_local = effective_provider != "host"
    panel._perception_run_local = bool(run_local)
    if mode != "sim":
        panel._perception_provider_draft = effective_provider
    return PerceptionConfig(
        enabled=True,
        detector_config=str(panel._perception_config_path_draft),
        mode=mode,
        detector=(
            _local_detector_mode(str(panel._perception_detector_draft))
            if run_local
            else str(panel._perception_detector_draft)
        ),
        provider=effective_provider,
        preview_bind=str(getattr(panel, "_perception_preview_bind", "")),
        preview_endpoint=str(getattr(panel, "_perception_preview_endpoint", "")),
        preview_jpeg_quality=int(getattr(panel, "_perception_preview_jpeg_quality", 75)),
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

    _draw_config_path_row(panel)
    imgui.separator()

    if _begin_section("Camera", "camera"):
        _draw_camera_mode_row(panel)

        cfg_preview = _build_perception_config(panel)
        run_local = bool(cfg_preview.run_local)

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

        _draw_model_row(panel)
        pipeline_idx = _draw_detection_row(panel)
        _draw_tracker_row(panel, disabled=(pipeline_idx == 0))

        running = bool(panel.state.perception_running)
        _draw_camera_controls(panel, running=running)
        if run_local:
            _control_label(panel, "Debug")
            if imgui.button("Save Snapshot"):
                panel.service.capture_perception_frame()
        _end_section()

    imgui.separator()
    if _begin_section("Target", "target"):
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

        _draw_ball_xyz_row(panel)
        _end_section()

    imgui.separator()
    if _begin_section("Ready Pose", "ready_pose"):
        _draw_ready_pose_dir_editor(panel)
        _end_section()

    imgui.separator()
    if _begin_section("Pick", "pick"):
        pick_running = bool(panel.state.pick_running) or bool(panel.service.pick_e2e_running())
        cfg = _build_perception_config(panel)
        _draw_pick_actions(panel, cfg, pick_running=pick_running)
        _end_section()

    imgui.separator()
    if _begin_section("Tracking", "tracking"):
        _draw_tracking_controls(panel)
        _end_section()
