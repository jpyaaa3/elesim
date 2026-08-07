from __future__ import annotations

import math
import time
from pathlib import Path

import imgui

from engine.core.config_loader import PerceptionConfig
from engine.behaviors.gaze.stabilizer import gaze_config_to_dict
from ui.file_dialog import browse_open_file_path
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
_ROLL_SCAN_BUTTON_W = 128.0
_BALL_MOVE_W = 58.0
_GAZE_MODE_BUTTON_W = 92.0
_PICK_ACTION_W = 108.0
_MODEL_CONFIGS = {
    "yolo": "model_presets/visual_servoing/detector.yolo.example.json",
    "hsv": "model_presets/visual_servoing/detector.sim_hsv.json",
    "green_hsv": "model_presets/visual_servoing/detector.real_green_hsv.json",
}
_GAZE_REACTIVE_FIELDS = (
    ("uv_gain", "UV gain", 0.05, 8.0, "%.2f"),
    ("center_u_gain", "U gain", 0.0, 80.0, "%.1f"),
    ("center_v_gain", "V gain", 0.0, 80.0, "%.1f"),
    ("center_tol", "Tol", 0.0, 0.5, "%.3f"),
    ("center_seg_max", "Seg cap", 0.0, 20.0, "%.1f"),
    ("max_seg_du_per_tick", "Tick cap", 0.0, 20.0, "%.1f"),
    ("command_ref_max_lead", "Lead cap", 0.0, 120.0, "%.1f"),
    ("cmd_settle_s", "Settle", 0.0, 0.5, "%.3f"),
    ("center_v_kd", "V kd", 0.0, 30.0, "%.1f"),
    ("center_d_seg_max", "D cap", 0.0, 20.0, "%.1f"),
)
_GAZE_PREVIEW_FIELDS = (
    ("preview_b_pitch", "B pitch", -1.0, 1.0, "%.3f"),
    ("preview_tau_s", "Tau", 0.0, 1.0, "%.3f"),
    ("preview_r_s1", "R s1", 0.0001, 1.0, "%.4f"),
    ("preview_r_s2", "R s2", 0.0001, 1.0, "%.4f"),
    ("preview_max_du_seg", "Prev cap", 0.0, 20.0, "%.1f"),
    ("preview_lowpass_alpha", "LP alpha", 0.0, 1.0, "%.2f"),
)


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


def _gaze_config_signature(cfg: dict) -> str:
    try:
        return "|".join(f"{str(k)}={repr(cfg[k])}" for k in sorted(cfg.keys()))
    except Exception:
        return ""


def _sync_gaze_config_draft(panel) -> None:
    host = getattr(panel, "_host_state", None)
    host_cfg = getattr(host, "gaze_config", None) if host is not None else None
    if isinstance(host_cfg, dict) and host_cfg:
        cfg = dict(host_cfg)
        source = "Jetson"
    else:
        cfg = gaze_config_to_dict(panel.service.gaze_config)
        source = "local"
    sig = _gaze_config_signature(cfg)
    active = bool(getattr(imgui, "is_any_item_active", lambda: False)())
    pending_until = float(getattr(panel, "_gaze_config_pending_until", 0.0))
    if source == "Jetson" and time.time() < pending_until:
        return
    if sig and sig != str(getattr(panel, "_gaze_config_seen_signature", "")) and not active:
        panel._gaze_config_draft.update(cfg)
        panel._gaze_config_seen_signature = sig
        panel._gaze_config_last_source = source


def _apply_gaze_config_patch(panel, patch: dict[str, object]) -> None:
    if not patch:
        return
    try:
        cfg = panel.service.update_gaze_stabilizer_config(patch)
        panel._gaze_config_draft.update(dict(patch))
        panel._gaze_config_seen_signature = _gaze_config_signature(gaze_config_to_dict(cfg))
        panel._gaze_config_last_source = "local"
        panel._gaze_config_pending_until = time.time() + 0.5
    except Exception as exc:
        print(f"[gaze] config update failed: {exc}")


def _draw_gaze_bool(panel, key: str, label: str) -> None:
    draft = bool(panel._gaze_config_draft.get(key, False))
    _control_label(panel, label)
    changed, value = imgui.checkbox(f"##gaze_{key}", draft)
    if changed:
        _apply_gaze_config_patch(panel, {key: bool(value)})


def _draw_gaze_float(panel, key: str, label: str, lo: float, hi: float, fmt: str) -> None:
    raw = panel._gaze_config_draft.get(key, 0.0)
    try:
        value = float(raw)
    except (TypeError, ValueError):
        value = 0.0
    changed, next_value = _input_float(
        panel,
        label,
        f"gaze_{key}",
        float(value),
        step=0.0,
        step_fast=0.0,
        format=fmt,
    )
    if changed:
        clipped = max(float(lo), min(float(hi), float(next_value)))
        _apply_gaze_config_patch(panel, {key: clipped})


def _draw_gaze_mode_row(panel) -> None:
    mode = str(panel._gaze_config_draft.get("walking_gaze_mode", "uv") or "uv").strip().lower()
    _control_label(panel, "Walk mode")
    if _mode_button(panel, "UV", "gaze_mode_uv", mode == "uv", width=_GAZE_MODE_BUTTON_W):
        _apply_gaze_config_patch(panel, {"walking_gaze_mode": "uv"})
    imgui.same_line()
    if _mode_button(panel, "UV + FF", "gaze_mode_uvff", mode == "uv_ff", width=_GAZE_MODE_BUTTON_W):
        _apply_gaze_config_patch(panel, {"walking_gaze_mode": "uv_ff"})
    imgui.same_line()
    if _mode_button(
        panel,
        "Pitch",
        "gaze_mode_pitch",
        mode == "pitch_preview",
        width=_GAZE_MODE_BUTTON_W,
    ):
        _apply_gaze_config_patch(panel, {"walking_gaze_mode": "pitch_preview", "preview_enable": True})


def _draw_gaze_tuning_controls(panel) -> None:
    _sync_gaze_config_draft(panel)
    source = str(getattr(panel, "_gaze_config_last_source", "local"))
    imgui.text_disabled(f"Gaze config: {source}")
    _draw_gaze_mode_row(panel)
    _draw_gaze_float(panel, "hz", "Rate", 1.0, 60.0, "%.1f")
    _draw_gaze_bool(panel, "preview_enable", "Preview")
    _draw_gaze_bool(panel, "enable_base_ff", "Base FF")
    _draw_gaze_bool(panel, "command_ref_enable", "Cmd ref")
    imgui.separator()
    imgui.text_disabled("Reactive UV")
    for key, label, lo, hi, fmt in _GAZE_REACTIVE_FIELDS:
        _draw_gaze_float(panel, key, label, lo, hi, fmt)
    imgui.separator()
    imgui.text_disabled("Pitch preview")
    for key, label, lo, hi, fmt in _GAZE_PREVIEW_FIELDS:
        _draw_gaze_float(panel, key, label, lo, hi, fmt)


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
    if "real_green" in path or "green_hsv" in path:
        return "green_hsv"
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
    imgui.same_line()
    if _mode_button(panel, "Green", "model_green_hsv", model == "green_hsv"):
        _set_detector_model(panel, "green_hsv")


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


def _draw_actual_rate_row(panel, *, run_local: bool) -> None:
    hz = float(getattr(panel.state, "perception_hz", 0.0))
    host = getattr(panel, "_host_state", None)
    if not bool(run_local) and host is not None:
        hz = float(getattr(host, "perception_hz", hz))
    source = "local" if bool(run_local) else "Jetson"
    _control_label(panel, "Actual")
    imgui.text(f"{hz:.1f} Hz  {source}")


def browse_detector_config_path(initial_path: str) -> str | None:
    try:
        root_path = _project_root()
        default_dir = root_path / "model_presets" / "visual_servoing"
        initial = str(initial_path or "").strip()
        initial_path_obj = Path(initial).expanduser() if initial else default_dir
        if not initial_path_obj.is_absolute():
            initial_path_obj = root_path / initial_path_obj
        initial_dir = initial_path_obj.parent if initial_path_obj.suffix else initial_path_obj
        if not initial_dir.is_dir():
            initial_dir = default_dir if default_dir.is_dir() else root_path

        selected = browse_open_file_path(
            title="Select detector config JSON",
            initial_dir=str(initial_dir),
            extensions=(".json",),
        )
        if not selected:
            return None
        selected_path = Path(selected)
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
        panel.request_file_browse(kind="perception_detector", initial_path=panel._perception_config_path_draft)


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


def _draw_ball_dir_row(panel) -> None:
    _control_label(panel, "Ball dir")
    x, y, z = panel.state.mock_object_preferred_dir()
    spacing_x = float(getattr(imgui.get_style().item_spacing, "x", scaled(panel, 8.0)))
    set_w = scaled(panel, _BALL_MOVE_W)
    input_w = max(scaled(panel, 36.0), (_field_width(panel) - set_w - spacing_x * 3.0) / 3.0)
    changed = False
    values = []
    for idx, value in enumerate((x, y, z)):
        if idx > 0:
            imgui.same_line()
        imgui.push_item_width(input_w)
        try:
            changed_i, value_i = imgui.input_float(
                f"##ball_dir_{idx}",
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
    if imgui.button("Set##ball_dir", set_w, 0.0) or changed:
        panel.state.set_mock_object_preferred_dir(
            float(values[0]),
            float(values[1]),
            float(values[2]),
        )


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
    walk_mode = str(getattr(panel.service._gaze_cfg, "walking_gaze_mode", "uv_ff") or "uv_ff")
    imgui.text_disabled(f"Walk + Track uses gaze_walking_mode={walk_mode}")
    if _button(panel, "Stop + Pick##visual_demo4", _TRACK_DEMO_BUTTON_W):
        panel.service.start_demo4_stop_and_grasp()


def _sync_roll_scan_plan_text(panel) -> None:
    """
    Cache the plan line and the fitting-backend status.

    Both are static for a given config but involve a filesystem probe, so they
    are refreshed on a timer rather than every frame.
    """
    now = time.time()
    if now < float(getattr(panel, "_roll_scan_text_until", 0.0)):
        return
    panel._roll_scan_text_until = now + 5.0
    try:
        plan = panel.service.roll_scan_plan_text()
    except Exception as exc:  # noqa: BLE001
        plan = f"plan unavailable: {exc}"
    try:
        backend = panel.service.roll_scan_backend_status()
    except Exception as exc:  # noqa: BLE001
        backend = f"geometry backend: unknown ({exc})"
    if "MISSING" in backend or "incomplete" in backend:
        # say so before the button is pressed: the scan refuses to move the arm
        # when the fit cannot run, and the reason belongs next to the control
        plan = f"{plan}\n{backend}"
    panel._roll_scan_plan_text = plan


def _roll_scan_view(panel) -> dict:
    """
    Scan status, wherever it is running.

    A local scan writes straight into PanelState; a host-side scan arrives in
    HostState.roll_scan. Prefer whichever one says it is running, so the panel
    does not go blank the moment the scan moves to the Jetson.
    """
    st = panel.state
    local = {
        "running": bool(getattr(st, "roll_scan_running", False)),
        "phase": str(getattr(st, "roll_scan_phase", "idle") or "idle"),
        "msg": str(getattr(st, "roll_scan_msg", "") or ""),
        "stop_index": int(getattr(st, "roll_scan_stop_index", 0)),
        "n_stops": int(getattr(st, "roll_scan_n_stops", 0)),
        "sweep": int(getattr(st, "roll_scan_sweep", 0)),
        "n_sweeps": int(getattr(st, "roll_scan_n_sweeps", 0)),
        "roll_actual_deg": float(getattr(st, "roll_scan_roll_actual_deg", 0.0)),
        "frames_kept": int(getattr(st, "roll_scan_frames_kept", 0)),
        "points_kept": int(getattr(st, "roll_scan_points_kept", 0)),
        "diameter_mm": getattr(st, "roll_scan_diameter_mm", None),
        "arc_span_deg": getattr(st, "roll_scan_arc_span_deg", None),
        "residual_rms_mm": getattr(st, "roll_scan_residual_rms_mm", None),
        "surface": str(getattr(st, "roll_scan_surface", "") or ""),
        "source": "local",
    }
    host = getattr(panel, "_host_state", None)
    raw = getattr(host, "roll_scan", None) if host is not None else None
    if isinstance(raw, dict) and raw:
        remote = {
            "running": bool(raw.get("roll_scan_running", False)),
            "phase": str(raw.get("roll_scan_phase", "idle") or "idle"),
            "msg": str(raw.get("roll_scan_msg", "") or ""),
            "stop_index": int(raw.get("roll_scan_stop_index", 0) or 0),
            "n_stops": int(raw.get("roll_scan_n_stops", 0) or 0),
            "sweep": int(raw.get("roll_scan_sweep", 0) or 0),
            "n_sweeps": int(raw.get("roll_scan_n_sweeps", 0) or 0),
            "roll_actual_deg": float(raw.get("roll_scan_roll_actual_deg", 0.0) or 0.0),
            "frames_kept": int(raw.get("roll_scan_frames_kept", 0) or 0),
            "points_kept": int(raw.get("roll_scan_points_kept", 0) or 0),
            "diameter_mm": raw.get("roll_scan_diameter_mm", None),
            "arc_span_deg": raw.get("roll_scan_arc_span_deg", None),
            "residual_rms_mm": raw.get("roll_scan_residual_rms_mm", None),
            "surface": str(raw.get("roll_scan_surface", "") or ""),
            "source": "Jetson",
        }
        if remote["running"] or not local["running"]:
            return remote
    return local


def _draw_roll_scan_controls(panel) -> None:
    view = _roll_scan_view(panel)
    running = bool(view["running"])

    if _button(panel, "Scan Geometry##roll_scan_start", _ROLL_SCAN_BUTTON_W):
        panel.service.start_roll_scan()
    imgui.same_line()
    if _button(panel, "Stop Scan##roll_scan_stop", _ROLL_SCAN_BUTTON_W):
        panel.service.stop_roll_scan()

    imgui.text_disabled(str(getattr(panel, "_roll_scan_plan_text", "") or ""))

    phase = str(view["phase"])
    if running:
        n = int(view["n_stops"])
        i = int(view["stop_index"])
        frac = 0.0 if n <= 0 else min(1.0, i / float(n))
        bar = getattr(imgui, "progress_bar", None)
        overlay = f"{phase}  {i}/{n}"
        if callable(bar):
            try:
                bar(float(frac), (0.0, 0.0), overlay)
            except TypeError:  # older imgui binding without an overlay arg
                bar(float(frac))
                imgui.same_line()
                imgui.text(overlay)
        else:
            imgui.text(overlay)
        imgui.text(
            f"sweep {view['sweep']}/{view['n_sweeps']}   roll {view['roll_actual_deg']:+.1f} deg"
            f"   frames {view['frames_kept']}   pts {view['points_kept']}"
        )
    else:
        tag = "FAILED" if phase == "failed" else phase.upper()
        imgui.text(f"{tag}  ({view['source']})")

    d = view.get("diameter_mm")
    if d is not None:
        arc = view.get("arc_span_deg")
        rms = view.get("residual_rms_mm")
        imgui.text(
            f"d = {float(d):.1f} mm   arc {float(arc or 0.0):.0f} deg"
            f"   wall rms {float(rms or 0.0):.2f} mm   {view['surface']}"
        )
    msg = str(view["msg"])
    if msg:
        imgui.text_wrapped(msg)


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


def _pick_status_color(panel) -> tuple[float, float, float]:
    st = panel.state
    if bool(getattr(st, "pick_failed", False)):
        return (0.82, 0.18, 0.16)
    if bool(getattr(st, "pick_running", False)) or bool(panel.service.pick_e2e_running()):
        return (0.13, 0.48, 0.78)
    phase = str(getattr(st, "pick_phase", "")).strip().lower()
    if phase == "done":
        return (0.13, 0.62, 0.30)
    return (0.34, 0.36, 0.40)


def _pick_dashboard_value(label: str, value: str, *, color: tuple[float, float, float] | None = None) -> None:
    imgui.text(str(label))
    imgui.same_line()
    if color is None:
        imgui.text(str(value))
    else:
        imgui.text_colored(str(value), float(color[0]), float(color[1]), float(color[2]))


def _pick_dashboard_object_world(panel) -> tuple[float, float, float] | None:
    obj = getattr(panel.state, "perception_world_xyz", None)
    if obj is not None:
        return tuple(float(v) for v in obj)
    getter = getattr(panel.service, "_mobile_pick_object_world", None)
    if callable(getter):
        try:
            got = getter()
            if got is not None:
                return tuple(float(v) for v in got)
        except Exception:
            return None
    return None


def _pick_dashboard_base_pos(host) -> tuple[float, float, float] | None:
    if host is None:
        return None
    sim_pos = getattr(host, "go2_sim_base_pos", None)
    if sim_pos is not None:
        return tuple(float(v) for v in sim_pos)
    pos = getattr(host, "go2_base_pos", None)
    if pos is not None:
        return tuple(float(v) for v in pos)
    return None


def _pick_dashboard_distance_text(panel) -> str:
    host = getattr(panel, "_host_state", None)
    base = _pick_dashboard_base_pos(host)
    obj = _pick_dashboard_object_world(panel)
    if base is None or obj is None:
        return "-"
    dist = math.hypot(float(base[0]) - float(obj[0]), float(base[1]) - float(obj[1]))
    try:
        pk = panel.service._pick_config_effective()
        handoff_m = float(pk.mobile_handoff_distance_m)
        soft_m = handoff_m + max(float(getattr(pk, "mobile_handoff_timeout_slack_m", 0.0)), 0.0)
    except Exception:
        handoff_m = 0.0
        soft_m = 0.0
    if handoff_m > 1e-6:
        if soft_m > handoff_m + 1e-6:
            return "%.2fm / %.2fm (%.2fm)" % (float(dist), float(handoff_m), float(soft_m))
        return "%.2fm / %.2fm" % (float(dist), float(handoff_m))
    return "%.2fm" % float(dist)


def _pick_dashboard_uv_text(panel) -> str:
    uv = getattr(panel.state, "perception_center_uv", None)
    scale = float(getattr(panel.state, "perception_image_scale", 0.0) or 0.0)
    if uv is None:
        return "-"
    return "(%+.2f,%+.2f)  s=%.3f" % (float(uv[0]), float(uv[1]), scale)


def _pick_dashboard_go2_text(panel) -> str:
    host = getattr(panel, "_host_state", None)
    vel = getattr(host, "go2_vel", None) if host is not None else None
    if vel is None:
        return "-"
    return "vx=%+.2f vy=%+.2f wz=%+.2f" % (float(vel[0]), float(vel[1]), float(vel[2]))


def _draw_pick_dashboard(panel) -> None:
    st = panel.state
    phase = str(getattr(st, "pick_phase", "idle") or "idle")
    status = str(getattr(st, "pick_status_msg", "") or "")
    gaze = "running/%s" % str(getattr(st, "gaze_mode", "")) if bool(getattr(st, "gaze_running", False)) else "idle"
    tracker = "%s  ok=%d" % (
        str(getattr(st, "perception_tracker_phase", "")),
        int(getattr(st, "perception_track_ok_frames", 0)),
    )
    width = float(getattr(imgui, "get_content_region_available_width", lambda: 260.0)())
    col_w = max(scaled(panel, 130.0), (width - scaled(panel, 12.0)) * 0.5)
    _pick_dashboard_value("Phase", phase, color=_pick_status_color(panel))
    imgui.same_line(col_w)
    _pick_dashboard_value("Distance", _pick_dashboard_distance_text(panel))
    _pick_dashboard_value("Target", _pick_dashboard_uv_text(panel))
    imgui.same_line(col_w)
    _pick_dashboard_value("Tracker", tracker)
    _pick_dashboard_value("Gaze", gaze)
    imgui.same_line(col_w)
    _pick_dashboard_value("GO2", _pick_dashboard_go2_text(panel))
    if status:
        _pick_dashboard_value("Status", status, color=_pick_status_color(panel))


def _pick_action_button(panel, label: str, width: float, *, disabled: bool) -> bool:
    disabled_token = begin_disabled_ui(bool(disabled))
    try:
        clicked = _button(panel, label, width)
    finally:
        end_disabled_ui(disabled_token)
    return (not bool(disabled)) and bool(clicked)


def _draw_pick_actions(panel, cfg: PerceptionConfig, *, pick_running: bool) -> None:
    if _pick_action_button(panel, "Walk + Grasp##pick_mobile", _PICK_ACTION_W, disabled=pick_running):
        panel.service.update_perception_config(cfg)
        panel.service.start_mobile_gaze_lji_pick_e2e()
    imgui.same_line()
    if _pick_action_button(panel, "LJI Grasp##pick_lji_only", _PICK_ACTION_W, disabled=pick_running):
        panel.service.update_perception_config(cfg)
        panel.service.start_lji_grasp_only()
    imgui.same_line()
    if _draw_pick_stop_button(panel, disabled=not pick_running):
        panel.service.stop_pick_e2e()
    _draw_pick_dashboard(panel)


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
        "View dist",
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
        _draw_actual_rate_row(panel, run_local=run_local)

        _draw_model_row(panel)
        pipeline_idx = _draw_detection_row(panel)
        _draw_tracker_row(panel, disabled=(pipeline_idx == 0))

        running = bool(panel.state.perception_running)
        _draw_camera_controls(panel, running=running)
        _control_label(panel, "Debug")
        changed_rec_overlay, rec_overlay = imgui.checkbox(
            "Record overlay##perception_record_overlay",
            bool(panel.state.perception_record_with_overlay),
        )
        if changed_rec_overlay:
            panel.state.set_perception_record_overlay(bool(rec_overlay))
        imgui.same_line()
        if imgui.button("Save Snapshot"):
            panel.service.capture_perception_frame()
        imgui.same_line()
        rec_label = "Stop Record" if bool(panel.state.perception_recording) else "Start Record"
        if imgui.button(rec_label):
            panel.service.toggle_perception_recording()
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
        _draw_ball_dir_row(panel)
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
        imgui.separator()
        _draw_gaze_tuning_controls(panel)
        _end_section()

    imgui.separator()
    if _begin_section("Geometry Scan", "roll_scan"):
        _sync_roll_scan_plan_text(panel)
        _draw_roll_scan_controls(panel)
        _end_section()
