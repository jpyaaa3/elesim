from __future__ import annotations

import math
import time

import imgui

from ui.helpers import begin_collapsible_section, end_collapsible_section, panel_header, scaled, section_title


_HOST_STALE_S = 2.0
_STATUS_LABEL_W = 82.0
_CURRENT_YELLOW_COLOR = (1.0, 0.67, 0.08)
_CURRENT_RED_COLOR = (1.0, 0.18, 0.18)
_DEFAULT_TEXT_COLOR = (0.10, 0.11, 0.13, 1.0)
_GO2_VEL_COLOR = (0.12, 0.44, 0.95)
_FLOAT3_LABEL_W = 94.0
_FLOAT3_WIDTH_SCALE = 0.92
_GO2_LEG_TORQUE_INDEX = {
    ("FL", "Hip"): 0,
    ("FL", "Thigh"): 1,
    ("FL", "Calf"): 2,
    ("FR", "Hip"): 3,
    ("FR", "Thigh"): 4,
    ("FR", "Calf"): 5,
    ("RL", "Hip"): 6,
    ("RL", "Thigh"): 7,
    ("RL", "Calf"): 8,
    ("RR", "Hip"): 9,
    ("RR", "Thigh"): 10,
    ("RR", "Calf"): 11,
}


def _imgui_scale() -> float:
    try:
        return max(0.1, float(getattr(imgui.get_io(), "font_global_scale", 1.0) or 1.0))
    except Exception:
        return 1.0


def _scaled_px(value: float) -> float:
    return float(value) * _imgui_scale()


def _fmt_xyz(vec: tuple[float, float, float] | None, *, signed: bool = False) -> str:
    if vec is None:
        return "-"
    if signed:
        return "(%+.3f, %+.3f, %+.3f)" % (float(vec[0]), float(vec[1]), float(vec[2]))
    return "(%.3f, %.3f, %.3f)" % (float(vec[0]), float(vec[1]), float(vec[2]))


def _normalized_xyz(vec: tuple[float, float, float] | None) -> tuple[float, float, float] | None:
    if vec is None:
        return None
    x, y, z = float(vec[0]), float(vec[1]), float(vec[2])
    norm = math.sqrt(x * x + y * y + z * z)
    if norm <= 1e-9:
        return None
    return (x / norm, y / norm, z / norm)


def _heartbeat_tag(last_update_s: float, *, active: bool, stale_s: float = 2.0) -> str:
    if not active:
        return "OFF"
    if float(last_update_s) <= 0.0:
        return "WAIT"
    age = max(0.0, time.time() - float(last_update_s))
    if age <= float(stale_s):
        return "LIVE"
    return f"STALE {age:.1f}s"


def _fmt_uv(uv: tuple[float, float] | None) -> str:
    if uv is None:
        return "-"
    return f"({float(uv[0]):+.3f}, {float(uv[1]):+.3f})"


def _blank(value: object) -> str:
    text = str(value or "").strip()
    return text if text else "-"


def _calc_text_size(text: str) -> tuple[float, float]:
    calc = getattr(imgui, "calc_text_size", None)
    if callable(calc):
        size = calc(str(text))
        if hasattr(size, "x") and hasattr(size, "y"):
            return float(size.x), float(size.y)
        return float(size[0]), float(size[1])
    return float(len(str(text)) * _scaled_px(8.0)), _scaled_px(14.0)


def _xy(pos) -> tuple[float, float]:
    if hasattr(pos, "x") and hasattr(pos, "y"):
        return float(pos.x), float(pos.y)
    return float(pos[0]), float(pos[1])


def _content_region_available_size() -> tuple[float, float] | None:
    getter = getattr(imgui, "get_content_region_available", None)
    if not callable(getter):
        return None
    try:
        return _xy(getter())
    except Exception:
        return None


def _color_u32(r: float, g: float, b: float, a: float = 1.0) -> int:
    getter = getattr(imgui, "get_color_u32_rgba", None)
    if callable(getter):
        return int(getter(float(r), float(g), float(b), float(a)))
    ri = max(0, min(255, int(float(r) * 255.0)))
    gi = max(0, min(255, int(float(g) * 255.0)))
    bi = max(0, min(255, int(float(b) * 255.0)))
    ai = max(0, min(255, int(float(a) * 255.0)))
    return (ai << 24) | (bi << 16) | (gi << 8) | ri


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


def _draw_text(draw_list, x: float, y: float, color: int, text: str) -> None:
    for args in (
        ((x, y), color, str(text)),
        (x, y, color, str(text)),
    ):
        try:
            draw_list.add_text(*args)
            return
        except TypeError:
            continue


def _style_text_color_u32() -> int:
    getter = getattr(imgui, "get_style_color_vec4", None)
    color_idx = getattr(imgui, "COLOR_TEXT", None)
    if callable(getter) and color_idx is not None:
        try:
            color = getter(color_idx)
            if hasattr(color, "x"):
                return _color_u32(color.x, color.y, color.z, color.w)
            return _color_u32(color[0], color[1], color[2], color[3])
        except Exception:
            pass
    return _color_u32(*_DEFAULT_TEXT_COLOR)


def _line(label: str, value: object, *, color: tuple[float, float, float] | None = None) -> None:
    text = f"{label}: {_blank(value)}"
    if color is None:
        imgui.text(text)
    else:
        imgui.text_colored(text, float(color[0]), float(color[1]), float(color[2]))


def _draw_collapsible_section(label: str, draw_fn, panel) -> None:
    item_id = str(label).lower().replace(" ", "_").replace("/", "_")
    token = begin_collapsible_section(label, item_id, namespace="status")
    if token is None:
        return
    try:
        draw_fn(panel)
    finally:
        end_collapsible_section(token)


def _text_value(value: object, *, color: tuple[float, float, float] | None = None) -> None:
    if color is None:
        imgui.text(_blank(value))
    else:
        imgui.text_colored(_blank(value), float(color[0]), float(color[1]), float(color[2]))


def _readonly_text_field(label: str, value: object, identifier: str) -> None:
    text = _blank(value)
    imgui.text(str(label))
    imgui.same_line(_scaled_px(_STATUS_LABEL_W))
    _readonly_text_value(text, identifier)


def _readonly_text_value(value: object, identifier: str) -> None:
    text = _blank(value)
    width_getter = getattr(imgui, "get_content_region_available_width", None)
    field_w = max(_scaled_px(80.0), float(width_getter()) if callable(width_getter) else _scaled_px(220.0))
    _readonly_text_box(text, identifier, field_w)


def _readonly_text_box(
    value: object,
    identifier: str,
    width: float,
    *,
    text_color: tuple[float, float, float] | None = None,
) -> None:
    text = _blank(value)
    flags = getattr(
        imgui,
        "INPUT_TEXT_READ_ONLY",
        getattr(imgui, "INPUT_TEXT_FLAGS_READ_ONLY", 1 << 14),
    )
    pushed = 0
    if text_color is not None:
        try:
            imgui.push_style_color(
                imgui.COLOR_TEXT,
                float(text_color[0]),
                float(text_color[1]),
                float(text_color[2]),
                1.0,
            )
            pushed += 1
        except Exception:
            pushed = 0
    imgui.push_item_width(float(width))
    try:
        try:
            imgui.input_text(f"##{identifier}", text, max(64, len(text) + 1), flags=flags)
        except TypeError:
            imgui.input_text(f"##{identifier}", text, max(64, len(text) + 1), flags)
    finally:
        imgui.pop_item_width()
        if pushed:
            imgui.pop_style_color(pushed)


def _right_align_next_field(field_width: float, available_width: float) -> None:
    set_x = getattr(imgui, "set_cursor_pos_x", None)
    get_x = getattr(imgui, "get_cursor_pos_x", None)
    if callable(set_x) and callable(get_x):
        set_x(float(get_x()) + max(0.0, float(available_width) - float(field_width)))


def _readonly_float3_field(
    panel,
    label: str,
    vec: tuple[float, float, float] | None,
    identifier: str,
    *,
    format: str = "%.3f",
    text_color: tuple[float, float, float] | None = None,
) -> None:
    imgui.text(str(label))
    imgui.same_line(scaled(panel, _FLOAT3_LABEL_W))
    width_getter = getattr(imgui, "get_content_region_available_width", None)
    available = max(1.0, float(width_getter()) if callable(width_getter) else 260.0)
    style = imgui.get_style()
    spacing = float(getattr(style, "item_spacing", (8.0, 0.0))[0])
    field_w = max(1.0, available * _FLOAT3_WIDTH_SCALE)
    _right_align_next_field(field_w, available)
    component_w = max(scaled(panel, 40.0), (field_w - spacing * 2.0) / 3.0)
    if vec is None:
        values = ("-", "-", "-")
    else:
        values = tuple(format % float(vec[idx]) for idx in range(3))
    for idx, value in enumerate(values):
        if idx > 0:
            imgui.same_line()
        _readonly_text_box(value, f"{identifier}_{idx}", component_w, text_color=text_color)


def _go2_forward_dir(rpy: tuple[float, float, float] | None) -> tuple[float, float, float] | None:
    if rpy is None:
        return None
    try:
        _, pitch, yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])
    except (TypeError, ValueError, IndexError):
        return None
    cp = math.cos(pitch)
    return _normalized_xyz((math.cos(yaw) * cp, math.sin(yaw) * cp, -math.sin(pitch)))


def _fmt_go2_yaw_rate(value: float) -> str:
    yaw = float(value)
    if abs(yaw) < 0.005:
        return "0.00"
    return "%.2f %s" % (abs(yaw), "CCW" if yaw > 0.0 else "CW")


def _zero_small(value: float, eps: float = 0.005) -> float:
    value_f = float(value)
    return 0.0 if abs(value_f) < float(eps) else value_f


def _fmt_duration_s(value: object) -> str:
    try:
        seconds = float(value)
    except (TypeError, ValueError):
        return "-"
    if not math.isfinite(seconds) or seconds < 0.0:
        return "-"
    if seconds < 60.0:
        return "%.2fs" % seconds
    minutes = int(seconds // 60.0)
    rem = seconds - minutes * 60.0
    return "%d:%05.2f" % (minutes, rem)


def _fmt_factor(value: object) -> str:
    try:
        factor = float(value)
    except (TypeError, ValueError):
        return "-"
    if not math.isfinite(factor) or factor <= 0.0:
        return "-"
    return "%.2fx" % factor


def _ui_local_sim_clock(panel, sim_time_s: float, sim_steps: int) -> tuple[float | None, float | None]:
    now = time.perf_counter()
    if int(sim_steps) <= 0 or float(sim_time_s) <= 0.0:
        panel._status_sim_clock_start_igt = None
        panel._status_sim_clock_start_wall = None
        panel._status_sim_clock_last_steps = None
        return None, None

    start_igt = getattr(panel, "_status_sim_clock_start_igt", None)
    start_wall = getattr(panel, "_status_sim_clock_start_wall", None)
    last_steps = getattr(panel, "_status_sim_clock_last_steps", None)
    if (
        start_igt is None
        or start_wall is None
        or last_steps is None
        or int(sim_steps) < int(last_steps)
        or float(sim_time_s) < float(start_igt)
    ):
        panel._status_sim_clock_start_igt = float(sim_time_s)
        panel._status_sim_clock_start_wall = float(now)
        panel._status_sim_clock_last_steps = int(sim_steps)
        return 0.0, None

    panel._status_sim_clock_last_steps = int(sim_steps)
    elapsed = max(0.0, float(now) - float(start_wall))
    if elapsed <= 1e-6:
        return elapsed, None
    rtf = max(0.0, (float(sim_time_s) - float(start_igt)) / elapsed)
    return elapsed, rtf


def _readonly_go2_velocity_field(
    panel,
    label: str,
    vel: tuple[float, float, float] | None,
    identifier: str,
) -> None:
    imgui.text(str(label))
    imgui.same_line(scaled(panel, _FLOAT3_LABEL_W))
    width_getter = getattr(imgui, "get_content_region_available_width", None)
    available = max(1.0, float(width_getter()) if callable(width_getter) else 280.0)
    style = imgui.get_style()
    spacing = float(getattr(style, "item_spacing", (8.0, 0.0))[0])
    field_w = max(1.0, available * _FLOAT3_WIDTH_SCALE)
    _right_align_next_field(field_w, available)
    component_w = max(scaled(panel, 40.0), (field_w - spacing * 2.0) / 3.0)
    if vel is None:
        values = ("-", "-", "-")
    else:
        values = (
            "%+.2f" % _zero_small(float(vel[0])),
            "%+.2f" % _zero_small(float(vel[1])),
            _fmt_go2_yaw_rate(_zero_small(float(vel[2]))),
        )
    for idx, value in enumerate(values):
        if idx > 0:
            imgui.same_line()
        _readonly_text_box(value, f"{identifier}_{idx}", component_w, text_color=_GO2_VEL_COLOR)


def _table_child_height(panel) -> float:
    getter = getattr(imgui, "get_text_line_height_with_spacing", None)
    line_h = float(getter()) if callable(getter) else scaled(panel, 20.0)
    return line_h * 2.0 + scaled(panel, 18.0)


def _compact_table_child_height(panel, rows: int) -> float:
    getter = getattr(imgui, "get_text_line_height_with_spacing", None)
    line_h = float(getter()) if callable(getter) else scaled(panel, 20.0)
    return line_h * int(rows) + scaled(panel, 46.0)


def _draw_columns_table(
    panel,
    identifier: str,
    headers: tuple[str, ...],
    values: tuple[object, ...],
    *,
    colors: tuple[tuple[float, float, float] | None, ...] | None = None,
    center: bool = False,
) -> None:
    columns = getattr(imgui, "columns", None)
    next_column = getattr(imgui, "next_column", None)
    if not callable(columns) or not callable(next_column):
        _line(" / ".join(headers), " / ".join(_blank(v) for v in values))
        return
    count = len(headers)
    colors = colors or tuple(None for _ in headers)
    flags = getattr(imgui, "WINDOW_NO_SCROLLBAR", 0) | getattr(imgui, "WINDOW_NO_SCROLL_WITH_MOUSE", 0)
    imgui.begin_child(str(identifier), 0.0, _table_child_height(panel), True, flags=flags)
    try:
        table_w = max(1.0, float(imgui.get_content_region_available_width()))
        col_w = table_w / max(1, count)
        cell_x0 = float(getattr(imgui, "get_cursor_pos_x", lambda: 0.0)())
        for args in ((count, f"{identifier}_cols", False), (count, f"{identifier}_cols"), (count,)):
            try:
                columns(*args)
                break
            except TypeError:
                continue
        else:
            _line(" / ".join(headers), " / ".join(_blank(v) for v in values))
            return
        for idx, header in enumerate(headers):
            if center:
                text_w, _ = _calc_text_size(str(header))
                imgui.set_cursor_pos_x(cell_x0 + col_w * idx + max(0.0, (col_w - text_w) * 0.5))
            imgui.text(str(header))
            next_column()
        imgui.separator()
        for idx, (value, color) in enumerate(zip(values, colors)):
            if center:
                text_w, _ = _calc_text_size(_blank(value))
                imgui.set_cursor_pos_x(cell_x0 + col_w * idx + max(0.0, (col_w - text_w) * 0.5))
            _text_value(value, color=color)
            next_column()
        for args in ((1, f"{identifier}_cols", False), (1, f"{identifier}_cols"), (1,)):
            try:
                columns(*args)
                break
            except TypeError:
                continue
    finally:
        imgui.end_child()


def _draw_go2_torque_table(
    panel,
    identifier: str,
    values: dict[tuple[str, str], object],
) -> None:
    draw_list_getter = getattr(imgui, "get_window_draw_list", None)
    if not callable(draw_list_getter):
        for left_leg, right_leg in (("FL", "FR"), ("RL", "RR")):
            text = "  ".join(
                f"{joint_label} {_blank(values.get((left_leg, joint_name)))} | "
                f"{_blank(values.get((right_leg, joint_name)))} {joint_label}"
                for joint_label, joint_name in (("H", "Hip"), ("T", "Thigh"), ("C", "Calf"))
            )
            _line(f"{left_leg}/{right_leg}", text)
        return

    row_count = 6
    pairs = (("FL", "FR"), ("RL", "RR"))
    joints = (("H", "Hip"), ("T", "Thigh"), ("C", "Calf"))
    flags = getattr(imgui, "WINDOW_NO_SCROLLBAR", 0) | getattr(imgui, "WINDOW_NO_SCROLL_WITH_MOUSE", 0)
    child_h = _compact_table_child_height(panel, row_count)
    imgui.begin_child(str(identifier), 0.0, child_h, True, flags=flags)
    try:
        available_size = _content_region_available_size()
        table_w = max(
            1.0,
            float(available_size[0]) if available_size is not None else float(imgui.get_content_region_available_width()),
        )
        col_weights = (0.54, 0.46, 1.32, 1.32, 0.46, 0.54)
        weight_total = sum(col_weights)
        pad_x = scaled(panel, 6.0)
        pad_y = scaled(panel, 4.0)
        content_w = max(1.0, table_w - pad_x * 2.0)
        available_h = float(available_size[1]) if available_size is not None else child_h - scaled(panel, 28.0)
        content_h = max(1.0, available_h - pad_y * 2.0)
        col_widths = tuple(content_w * weight / weight_total for weight in col_weights)
        col_offsets: list[float] = []
        offset = 0.0
        for col_width in col_widths:
            col_offsets.append(offset)
            offset += col_width
        screen_x0, screen_y0 = _xy(imgui.get_cursor_screen_pos())
        text_color = _style_text_color_u32()
        line_color = _color_u32(0.46, 0.48, 0.52, 0.85)
        line_thickness = scaled(panel, 1.0)
        row_h = content_h / row_count
        x0 = screen_x0 + pad_x
        x1 = x0 + content_w
        y0 = screen_y0 + pad_y
        y1 = y0 + content_h
        draw_list = draw_list_getter()

        def draw_cell(row_idx: int, col_idx: int, text: object, *, placeholder: bool = True) -> None:
            value = _blank(text) if placeholder else str(text or "")
            text_w, _ = _calc_text_size(value)
            _, text_h = _calc_text_size(value)
            cell_x = x0 + col_offsets[col_idx]
            cell_y = y0 + row_h * row_idx
            text_x = cell_x + max(0.0, (col_widths[col_idx] - text_w) * 0.5)
            text_y = cell_y + max(0.0, (row_h - text_h) * 0.5)
            _draw_text(draw_list, text_x, text_y, text_color, value)

        row_idx = 0
        for left_leg, right_leg in pairs:
            for joint_idx, (joint_label, joint_name) in enumerate(joints):
                draw_cell(row_idx, 0, left_leg if joint_idx == 1 else "", placeholder=False)
                draw_cell(row_idx, 1, joint_label, placeholder=False)
                draw_cell(row_idx, 2, values.get((left_leg, joint_name)))
                draw_cell(row_idx, 3, values.get((right_leg, joint_name)))
                draw_cell(row_idx, 4, joint_label, placeholder=False)
                draw_cell(row_idx, 5, right_leg if joint_idx == 1 else "", placeholder=False)
                row_idx += 1

        divider_xs = (
            x0 + sum(col_widths[:1]),
            x0 + sum(col_widths[:3]),
            x0 + sum(col_widths[:5]),
        )
        for x in divider_xs:
            _draw_line(draw_list, x, y0, x, y1, line_color, line_thickness)
        _draw_line(draw_list, x0, y0 + row_h * 3.0, x1, y0 + row_h * 3.0, line_color, line_thickness)
    finally:
        imgui.end_child()


def _fmt_ma(value: object) -> str:
    if value is None:
        return "-"
    try:
        return "%dmA" % int(value)
    except (TypeError, ValueError):
        return "-"


def _current_color(panel, value: object) -> tuple[float, float, float] | None:
    if value is None:
        return None
    try:
        current_abs = abs(int(value))
    except (TypeError, ValueError):
        return None
    red = abs(int(getattr(panel, "_current_limit_ma", 2500) or 0))
    yellow = abs(int(getattr(panel, "_current_yellow_ma", 1800) or 0))
    if red > 0 and current_abs > red:
        return _CURRENT_RED_COLOR
    if yellow > 0 and current_abs > yellow:
        return _CURRENT_YELLOW_COLOR
    return None


def _motor_current(host, *names: str) -> int | None:
    if host is None:
        return None
    currents = getattr(host, "motor_currents_ma", {}) or {}
    normalized = {
        str(key).strip().lower().replace("_", "").replace("-", ""): value
        for key, value in dict(currents).items()
    }
    for name in names:
        key = str(name).strip().lower().replace("_", "").replace("-", "")
        if key in normalized:
            try:
                return int(normalized[key])
            except (TypeError, ValueError):
                return None
    return None


def _fmt_go2_torque(value: object) -> str:
    if value is None:
        return "-"
    try:
        v = float(value)
    except (TypeError, ValueError):
        return "-"
    return "%.2fNm" % v


def _go2_joint_torque(host, leg: str, joint: str) -> float | None:
    if host is not None:
        torques = getattr(host, "go2_leg_torque_nm", None)
        idx = _GO2_LEG_TORQUE_INDEX.get((str(leg).strip().upper(), str(joint).strip()))
        if idx is not None and isinstance(torques, (list, tuple)) and len(torques) == 12:
            try:
                return float(torques[int(idx)])
            except (TypeError, ValueError, IndexError):
                pass
    return None


def _host_status(host) -> tuple[str, tuple[float, float, float] | None]:
    if host is None or not bool(getattr(host, "connected", False)):
        return "Offline", (0.02, 0.02, 0.02)
    try:
        rx_age = float(getattr(host, "rx_age_s", float("inf")))
    except (TypeError, ValueError):
        rx_age = float("inf")
    if not math.isfinite(rx_age):
        return "Waiting", (0.46, 0.48, 0.52)
    if rx_age > _HOST_STALE_S:
        return "Stopped (%.1fs)" % rx_age, (0.90, 0.12, 0.12)
    return "Online", (0.10, 0.55, 0.18)


def _draw_hardware_brief(panel) -> None:
    host = panel._host_state
    host_text, host_color = _host_status(host)
    device = getattr(host, "device", "") if host is not None else ""
    ports = tuple(getattr(host, "ports", ()) or ()) if host is not None else ()
    camera_on = bool(getattr(panel.state, "perception_running", False))
    camera_text = "On" if camera_on else "Off"
    camera_color = (0.10, 0.55, 0.18) if camera_on else (0.46, 0.48, 0.52)
    _draw_columns_table(
        panel,
        "hardware_host_table",
        ("Host", "Device", "Port", "Camera"),
        (host_text, device, ", ".join(str(p) for p in ports), camera_text),
        colors=(host_color, None, None, camera_color),
        center=True,
    )

    go2_rows = ("FL", "FR", "RL", "RR")
    go2_cols = ("Hip", "Thigh", "Calf")
    go2_torque_values = {
        (leg, joint): _fmt_go2_torque(_go2_joint_torque(host, leg, joint))
        for leg in go2_rows
        for joint in go2_cols
    }
    _draw_go2_torque_table(
        panel,
        "hardware_go2_torque_table",
        go2_torque_values,
    )

    gripper_current = _motor_current(host, "gripper", "claw")
    if gripper_current is None and host is not None:
        try:
            gripper_current = int(getattr(host, "claw_current", 0))
        except (TypeError, ValueError):
            gripper_current = None
    current_values = (
        _motor_current(host, "linear"),
        _motor_current(host, "roll"),
        _motor_current(host, "seg1", "s1"),
        _motor_current(host, "seg2", "s2"),
        gripper_current,
    )
    _draw_columns_table(
        panel,
        "hardware_current_table",
        ("Linear", "Roll", "Seg1", "Seg2", "Gripper"),
        tuple(_fmt_ma(value) for value in current_values),
        colors=tuple(_current_color(panel, value) for value in current_values),
        center=True,
    )

    reply_reason = str(getattr(host, "reply_reason", "") or "").strip()
    reply_ok = bool(getattr(host, "reply_ok", True)) if host is not None else True
    _line("Command status", reply_reason, color=(1.0, 0.35, 0.35) if reply_reason and not reply_ok else None)


def _draw_arm_brief(panel) -> None:
    host = panel._host_state
    tip_xyz = host.actual_tip_xyz if host is not None else None
    tip_dir = host.actual_tip_dir if host is not None else None
    _readonly_float3_field(panel, "Tip xyz", tip_xyz, "status_tip_xyz", format="%.3f")
    _readonly_float3_field(panel, "Tip dir", _normalized_xyz(tip_dir), "status_tip_dir", format="%+.3f")


def _draw_go2_brief(panel) -> None:
    host = panel._host_state
    pos = getattr(host, "go2_base_pos", None) if host is not None else None
    rpy = getattr(host, "go2_base_rpy", None) if host is not None else None
    lin_vel = getattr(host, "go2_base_lin_vel_body", None) if host is not None else None
    ang_vel = getattr(host, "go2_base_ang_vel", None) if host is not None else None
    vel = None
    if lin_vel is not None and ang_vel is not None:
        try:
            vel = (float(lin_vel[0]), float(lin_vel[1]), float(ang_vel[2]))
        except (TypeError, ValueError, IndexError):
            vel = None
    _readonly_float3_field(panel, "GO2 xyz", pos, "status_go2_xyz", format="%+.3f")
    _readonly_float3_field(panel, "GO2 dir", _go2_forward_dir(rpy), "status_go2_dir", format="%+.3f")
    _readonly_go2_velocity_field(panel, "GO2 vel", vel, "status_go2_vel")


def _draw_sim_brief(panel) -> None:
    host = panel._host_state
    sim_steps = int(getattr(host, "sim_step_count", 0) or 0) if host is not None else 0
    sim_time = float(getattr(host, "sim_time_s", 0.0) or 0.0) if host is not None else 0.0
    if host is None or (sim_steps <= 0 and sim_time <= 0.0):
        _ui_local_sim_clock(panel, 0.0, 0)
        _line("Sim IGT", "-")
        _line("Real elapsed", "-")
        _line("Sim RTF", "-")
        _line("Sim steps", "-")
        return
    real_elapsed, local_rtf = _ui_local_sim_clock(panel, sim_time, sim_steps)
    _line("Sim IGT", _fmt_duration_s(sim_time))
    _line("Real elapsed", _fmt_duration_s(real_elapsed))
    _line("Sim RTF", _fmt_factor(local_rtf))
    _line("Sim steps", str(sim_steps))


def _draw_ik_brief(panel) -> None:
    st = panel.state
    status = "idle"
    if bool(st.ik_running):
        status = "running"
    if bool(st.ik_converged):
        status = "converged"
    if bool(st.ik_failed):
        status = "failed"
    _line("IK", "%s  err=%.2fmm" % (status, float(st.ik_err_m) * 1000.0))
    _line(
        "IK solution",
        "roll=%.3f  theta1=%.3f  theta2=%.3f"
        % (float(st.ik_sol_roll), float(st.ik_sol_theta1), float(st.ik_sol_theta2)),
    )
    _line(
        "IK track error",
        "roll=%.3f  theta1=%.3f  theta2=%.3f  bend_max=%.3f"
        % (
            float(st.ik_track_roll_err_rad),
            float(st.ik_track_theta1_err_rad),
            float(st.ik_track_theta2_err_rad),
            float(st.ik_track_bend_max_err_rad),
        ),
    )
    _line("IK msg", st.ik_status_msg)


def _draw_pick_brief(panel) -> None:
    st = panel.state
    pick_running = bool(st.pick_running) or bool(panel.service.pick_e2e_running())
    pick_status = "running" if pick_running else "idle"
    if bool(st.pick_failed):
        pick_status = "failed"
    _line("Pick", "%s  phase=%s" % (pick_status, _blank(st.pick_phase)))
    _line("Pick msg", st.pick_status_msg)
    _line("Sag model", st.sag_model_path)
    _line("Sag status", getattr(panel, "_sag_status_text", ""))


def draw_live_visual_status(panel, *, show_separators: bool = True, show_title: bool = True) -> None:
    """Perception / host relay / gaze heartbeat shown at panel top."""
    st = panel.state
    now = time.time()
    run_local = bool(getattr(panel, "_perception_run_local", True))

    if show_separators:
        imgui.separator()
    if show_title:
        section_title("Vision / Gaze")
    _line("Perception source", "local" if run_local else "remote")
    _line("Detector config", getattr(panel, "_perception_config_path_draft", ""))
    _line(
        "Visual target",
        "label=%s  scale=%.3f  uv=(%.2f, %.2f)"
        % (
            _blank(st.visual_target_label),
            float(st.visual_target_scale),
            float(st.visual_target_uv_u),
            float(st.visual_target_uv_v),
        ),
    )

    host = panel._host_state
    host_age = -1.0
    host_live = False
    if host is not None and bool(getattr(host, "connected", False)):
        if float(host.perceived_timestamp_s) > 0.0:
            host_age = max(0.0, now - float(host.perceived_timestamp_s))
        host_live = host.perceived_center_uv is not None and host_age >= 0.0 and host_age <= 0.75

    perc_active = bool(st.perception_running)
    if not run_local and host_live:
        perc_active = True
    perc_tag = _heartbeat_tag(st.perception_last_update_s, active=perc_active)
    if st.perception_failed:
        perc_tag = "FAILED"
    elif not perc_active:
        perc_tag = "OFF"
    elif not run_local:
        perc_tag = "REMOTE"

    _line(
        "Perception",
        "[%s]  frame=%d  center_uv=%s  det=%s  conf=%.2f"
        % (
            perc_tag,
            int(st.perception_frame_idx),
            _fmt_uv(st.perception_center_uv),
            str(st.perception_label) or "(none)",
            float(st.perception_confidence),
        ),
    )
    bw = st.perception_bbox_wh
    _line(
        "Perception track",
        "phase=%s  ok=%d  scale=%.3f  bbox=%dx%d  backend=%s"
        % (
            _blank(st.perception_tracker_phase),
            int(st.perception_track_ok_frames),
            float(st.perception_image_scale),
            int(bw[0]),
            int(bw[1]),
            _blank(st.perception_tracker_backend),
        ),
    )
    _line("Camera XYZ [m]", _fmt_xyz(st.perception_camera_xyz, signed=True))
    _line("World XYZ [m]", _fmt_xyz(st.perception_world_xyz, signed=True))
    _line("Last capture", st.perception_last_capture_path)
    _line(
        "Recording",
        ("ON  " if bool(st.perception_recording) else "OFF ")
        + ("[overlay] " if bool(st.perception_record_with_overlay) else "[raw] ")
        + (str(st.perception_last_record_path) if str(st.perception_last_record_path).strip() else "-"),
    )
    _line("Perception msg", st.perception_status_msg)

    if host is None or not bool(getattr(host, "connected", False)):
        relay_text = "[OFF]  uv=-  scale=-  label=-  age=-"
    else:
        host_age = -1.0
        if float(host.perceived_timestamp_s) > 0.0:
            host_age = max(0.0, now - float(host.perceived_timestamp_s))
        local_age = -1.0
        if float(st.perception_last_update_s) > 0.0:
            local_age = max(0.0, now - float(st.perception_last_update_s))
        host_tag = "OFF"
        if host.perceived_center_uv is not None and host_age >= 0.0 and host_age <= 0.75:
            host_tag = "LIVE"
        elif host.perceived_center_uv is not None and host_age > 0.75:
            host_tag = f"STALE {host_age:.1f}s"
        elif host_age >= 0.0:
            host_tag = f"WAIT {host_age:.1f}s"
        if (
            run_local
            and perc_active
            and local_age >= 0.0
            and local_age <= 0.75
            and host_age > 0.75
            and st.perception_center_uv is not None
        ):
            host_tag = f"{host_tag} (local ok {local_age:.1f}s)"
        scale_str = "-" if host.perceived_scale is None else f"{float(host.perceived_scale):.3f}"
        age_str = "-" if host_age < 0.0 else "%.2fs" % float(host_age)
        relay_text = "[%s]  uv=%s  scale=%s  label=%s  age=%s" % (
            host_tag,
            _fmt_uv(host.perceived_center_uv),
            scale_str,
            str(host.perceived_object_label) or "(none)",
            age_str,
        )
    _line("Host relay", relay_text)

    gaze_tag = "RUNNING" if bool(st.gaze_running) else "OFF"
    _line(
        "Gaze",
        "[%s]  mode=%s  updates=%d  u_err=%+.3f  v_err=%+.3f"
        % (
            gaze_tag,
            str(st.gaze_mode) or "idle",
            int(st.gaze_update_count),
            float(st.gaze_u_err),
            float(st.gaze_v_err),
        ),
    )
    _line(
        "Gaze delta",
        "du roll/s1/s2=%+.4f / %+.4f / %+.4f  obs_age=%.2fs  target_uv=(%.2f, %.2f)"
        % (
            float(st.gaze_du_roll),
            float(st.gaze_du_s1),
            float(st.gaze_du_s2),
            float(st.gaze_obs_age_s),
            float(st.visual_target_uv_u),
            float(st.visual_target_uv_v),
        )
    )
    _line("Gaze msg", st.gaze_status_msg)

    needed = (
        (run_local and not st.perception_running)
        or st.perception_center_uv is None
        or (host is not None and host.perceived_center_uv is None)
    )
    note = ""
    if bool(st.gaze_running) and needed:
        if run_local:
            note = (
                "Gaze needs Perception Start + host UV relay. "
                "Check sim camera, target label, and that a target is visible."
            )
        else:
            note = (
                "Gaze needs Jetson perception_worker + host UV relay. "
                "Check RealSense, target label, and worker process on Jetson."
            )
    elif host is not None and run_local and perc_active and st.perception_center_uv is not None:
        host_age_note = -1.0
        if float(getattr(host, "perceived_timestamp_s", 0.0)) > 0.0:
            host_age_note = max(0.0, now - float(host.perceived_timestamp_s))
        if host_age_note > 0.75:
            note = "Perception is live locally but host relay is stale."
    _line("Vision note", note)

    if show_separators:
        imgui.separator()


def _draw_status_sections_single(panel) -> None:
    _draw_collapsible_section("Hardware", _draw_hardware_brief, panel)
    imgui.separator()
    _draw_collapsible_section("Sim", _draw_sim_brief, panel)
    imgui.separator()
    _draw_collapsible_section("Arm", _draw_arm_brief, panel)
    imgui.separator()
    _draw_collapsible_section("GO2", _draw_go2_brief, panel)
    imgui.separator()
    _draw_collapsible_section("IK", _draw_ik_brief, panel)
    imgui.separator()
    _draw_collapsible_section("Pick / Sag", _draw_pick_brief, panel)
    imgui.separator()
    _draw_collapsible_section(
        "Vision / Gaze",
        lambda p: draw_live_visual_status(p, show_separators=False, show_title=False),
        panel,
    )


def _draw_ui_resolution_controls(panel) -> None:
    presets_fn = getattr(panel, "ui_resolution_presets", None)
    scale_fn = getattr(panel, "ui_resolution_scale", None)
    requested_scale_fn = getattr(panel, "ui_resolution_requested_scale", None)
    set_scale = getattr(panel, "set_ui_resolution_scale", None)
    radio_button = getattr(imgui, "radio_button", None)
    if not callable(presets_fn) or not callable(scale_fn) or not callable(set_scale) or not callable(radio_button):
        return
    presets = tuple(presets_fn())
    if not presets:
        return
    current_fn = requested_scale_fn if callable(requested_scale_fn) else scale_fn
    current = float(current_fn())
    imgui.text("UI Resolution")
    for idx, (label, scale) in enumerate(presets):
        if idx > 0:
            imgui.same_line()
        active = abs(float(scale) - current) <= 1e-6
        clicked = bool(radio_button(f"{label}##ui_resolution_{idx}", active))
        if clicked:
            set_scale(float(scale))


def draw_status_panel(panel) -> None:
    if not panel._status_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._status_header_init_open = True
    if not panel_header("Status", visible=True)[0]:
        return

    _draw_status_sections_single(panel)


def draw_resolution_panel(panel) -> None:
    if not getattr(panel, "_resolution_header_init_open", False):
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._resolution_header_init_open = True
    if not panel_header("Resolution", visible=True)[0]:
        return
    _draw_ui_resolution_controls(panel)


def draw_gaze_status_compact(panel) -> None:
    st = panel.state
    tag = "RUNNING" if bool(st.gaze_running) else "OFF"
    imgui.text(
        "Gaze [%s] mode=%s updates=%d u_err=%+.3f v_err=%+.3f"
        % (
            tag,
            str(st.gaze_mode) or "idle",
            int(st.gaze_update_count),
            float(st.gaze_u_err),
            float(st.gaze_v_err),
        )
    )
    if str(st.gaze_status_msg).strip():
        imgui.text_wrapped(str(st.gaze_status_msg))
