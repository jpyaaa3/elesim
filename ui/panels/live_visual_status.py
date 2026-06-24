from __future__ import annotations

import math
import time

import imgui

from ui.helpers import panel_header


_HOST_STALE_S = 2.0
_STATUS_LABEL_W = 96.0
_CURRENT_YELLOW_COLOR = (1.0, 0.67, 0.08)
_CURRENT_RED_COLOR = (1.0, 0.18, 0.18)


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
    return float(len(str(text)) * 8), 14.0


def _line(label: str, value: object, *, color: tuple[float, float, float] | None = None) -> None:
    text = f"{label}: {_blank(value)}"
    if color is None:
        imgui.text(text)
    else:
        imgui.text_colored(text, float(color[0]), float(color[1]), float(color[2]))


def _section_title(text: str) -> None:
    imgui.text(str(text))


def _draw_collapsible_section(label: str, draw_fn, panel) -> None:
    tree_node = getattr(imgui, "tree_node", None)
    tree_pop = getattr(imgui, "tree_pop", None)
    set_open = getattr(imgui, "set_next_item_open", None)
    if not callable(tree_node) or not callable(tree_pop):
        _section_title(label)
        draw_fn(panel)
        return
    if callable(set_open):
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        set_open(True, cond)
    item_id = str(label).lower().replace(" ", "_").replace("/", "_")
    if tree_node(f"{label}##status_section_{item_id}"):
        try:
            draw_fn(panel)
        finally:
            tree_pop()


def _text_value(value: object, *, color: tuple[float, float, float] | None = None) -> None:
    if color is None:
        imgui.text(_blank(value))
    else:
        imgui.text_colored(_blank(value), float(color[0]), float(color[1]), float(color[2]))


def _readonly_text_field(label: str, value: object, identifier: str) -> None:
    text = _blank(value)
    imgui.text(str(label))
    imgui.same_line(_STATUS_LABEL_W)
    _readonly_text_value(text, identifier)


def _readonly_text_value(value: object, identifier: str) -> None:
    text = _blank(value)
    width_getter = getattr(imgui, "get_content_region_available_width", None)
    field_w = max(80.0, float(width_getter()) if callable(width_getter) else 220.0)
    _readonly_text_box(text, identifier, field_w)


def _readonly_text_box(value: object, identifier: str, width: float) -> None:
    text = _blank(value)
    flags = getattr(
        imgui,
        "INPUT_TEXT_READ_ONLY",
        getattr(imgui, "INPUT_TEXT_FLAGS_READ_ONLY", 1 << 14),
    )
    imgui.push_item_width(float(width))
    try:
        try:
            imgui.input_text(f"##{identifier}", text, max(64, len(text) + 1), flags=flags)
        except TypeError:
            imgui.input_text(f"##{identifier}", text, max(64, len(text) + 1), flags)
    finally:
        imgui.pop_item_width()


def _readonly_float3_field(
    label: str,
    vec: tuple[float, float, float] | None,
    identifier: str,
    *,
    format: str = "%.3f",
) -> None:
    imgui.text(str(label))
    imgui.same_line(_STATUS_LABEL_W)
    width_getter = getattr(imgui, "get_content_region_available_width", None)
    available = max(120.0, float(width_getter()) if callable(width_getter) else 260.0)
    style = imgui.get_style()
    spacing = float(getattr(style, "item_spacing", (8.0, 0.0))[0])
    component_w = max(52.0, (available - spacing * 2.0) / 3.0)
    if vec is None:
        values = ("-", "-", "-")
    else:
        values = tuple(format % float(vec[idx]) for idx in range(3))
    for idx, value in enumerate(values):
        if idx > 0:
            imgui.same_line()
        _readonly_text_box(value, f"{identifier}_{idx}", component_w)


def _table_child_height() -> float:
    getter = getattr(imgui, "get_text_line_height_with_spacing", None)
    line_h = float(getter()) if callable(getter) else 20.0
    return line_h * 2.0 + 18.0


def _draw_columns_table(
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
    imgui.begin_child(str(identifier), 0.0, _table_child_height(), True, flags=flags)
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


def _host_status(host) -> tuple[str, tuple[float, float, float] | None]:
    if host is None or not bool(getattr(host, "connected", False)):
        return "OFF", (0.70, 0.36, 0.05)
    try:
        rx_age = float(getattr(host, "rx_age_s", float("inf")))
    except (TypeError, ValueError):
        rx_age = float("inf")
    if not math.isfinite(rx_age):
        return "NO REPLY", (0.70, 0.36, 0.05)
    if rx_age > _HOST_STALE_S:
        return "STALE %.1fs" % rx_age, (0.85, 0.46, 0.10)
    return "OK", None


def _draw_hardware_brief(panel) -> None:
    host = panel._host_state
    host_text, host_color = _host_status(host)
    device = getattr(host, "device", "") if host is not None else ""
    ports = tuple(getattr(host, "ports", ()) or ()) if host is not None else ()
    _draw_columns_table(
        "hardware_host_table",
        ("Host", "Device", "Port"),
        (host_text, device, ", ".join(str(p) for p in ports)),
        colors=(host_color, None, None),
        center=True,
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
    _readonly_float3_field("Tip xyz", tip_xyz, "status_tip_xyz", format="%.3f")
    _readonly_float3_field("Tip dir", _normalized_xyz(tip_dir), "status_tip_dir", format="%+.3f")


def _draw_go2_brief(panel) -> None:
    enabled = bool(getattr(panel, "_use_go2", False))
    host = panel._host_state
    vel = (0.0, 0.0, 0.0) if host is None else tuple(float(v) for v in getattr(host, "go2_vel", (0.0, 0.0, 0.0)))
    _line("GO2", "enabled" if enabled else "disabled")
    _line(
        "GO2 teleop step",
        "vx=%.2fm/s  vy=%.2fm/s  wz=%.2frad/s"
        % (
            float(getattr(panel, "_go2_teleop_vx_mps", 0.0)),
            float(getattr(panel, "_go2_teleop_vy_mps", 0.0)),
            float(getattr(panel, "_go2_teleop_wz_radps", 0.0)),
        ),
    )
    _line("GO2 vel", "vx=%+.2f  vy=%+.2f  wz=%+.2f" % vel)
    _line("GO2 base pos [m]", _fmt_xyz(getattr(host, "go2_base_pos", None) if host is not None else None, signed=True))
    _line("GO2 base rpy [rad]", _fmt_xyz(getattr(host, "go2_base_rpy", None) if host is not None else None, signed=True))


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
        _section_title("Vision / Gaze")
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

    perc_active = bool(st.perception_running) and not bool(st.perception_failed)
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


def draw_status_panel(panel) -> None:
    if not panel._status_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._status_header_init_open = True
    if not panel_header("Status", visible=True)[0]:
        return

    _draw_status_sections_single(panel)


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
