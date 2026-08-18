from __future__ import annotations

import math
import time

import glfw
import imgui

from elesim_ui.helpers import (
    _button,
    _color_u32,
    _draw_line,
    _draw_rect_filled,
    _draw_text,
    _draw_triangle_filled,
    _imgui_scale,
    _xy,
    panel_header,
    scaled,
    ui_scale,
)


_PAD_MIN_CELL_W = 36.0
_PAD_MAX_CELL_W = 56.0
_SHAPE_ROUNDING = 6.0


def _calc_text_size(text: str) -> tuple[float, float]:
    calc = getattr(imgui, "calc_text_size", None)
    if callable(calc):
        return _xy(calc(str(text)))
    scale = _imgui_scale()
    return float(len(str(text)) * 8.0 * scale), 14.0 * scale


def _style_spacing_x() -> float:
    style = getattr(imgui, "get_style", lambda: None)()
    spacing = getattr(style, "item_spacing", None)
    if spacing is None:
        return 8.0 * _imgui_scale()
    if hasattr(spacing, "x"):
        return float(spacing.x)
    return float(spacing[0])


def _center_next_item(item_width: float, available_width: float) -> None:
    get_x = getattr(imgui, "get_cursor_pos_x", None)
    set_x = getattr(imgui, "set_cursor_pos_x", None)
    if not callable(get_x) or not callable(set_x):
        return
    offset = max(0.0, (float(available_width) - float(item_width)) * 0.5)
    set_x(float(get_x()) + offset)


def _draw_circle_filled(draw_list, x: float, y: float, radius: float, color: int) -> None:
    for args in (
        ((x, y), radius, color, 28),
        (x, y, radius, color, 28),
        ((x, y), radius, color),
        (x, y, radius, color),
    ):
        try:
            draw_list.add_circle_filled(*args)
            return
        except TypeError:
            continue


def _draw_stroked_polyline(
    draw_list,
    points: list[tuple[float, float]],
    color: int,
    thickness: float,
    *,
    cap_first: bool = True,
    cap_last: bool = True,
) -> None:
    radius = max(1.0, float(thickness) * 0.5)
    for p1, p2 in zip(points, points[1:]):
        _draw_line(draw_list, p1[0], p1[1], p2[0], p2[1], color, thickness)
    cap_points = points
    if not cap_first:
        cap_points = cap_points[1:]
    if not cap_last:
        cap_points = cap_points[:-1]
    for px, py in cap_points:
        _draw_circle_filled(draw_list, px, py, radius, color)


def _draw_arrow_head(
    draw_list,
    tip: tuple[float, float],
    prev: tuple[float, float],
    color: int,
    size: float,
    extension: float = 0.0,
) -> None:
    dx = tip[0] - prev[0]
    dy = tip[1] - prev[1]
    length = math.hypot(dx, dy)
    if length <= 0.001:
        return

    ux = dx / length
    uy = dy / length
    nx = -uy
    ny = ux
    head_len = size * 0.28
    head_width = size * 0.24
    tip = (tip[0] + ux * float(extension), tip[1] + uy * float(extension))
    base = (tip[0] - ux * head_len, tip[1] - uy * head_len)
    left = (base[0] + nx * head_width * 0.5, base[1] + ny * head_width * 0.5)
    right = (base[0] - nx * head_width * 0.5, base[1] - ny * head_width * 0.5)
    _draw_triangle_filled(draw_list, (tip, left, right), color)


def _draw_centered_text(draw_list, x: float, y: float, color: int, text: str) -> None:
    tw, th = _calc_text_size(str(text))
    _draw_text(draw_list, float(x) - tw * 0.5, float(y) - th * 0.5, color, str(text))


def _draw_space_symbol(draw_list, cx: float, cy: float, size: float, color: int, *, width: float | None = None) -> None:
    width = float(width) if width is not None else float(size) * 0.34
    height = float(size) * 0.10
    thickness = max(1.2, float(size) * 0.045)
    left = float(cx) - width * 0.5
    right = float(cx) + width * 0.5
    top = float(cy) - height * 0.5
    bottom = float(cy) + height * 0.5
    half = thickness * 0.5
    _draw_rect_filled(draw_list, left - half, top, left + half, bottom + half, color, 0.0)
    _draw_rect_filled(draw_list, left - half, bottom - half, right + half, bottom + half, color, 0.0)
    _draw_rect_filled(draw_list, right - half, top, right + half, bottom + half, color, 0.0)


def _draw_turn_arrow(draw_list, x: float, y: float, size: float, *, direction: str, color: int) -> None:
    cx = x + size * 0.5
    cy = y + size * 0.52
    radius = size * 0.32
    thickness = size * 0.095

    if direction == "left":
        start_deg, end_deg = 60.0, 300.0
    else:
        start_deg, end_deg = 120.0, -120.0

    points = []
    for i in range(57):
        t = i / 56.0
        a = math.radians(start_deg + (end_deg - start_deg) * t)
        points.append((cx + math.cos(a) * radius, cy + math.sin(a) * radius))

    _draw_stroked_polyline(draw_list, points, color, thickness, cap_first=False)

    tip = points[0]
    _draw_arrow_head(draw_list, tip, points[1], color, size, extension=thickness * 2.0)


def _shape_button(
    panel,
    kind: str,
    widget_id: str,
    size: float,
    *,
    text: str = "",
    force_active: bool = False,
) -> tuple[bool, bool]:
    if not callable(getattr(imgui, "invisible_button", None)) or not callable(getattr(imgui, "get_window_draw_list", None)):
        clicked = bool(imgui.button(text or kind, float(size), float(size)))
        return clicked, bool(imgui.is_item_active()) or bool(force_active)

    x, y = _xy(imgui.get_cursor_screen_pos())
    clicked = bool(imgui.invisible_button(f"##{widget_id}", float(size), float(size)))
    active = bool(imgui.is_item_active()) or bool(force_active)
    hovered = bool(getattr(imgui, "is_item_hovered", lambda: False)())
    draw_list = imgui.get_window_draw_list()

    if kind == "stop":
        base = (0.80, 0.82, 0.85)
        hover = (0.72, 0.75, 0.80)
        down = (0.58, 0.62, 0.68)
        glyph = "STOP"
    elif kind in ("turn_left", "turn_right"):
        base = (0.82, 0.84, 0.87)
        hover = (0.76, 0.79, 0.84)
        down = (0.66, 0.70, 0.76)
        glyph = text
    else:
        base = (0.86, 0.90, 0.95)
        hover = (0.78, 0.86, 0.98)
        down = (0.66, 0.78, 0.96)
        glyph = ""

    fill = down if active else hover if hovered else base
    fg = _color_u32(0.12, 0.14, 0.17, 1.0)
    bg = _color_u32(*fill, 1.0)
    _draw_rect_filled(draw_list, x, y, x + size, y + size, bg, scaled(panel, _SHAPE_ROUNDING))
    cutout = bg

    cx = x + size * 0.5
    cy = y + size * 0.5
    pad = size * 0.24
    if kind == "up":
        _draw_triangle_filled(draw_list, ((cx, y + pad), (x + size - pad, y + size - pad), (x + pad, y + size - pad)), fg)
        _draw_centered_text(draw_list, cx, y + size * 0.60, cutout, "W")
    elif kind == "down":
        _draw_triangle_filled(draw_list, ((x + pad, y + pad), (x + size - pad, y + pad), (cx, y + size - pad)), fg)
        _draw_centered_text(draw_list, cx, y + size * 0.40, cutout, "S")
    elif kind == "left":
        _draw_triangle_filled(draw_list, ((x + pad, cy), (x + size - pad, y + pad), (x + size - pad, y + size - pad)), fg)
        _draw_centered_text(draw_list, x + size * 0.60, cy, cutout, "A")
    elif kind == "right":
        _draw_triangle_filled(draw_list, ((x + pad, y + pad), (x + size - pad, cy), (x + pad, y + size - pad)), fg)
        _draw_centered_text(draw_list, x + size * 0.40, cy, cutout, "D")
    elif kind in ("turn_left", "turn_right"):
        _draw_turn_arrow(draw_list, x, y, size, direction="left" if kind == "turn_left" else "right", color=fg)
        _draw_centered_text(draw_list, cx, cy, fg, "Q" if kind == "turn_left" else "E")
    elif kind == "stop":
        _draw_centered_text(draw_list, cx, y + size * 0.42, fg, glyph)
        glyph_w, _ = _calc_text_size(glyph)
        _draw_space_symbol(draw_list, cx, y + size * 0.64, size, fg, width=glyph_w)
    else:
        tw, th = _calc_text_size(glyph)
        _draw_text(draw_list, cx - tw * 0.5, cy - th * 0.5, fg, glyph)

    return clicked, active


def _send_go2_velocity(
    panel,
    *,
    vx: float,
    vy: float,
    wz: float,
    force: bool = False,
) -> bool:
    """Send teleop at a bounded cadence instead of once per ImGui frame.

    The Robot deadman only needs a modest keep-alive rate.  Rendering at
    display refresh (often 60--144 Hz) used to enqueue one DDS request for
    every frame, which competed with snapshots and made the UI feel sticky.
    ``force`` is reserved for the first command and the stop edge so a safety
    transition is never delayed by the rate limiter.
    """

    now = time.monotonic()
    period = max(
        0.01,
        float(getattr(panel, "_go2_send_period_s", 1.0 / 20.0)),
    )
    last = float(getattr(panel, "_go2_last_command_at", 0.0))
    if not force and now - last < period:
        return False
    panel.service.send_go2_velocity(
        vx=float(vx),
        vy=float(vy),
        wz=float(wz),
    )
    panel._go2_last_command_at = now
    return True


def _stop_go2(panel) -> None:
    # Holding SPACE used to submit a zero command every render frame.  One
    # forced stop is sufficient; the next non-zero command starts a new
    # deadman interval and clears this edge marker.
    if not bool(getattr(panel, "_go2_stop_sent", False)) or bool(
        getattr(panel, "_go2_was_active", False)
    ):
        _send_go2_velocity(panel, vx=0.0, vy=0.0, wz=0.0, force=True)
    panel._go2_was_active = False
    panel._go2_stop_sent = True


def _keyboard_teleop_enabled(panel) -> bool:
    window = getattr(panel, "_glfw_window", None)
    if window is None:
        return False
    try:
        if glfw.get_window_attrib(window, glfw.FOCUSED) != glfw.TRUE:
            return False
    except Exception:
        return False
    try:
        io = imgui.get_io()
        if bool(getattr(io, "want_text_input", False)):
            return False
    except Exception:
        pass
    return True


def _key_down(panel, key: int) -> bool:
    if not _keyboard_teleop_enabled(panel):
        return False
    try:
        return glfw.get_key(panel._glfw_window, int(key)) == glfw.PRESS
    except Exception:
        return False


def _draw_teleop_pad(panel, width: float) -> bool:
    spacing_x = _style_spacing_x()
    cell = max(scaled(panel, _PAD_MIN_CELL_W), min(scaled(panel, _PAD_MAX_CELL_W), (float(width) - spacing_x * 4.0) / 5.0))
    row_w = cell * 5.0 + spacing_x * 4.0
    active = False
    vx = 0.0
    vy = 0.0
    wz = 0.0
    key_forward = _key_down(panel, glfw.KEY_W)
    key_back = _key_down(panel, glfw.KEY_S)
    key_left = _key_down(panel, glfw.KEY_A)
    key_right = _key_down(panel, glfw.KEY_D)
    key_turn_left = _key_down(panel, glfw.KEY_Q)
    key_turn_right = _key_down(panel, glfw.KEY_E)
    key_stop = _key_down(panel, glfw.KEY_SPACE)

    _center_next_item(row_w, width)
    imgui.begin_group()
    imgui.dummy(cell, cell)
    imgui.same_line()
    imgui.dummy(cell, cell)
    imgui.same_line()
    _, held = _shape_button(panel, "up", "go2_forward_shape", cell, force_active=key_forward)
    if held:
        vx += float(panel._go2_teleop_vx_mps)
        active = True
    imgui.same_line()
    imgui.dummy(cell, cell)
    imgui.same_line()
    imgui.dummy(cell, cell)

    _, held = _shape_button(panel, "turn_left", "go2_turn_left_shape", cell, force_active=key_turn_left)
    if held:
        wz += float(panel._go2_teleop_wz_radps)
        active = True
    imgui.same_line()
    _, held = _shape_button(panel, "left", "go2_left_shape", cell, force_active=key_left)
    if held:
        vy += float(panel._go2_teleop_vy_mps)
        active = True
    imgui.same_line()
    clicked, _ = _shape_button(panel, "stop", "go2_stop_shape", cell, force_active=key_stop)
    if clicked or key_stop:
        _stop_go2(panel)
    imgui.same_line()
    _, held = _shape_button(panel, "right", "go2_right_shape", cell, force_active=key_right)
    if held:
        vy -= float(panel._go2_teleop_vy_mps)
        active = True
    imgui.same_line()
    _, held = _shape_button(panel, "turn_right", "go2_turn_right_shape", cell, force_active=key_turn_right)
    if held:
        wz -= float(panel._go2_teleop_wz_radps)
        active = True

    imgui.dummy(cell, cell)
    imgui.same_line()
    imgui.dummy(cell, cell)
    imgui.same_line()
    _, held = _shape_button(panel, "down", "go2_back_shape", cell, force_active=key_back)
    if held:
        vx -= float(panel._go2_teleop_vx_mps)
        active = True
    imgui.same_line()
    imgui.dummy(cell, cell)
    imgui.same_line()
    imgui.dummy(cell, cell)
    imgui.end_group()

    if key_stop:
        return False
    if active:
        # Send the first command of a new deadman interval immediately even
        # if the one-shot stop was emitted on the previous frame.
        first_active = bool(getattr(panel, "_go2_stop_sent", False))
        _send_go2_velocity(
            panel,
            vx=float(vx),
            vy=float(vy),
            wz=float(wz),
            force=first_active,
        )
        panel._go2_was_active = True
        panel._go2_stop_sent = False
    elif panel._go2_was_active:
        _stop_go2(panel)
    return active


def _draw_posture(panel, width: float) -> None:
    scale = ui_scale(panel)
    btn_w = max(86.0, min(130.0, ((float(width) / scale) - 16.0) / 3.0))
    host = getattr(panel, "_host_state", None)
    last_pose = str(getattr(host, "go2_sport_pose", "") if host is not None else "").strip().lower()
    standing_poses = {"stand_up", "balance_stand", "recovery_stand", "static_walk", "trot_run", "economic_gait"}
    is_standing = last_pose in standing_poses
    sit_stand_label = "Sit" if is_standing else "Stand"
    sit_stand_pose = "stand_down" if is_standing else "stand_up"
    if is_standing and _button(panel, "Balance##go2_balance", btn_w):
        _stop_go2(panel)
        panel.service.send_go2_sport_pose(pose="balance_stand")
    elif not is_standing:
        imgui.text_disabled("Balance")
    imgui.same_line()
    if _button(panel, f"{sit_stand_label}##go2_sit_stand", btn_w):
        _stop_go2(panel)
        panel.service.send_go2_sport_pose(pose=sit_stand_pose)
    imgui.same_line()
    if is_standing and _button(panel, "Recover##go2_recovery", btn_w):
        _stop_go2(panel)
        panel.service.send_go2_sport_pose(pose="recovery_stand")
    elif not is_standing:
        imgui.text_disabled("Recover")


def draw_go2_panel(panel) -> None:
    if not panel._use_go2:
        if bool(getattr(panel, "_go2_was_active", False)):
            _stop_go2(panel)
        return
    if not panel._go2_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._go2_header_init_open = True
    if not panel_header("GO2 Locomotion", visible=True)[0]:
        # A collapsed panel must not leave a held keyboard/pad command alive
        # until the remote deadman happens to expire.
        if bool(getattr(panel, "_go2_was_active", False)):
            _stop_go2(panel)
        return

    width = max(scaled(panel, 220.0), float(imgui.get_content_region_available_width()))
    _draw_teleop_pad(panel, width)

    changed_avoid, enabled = imgui.checkbox(
        "Obstacle Avoid",
        bool(getattr(panel, "_go2_obstacles_avoid_enabled", False)),
    )
    if changed_avoid:
        panel._go2_obstacles_avoid_enabled = bool(enabled)
        panel.service.send_go2_obstacles_avoid(enabled=bool(enabled))

    _draw_posture(panel, width)
