from __future__ import annotations

import math

import imgui

from ui.helpers import panel_header, scaled, ui_scale


_PAD_MIN_CELL_W = 36.0
_PAD_MAX_CELL_W = 56.0
_PAD_H = 30.0
_SMALL_H = 26.0
_SHAPE_ROUNDING = 6.0


def _button(panel, label: str, width: float, height: float = _SMALL_H) -> bool:
    return bool(imgui.button(label, scaled(panel, width), scaled(panel, height)))


def _hold_button(panel, label: str, width: float, height: float = _PAD_H) -> bool:
    imgui.button(label, scaled(panel, width), scaled(panel, height))
    return bool(imgui.is_item_active())


def _xy(pos) -> tuple[float, float]:
    if hasattr(pos, "x") and hasattr(pos, "y"):
        return float(pos.x), float(pos.y)
    return float(pos[0]), float(pos[1])


def _color_u32(r: float, g: float, b: float, a: float = 1.0) -> int:
    getter = getattr(imgui, "get_color_u32_rgba", None)
    if callable(getter):
        return int(getter(float(r), float(g), float(b), float(a)))
    ri = max(0, min(255, int(float(r) * 255.0)))
    gi = max(0, min(255, int(float(g) * 255.0)))
    bi = max(0, min(255, int(float(b) * 255.0)))
    ai = max(0, min(255, int(float(a) * 255.0)))
    return (ai << 24) | (bi << 16) | (gi << 8) | ri


def _calc_text_size(text: str) -> tuple[float, float]:
    calc = getattr(imgui, "calc_text_size", None)
    if callable(calc):
        return _xy(calc(str(text)))
    return float(len(str(text)) * 8), 14.0


def _style_spacing_x() -> float:
    style = getattr(imgui, "get_style", lambda: None)()
    spacing = getattr(style, "item_spacing", None)
    if spacing is None:
        return 8.0
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
    head_len = max(10.0, size * 0.28)
    head_width = max(9.0, size * 0.24)
    tip = (tip[0] + ux * float(extension), tip[1] + uy * float(extension))
    base = (tip[0] - ux * head_len, tip[1] - uy * head_len)
    left = (base[0] + nx * head_width * 0.5, base[1] + ny * head_width * 0.5)
    right = (base[0] - nx * head_width * 0.5, base[1] - ny * head_width * 0.5)
    _draw_triangle_filled(draw_list, (tip, left, right), color)


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


def _draw_turn_arrow(draw_list, x: float, y: float, size: float, *, direction: str, color: int) -> None:
    cx = x + size * 0.5
    cy = y + size * 0.52
    radius = size * 0.32
    thickness = max(4.2, size * 0.095)

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


def _shape_button(panel, kind: str, widget_id: str, size: float, *, text: str = "") -> tuple[bool, bool]:
    if not callable(getattr(imgui, "invisible_button", None)) or not callable(getattr(imgui, "get_window_draw_list", None)):
        clicked = bool(imgui.button(text or kind, float(size), float(size)))
        return clicked, bool(imgui.is_item_active())

    x, y = _xy(imgui.get_cursor_screen_pos())
    clicked = bool(imgui.invisible_button(f"##{widget_id}", float(size), float(size)))
    active = bool(imgui.is_item_active())
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

    cx = x + size * 0.5
    cy = y + size * 0.5
    pad = size * 0.24
    if kind == "up":
        _draw_triangle_filled(draw_list, ((cx, y + pad), (x + size - pad, y + size - pad), (x + pad, y + size - pad)), fg)
    elif kind == "down":
        _draw_triangle_filled(draw_list, ((x + pad, y + pad), (x + size - pad, y + pad), (cx, y + size - pad)), fg)
    elif kind == "left":
        _draw_triangle_filled(draw_list, ((x + pad, cy), (x + size - pad, y + pad), (x + size - pad, y + size - pad)), fg)
    elif kind == "right":
        _draw_triangle_filled(draw_list, ((x + pad, y + pad), (x + size - pad, cy), (x + pad, y + size - pad)), fg)
    elif kind in ("turn_left", "turn_right"):
        _draw_turn_arrow(draw_list, x, y, size, direction="left" if kind == "turn_left" else "right", color=fg)
    else:
        tw, th = _calc_text_size(glyph)
        _draw_text(draw_list, cx - tw * 0.5, cy - th * 0.5, fg, glyph)

    return clicked, active


def _stop_go2(panel) -> None:
    panel.service.send_go2_velocity(vx=0.0, vy=0.0, wz=0.0)
    panel._go2_was_active = False


def _draw_teleop_pad(panel, width: float) -> bool:
    spacing_x = _style_spacing_x()
    cell = max(scaled(panel, _PAD_MIN_CELL_W), min(scaled(panel, _PAD_MAX_CELL_W), (float(width) - spacing_x * 4.0) / 5.0))
    row_w = cell * 5.0 + spacing_x * 4.0
    active = False
    vx = 0.0
    vy = 0.0
    wz = 0.0

    _center_next_item(row_w, width)
    imgui.begin_group()
    imgui.dummy(cell, cell)
    imgui.same_line()
    imgui.dummy(cell, cell)
    imgui.same_line()
    _, held = _shape_button(panel, "up", "go2_forward_shape", cell)
    if held:
        vx += float(panel._go2_teleop_vx_mps)
        active = True
    imgui.same_line()
    imgui.dummy(cell, cell)
    imgui.same_line()
    imgui.dummy(cell, cell)

    _, held = _shape_button(panel, "turn_left", "go2_turn_left_shape", cell)
    if held:
        wz += float(panel._go2_teleop_wz_radps)
        active = True
    imgui.same_line()
    _, held = _shape_button(panel, "left", "go2_left_shape", cell)
    if held:
        vy += float(panel._go2_teleop_vy_mps)
        active = True
    imgui.same_line()
    clicked, _ = _shape_button(panel, "stop", "go2_stop_shape", cell)
    if clicked:
        _stop_go2(panel)
    imgui.same_line()
    _, held = _shape_button(panel, "right", "go2_right_shape", cell)
    if held:
        vy -= float(panel._go2_teleop_vy_mps)
        active = True
    imgui.same_line()
    _, held = _shape_button(panel, "turn_right", "go2_turn_right_shape", cell)
    if held:
        wz -= float(panel._go2_teleop_wz_radps)
        active = True

    imgui.dummy(cell, cell)
    imgui.same_line()
    imgui.dummy(cell, cell)
    imgui.same_line()
    _, held = _shape_button(panel, "down", "go2_back_shape", cell)
    if held:
        vx -= float(panel._go2_teleop_vx_mps)
        active = True
    imgui.same_line()
    imgui.dummy(cell, cell)
    imgui.same_line()
    imgui.dummy(cell, cell)
    imgui.end_group()

    if active:
        panel.service.send_go2_velocity(vx=float(vx), vy=float(vy), wz=float(wz))
        panel._go2_was_active = True
    elif panel._go2_was_active:
        _stop_go2(panel)
    return active


def _draw_posture(panel, width: float) -> None:
    scale = ui_scale(panel)
    btn_w = max(86.0, min(130.0, ((float(width) / scale) - 16.0) / 3.0))
    if _button(panel, "Balance##go2_balance", btn_w):
        _stop_go2(panel)
        panel.service.send_go2_sport_pose(pose="balance_stand")
    imgui.same_line()
    if _button(panel, "Lie Down##go2_lie_down", btn_w):
        _stop_go2(panel)
        panel.service.send_go2_sport_pose(pose="stand_down")
    imgui.same_line()
    if _button(panel, "Recover##go2_recovery", btn_w):
        _stop_go2(panel)
        panel.service.send_go2_sport_pose(pose="recovery_stand")


def draw_go2_panel(panel) -> None:
    if not panel._use_go2:
        return
    if not panel._go2_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._go2_header_init_open = True
    if not panel_header("GO2 Locomotion", visible=True)[0]:
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

    imgui.text("Gaze / Demo")
    if _button(panel, "Gaze Stand##go2_gaze_stand", 96.0):
        panel.service.start_gaze_stabilizer_standing()
    imgui.same_line()
    if _button(panel, "Gaze Walk##go2_gaze_walk", 96.0):
        panel.service.start_gaze_stabilizer_walking()
    imgui.same_line()
    if _button(panel, "Stop Gaze##go2_stop_gaze", 96.0):
        panel.service.stop_gaze_stabilizer()
    if _button(panel, "Demo 4: Stop + Grasp##go2_demo4", 176.0):
        panel.service.start_demo4_stop_and_grasp()
