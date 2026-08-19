from __future__ import annotations

from typing import Optional

import imgui

from elesim_ui.theme import FONT_SPEC

_PANEL_HEADER_FONT = None


def set_panel_header_font(font) -> None:
    global _PANEL_HEADER_FONT
    _PANEL_HEADER_FONT = font


def ui_scale(panel) -> float:
    scale_fn = getattr(panel, "ui_resolution_scale", None)
    if callable(scale_fn):
        try:
            return max(0.1, float(scale_fn()))
        except Exception:
            return 1.0
    return 1.0


def scaled(panel, value: float) -> float:
    return float(value) * ui_scale(panel)


def _imgui_scale() -> float:
    try:
        return max(0.1, float(getattr(imgui.get_io(), "font_global_scale", 1.0) or 1.0))
    except Exception:
        return 1.0


def _button(panel, label: str, width: float, height: float = 26.0) -> bool:
    return bool(imgui.button(label, scaled(panel, width), scaled(panel, height)))


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


def _draw_rect_filled(
    draw_list,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    color: int,
    rounding: float = 0.0,
) -> None:
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


def _draw_line(
    draw_list,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    color: int,
    thickness: float = 1.0,
) -> None:
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


def _draw_triangle_filled(
    draw_list,
    points: tuple[tuple[float, float], tuple[float, float], tuple[float, float]],
    color: int,
) -> None:
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


def toggle_switch(
    panel,
    label: str,
    enabled: bool,
    *,
    on_color: tuple[float, float, float] = (0.13, 0.62, 0.30),
    on_hover_color: tuple[float, float, float] = (0.16, 0.68, 0.34),
) -> bool:
    frame_h = getattr(imgui, "get_frame_height", None)
    height = float(frame_h()) if callable(frame_h) else scaled(panel, 26.0)
    pad = scaled(panel, 3.0)
    cell = max(1.0, height - pad * 2.0)
    width = cell * 2.0 + pad * 2.0
    if not callable(getattr(imgui, "invisible_button", None)) or not callable(getattr(imgui, "get_window_draw_list", None)):
        return bool(imgui.button(f"##{label}", width, 0.0))

    x, y = _xy(imgui.get_cursor_screen_pos())
    clicked = bool(imgui.invisible_button(f"##{label}", float(width), float(height)))
    active = bool(imgui.is_item_active())
    hovered = bool(getattr(imgui, "is_item_hovered", lambda: False)())
    draw_list = imgui.get_window_draw_list()

    bg_fill = (0.66, 0.70, 0.76) if active else (0.76, 0.79, 0.84) if hovered else (0.82, 0.84, 0.87)
    _draw_rect_filled(draw_list, x, y, x + width, y + height, _color_u32(*bg_fill, 1.0), scaled(panel, 4.0))

    ly = y + (height - cell) * 0.5
    left_x = x + pad
    right_x = left_x + cell
    rounding = scaled(panel, 3.0)

    if enabled:
        fill = tuple(float(v) for v in (on_hover_color if hovered and not active else on_color))
        _draw_rect_filled(draw_list, right_x, ly, right_x + cell, ly + cell, _color_u32(*fill, 1.0), rounding)
    else:
        fill = (0.98, 0.99, 1.00) if hovered and not active else (0.95, 0.96, 0.98)
        _draw_rect_filled(draw_list, left_x, ly, left_x + cell, ly + cell, _color_u32(*fill, 1.0), rounding)

    return clicked


def begin_disabled_ui(disabled: bool) -> Optional[str]:
    if not disabled:
        return None
    begin_disabled = getattr(imgui, "begin_disabled", None)
    if callable(begin_disabled):
        begin_disabled()
        return "begin_disabled"
    item_disabled = getattr(imgui, "ITEM_DISABLED", None)
    push_item_flag = getattr(imgui, "push_item_flag", None)
    push_style_var = getattr(imgui, "push_style_var", None)
    style_alpha = getattr(imgui, "STYLE_ALPHA", None)
    if item_disabled is not None and callable(push_item_flag):
        push_item_flag(item_disabled, True)
        if style_alpha is not None and callable(push_style_var):
            push_style_var(style_alpha, imgui.get_style().alpha * 0.5)
            return "push_item_flag+alpha"
        return "push_item_flag"
    return None


def end_disabled_ui(token: Optional[str]) -> None:
    if token is None:
        return
    if token == "begin_disabled":
        end_disabled = getattr(imgui, "end_disabled", None)
        if callable(end_disabled):
            end_disabled()
        return
    if token == "push_item_flag+alpha":
        pop_style_var = getattr(imgui, "pop_style_var", None)
        if callable(pop_style_var):
            pop_style_var()
    pop_item_flag = getattr(imgui, "pop_item_flag", None)
    if callable(pop_item_flag):
        pop_item_flag()


def panel_header(label: str, *, visible: bool = True):
    if _PANEL_HEADER_FONT is not None:
        try:
            imgui.push_font(_PANEL_HEADER_FONT)
            try:
                return imgui.collapsing_header(label, visible=visible)
            finally:
                imgui.pop_font()
        except Exception:
            pass
    set_scale = getattr(imgui, "set_window_font_scale", None)
    if not callable(set_scale):
        return imgui.collapsing_header(label, visible=visible)
    set_scale(float(FONT_SPEC.title_fallback_scale))
    try:
        return imgui.collapsing_header(label, visible=visible)
    finally:
        set_scale(1.0)


def section_title(text: str) -> None:
    if _PANEL_HEADER_FONT is not None:
        try:
            imgui.push_font(_PANEL_HEADER_FONT)
            try:
                imgui.text(str(text))
            finally:
                imgui.pop_font()
            return
        except Exception:
            pass
    imgui.text(str(text))


def begin_collapsible_section(
    label: str,
    item_id: str,
    *,
    namespace: str,
    default_open: bool = True,
) -> Optional[str]:
    tree_node = getattr(imgui, "tree_node", None)
    tree_pop = getattr(imgui, "tree_pop", None)
    set_open = getattr(imgui, "set_next_item_open", None)
    if not callable(tree_node) or not callable(tree_pop):
        section_title(label)
        return "plain"
    if bool(default_open) and callable(set_open):
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        set_open(True, cond)
    text = f"{label}##{namespace}_section_{item_id}"
    if _PANEL_HEADER_FONT is not None:
        try:
            imgui.push_font(_PANEL_HEADER_FONT)
            try:
                opened = bool(tree_node(text))
            finally:
                imgui.pop_font()
            return "tree" if opened else None
        except Exception:
            pass
    return "tree" if bool(tree_node(text)) else None


def end_collapsible_section(token: Optional[str]) -> None:
    if token != "tree":
        return
    tree_pop = getattr(imgui, "tree_pop", None)
    if callable(tree_pop):
        tree_pop()
