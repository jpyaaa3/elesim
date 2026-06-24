from __future__ import annotations

from typing import Optional

import imgui

from ui.theme import FONT_SPEC

_PANEL_HEADER_FONT = None


def set_panel_header_font(font) -> None:
    global _PANEL_HEADER_FONT
    _PANEL_HEADER_FONT = font


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
