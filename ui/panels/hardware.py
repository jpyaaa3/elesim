from __future__ import annotations

import imgui

from ui.helpers import panel_header, scaled


class _EmptyHardwareState:
    connected = False
    tx_seq = 0
    rx_age_s = 0.0
    device = ""
    ports = ()
    torque_enabled = False
    reply_ok = True
    reply_reason = ""
    safety_fault = ""
    motor_currents_ma = {}


_CONTROL_LABEL_W = 96.0
_PORT_LABEL_W = 66.0
_SWITCH_W = 58.0
_WARN_W = 28.0
_SEARCH_W = 72.0


def _control_label(panel, text: str) -> None:
    imgui.text(str(text))
    imgui.same_line(scaled(panel, _CONTROL_LABEL_W))


def _switch_button(panel, label: str, enabled: bool, *, width: float = 58.0) -> bool:
    text = "ON" if enabled else "OFF"
    pushed = False
    if enabled:
        colors = (
            (imgui.COLOR_BUTTON, 0.12, 0.58, 0.26, 1.0),
            (imgui.COLOR_BUTTON_HOVERED, 0.16, 0.66, 0.32, 1.0),
            (imgui.COLOR_BUTTON_ACTIVE, 0.08, 0.44, 0.20, 1.0),
        )
    else:
        colors = (
            (imgui.COLOR_BUTTON, 0.82, 0.84, 0.87, 1.0),
            (imgui.COLOR_BUTTON_HOVERED, 0.76, 0.79, 0.84, 1.0),
            (imgui.COLOR_BUTTON_ACTIVE, 0.66, 0.70, 0.76, 1.0),
        )
    try:
        for color in colors:
            imgui.push_style_color(*color)
        pushed = True
    except Exception:
        pushed = False
    try:
        return bool(imgui.button(f"{text}##{label}", scaled(panel, width), 0.0))
    finally:
        if pushed:
            imgui.pop_style_color(3)


def _warn_button(panel, label: str) -> bool:
    pushed = False
    try:
        imgui.push_style_color(imgui.COLOR_BUTTON, 0.93, 0.48, 0.18, 1.0)
        imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, 1.0, 0.56, 0.22, 1.0)
        imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, 0.78, 0.34, 0.10, 1.0)
        pushed = True
    except Exception:
        pushed = False
    try:
        return bool(imgui.button(f"!##{label}", scaled(panel, _WARN_W), 0.0))
    finally:
        if pushed:
            imgui.pop_style_color(3)


def draw_hardware_panel(panel) -> None:
    if not panel._hw_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._hw_header_init_open = True
    if panel_header("Hardware", visible=True)[0]:
        state = panel._host_state if panel._host_state is not None else panel.service.current_host_state()
        if state is None:
            state = _EmptyHardwareState()
        current_device = str(state.device or "").strip()
        if current_device and not panel._port_input:
            panel._port_input = current_device

        imgui.text("Port")
        imgui.same_line(scaled(panel, _PORT_LABEL_W))
        search_w = scaled(panel, _SEARCH_W)
        spacing_x = float(getattr(imgui.get_style().item_spacing, "x", scaled(panel, 8.0)))
        port_input_w = max(1.0, float(imgui.get_content_region_available_width()) - search_w - spacing_x)
        imgui.push_item_width(port_input_w)
        changed_port, new_port = imgui.input_text("##hardware_port", panel._port_input, 256)
        imgui.pop_item_width()
        if changed_port:
            panel._port_input = str(new_port)
        imgui.same_line()
        if imgui.button("Search", search_w, 0.0):
            panel.service.request_ports()

        _control_label(panel, "Apply Port")
        if _switch_button(panel, "hardware_port_switch", bool(current_device), width=_SWITCH_W):
            if current_device:
                panel.state.set_torque_lock_bypass(False)
                panel.service.disconnect_device()
                panel._port_input = ""
            else:
                panel.state.set_torque_lock_bypass(bool(state.torque_enabled))
                panel.service.set_device(panel._port_input.strip())
        imgui.same_line()
        if _warn_button(panel, "hardware_port_abort"):
            panel.state.set_torque_lock_bypass(False)
            panel.service.disconnect_device()
            panel._port_input = ""

        _control_label(panel, "Torque")
        if _switch_button(panel, "hardware_torque_switch", bool(state.torque_enabled), width=_SWITCH_W):
            if state.torque_enabled:
                panel.state.set_torque_lock_bypass(False)
                panel.service.torque_off()
            else:
                resume = bool(panel.state.torque_lock_bypass)
                panel.service.torque_on(resume=resume)
                if not resume:
                    panel.state.set_torque_lock_bypass(False)
        imgui.same_line()
        if _warn_button(panel, "hardware_torque_abort"):
            panel.state.set_torque_lock_bypass(False)
            panel.service.torque_off()
