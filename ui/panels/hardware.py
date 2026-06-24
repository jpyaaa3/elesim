from __future__ import annotations

import imgui

from ui.helpers import panel_header


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
_SWITCH_W = 58.0
_WARN_W = 28.0


def _control_label(text: str) -> None:
    imgui.text(str(text))
    imgui.same_line(_CONTROL_LABEL_W)


def _reserved_text(text: str, *, color: tuple[float, float, float] | None = None) -> None:
    if str(text).strip():
        if color is None:
            imgui.text(str(text))
        else:
            imgui.text_colored(str(text), float(color[0]), float(color[1]), float(color[2]))
    else:
        imgui.dummy(1.0, float(imgui.get_text_line_height()))


def _is_device_reply(reason_key: str) -> bool:
    return (
        reason_key == "ports"
        or "device" in reason_key
        or "port" in reason_key
        or reason_key.startswith("failed to open")
    )


def _device_hint(reply_reason: str, *, reply_ok: bool, ports: list[str]) -> tuple[str, bool]:
    key = str(reply_reason).strip().lower()
    if not key:
        return "", False
    if key == "ports":
        return ("No serial ports found" if not ports else ""), True
    if "empty device" in key:
        return "No devices detected", True
    if "device disconnected" in key:
        return "Device disconnected", True
    if "failed to open device" in key:
        return str(reply_reason).replace("failed to open device", "Failed to open device", 1), bool(reply_ok)
    if _is_device_reply(key):
        return str(reply_reason), bool(reply_ok)
    return "", False


def _switch_button(label: str, enabled: bool, *, width: float = 58.0) -> bool:
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
        return bool(imgui.button(f"{text}##{label}", float(width), 0.0))
    finally:
        if pushed:
            imgui.pop_style_color(3)


def _warn_button(label: str) -> bool:
    pushed = False
    try:
        imgui.push_style_color(imgui.COLOR_BUTTON, 0.95, 0.78, 0.32, 1.0)
        imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, 1.0, 0.70, 0.18, 1.0)
        imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, 0.86, 0.48, 0.10, 1.0)
        pushed = True
    except Exception:
        pushed = False
    try:
        return bool(imgui.button(f"!##{label}", 28.0, 0.0))
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
        hardware_ready = bool(state.connected and current_device)
        if hardware_ready:
            imgui.text("Host: OK")
        else:
            imgui.text_colored("Please connect the hardware!", 0.70, 0.36, 0.05)
        if current_device and not panel._port_input:
            panel._port_input = current_device

        imgui.text("Port:")
        imgui.same_line()
        imgui.push_item_width(max(120.0, float(imgui.get_content_region_available_width()) - 112.0))
        changed_port, new_port = imgui.input_text("##hardware_port", panel._port_input, 256)
        imgui.pop_item_width()
        if changed_port:
            panel._port_input = str(new_port)
        imgui.same_line()
        if imgui.button("Search"):
            panel.service.request_ports()

        ports = list(state.ports)
        if ports:
            imgui.text("Detected Ports:")
            imgui.same_line()
            for idx, port in enumerate(ports):
                if imgui.small_button(f"{port}##port_{idx}"):
                    panel._port_input = str(port)
                if (idx + 1) < len(ports):
                    imgui.same_line()

        reply_reason = str(state.reply_reason or "").strip()
        reply_reason_key = reply_reason.lower()
        is_perception_reason = reply_reason.lower().startswith("perception")
        device_hint, device_hint_ok = _device_hint(
            reply_reason,
            reply_ok=bool(state.reply_ok),
            ports=ports,
        )
        _reserved_text(
            device_hint,
            color=None if device_hint_ok else (1.0, 0.35, 0.35),
        )

        _control_label("Apply Port")
        if _switch_button("hardware_port_switch", bool(current_device), width=_SWITCH_W):
            if current_device:
                panel.state.set_torque_lock_bypass(False)
                panel.service.disconnect_device()
                panel._port_input = ""
            else:
                panel.state.set_torque_lock_bypass(bool(state.torque_enabled))
                panel.service.set_device(panel._port_input.strip())
        imgui.same_line()
        if _warn_button("hardware_port_abort"):
            panel.state.set_torque_lock_bypass(False)
            panel.service.disconnect_device()
            panel._port_input = ""

        if reply_reason:
            if bool(state.reply_ok):
                if not _is_device_reply(reply_reason_key) and not is_perception_reason:
                    imgui.text(f"Host: {reply_reason}")
            elif not _is_device_reply(reply_reason_key) and not is_perception_reason:
                imgui.text_colored(f"Host: {reply_reason}", 1.0, 0.35, 0.35)
        if str(state.safety_fault).strip():
            imgui.text_colored(f"Safety fault: {state.safety_fault}", 1.0, 0.25, 0.25)
        if state.motor_currents_ma:
            currents_text = ", ".join(f"{k}={int(v)}mA" for k, v in state.motor_currents_ma.items())
            imgui.text_wrapped(f"Currents: {currents_text}")

        _control_label("Torque")
        if _switch_button("hardware_torque_switch", bool(state.torque_enabled), width=_SWITCH_W):
            if state.torque_enabled:
                panel.state.set_torque_lock_bypass(False)
                panel.service.torque_off()
            else:
                resume = bool(panel.state.torque_lock_bypass)
                panel.service.torque_on(resume=resume)
                if not resume:
                    panel.state.set_torque_lock_bypass(False)
        imgui.same_line()
        if _warn_button("hardware_torque_abort"):
            panel.state.set_torque_lock_bypass(False)
            panel.service.torque_off()
