from __future__ import annotations

import imgui

from elesim_ui.helpers import panel_header, scaled, toggle_switch


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
    motor_positions_raw = {}
    motor_positions_deg = {}


_CONTROL_LABEL_W = 96.0
_PORT_LABEL_W = 66.0
_WARN_W = 28.0
_SEARCH_W = 72.0
_MOTOR_ID_BY_NAME = {
    "linear": "1",
    "roll": "2",
    "seg1": "3",
    "seg2": "4",
    "claw": "5",
}


def _control_label(panel, text: str) -> None:
    imgui.text(str(text))
    imgui.same_line(scaled(panel, _CONTROL_LABEL_W))


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


def _draw_motor_position_rows(panel, state) -> None:
    raw = getattr(state, "motor_positions_raw", {}) or {}
    deg = getattr(state, "motor_positions_deg", {}) or {}
    if not raw and not deg:
        return
    imgui.separator()
    _control_label(panel, "Motor Pos")
    imgui.text("raw / deg")
    for name in ("linear", "roll", "seg1", "seg2", "claw"):
        # Robot telemetry is keyed by Dynamixel ID ("1".."5"); the UI uses
        # canonical joint names. Accept either form so valid feedback is not
        # silently hidden from the Hardware panel.
        motor_id = _MOTOR_ID_BY_NAME[name]
        raw_value = raw.get(name, raw.get(motor_id, raw.get(int(motor_id))))
        deg_value = deg.get(name, deg.get(motor_id, deg.get(int(motor_id))))
        if raw_value is None and deg_value is None:
            continue
        raw_text = "-" if raw_value is None else str(raw_value)
        try:
            deg_text = "-" if deg_value is None else f"{float(deg_value):.1f}"
        except (TypeError, ValueError):
            deg_text = "-"
        imgui.text(f"{name:<6} {raw_text:>6} / {deg_text:>7}")


def _draw_motor_fault_recovery(panel, state) -> None:
    fault = str(getattr(state, "safety_fault", "") or "")
    if not fault:
        panel._motor_reboot_confirmed = False
        return

    imgui.separator()
    if "hardware error occurred" not in fault.lower():
        panel._motor_reboot_confirmed = False
        imgui.text("Safety fault")
        imgui.text_wrapped(fault)
        imgui.text_wrapped(
            "This fault can be cleared only after Robot verifies Torque is OFF, "
            "telemetry is healthy, and the current is back within its limit. "
            "Clearing the fault does not turn Torque back on."
        )
        if bool(state.torque_enabled):
            imgui.text("Turn Torque OFF before clearing the safety fault.")
        elif imgui.button("Recheck hardware and clear safety fault##clear_safety_fault"):
            panel.service.clear_safety_fault()
        return

    imgui.text("DYNAMIXEL hardware shutdown")
    imgui.text_wrapped(fault)
    imgui.text_wrapped(
        "A DYNAMIXEL reboot may briefly restore torque. Physically support the arm, "
        "keep clear, and leave Torque OFF after recovery."
    )
    changed, confirmed = imgui.checkbox(
        "Arm is physically supported and the area is clear##motor_reboot_ack",
        bool(getattr(panel, "_motor_reboot_confirmed", False)),
    )
    if changed:
        panel._motor_reboot_confirmed = bool(confirmed)
    if bool(state.torque_enabled):
        imgui.text("Turn Torque OFF before motor recovery.")
    elif panel._motor_reboot_confirmed and imgui.button(
        "Reboot faulted motor and clear safety latch##motor_reboot"
    ):
        panel.service.reboot_faulted_motor(arm_supported=True)
        panel._motor_reboot_confirmed = False


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
        if toggle_switch(
            panel,
            "hardware_port_switch",
            bool(current_device),
        ):
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
        if toggle_switch(
            panel,
            "hardware_torque_switch",
            bool(state.torque_enabled),
        ):
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

        _draw_motor_position_rows(panel, state)
        _draw_motor_fault_recovery(panel, state)
