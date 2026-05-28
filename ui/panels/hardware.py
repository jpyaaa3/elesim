from __future__ import annotations

import imgui


def draw_hardware_panel(panel) -> None:
    if (not panel._use_hardware) or (not panel.service.has_client()):
        return
    if not panel._hw_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._hw_header_init_open = True
    if imgui.collapsing_header("Hardware", visible=True)[0]:
        state = panel._host_state if panel._host_state is not None else panel.service.current_host_state()
        if state is None:
            imgui.text("Host: OFF")
            return
        imgui.text(f"Host: {'OK' if state.connected else 'OFF'}")
        imgui.text(f"tx_seq={state.tx_seq} rx_age={state.rx_age_s:.2f}s")
        current_device = str(state.device or "").strip()
        if current_device:
            imgui.text(f"Current Port: {current_device}")
            if not panel._port_input:
                panel._port_input = current_device
        changed_port, new_port = imgui.input_text("Port", panel._port_input, 256)
        if changed_port:
            panel._port_input = str(new_port)
        if imgui.button("Search Ports"):
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
        if imgui.button("Apply Port"):
            panel.service.set_device(panel._port_input.strip())
        imgui.same_line()
        if imgui.button("Disconnect Port"):
            panel.service.disconnect_device()
            panel._port_input = ""
        reply_reason = str(state.reply_reason or "").strip()
        if reply_reason:
            if bool(state.reply_ok):
                if reply_reason == "ports":
                    if not ports:
                        imgui.text("No serial ports found")
                else:
                    imgui.text(f"Host: {reply_reason}")
            else:
                imgui.text_colored(f"Host: {reply_reason}", 1.0, 0.35, 0.35)
        if str(state.safety_fault).strip():
            imgui.text_colored(f"Safety fault: {state.safety_fault}", 1.0, 0.25, 0.25)
        if state.motor_currents_ma:
            currents_text = ", ".join(f"{k}={int(v)}mA" for k, v in state.motor_currents_ma.items())
            imgui.text_wrapped(f"Currents: {currents_text}")
        if imgui.button("Torque On"):
            panel.service.torque_on()
        imgui.same_line()
        if imgui.button("Torque Off"):
            panel.service.torque_off()
