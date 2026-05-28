from __future__ import annotations

import imgui


def draw_pick_fsm_panel(panel) -> None:
    if not panel._pick_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._pick_header_init_open = True

    if imgui.collapsing_header("Pick FSM", visible=True)[0]:
        state = panel._host_state if panel._host_state is not None else panel.service.current_host_state()
        if state is None:
            imgui.text("Host: OFF")
            return
        if panel.service.has_client():
            if imgui.button("Start Pick"):
                panel.service.pick_start()
            imgui.same_line()
            if imgui.button("Stop Pick"):
                panel.service.pick_stop()
            imgui.same_line()
            if imgui.button("Reset Attempt"):
                panel.service.pick_reset()
        pick_stage = str(state.pick_stage or "").strip()
        if not pick_stage:
            imgui.text("Pick stage: -")
            imgui.text("Pick error: -")
            imgui.text("Pick uncertainty: -")
            imgui.text("Pick attempt: 0")
            return
        pick_error_txt = "-" if state.pick_error_m is None else f"{float(state.pick_error_m):.4f} m"
        pick_uncertainty_txt = "-" if state.pick_uncertainty is None else f"{float(state.pick_uncertainty):.6f}"
        imgui.text(f"Pick stage: {pick_stage}")
        imgui.text(f"Pick error: {pick_error_txt}")
        imgui.text(f"Pick uncertainty: {pick_uncertainty_txt}")
        imgui.text(f"Pick attempt: {int(state.pick_attempt)}")
