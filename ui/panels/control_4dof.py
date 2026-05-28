from __future__ import annotations

import imgui

from ui.helpers import begin_disabled_ui, end_disabled_ui


def _draw_offset_editor(panel, *, label: str, draft_attr: str, axis: str) -> None:
    imgui.same_line()
    imgui.text("offset")
    imgui.same_line()
    imgui.push_item_width(70.0)
    changed, new_value = imgui.input_float(f"##{label}_offset", float(getattr(panel, draft_attr)), 0.0, 0.0, format="%.1f")
    imgui.pop_item_width()
    if changed:
        setattr(panel, draft_attr, float(new_value))
    imgui.same_line()
    if imgui.button(f"Apply##{label}_offset_apply"):
        panel.service.set_display_offset(axis, float(getattr(panel, draft_attr)))


def draw_control_4dof_panel(panel) -> None:
    if not panel._ctrl_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._ctrl_header_init_open = True
    if not imgui.collapsing_header("4-DOF Controls", visible=True)[0]:
        return

    link_state = panel._host_state if panel._host_state is not None else None
    sliders_locked = bool(
        panel._use_hardware and ((not panel.service.has_client()) or link_state is None or not bool(link_state.torque_enabled))
    )
    u_now = panel.service.current_control_u()
    cfg = panel.service.control_mapping()

    disable_token = begin_disabled_ui(sliders_locked)
    changed_linear, u_linear = imgui.slider_float(
        "linear [u]", float(u_now.u_linear),
        float(cfg.linear_u_min), float(cfg.linear_u_max),
        format="%.1f"
    )
    end_disabled_ui(disable_token)
    _draw_offset_editor(panel, label="linear", draft_attr="_offset_linear_draft", axis="linear")

    disable_token = begin_disabled_ui(sliders_locked)
    changed_rdeg, u_roll = imgui.slider_float(
        "roll [u]", float(u_now.u_roll),
        float(cfg.roll_u_min), float(cfg.roll_u_max),
        format="%.1f"
    )
    end_disabled_ui(disable_token)
    _draw_offset_editor(panel, label="roll", draft_attr="_offset_roll_draft", axis="roll")

    disable_token = begin_disabled_ui(sliders_locked)
    changed_s1, u_s1 = imgui.slider_float(
        "seg1 [u]", float(u_now.u_s1),
        float(cfg.seg_u_min), float(cfg.seg_u_max),
        format="%.1f"
    )
    end_disabled_ui(disable_token)
    _draw_offset_editor(panel, label="s1", draft_attr="_offset_s1_draft", axis="s1")

    disable_token = begin_disabled_ui(sliders_locked)
    changed_s2, u_s2 = imgui.slider_float(
        "seg2 [u]", float(u_now.u_s2),
        float(cfg.seg_u_min), float(cfg.seg_u_max),
        format="%.1f"
    )
    end_disabled_ui(disable_token)
    _draw_offset_editor(panel, label="s2", draft_attr="_offset_s2_draft", axis="s2")

    changed_any = bool((not sliders_locked) and (changed_linear or changed_rdeg or changed_s1 or changed_s2))
    if panel.state.ik_running and changed_any:
        panel.state.clear_ik_status()
    if changed_any:
        partial_u: dict[str, float] = {}
        if changed_linear:
            partial_u["linear"] = float(u_linear)
        if changed_rdeg:
            partial_u["roll"] = float(u_roll)
        if changed_s1:
            partial_u["s1"] = float(u_s1)
        if changed_s2:
            partial_u["s2"] = float(u_s2)
        panel.service.apply_partial_control_u(partial_u)
    if sliders_locked:
        imgui.text("Sliders locked until Torque On")

    tip_xyz = link_state.actual_tip_xyz if link_state is not None else None
    if tip_xyz is None:
        imgui.text("Tip xyz [m]: unavailable")
    else:
        imgui.text(
            "Tip xyz [m]: (%.3f, %.3f, %.3f)"
            % (float(tip_xyz[0]), float(tip_xyz[1]), float(tip_xyz[2]))
        )

    if imgui.button("Open Gripper"):
        panel.state.set_claw_closed(False)
        panel.service.send_claw_command(closed=False)
    imgui.same_line()
    if imgui.button("Close Gripper"):
        panel.state.set_claw_closed(True)
        panel.service.send_claw_command(closed=True)
    if imgui.button("Extend Arm"):
        panel.service.home_controls()
    imgui.same_line()
    if imgui.button("Calibration"):
        panel.service.start_calibration()
    _, paused = imgui.checkbox("Lock", panel.state.paused)
    panel.state.set_paused(bool(paused))
    if str(panel.state.calibration_status_msg).strip():
        imgui.text_wrapped(f"Calibration: {panel.state.calibration_status_msg}")
