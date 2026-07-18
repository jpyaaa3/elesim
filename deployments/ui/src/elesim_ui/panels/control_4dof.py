from __future__ import annotations

import imgui

import elesim_protocol.messages as proto
from elesim_ui.helpers import begin_disabled_ui, end_disabled_ui, panel_header, scaled, toggle_switch


_CONTROL_LABEL_W = 66.0
_COMMAND_LABEL_W = 96.0
_OFFSET_INPUT_W = 62.0
_ROW_GAP_W = 10.0
_MIN_SLIDER_W = 132.0
_OFFSET_BUTTON_W = 118.0
_SWITCH_W = 58.0
_WARN_W = 28.0
_EXTEND_ARM_W = 104.0
_RESPAWN_W = 84.0


def _push_locked_slider_style() -> int:
    pushed = 0
    for color in (
        (imgui.COLOR_FRAME_BACKGROUND, 0.78, 0.80, 0.83, 1.0),
        (imgui.COLOR_FRAME_BACKGROUND_HOVERED, 0.78, 0.80, 0.83, 1.0),
        (imgui.COLOR_FRAME_BACKGROUND_ACTIVE, 0.78, 0.80, 0.83, 1.0),
        (imgui.COLOR_SLIDER_GRAB, 0.54, 0.56, 0.60, 1.0),
        (imgui.COLOR_SLIDER_GRAB_ACTIVE, 0.50, 0.52, 0.56, 1.0),
    ):
        try:
            imgui.push_style_color(*color)
            pushed += 1
        except Exception:
            break
    return pushed


def _control_label(panel, text: str) -> None:
    imgui.text(str(text))
    imgui.same_line(scaled(panel, _COMMAND_LABEL_W))


def _warn_button(panel, label: str) -> bool:
    pushed = 0
    for color in (
        (imgui.COLOR_BUTTON, 0.93, 0.48, 0.18, 1.0),
        (imgui.COLOR_BUTTON_HOVERED, 1.0, 0.56, 0.22, 1.0),
        (imgui.COLOR_BUTTON_ACTIVE, 0.78, 0.34, 0.10, 1.0),
    ):
        try:
            imgui.push_style_color(*color)
            pushed += 1
        except Exception:
            break
    try:
        return bool(imgui.button(f"!##{label}", scaled(panel, _WARN_W), 0.0))
    finally:
        if pushed:
            imgui.pop_style_color(pushed)


def _reload_offset_drafts(panel) -> None:
    linear_off, roll_off, s1_off, s2_off, rev = panel.state.offset_values()
    panel._offset_linear_draft = float(linear_off)
    panel._offset_roll_draft = float(roll_off)
    panel._offset_s1_draft = float(s1_off)
    panel._offset_s2_draft = float(s2_off)
    panel._offset_revision_seen = int(rev)


def _apply_offset_drafts(panel) -> None:
    current_linear, current_roll, current_s1, current_s2, _ = panel.state.offset_values()
    current = {
        "linear": float(current_linear),
        "roll": float(current_roll),
        "s1": float(current_s1),
        "s2": float(current_s2),
    }
    for axis, draft_attr in (
        ("linear", "_offset_linear_draft"),
        ("roll", "_offset_roll_draft"),
        ("s1", "_offset_s1_draft"),
        ("s2", "_offset_s2_draft"),
    ):
        value = float(getattr(panel, draft_attr))
        if abs(value - current[axis]) > 1e-9:
            panel.service.set_display_offset(axis, value)


def _draw_offset_input(panel, *, row_id: str, draft_attr: str, editing: bool) -> None:
    value = float(getattr(panel, draft_attr))
    input_w = scaled(panel, _OFFSET_INPUT_W)
    if not editing:
        pushed_colors = 0
        try:
            imgui.push_style_color(imgui.COLOR_BUTTON, 0.82, 0.84, 0.87, 1.0)
            pushed_colors += 1
            imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, 0.82, 0.84, 0.87, 1.0)
            pushed_colors += 1
            imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, 0.82, 0.84, 0.87, 1.0)
            pushed_colors += 1
            imgui.push_style_color(imgui.COLOR_TEXT, 0.42, 0.44, 0.48, 1.0)
            pushed_colors += 1
        except Exception:
            pass
        disable_token = begin_disabled_ui(True)
        try:
            imgui.button(f"{value:.1f}##{row_id}_offset_disabled", input_w, 0.0)
        finally:
            end_disabled_ui(disable_token)
            if pushed_colors:
                imgui.pop_style_color(pushed_colors)
        return

    imgui.push_item_width(input_w)
    try:
        changed, new_value = imgui.input_float(
            f"##{row_id}_offset",
            value,
            0.0,
            0.0,
            format="%.1f",
        )
    finally:
        imgui.pop_item_width()
    if changed:
        setattr(panel, draft_attr, float(new_value))


def _draw_control_row(
    panel,
    *,
    label: str,
    row_id: str,
    value: float,
    min_value: float,
    max_value: float,
    draft_attr: str,
    sliders_locked: bool,
    editing_offsets: bool,
) -> tuple[bool, float]:
    avail_w = max(1.0, float(imgui.get_content_region_available_width()))
    label_w = scaled(panel, _CONTROL_LABEL_W)
    input_w = scaled(panel, _OFFSET_INPUT_W)
    gap_w = scaled(panel, _ROW_GAP_W)
    slider_w = max(
        scaled(panel, _MIN_SLIDER_W),
        avail_w - label_w - input_w - gap_w,
    )
    input_x = label_w + slider_w + gap_w

    imgui.text(str(label))
    imgui.same_line(label_w)
    disable_token = begin_disabled_ui(sliders_locked)
    pushed_slider_colors = _push_locked_slider_style() if sliders_locked else 0
    imgui.push_item_width(slider_w)
    try:
        changed, new_value = imgui.slider_float(
            f"##{row_id}_slider",
            float(value),
            float(min_value),
            float(max_value),
            format="%.1f",
        )
    finally:
        imgui.pop_item_width()
        if pushed_slider_colors:
            imgui.pop_style_color(pushed_slider_colors)
        end_disabled_ui(disable_token)

    imgui.same_line(input_x)
    _draw_offset_input(
        panel,
        row_id=row_id,
        draft_attr=draft_attr,
        editing=bool(editing_offsets),
    )
    return bool(changed), float(new_value)


def _draw_lock_and_offset_row(panel, *, editing_offsets: bool) -> None:
    row_x = float(imgui.get_cursor_pos_x())
    row_w = max(1.0, float(imgui.get_content_region_available_width()))
    button_w = scaled(panel, _OFFSET_BUTTON_W)
    _, paused = imgui.checkbox("Lock", bool(panel.state.paused))
    panel.state.set_paused(bool(paused))

    button_x = row_x + row_w - button_w
    current_x = float(imgui.get_cursor_pos_x())
    if button_x > current_x:
        imgui.same_line(button_x)
    else:
        imgui.same_line()
    if editing_offsets:
        if imgui.button("Apply Offset", button_w, 0.0):
            _apply_offset_drafts(panel)
            panel._offset_editing = False
            panel.sync_offset_drafts()
    else:
        if imgui.button("Change Offset", button_w, 0.0):
            _reload_offset_drafts(panel)
            panel._offset_editing = True


def _draw_gripper_row(panel) -> None:
    _control_label(panel, "Gripper")
    claw_closed = bool(panel.state.claw_closed)
    if toggle_switch(
        panel,
        "gripper_close_switch",
        claw_closed,
        on_color=(0.94, 0.82, 0.42),
        on_hover_color=(1.0, 0.88, 0.48),
    ):
        next_closed = not claw_closed
        panel.state.set_claw_closed(next_closed)
        panel.service.send_claw_command(closed=next_closed)
    imgui.same_line()
    if _warn_button(panel, "gripper_open_abort"):
        panel.state.set_claw_closed(False)
        panel.service.send_claw_command(closed=False)


def _draw_preset_row(panel) -> None:
    _control_label(panel, "Preset")
    if imgui.button("Home", scaled(panel, _SWITCH_W), 0.0):
        panel.service.home_controls()
    imgui.same_line()
    if imgui.button("Extend Arm", scaled(panel, _EXTEND_ARM_W), 0.0):
        panel.service.extend_arm_controls()


def _draw_respawn_row(panel) -> None:
    _control_label(panel, "Respawn")
    if imgui.button("Respawn", scaled(panel, _RESPAWN_W), 0.0):
        panel.service.reset_simulation()
        panel._go2_was_active = False


def draw_control_4dof_panel(panel) -> None:
    if not panel._ctrl_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._ctrl_header_init_open = True
    if not panel_header("4-DOF Controls", visible=True)[0]:
        return

    link_state = panel._host_state if panel._host_state is not None else None
    if link_state is not None and bool(link_state.torque_enabled) and bool(panel.state.torque_lock_bypass):
        panel.state.set_torque_lock_bypass(False)
    torque_lock_bypass = bool(
        panel.state.torque_lock_bypass
        and panel.service.has_client()
        and link_state is not None
    )
    slider_lock_paused = bool(panel.state.paused)
    sliders_locked = bool(
        slider_lock_paused
        or (
            panel._use_hardware
            and (
                (not panel.service.has_client())
                or link_state is None
                or (not bool(link_state.torque_enabled) and not torque_lock_bypass)
            )
        )
    )
    u_now = panel.service.current_control_u()
    cfg = panel.service.control_mapping()
    editing_offsets = bool(getattr(panel, "_offset_editing", False))

    changed_linear, u_linear = _draw_control_row(
        panel,
        label="Linear",
        row_id="linear",
        value=float(u_now.u_linear),
        min_value=float(cfg.linear_u_min),
        max_value=float(proto.linear_motor_u_limit(cfg)),
        draft_attr="_offset_linear_draft",
        sliders_locked=sliders_locked,
        editing_offsets=editing_offsets,
    )

    changed_rdeg, u_roll = _draw_control_row(
        panel,
        label="Roll",
        row_id="roll",
        value=float(u_now.u_roll),
        min_value=float(cfg.roll_u_min),
        max_value=float(cfg.roll_u_max),
        draft_attr="_offset_roll_draft",
        sliders_locked=sliders_locked,
        editing_offsets=editing_offsets,
    )

    changed_s1, u_s1 = _draw_control_row(
        panel,
        label="Seg1",
        row_id="s1",
        value=float(u_now.u_s1),
        min_value=float(cfg.seg_u_min),
        max_value=float(cfg.seg_u_max),
        draft_attr="_offset_s1_draft",
        sliders_locked=sliders_locked,
        editing_offsets=editing_offsets,
    )

    changed_s2, u_s2 = _draw_control_row(
        panel,
        label="Seg2",
        row_id="s2",
        value=float(u_now.u_s2),
        min_value=float(cfg.seg_u_min),
        max_value=float(cfg.seg_u_max),
        draft_attr="_offset_s2_draft",
        sliders_locked=sliders_locked,
        editing_offsets=editing_offsets,
    )

    _draw_lock_and_offset_row(panel, editing_offsets=editing_offsets)

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
    _draw_gripper_row(panel)
    _draw_preset_row(panel)
    _draw_respawn_row(panel)
