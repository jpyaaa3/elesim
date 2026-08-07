from __future__ import annotations

import imgui

from elesim_protocol import linear_motor_u_limit
from elesim_ui.helpers import begin_disabled_ui, draw_float3_input, end_disabled_ui, panel_header, scaled

_LABEL_W = 96.0
_BUTTON_W = 84.0
_MARKER_SYNC_TOL = 1e-6

_PHASE_COLORS = {
    "idle": (0.72, 0.72, 0.75),
    "planning": (1.0, 0.85, 0.2),
    "planned": (0.35, 0.85, 0.45),
    "executing": (0.35, 0.70, 1.0),
    "done": (0.35, 0.85, 0.45),
    "failed": (1.0, 0.40, 0.35),
    "cancelled": (1.0, 0.62, 0.22),
}


def _draw_target_u_row(
    panel, *, label: str, row_id: str, value: float, min_value: float, max_value: float
) -> tuple[bool, float]:
    imgui.text(str(label))
    imgui.same_line(scaled(panel, _LABEL_W))
    width = max(scaled(panel, 120.0), float(imgui.get_content_region_available_width()))
    imgui.push_item_width(width)
    try:
        changed, new_value = imgui.slider_float(
            f"##planned_move_{row_id}", float(value), float(min_value), float(max_value), format="%.1f"
        )
    finally:
        imgui.pop_item_width()
    return bool(changed), float(new_value)


def _sync_planned_move_target_from_telemetry(panel) -> None:
    host_state = getattr(panel, "_host_state", None)
    marker_xyz = getattr(host_state, "planned_move_target_xyz", None) if host_state is not None else None
    if marker_xyz is None:
        return
    last = panel._planned_move_last_marker_xyz
    if last is None or any(abs(float(marker_xyz[i]) - float(last[i])) > _MARKER_SYNC_TOL for i in range(3)):
        panel._planned_move_target_xyz = [float(marker_xyz[0]), float(marker_xyz[1]), float(marker_xyz[2])]
        panel._planned_move_last_marker_xyz = tuple(float(v) for v in marker_xyz)


def draw_planned_move_panel(panel) -> None:
    """RRT-planned collision-checked move: Generate shows the path as debug
    markers in the Simulator without moving the arm; Execute streams it.

    Supports two target-entry modes: raw joint/actuator values (mirrors the
    main 4-DOF sliders exactly) or a Cartesian tip target (solved via IK on
    the Controller before planning). The task-space target is also shown as
    a marker in the Simulator -- editing the fields here pushes the marker's
    position live; the fields also adopt the marker's position from
    telemetry, which is what lets a future mouse-dragged marker update these
    fields too.
    """
    if not panel._planned_move_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._planned_move_header_init_open = True

    if not panel_header("Planned Move", visible=True)[0]:
        return

    if not hasattr(panel, "_planned_move_mode"):
        panel._planned_move_mode = "joint"
    if not hasattr(panel, "_planned_move_target_xyz"):
        panel._planned_move_target_xyz = [0.0, 0.0, 0.0]
        panel._planned_move_last_marker_xyz = None
    if not hasattr(panel, "_planned_move_hold_dir"):
        panel._planned_move_hold_dir = False

    _sync_planned_move_target_from_telemetry(panel)

    joint_mode = panel._planned_move_mode == "joint"
    if imgui.radio_button("Joint##planned_move_mode", joint_mode):
        panel._planned_move_mode = "joint"
        joint_mode = True
    imgui.same_line()
    if imgui.radio_button("Task-space##planned_move_mode", not joint_mode):
        panel._planned_move_mode = "task_space"
        joint_mode = False

    if joint_mode:
        cfg = panel.service.control_mapping()
        if not hasattr(panel, "_planned_move_target_u"):
            u_now = panel.service.current_control_u()
            panel._planned_move_target_u = [
                float(u_now.u_linear),
                float(u_now.u_roll),
                float(u_now.u_s1),
                float(u_now.u_s2),
            ]
        target_u = panel._planned_move_target_u

        _, target_u[0] = _draw_target_u_row(
            panel,
            label="Target Linear",
            row_id="linear",
            value=target_u[0],
            min_value=float(cfg.linear_u_min),
            max_value=float(linear_motor_u_limit(cfg)),
        )
        _, target_u[1] = _draw_target_u_row(
            panel,
            label="Target Roll",
            row_id="roll",
            value=target_u[1],
            min_value=float(cfg.roll_u_min),
            max_value=float(cfg.roll_u_max),
        )
        _, target_u[2] = _draw_target_u_row(
            panel,
            label="Target Seg1",
            row_id="s1",
            value=target_u[2],
            min_value=float(cfg.seg_u_min),
            max_value=float(cfg.seg_u_max),
        )
        _, target_u[3] = _draw_target_u_row(
            panel,
            label="Target Seg2",
            row_id="s2",
            value=target_u[3],
            min_value=float(cfg.seg_u_min),
            max_value=float(cfg.seg_u_max),
        )
        panel._planned_move_target_u = target_u
    else:
        changed, (x, y, z) = draw_float3_input(
            panel,
            "Target xyz",
            tuple(panel._planned_move_target_xyz),
            "planned_move_target",
            format="%.4f",
            label_w=_LABEL_W,
        )
        if changed:
            panel._planned_move_target_xyz = [x, y, z]
            panel._planned_move_last_marker_xyz = (x, y, z)

        changed_hold, hold = imgui.checkbox("Hold current direction", bool(panel._planned_move_hold_dir))
        if changed_hold:
            panel._planned_move_hold_dir = bool(hold)

        if changed or changed_hold:
            panel.service.send_planned_move_target(
                *panel._planned_move_target_xyz, hold_current_direction=bool(panel._planned_move_hold_dir)
            )

    status = panel.service.planned_move_status()
    phase = str(status.get("phase", "idle"))
    message = str(status.get("message", ""))
    waypoint_count = int(status.get("waypoint_count", 0))

    imgui.text("Status:")
    imgui.same_line(scaled(panel, _LABEL_W))
    imgui.text_colored(phase, *_PHASE_COLORS.get(phase, (1.0, 1.0, 1.0)))
    if message:
        imgui.text_wrapped(f"  {message}")
    if waypoint_count:
        imgui.text(f"  {waypoint_count} waypoint(s) remaining")

    generate_disabled = phase in ("planning", "executing")
    preview_disabled = phase != "planned"
    execute_disabled = phase != "planned"
    cancel_disabled = phase not in ("planning", "executing")
    button_w = scaled(panel, _BUTTON_W)

    token = begin_disabled_ui(generate_disabled)
    generate_clicked = imgui.button("Generate", button_w, 0.0)
    end_disabled_ui(token)
    if generate_clicked and not generate_disabled:
        if joint_mode:
            # Send raw display-u values -- the Controller applies the exact same
            # offset-aware conversion apply_control_u() uses for the main 4-DOF
            # sliders, so a given u value always means the same target regardless
            # of which panel it was entered through.
            target_u = panel._planned_move_target_u
            panel.service.start_planned_move_generate(
                target_u_linear=float(target_u[0]),
                target_u_roll=float(target_u[1]),
                target_u_s1=float(target_u[2]),
                target_u_s2=float(target_u[3]),
            )
        else:
            panel.service.start_planned_move_generate_task_space(
                target_xyz=tuple(panel._planned_move_target_xyz),
                hold_current_direction=bool(panel._planned_move_hold_dir),
            )

    imgui.same_line()
    token = begin_disabled_ui(preview_disabled)
    preview_clicked = imgui.button("Preview", button_w, 0.0)
    end_disabled_ui(token)
    if preview_clicked and not preview_disabled:
        panel.service.start_planned_move_preview()

    imgui.same_line()
    token = begin_disabled_ui(execute_disabled)
    execute_clicked = imgui.button("Execute", button_w, 0.0)
    end_disabled_ui(token)
    if execute_clicked and not execute_disabled:
        panel.service.start_planned_move_execute()

    imgui.same_line()
    token = begin_disabled_ui(cancel_disabled)
    cancel_clicked = imgui.button("Cancel", button_w, 0.0)
    end_disabled_ui(token)
    if cancel_clicked and not cancel_disabled:
        panel.service.stop_pick_e2e()
