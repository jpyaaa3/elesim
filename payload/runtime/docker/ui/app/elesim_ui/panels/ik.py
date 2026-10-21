from __future__ import annotations

import math
import time

import imgui

from elesim_ui.helpers import panel_header, scaled, begin_disabled_ui, end_disabled_ui


_IK_LABEL_W = 82.0


def _valid_xyz(value) -> tuple[float, float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        xyz = tuple(float(component) for component in value)
    except (TypeError, ValueError, OverflowError):
        return None
    if not all(math.isfinite(component) for component in xyz):
        return None
    return xyz


def _style_spacing_x(panel) -> float:
    style = getattr(imgui, "get_style", lambda: None)()
    spacing = getattr(style, "item_spacing", None)
    if spacing is None:
        return scaled(panel, 8.0)
    if hasattr(spacing, "x"):
        return float(spacing.x)
    return float(spacing[0])


def _draw_float3_input(
    panel,
    label: str,
    values: tuple[float, float, float],
    identifier: str,
    *,
    format: str,
) -> tuple[bool, tuple[float, float, float]]:
    imgui.text(str(label))
    imgui.same_line(scaled(panel, _IK_LABEL_W))
    width_getter = getattr(imgui, "get_content_region_available_width", None)
    available = max(1.0, float(width_getter()) if callable(width_getter) else scaled(panel, 260.0))
    spacing = _style_spacing_x(panel)
    component_w = max(scaled(panel, 40.0), (available - spacing * 2.0) / 3.0)

    changed_any = False
    out = [float(values[0]), float(values[1]), float(values[2])]
    for idx in range(3):
        if idx > 0:
            imgui.same_line()
        imgui.push_item_width(component_w)
        try:
            changed, new_value = imgui.input_float(
                f"##{identifier}_{idx}",
                float(out[idx]),
                0.0,
                0.0,
                format=format,
            )
        finally:
            imgui.pop_item_width()
        if changed:
            out[idx] = float(new_value)
            changed_any = True
    return changed_any, (float(out[0]), float(out[1]), float(out[2]))


def draw_ik_panel(panel) -> None:
    if not panel._ik_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._ik_header_init_open = True

    if panel_header("Inverse Kinematics", visible=True)[0]:
        changed, (x, y, z) = _draw_float3_input(
            panel,
            "Target xyz",
            (
                float(panel.state.target_x),
                float(panel.state.target_y),
                float(panel.state.target_z),
            ),
            "ik_target",
            format="%.4f",
        )
        if changed:
            panel.state.set_target(float(x), float(y), float(z))
            panel.service.send_current_target_meta(source="target")
        imgui.text_disabled("Observer: green marker = Target xyz")

        changed_dir, (vx, vy, vz) = _draw_float3_input(
            panel,
            "Target dir",
            (
                float(panel.state.target_vx),
                float(panel.state.target_vy),
                float(panel.state.target_vz),
            ),
            "ik_target_dir",
            format="%.3f",
        )
        if changed_dir:
            panel.state.set_target_dir(float(vx), float(vy), float(vz))
            panel.service.send_current_target_meta(source="target")

        if imgui.button("Solve"):
            panel.service.start_ik_solve()
        imgui.same_line()
        if imgui.button("Stop"):
            panel.state.clear_ik_status()
        _draw_curobo(panel)


def _path_request(panel, name, *args):
    action = {"name": name, "pending": True, "error": "", "started": time.monotonic()}
    panel._curobo_action = action
    def done(_result):
        action["pending"] = False
    def failed(reason):
        action["pending"], action["error"] = False, str(reason)
    try:
        getattr(panel.service, name)(*args, on_result=done, on_error=failed)
    except Exception as exc:
        failed(exc)


def _draw_curobo(panel):
    status = panel.service.session.service_value("arm_trajectory", {}) or {}
    if not status:
        return
    current_xyz = _valid_xyz(status.get("current_xyz"))
    imgui.separator()
    imgui.text("cuRobo XYZ path (metres, world frame)")
    imgui.text_disabled("Position only; Target dir is not constrained.")
    imgui.text_disabled("Target XYZ is the 20 mm object centre; the path places it between the open claws.")
    if status.get("preview"):
        imgui.text_disabled("Observer: translucent green Ghost previews the planned path; Robot stays still.")
    phase = str(status.get("phase", "idle"))
    available = bool(status.get("available", False))
    active = phase in {"planning", "executing", "settling"}
    action = getattr(panel, "_curobo_action", {})
    pending = bool(action.get("pending"))
    if active:
        imgui.text_disabled("Use current XYZ (trajectory active)")
    elif current_xyz is None:
        imgui.text_disabled("Use current XYZ (measured pose unavailable)")
    else:
        if imgui.button("Use current XYZ"):
            # Keep the data check even when the button is hidden/disabled:
            # a remote snapshot can omit the measured pose.
            if current_xyz is not None:
                panel.state.set_target(*current_xyz)
    token = begin_disabled_ui(not available or active or pending or not status.get("can_plan", True))
    try:
        if imgui.button("Plan XYZ"):
            _path_request(panel, "plan_arm_to_target")
    finally:
        end_disabled_ui(token)
    imgui.same_line()
    token = begin_disabled_ui(phase != "ready" or not status.get("plan_id") or pending)
    try:
        if imgui.button("Execute path"):
            _path_request(panel, "execute_arm_trajectory", str(status["plan_id"]))
    finally:
        end_disabled_ui(token)
    imgui.same_line()
    if imgui.button("Cancel path"):
        _path_request(panel, "cancel_arm_trajectory")
    action = getattr(panel, "_curobo_action", {})
    if action.get("pending"):
        imgui.text_wrapped(f"Request sent; waiting for Pilot acknowledgement ({time.monotonic()-action['started']:.1f} s)")
    if action.get("error"):
        imgui.text_wrapped("Request failed: " + str(action["error"]))
    tracking = status.get("tracking")
    if tracking and tracking.get("error"):
        error = tracking["error"]
        imgui.text_wrapped(f"Tracking: Linear {error[0]*1000:.2f} mm | "
                          f"Roll {error[1]:.4f}, Bend1 {error[2]:.4f}, Bend2 {error[3]:.4f} rad")
    if tracking and tracking.get("goal_error"):
        error = tracking["goal_error"]
        imgui.text_wrapped(f"Goal residual: Linear {error[0]*1000:.2f} mm | "
                          f"Roll {error[1]:.4f}, Bend1 {error[2]:.4f}, Bend2 {error[3]:.4f} rad "
                          "(limit 8 mm / 0.12 rad)")
    if status.get("blocked_reason"):
        imgui.text_wrapped("Planning unavailable: " + str(status["blocked_reason"]))
    imgui.text_wrapped(f"Stage: {status.get('stage', phase)} | planning elapsed: {status.get('elapsed_s', 0):.1f} s")
    imgui.text(f"{phase} | {status.get('waypoint', 0)}/{status.get('waypoints', 0)}"
               f" | {status.get('duration_s', 0):.2f} s")
    if current_xyz is not None:
        xyz = current_xyz
        imgui.text(f"Current XYZ: {xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f} m")
    if status.get("target_xyz") is not None:
        xyz = status["target_xyz"]
        imgui.text(f"Object center XYZ: {xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f} m")
    if status.get("planned_xyz") is not None:
        xyz = status["planned_xyz"]
        imgui.text(f"Planned claw-center XYZ: {xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f} m")
    imgui.text_wrapped(str(status.get("world", "")))
    if not available:
        imgui.text_wrapped("Configure curobo_world on the Jetson Pilot to enable planning.")
    if status.get("reason"):
        imgui.text_wrapped(str(status["reason"]))
