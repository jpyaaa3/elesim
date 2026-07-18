from __future__ import annotations

import imgui

from elesim_ui.helpers import panel_header, scaled


_IK_LABEL_W = 82.0


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
