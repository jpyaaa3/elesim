from __future__ import annotations

import imgui

from elesim_ui.helpers import draw_float3_input as _draw_float3_input
from elesim_ui.helpers import panel_header


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
