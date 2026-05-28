from __future__ import annotations

import imgui


def draw_ik_panel(panel) -> None:
    if not panel._ik_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._ik_header_init_open = True

    if imgui.collapsing_header("Inverse Kinematics", visible=True)[0]:
        ret = imgui.input_float3(
            "target [m]",
            panel.state.target_x,
            panel.state.target_y,
            panel.state.target_z,
            format="%.4f",
        )
        if isinstance(ret, tuple) and len(ret) == 2:
            changed, (x, y, z) = ret
        else:
            changed, x, y, z = ret
        if changed:
            panel.state.set_target(float(x), float(y), float(z))
            panel.service.send_current_target_meta(source="target")

        dir_ret = imgui.input_float3(
            "target dir",
            panel.state.target_vx,
            panel.state.target_vy,
            panel.state.target_vz,
            format="%.3f",
        )
        if isinstance(dir_ret, tuple) and len(dir_ret) == 2:
            changed_dir, (vx, vy, vz) = dir_ret
        else:
            changed_dir, vx, vy, vz = dir_ret
        if changed_dir:
            panel.state.set_target_dir(float(vx), float(vy), float(vz))
            panel.service.send_current_target_meta(source="target")

        if imgui.button("Solve IK"):
            panel.service.start_ik_solve()
        imgui.same_line()
        if imgui.button("Tweak"):
            panel.service.start_tweak()
        imgui.same_line()
        if imgui.button("Stop IK"):
            panel.state.clear_ik_status()

        status = "idle"
        if panel.state.ik_running:
            status = "running"
        if panel.state.ik_converged:
            status = "converged"
        if panel.state.ik_failed:
            status = "failed"
        imgui.text(f"IK status: {status} | err: {panel.state.ik_err_m*1000:.2f} mm")
        if str(panel.state.ik_status_msg).strip():
            imgui.text_wrapped(str(panel.state.ik_status_msg))
