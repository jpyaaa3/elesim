from __future__ import annotations

import imgui
import time


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

        imgui.separator()
        obs = panel.service.current_visual_observation(panel._host_state)
        perceived_label = ""
        perceived_conf = 0.0
        perceived_uv = None
        perceived_scale = None
        perceived_age = None
        if obs is not None:
            perceived_label = str(obs.label)
            perceived_conf = float(obs.confidence)
            perceived_uv = tuple(obs.center_uv)
            perceived_scale = float(obs.scale)
            perceived_age = max(0.0, float(time.time() - float(obs.timestamp_s)))
        elif panel._host_state is not None and float(panel._host_state.perceived_timestamp_s) > 0.0:
            perceived_label = str(panel._host_state.perceived_object_label)
            perceived_conf = float(panel._host_state.perceived_object_confidence)
            perceived_uv = panel._host_state.perceived_center_uv
            perceived_scale = panel._host_state.perceived_scale
            perceived_age = max(0.0, float(time.time() - float(panel._host_state.perceived_timestamp_s)))

        imgui.text("Visual Servo")
        label_text = perceived_label if perceived_label else "(none)"
        imgui.text(f"Detection: {label_text} | conf: {perceived_conf:.2f}")
        if perceived_uv is None or perceived_scale is None:
            imgui.text("Observation: unavailable")
        else:
            imgui.text(
                "Observation: uv=(%.3f, %.3f) scale=%.4f age=%.2fs"
                % (float(perceived_uv[0]), float(perceived_uv[1]), float(perceived_scale), float(perceived_age or 0.0))
            )

        changed_scale, new_scale = imgui.input_float(
            "target scale",
            float(panel.state.visual_target_scale),
            step=0.01,
            step_fast=0.05,
            format="%.3f",
        )
        if changed_scale:
            panel.state.visual_target_scale = max(0.001, float(new_scale))

        changed_label, new_label = imgui.input_text(
            "visual label",
            str(panel.state.visual_target_label),
            64,
        )
        if changed_label:
            panel.state.visual_target_label = str(new_label).strip()

        if imgui.button("Start Visual Servo"):
            panel.service.start_visual_servo()
        imgui.same_line()
        if imgui.button("Center Object"):
            panel.service.start_visual_centering()
        imgui.same_line()
        if imgui.button("Stop Visual Servo"):
            panel.service.stop_visual_servo()

        changed_tilt_deg, tilt_deg = imgui.input_float(
            "tilt angle [deg]",
            float(panel._visual_tilt_deg_draft),
            step=1.0,
            step_fast=5.0,
            format="%.1f",
        )
        if changed_tilt_deg:
            panel._visual_tilt_deg_draft = max(0.0, float(tilt_deg))
        if imgui.button("Apply Tilt Up"):
            panel.service.apply_visual_tilt_angle(direction=-1, angle_deg=float(panel._visual_tilt_deg_draft))
        imgui.same_line()
        if imgui.button("Apply Tilt Down"):
            panel.service.apply_visual_tilt_angle(direction=+1, angle_deg=float(panel._visual_tilt_deg_draft))
        clicked = imgui.button("Tilt Up")
        hovered = bool(getattr(imgui, "is_item_hovered", lambda: False)())
        mouse_down = bool(getattr(imgui, "is_mouse_down", lambda _btn=0: False)(0))
        active = bool(hovered and mouse_down)
        panel.run_repeat_button(
            "visual_tilt_up",
            clicked=bool(clicked),
            active=bool(active),
            action=lambda: panel.service.nudge_visual_tilt(-1),
        )
        imgui.same_line()
        clicked = imgui.button("Tilt Down")
        hovered = bool(getattr(imgui, "is_item_hovered", lambda: False)())
        mouse_down = bool(getattr(imgui, "is_mouse_down", lambda _btn=0: False)(0))
        active = bool(hovered and mouse_down)
        panel.run_repeat_button(
            "visual_tilt_down",
            clicked=bool(clicked),
            active=bool(active),
            action=lambda: panel.service.nudge_visual_tilt(+1),
        )

        visual_status = "idle"
        if panel.state.visual_running:
            visual_status = "running"
        if panel.state.visual_failed:
            visual_status = "failed"
        imgui.text(f"Visual status: {visual_status}")
        if str(panel.state.visual_status_msg).strip():
            imgui.text_wrapped(str(panel.state.visual_status_msg))
