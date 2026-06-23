from __future__ import annotations

import imgui

from ui.panels.live_visual_status import draw_gaze_status_compact


def _hold_button(label: str) -> bool:
    imgui.button(label)
    return bool(imgui.is_item_active())


def draw_go2_panel(panel) -> None:
    if not panel._use_go2:
        return
    if not panel._go2_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._go2_header_init_open = True
    if not imgui.collapsing_header("GO2 Locomotion", visible=True)[0]:
        return

    vx = 0.0
    vy = 0.0
    wz = 0.0
    active = False

    if _hold_button("Forward"):
        vx += float(panel._go2_teleop_vx_mps)
        active = True
    imgui.same_line()
    if _hold_button("Back"):
        vx -= float(panel._go2_teleop_vx_mps)
        active = True

    if _hold_button("Left"):
        vy += float(panel._go2_teleop_vy_mps)
        active = True
    imgui.same_line()
    if _hold_button("Right"):
        vy -= float(panel._go2_teleop_vy_mps)
        active = True

    if _hold_button("Turn Left"):
        wz += float(panel._go2_teleop_wz_radps)
        active = True
    imgui.same_line()
    if _hold_button("Turn Right"):
        wz -= float(panel._go2_teleop_wz_radps)
        active = True

    stop_now = imgui.button("Stop")
    if stop_now:
        panel.service.send_go2_velocity(vx=0.0, vy=0.0, wz=0.0)
        panel._go2_was_active = False
    imgui.same_line()
    if imgui.button("Reset Sim"):
        panel.service.reset_simulation()
        panel._go2_was_active = False
    elif active:
        panel.service.send_go2_velocity(vx=float(vx), vy=float(vy), wz=float(wz))
        panel._go2_was_active = True
    elif panel._go2_was_active:
        panel.service.send_go2_velocity(vx=0.0, vy=0.0, wz=0.0)
        panel._go2_was_active = False

    host_vel = (0.0, 0.0, 0.0)
    if panel._host_state is not None:
        host_vel = tuple(float(v) for v in panel._host_state.go2_vel)
    imgui.text(
        "Host go2_vel: vx=%.2f vy=%.2f wz=%.2f"
        % (float(host_vel[0]), float(host_vel[1]), float(host_vel[2]))
    )

    imgui.separator()
    imgui.text("Posture")
    if imgui.button("Balance Stand"):
        panel.service.send_go2_velocity(vx=0.0, vy=0.0, wz=0.0)
        panel.service.send_go2_sport_pose(pose="balance_stand")
        panel._go2_was_active = False
    imgui.same_line()
    if imgui.button("Lie Down"):
        panel.service.send_go2_velocity(vx=0.0, vy=0.0, wz=0.0)
        panel.service.send_go2_sport_pose(pose="stand_down")
        panel._go2_was_active = False
    imgui.same_line()
    if imgui.button("Recovery Stand"):
        panel.service.send_go2_velocity(vx=0.0, vy=0.0, wz=0.0)
        panel.service.send_go2_sport_pose(pose="recovery_stand")
        panel._go2_was_active = False

    imgui.separator()
    if imgui.button("Gaze Standing (3a)"):
        panel.service.start_gaze_stabilizer_standing()
    imgui.same_line()
    if imgui.button("Gaze Walking (3b)"):
        panel.service.start_gaze_stabilizer_walking()
    imgui.same_line()
    if imgui.button("Stop Gaze"):
        panel.service.stop_gaze_stabilizer()
    draw_gaze_status_compact(panel)
    if imgui.button("Demo 4: Stop + Grasp"):
        panel.service.start_demo4_stop_and_grasp()
