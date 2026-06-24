from __future__ import annotations

import imgui

from ui.helpers import panel_header
from ui.panels.live_visual_status import draw_gaze_status_compact


_PAD_H = 34.0
_SMALL_H = 26.0


def _button(label: str, width: float, height: float = _SMALL_H) -> bool:
    return bool(imgui.button(label, float(width), float(height)))


def _hold_button(label: str, width: float, height: float = _PAD_H) -> bool:
    imgui.button(label, float(width), float(height))
    return bool(imgui.is_item_active())


def _stop_go2(panel) -> None:
    panel.service.send_go2_velocity(vx=0.0, vy=0.0, wz=0.0)
    panel._go2_was_active = False


def _draw_status(panel) -> None:
    host_vel = (0.0, 0.0, 0.0)
    if panel._host_state is not None:
        host_vel = tuple(float(v) for v in panel._host_state.go2_vel)
    moving = any(abs(v) > 1e-3 for v in host_vel)
    if moving:
        imgui.text_colored("GO2 moving", 0.25, 0.85, 0.35)
    else:
        imgui.text_colored("GO2 idle", 0.65, 0.65, 0.65)
    imgui.same_line()
    imgui.text(
        "vx=%+.2f  vy=%+.2f  wz=%+.2f"
        % (float(host_vel[0]), float(host_vel[1]), float(host_vel[2]))
    )


def _draw_teleop_pad(panel, width: float) -> bool:
    cell = max(54.0, min(88.0, (float(width) - 16.0) / 3.0))
    row_w = cell * 3.0 + 16.0
    active = False
    vx = 0.0
    vy = 0.0
    wz = 0.0

    imgui.begin_group()
    imgui.dummy(cell, _PAD_H)
    imgui.same_line()
    if _hold_button("W##go2_forward", cell):
        vx += float(panel._go2_teleop_vx_mps)
        active = True
    imgui.same_line()
    imgui.dummy(cell, _PAD_H)

    if _hold_button("A##go2_left", cell):
        vy += float(panel._go2_teleop_vy_mps)
        active = True
    imgui.same_line()
    if _button("STOP##go2_stop", cell, _PAD_H):
        _stop_go2(panel)
    imgui.same_line()
    if _hold_button("D##go2_right", cell):
        vy -= float(panel._go2_teleop_vy_mps)
        active = True

    if _hold_button("Q##go2_turn_left", cell):
        wz += float(panel._go2_teleop_wz_radps)
        active = True
    imgui.same_line()
    if _hold_button("S##go2_back", cell):
        vx -= float(panel._go2_teleop_vx_mps)
        active = True
    imgui.same_line()
    if _hold_button("E##go2_turn_right", cell):
        wz -= float(panel._go2_teleop_wz_radps)
        active = True
    imgui.end_group()

    if width > row_w + 130.0:
        imgui.same_line()
        imgui.begin_group()
        imgui.text("Teleop")
        imgui.text("vx %.2f m/s" % float(panel._go2_teleop_vx_mps))
        imgui.text("vy %.2f m/s" % float(panel._go2_teleop_vy_mps))
        imgui.text("wz %.2f rad/s" % float(panel._go2_teleop_wz_radps))
        imgui.end_group()

    if active:
        panel.service.send_go2_velocity(vx=float(vx), vy=float(vy), wz=float(wz))
        panel._go2_was_active = True
    elif panel._go2_was_active:
        _stop_go2(panel)
    return active


def _draw_posture(panel, width: float) -> None:
    btn_w = max(86.0, min(130.0, (float(width) - 16.0) / 3.0))
    if _button("Balance##go2_balance", btn_w):
        _stop_go2(panel)
        panel.service.send_go2_sport_pose(pose="balance_stand")
    imgui.same_line()
    if _button("Lie Down##go2_lie_down", btn_w):
        _stop_go2(panel)
        panel.service.send_go2_sport_pose(pose="stand_down")
    imgui.same_line()
    if _button("Recover##go2_recovery", btn_w):
        _stop_go2(panel)
        panel.service.send_go2_sport_pose(pose="recovery_stand")


def draw_go2_panel(panel) -> None:
    if not panel._use_go2:
        return
    if not panel._go2_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._go2_header_init_open = True
    if not panel_header("GO2 Locomotion", visible=True)[0]:
        return

    width = max(220.0, float(imgui.get_content_region_available_width()))
    _draw_status(panel)
    imgui.separator()
    _draw_teleop_pad(panel, width)

    imgui.separator()
    if _button("Reset Sim##go2_reset", max(100.0, min(150.0, width * 0.35))):
        panel.service.reset_simulation()
        panel._go2_was_active = False
    imgui.same_line()
    changed_avoid, enabled = imgui.checkbox(
        "Obstacle Avoid",
        bool(getattr(panel, "_go2_obstacles_avoid_enabled", False)),
    )
    if changed_avoid:
        panel._go2_obstacles_avoid_enabled = bool(enabled)
        panel.service.send_go2_obstacles_avoid(enabled=bool(enabled))

    _draw_posture(panel, width)

    if imgui.tree_node("Gaze / Demo##go2_gaze_demo"):
        if _button("Gaze Stand##go2_gaze_stand", 96.0):
            panel.service.start_gaze_stabilizer_standing()
        imgui.same_line()
        if _button("Gaze Walk##go2_gaze_walk", 96.0):
            panel.service.start_gaze_stabilizer_walking()
        imgui.same_line()
        if _button("Stop Gaze##go2_stop_gaze", 96.0):
            panel.service.stop_gaze_stabilizer()
        draw_gaze_status_compact(panel)
        if _button("Demo 4: Stop + Grasp##go2_demo4", 176.0):
            panel.service.start_demo4_stop_and_grasp()
        imgui.tree_pop()
