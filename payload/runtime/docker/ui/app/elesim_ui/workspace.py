"""Single-window operator workspace; motion and video sources stay independent."""
from __future__ import annotations

import imgui

from .helpers import scaled
from .panels import (
    draw_control_4dof_panel, draw_floor_calibration_panel, draw_go2_panel,
    draw_hardware_panel, draw_ik_panel, draw_imu_model_panel, draw_mock_object_panel,
    draw_object_scan_panel, draw_perception_panel, draw_resolution_panel,
    draw_scene_panel, draw_status_panel,
)
from .panels.go2 import _stop_go2


_TABS = {
    "Scan": (draw_object_scan_panel, draw_scene_panel),
    "Manual": (draw_hardware_panel, draw_control_4dof_panel, draw_go2_panel,
               draw_ik_panel, draw_mock_object_panel),
    "Legacy": (draw_perception_panel,),
    "Status": (draw_status_panel, draw_imu_model_panel,
               draw_floor_calibration_panel, draw_resolution_panel),
}


def workspace_sizes(width: float, height: float, controls_width: float, scale: float, *, gap=None):
    """Keep both panes visible; stack them on narrow windows without scaling text."""
    gap = 8.0 * scale if gap is None else gap
    if width < 900.0 * scale:
        camera_height = max(1.0, (height-gap) * .55)
        return False, width, max(1.0, height-gap-camera_height), camera_height
    side = min(max(360.0*scale, controls_width*scale), 560.0*scale, width*.45)
    return True, side, height, height


def release_manual_input(panel):
    """A hidden teleop pad must release its velocity command immediately."""
    if getattr(panel, '_go2_was_active', False):
        _stop_go2(panel)
    panel._go2_respawn_blocked = True
    panel._control_u_dragging = False
    panel.flush_control_u_draft(force=True)
    panel._clear_respawn_shortcut()


def _draw_header(panel):
    width = float(imgui.get_content_region_available_width())
    imgui.text('EleSim')
    imgui.same_line()
    available = float(imgui.get_content_region_available_width())
    imgui.push_item_width(min(scaled(panel, 320), max(scaled(panel, 150), available*.4)))
    panel._draw_endpoint_selector(dropdown=True)
    imgui.pop_item_width()
    host = panel._host_state
    online = bool(getattr(host, 'connected', False))
    imgui.same_line()
    imgui.text_colored('ONLINE' if online else 'OFFLINE',
                       *((.12, .48, .25) if online else (.65, .30, .15)))
    imgui.same_line()
    imgui.text('Torque '+('ON' if getattr(host, 'torque_enabled', False) else 'OFF'))
    if imgui.get_content_region_available_width() > scaled(panel, 140):
        imgui.same_line()
    if imgui.button('Torque OFF##workspace-torque-off'):
        panel.state.set_torque_lock_bypass(False)
        panel.service.torque_off()
    if getattr(host, 'safety_fault', ''):
        imgui.text_colored(str(host.safety_fault), .78, .18, .18)

    if width >= scaled(panel, 1000) and not getattr(host, 'safety_fault', ''):
        imgui.same_line()
    changed, visible = imgui.checkbox('Controls##workspace-controls', panel._workspace_controls_visible)
    if changed:
        if not visible:
            release_manual_input(panel)
        panel._workspace_controls_visible = visible
        panel._workspace_tab_request = panel._workspace_tab
    for field, label, method in (
        ('gaze_running', 'Stop tracking', 'stop_gaze_stabilizer'),
        ('pick_running', 'Stop pick', 'stop_pick_e2e'),
    ):
        if getattr(panel.state, field, False):
            imgui.same_line()
            if imgui.button(label+'##workspace-'+field):
                getattr(panel.service, method)()
    if panel._workspace_error:
        imgui.text_wrapped(panel._workspace_error)
    imgui.separator()


def _draw_controls(panel, width: float, height: float):
    flags = imgui.WINDOW_NO_SCROLLBAR | imgui.WINDOW_NO_SCROLL_WITH_MOUSE
    imgui.begin_child('workspace-controls', width, height, True, flags=flags)
    try:
        requested = panel._workspace_tab_request
        if imgui.begin_tab_bar('workspace-tabs'):
            try:
                for name, drawers in _TABS.items():
                    flags = imgui.TAB_ITEM_SET_SELECTED if name == requested else 0
                    if imgui.begin_tab_item(name, flags=flags).selected:
                        try:
                            if name != panel._workspace_tab:
                                release_manual_input(panel)
                                panel._workspace_tab = name
                            imgui.begin_child('workspace-content-'+name, 0., 0., False)
                            try:
                                if name == 'Legacy':
                                    imgui.text_wrapped('Detection, gaze tracking and the existing UV/LJI pick workflow.')
                                inner = max(1., float(imgui.get_content_region_available_width()))
                                panel._draw_panel_stack(drawers, item_width=max(scaled(panel, 100), inner*.45))
                            finally:
                                imgui.end_child()
                        finally:
                            imgui.end_tab_item()
            finally:
                imgui.end_tab_bar()
                panel._workspace_tab_request = None
    finally:
        imgui.end_child()


def _draw_camera(panel, width: float, height: float):
    flags = imgui.WINDOW_NO_SCROLLBAR | imgui.WINDOW_NO_SCROLL_WITH_MOUSE
    imgui.begin_child('workspace-camera', width, height, True, flags=flags)
    try:
        if panel._camera_visible:
            imgui.text_wrapped('Camera Views is open in a separate window.')
            if imgui.button('Return camera to workspace'):
                panel._hide_camera_window()
        else:
            panel._draw_camera_contents()
    finally:
        imgui.end_child()


def draw_workspace(panel):
    _draw_header(panel)
    available = imgui.get_content_region_available()
    width, height = max(1., float(available.x)), max(1., float(available.y))
    if not panel._workspace_controls_visible:
        _draw_camera(panel, width, height)
        return
    horizontal, side, controls_height, camera_height = workspace_sizes(
        width, height, panel._workspace_controls_width, panel.ui_resolution_scale(),
        gap=float(imgui.get_style().item_spacing.y))
    if horizontal:
        _draw_controls(panel, side, controls_height)
        imgui.same_line(spacing=0)
        imgui.invisible_button('##workspace-divider', scaled(panel, 8), height)
        if imgui.is_item_active():
            panel._workspace_controls_width = (side + imgui.get_io().mouse_delta.x)/panel.ui_resolution_scale()
        if imgui.is_item_hovered() or imgui.is_item_active():
            imgui.set_mouse_cursor(imgui.MOUSE_CURSOR_RESIZE_EW)
        imgui.same_line(spacing=0)
        _draw_camera(panel, 0., camera_height)
    else:
        _draw_camera(panel, width, camera_height)
        _draw_controls(panel, width, controls_height)
