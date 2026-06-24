from __future__ import annotations

import time
import sys
from typing import Optional

import glfw
import imgui
from imgui.integrations.glfw import GlfwRenderer

from engine.controller import ControlService, HostState, PanelState
from engine.config_loader import PerceptionConfig, PickConfig
from engine.controller.perception_capture import load_mock_world_xyz_from_detector_path
from ui.helpers import set_panel_header_font
from ui.theme import CONTENT_FONT_CANDIDATES, FONT_SPEC, TITLE_FONT

from .panels import (
    draw_control_4dof_panel,
    draw_go2_panel,
    draw_hardware_panel,
    draw_ik_panel,
    draw_perception_panel,
    draw_sag_panel,
)

def _set_style_color(style, name: str, rgba: tuple[float, float, float, float]) -> None:
    color_id = getattr(imgui, name, None)
    if color_id is None:
        return
    try:
        style.colors[color_id] = rgba
    except Exception:
        pass


class ControlPanel:
    """External ImGui window that draws and edits PanelState."""

    def __init__(
        self,
        state: PanelState,
        service: ControlService,
        *,
        use_hardware: bool = False,
        use_go2: bool = False,
        go2_teleop_vx_mps: float = 0.35,
        go2_teleop_vy_mps: float = 0.25,
        go2_teleop_wz_radps: float = 0.80,
        perception_cfg: PerceptionConfig | None = None,
        pick_cfg: PickConfig | None = None,
    ):
        self.state = state
        self.service = service
        self._use_hardware = bool(use_hardware)
        self._use_go2 = bool(use_go2)
        self._go2_teleop_vx_mps = float(go2_teleop_vx_mps)
        self._go2_teleop_vy_mps = float(go2_teleop_vy_mps)
        self._go2_teleop_wz_radps = float(go2_teleop_wz_radps)
        pc = perception_cfg or PerceptionConfig()
        self._perception_run_local = bool(pc.run_local)
        pk = pick_cfg or PickConfig()
        self._stop = False
        self._hw_header_init_open = False
        self._ctrl_header_init_open = False
        self._ik_header_init_open = False
        self._go2_header_init_open = False
        self._perception_header_init_open = False
        self._sag_header_init_open = False
        self._perception_config_path_draft = str(pc.detector_config)
        self._perception_mode_draft = str(pc.mode)
        self._perception_detector_draft = str(pc.detector)
        self._perception_target_label_draft = str(pc.target_label)
        self._perception_yolo_device_draft = str(pc.yolo_device)
        self._perception_publish_hz_draft = float(pc.publish_hz)
        self._perception_show_preview_draft = bool(pc.show_preview)
        self._perception_pipeline_draft = str(pc.pipeline)
        self._perception_tracker_draft = str(pc.tracker)
        self.state.visual_target_label = str(pc.target_label).strip()
        self.state.visual_target_scale = float(pk.target_scale)
        self.state.visual_center_tol = float(pk.center_tol)
        self.state.visual_target_uv_u = float(pk.target_uv_u)
        self.state.visual_target_uv_v = float(pk.target_uv_v)
        self.state.visual_scale_tol = float(pk.scale_tol)
        self.state.visual_ready_distance_m = float(pk.ready_pose_standoff_m)
        self.state.visual_look_distance_m = float(pk.look_pose_standoff_m)
        mock_xyz = load_mock_world_xyz_from_detector_path(pc.resolved_detector_config_path())
        if mock_xyz is not None:
            self.state.set_mock_object_world_xyz(*mock_xyz)
        self._ctrl_window_init = False
        self._port_input = ""
        self._host_state: Optional[HostState] = None
        self._sag_model_path_draft = str(self.state.sag_model_path)
        self._sag_status_text = ""
        self._sag_status_ok = True
        linear_off, roll_off, s1_off, s2_off, rev = self.state.offset_values()
        self._offset_linear_draft = float(linear_off)
        self._offset_roll_draft = float(roll_off)
        self._offset_s1_draft = float(s1_off)
        self._offset_s2_draft = float(s2_off)
        self._offset_revision_seen = int(rev)
        self._go2_was_active = False
        self._go2_obstacles_avoid_enabled = False

    def _install_ui_font(self) -> None:
        io = imgui.get_io()
        fonts = getattr(io, "fonts", None)
        if fonts is None or not hasattr(fonts, "add_font_from_file_ttf"):
            return
        font_path = next((p for p in CONTENT_FONT_CANDIDATES if p.exists()), None)
        if font_path is None:
            return
        try:
            font = fonts.add_font_from_file_ttf(str(font_path), float(FONT_SPEC.content_px))
            if font is not None:
                io.font_default = font
                print(f"[ui] content font: {font_path} ({FONT_SPEC.content_px:.1f}px)")
            header_path = TITLE_FONT
            if header_path.exists():
                header_font = fonts.add_font_from_file_ttf(str(header_path), float(FONT_SPEC.title_px))
                if header_font is not None:
                    set_panel_header_font(header_font)
                    print(f"[ui] title font: {header_path} ({FONT_SPEC.title_px:.1f}px)")
        except Exception as exc:
            print(f"[ui] font load skipped: {exc}")

    def _install_ui_style(self) -> None:
        style = imgui.get_style()
        for attr, value in (
            ("window_rounding", 6.0),
            ("child_rounding", 5.0),
            ("frame_rounding", 4.0),
            ("grab_rounding", 4.0),
            ("popup_rounding", 5.0),
            ("scrollbar_rounding", 6.0),
            ("tab_rounding", 5.0),
            ("window_border_size", 1.0),
            ("child_border_size", 1.0),
            ("frame_border_size", 1.0),
        ):
            if hasattr(style, attr):
                try:
                    setattr(style, attr, value)
                except Exception:
                    pass
        for attr, value in (
            ("item_spacing", (8.0, 7.0)),
            ("frame_padding", (7.0, 4.0)),
            ("window_padding", (10.0, 10.0)),
            ("cell_padding", (6.0, 4.0)),
        ):
            if hasattr(style, attr):
                try:
                    current = getattr(style, attr)
                    current.x = float(value[0])
                    current.y = float(value[1])
                except Exception:
                    pass

        colors = {
            "COLOR_TEXT": (0.10, 0.11, 0.13, 1.00),
            "COLOR_TEXT_DISABLED": (0.48, 0.50, 0.54, 1.00),
            "COLOR_WINDOW_BACKGROUND": (0.94, 0.95, 0.96, 1.00),
            "COLOR_CHILD_BACKGROUND": (0.985, 0.985, 0.99, 1.00),
            "COLOR_POPUP_BACKGROUND": (1.00, 1.00, 1.00, 0.98),
            "COLOR_BORDER": (0.74, 0.76, 0.80, 1.00),
            "COLOR_BORDER_SHADOW": (1.00, 1.00, 1.00, 0.00),
            "COLOR_FRAME_BACKGROUND": (1.00, 1.00, 1.00, 1.00),
            "COLOR_FRAME_BACKGROUND_HOVERED": (0.91, 0.95, 1.00, 1.00),
            "COLOR_FRAME_BACKGROUND_ACTIVE": (0.84, 0.90, 1.00, 1.00),
            "COLOR_TITLE_BACKGROUND": (0.88, 0.89, 0.91, 1.00),
            "COLOR_TITLE_BACKGROUND_ACTIVE": (0.82, 0.86, 0.92, 1.00),
            "COLOR_TITLE_BACKGROUND_COLLAPSED": (0.90, 0.91, 0.93, 1.00),
            "COLOR_MENU_BAR_BACKGROUND": (0.91, 0.92, 0.94, 1.00),
            "COLOR_SCROLLBAR_BACKGROUND": (0.93, 0.94, 0.95, 1.00),
            "COLOR_SCROLLBAR_GRAB": (0.70, 0.72, 0.76, 1.00),
            "COLOR_SCROLLBAR_GRAB_HOVERED": (0.62, 0.65, 0.70, 1.00),
            "COLOR_SCROLLBAR_GRAB_ACTIVE": (0.52, 0.56, 0.62, 1.00),
            "COLOR_CHECK_MARK": (0.00, 0.45, 0.95, 1.00),
            "COLOR_SLIDER_GRAB": (0.00, 0.48, 1.00, 1.00),
            "COLOR_SLIDER_GRAB_ACTIVE": (0.00, 0.36, 0.86, 1.00),
            "COLOR_BUTTON": (0.90, 0.91, 0.93, 1.00),
            "COLOR_BUTTON_HOVERED": (0.82, 0.89, 0.98, 1.00),
            "COLOR_BUTTON_ACTIVE": (0.70, 0.82, 0.98, 1.00),
            "COLOR_HEADER": (0.86, 0.88, 0.91, 1.00),
            "COLOR_HEADER_HOVERED": (0.78, 0.86, 0.98, 1.00),
            "COLOR_HEADER_ACTIVE": (0.66, 0.78, 0.96, 1.00),
            "COLOR_SEPARATOR": (0.78, 0.80, 0.84, 1.00),
            "COLOR_SEPARATOR_HOVERED": (0.52, 0.66, 0.86, 1.00),
            "COLOR_SEPARATOR_ACTIVE": (0.34, 0.54, 0.82, 1.00),
            "COLOR_RESIZE_GRIP": (0.70, 0.72, 0.76, 0.45),
            "COLOR_RESIZE_GRIP_HOVERED": (0.42, 0.62, 0.90, 0.75),
            "COLOR_RESIZE_GRIP_ACTIVE": (0.22, 0.48, 0.86, 0.95),
            "COLOR_TAB": (0.86, 0.88, 0.91, 1.00),
            "COLOR_TAB_HOVERED": (0.75, 0.84, 0.98, 1.00),
            "COLOR_TAB_ACTIVE": (0.94, 0.97, 1.00, 1.00),
            "COLOR_TAB_UNFOCUSED": (0.88, 0.89, 0.91, 1.00),
            "COLOR_TAB_UNFOCUSED_ACTIVE": (0.92, 0.94, 0.97, 1.00),
            "COLOR_PLOT_LINES": (0.20, 0.45, 0.75, 1.00),
            "COLOR_PLOT_HISTOGRAM": (0.20, 0.45, 0.75, 1.00),
            "COLOR_TEXT_SELECTED_BACKGROUND": (0.30, 0.58, 0.95, 0.35),
            "COLOR_DRAG_DROP_TARGET": (0.00, 0.45, 0.95, 0.90),
            "COLOR_NAV_HIGHLIGHT": (0.00, 0.45, 0.95, 0.80),
        }
        for name, rgba in colors.items():
            _set_style_color(style, name, rgba)

    def stop(self) -> None:
        self._stop = True

    def sync_offset_drafts(self) -> None:
        linear_off, roll_off, s1_off, s2_off, rev = self.state.offset_values()
        if int(rev) == int(self._offset_revision_seen):
            return
        self._offset_linear_draft = float(linear_off)
        self._offset_roll_draft = float(roll_off)
        self._offset_s1_draft = float(s1_off)
        self._offset_s2_draft = float(s2_off)
        self._offset_revision_seen = int(rev)

    def _draw_panel_stack(self, drawers, *, item_width: float) -> None:
        imgui.push_item_width(float(item_width))
        try:
            first = True
            for draw in drawers:
                if not first:
                    imgui.separator()
                draw(self)
                first = False
        finally:
            imgui.pop_item_width()

    def _draw_controls_window(self) -> None:
        if not self._ctrl_window_init:
            cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
            io = imgui.get_io()
            imgui.set_next_window_position(0.0, 0.0, cond)
            imgui.set_next_window_size(float(io.display_size.x), float(io.display_size.y), cond)
            self._ctrl_window_init = True
        window_flags = getattr(imgui, "WINDOW_NO_TITLE_BAR", 0)
        imgui.begin("Arm Control###arm_control_window", True, flags=window_flags)
        avail_w = max(1.0, float(imgui.get_content_region_available_width()))
        style = imgui.get_style()
        spacing_x = float(getattr(style.item_spacing, "x", 8.0))
        if avail_w >= 720.0:
            col_w = max(300.0, (avail_w - spacing_x) * 0.5)
            item_w = max(120.0, col_w * 0.45)
            imgui.begin_child("left_controls", col_w, 0.0, True)
            self._draw_panel_stack(
                (
                    draw_hardware_panel,
                    draw_control_4dof_panel,
                    draw_go2_panel,
                ),
                item_width=item_w,
            )
            imgui.end_child()
            imgui.same_line()
            imgui.begin_child("right_controls", 0.0, 0.0, True)
            right_w = max(300.0, float(imgui.get_content_region_available_width()))
            self._draw_panel_stack(
                (
                    draw_ik_panel,
                    draw_perception_panel,
                    draw_sag_panel,
                ),
                item_width=max(120.0, right_w * 0.45),
            )
            imgui.end_child()
        else:
            self._draw_panel_stack(
                (
                    draw_hardware_panel,
                    draw_control_4dof_panel,
                    draw_go2_panel,
                    draw_ik_panel,
                    draw_perception_panel,
                    draw_sag_panel,
                ),
                item_width=max(120.0, avail_w * 0.45),
            )
        imgui.end()

    def run(self) -> None:
        if not glfw.init():
            raise SystemExit("glfw.init() failed.")

        glfw.window_hint(glfw.RESIZABLE, True)
        if sys.platform == "darwin":
            # pyimgui programmable pipeline renderer is stable on macOS with an explicit 3.3 core context.
            glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
            glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
            glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
            glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, glfw.TRUE)
        win_w = 800
        win_h = 600
        monitor = glfw.get_primary_monitor()
        if monitor is not None:
            mode = glfw.get_video_mode(monitor)
            if mode is not None:
                width = int(getattr(mode.size, "width", 0) or 0)
                height = int(getattr(mode.size, "height", 0) or 0)
                if width > 0 and height > 0:
                    win_w = int(width * 0.55)
                    win_h = int(height * 0.9)
        window = glfw.create_window(win_w, win_h, "Arm Control", None, None)
        if not window:
            glfw.terminate()
            raise SystemExit("Failed to create GLFW window.")

        glfw.make_context_current(window)

        imgui.create_context()
        self._install_ui_font()
        self._install_ui_style()
        impl = GlfwRenderer(window)

        try:
            while not glfw.window_should_close(window) and not self._stop:
                self._host_state = self.service.refresh_host_state()
                self.sync_offset_drafts()
                glfw.poll_events()
                impl.process_inputs()

                imgui.new_frame()
                self._draw_controls_window()
                imgui.render()

                impl.render(imgui.get_draw_data())
                glfw.swap_buffers(window)
                time.sleep(0.01)
        finally:
            impl.shutdown()
            glfw.terminate()
