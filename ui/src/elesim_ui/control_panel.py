from __future__ import annotations

import time
import sys
from typing import Callable, Optional

import glfw
import imgui
from imgui.integrations.glfw import GlfwRenderer

from elesim_ui.models import (
    ControlService,
    GazeStabilizerConfig,
    HardwareConfig,
    HostState,
    PanelState,
    PerceptionConfig,
    PickConfig,
    gaze_config_to_dict,
)
from elesim_ui.helpers import scaled, set_panel_header_font
from elesim_ui.sim_view import SimView
from elesim_ui.theme import CONTENT_FONT_CANDIDATES, FONT_SPEC, TITLE_FONT, add_font_with_korean_ranges
from elesim_ui import file_dialog

from .panels import (
    draw_control_4dof_panel,
    draw_go2_panel,
    draw_hardware_panel,
    draw_ik_panel,
    draw_perception_panel,
    draw_resolution_panel,
    draw_sag_panel,
    draw_status_panel,
)

_DEFAULT_SPACING_X = 8.0
_DEFAULT_WINDOW_PADDING_X = 11.0
_GO2_PAD_MIN_CELL_W = 36.0
_GO2_PAD_MIN_ROW_W = _GO2_PAD_MIN_CELL_W * 5.0 + _DEFAULT_SPACING_X * 4.0
_CONTROL_COLUMN_SCALE = 0.90
_FIRST_COLUMN_MIN_W = _GO2_PAD_MIN_ROW_W + _DEFAULT_WINDOW_PADDING_X * 2.0
_SECOND_COLUMN_MIN_W = 264.0
_CONTROL_COLUMN_MIN_W = max(_FIRST_COLUMN_MIN_W, _SECOND_COLUMN_MIN_W)
_THREE_COLUMN_BASE_W = _CONTROL_COLUMN_MIN_W / _CONTROL_COLUMN_SCALE
_THREE_COLUMN_MIN_W = _THREE_COLUMN_BASE_W * 3.0 + _DEFAULT_SPACING_X * 2.0
_TWO_COLUMN_MIN_W = 720.0
_INITIAL_WINDOW_W = 1440
_INITIAL_WINDOW_H = 900
_UI_RESOLUTION_PRESETS = (
    ("Small 80%", 0.80),
    ("Default 100%", 1.00),
    ("Large 120%", 1.20),
)


def _set_style_color(style, name: str, rgba: tuple[float, float, float, float]) -> None:
    color_id = getattr(imgui, name, None)
    if color_id is None:
        return
    try:
        style.colors[color_id] = rgba
    except Exception:
        pass


def draw_sim_video_panel(panel: "ControlPanel") -> None:
    panel._draw_sim_video()


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
        hardware_cfg: HardwareConfig | None = None,
        perception_cfg: PerceptionConfig | None = None,
        pick_cfg: PickConfig | None = None,
        gaze_cfg: GazeStabilizerConfig | None = None,
        sim_session: Optional[object] = None,
        endpoint_select: Optional[Callable[[str, str], None]] = None,
    ):
        self.state = state
        self.service = service
        self._sim_view = (
            None if sim_session is None else SimView(sim_session)
        )
        self._endpoint_select = endpoint_select
        self._endpoint_cache: list[object] = []
        self._active_endpoint_cache = ""
        self._endpoint_cache_at = 0.0
        self._use_hardware = bool(use_hardware)
        self._use_go2 = bool(use_go2)
        self._go2_teleop_vx_mps = float(go2_teleop_vx_mps)
        self._go2_teleop_vy_mps = float(go2_teleop_vy_mps)
        self._go2_teleop_wz_radps = float(go2_teleop_wz_radps)
        hw_cfg = hardware_cfg or HardwareConfig()
        self._current_yellow_ma = abs(int(hw_cfg.current_yellow_ma))
        self._current_limit_ma = abs(int(hw_cfg.current_limit_ma))
        pc = perception_cfg or PerceptionConfig()
        self._perception_provider_draft = str(getattr(pc, "provider", "") or ("local" if pc.run_local else "host"))
        self._perception_run_local = self._perception_provider_draft.strip().lower() != "host" and bool(pc.run_local)
        self._perception_real_provider_draft = (
            self._perception_provider_draft
            if str(pc.mode).strip().lower() != "sim"
            else "local"
        )
        pk = pick_cfg or PickConfig()
        gz = gaze_cfg or getattr(service, "gaze_config", GazeStabilizerConfig())
        self._stop = False
        self._hw_header_init_open = False
        self._ctrl_header_init_open = False
        self._ik_header_init_open = False
        self._go2_header_init_open = False
        self._perception_header_init_open = False
        self._status_header_init_open = False
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
        self._gaze_config_draft = gaze_config_to_dict(gz)
        self._gaze_config_seen_signature = ""
        self._gaze_config_last_source = "local"
        self._gaze_config_pending_until = 0.0
        self.state.visual_target_label = str(pc.target_label).strip()
        self.state.visual_target_scale = float(pk.target_scale)
        self.state.visual_center_tol = float(pk.center_tol)
        self.state.visual_target_uv_u = float(pk.target_uv_u)
        self.state.visual_target_uv_v = float(pk.target_uv_v)
        self.state.visual_scale_tol = float(pk.scale_tol)
        self.state.visual_ready_distance_m = float(pk.ready_pose_standoff_m)
        self.state.visual_look_distance_m = float(pk.look_pose_standoff_m)
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
        self._offset_editing = False
        # The motion sliders are fed by a 10 Hz remote snapshot, while the
        # ImGui frame runs much faster.  Keep a local draft during a drag so a
        # delayed Pilot snapshot cannot overwrite the value under the mouse.
        # Outbound updates are latest-only and rate limited; the Pilot already
        # coalesces target motion, so sending every mouse event only adds DDS
        # request/response traffic without improving control latency.
        self._control_u_draft: dict[str, float] | None = None
        self._control_u_dirty = False
        self._control_u_dragging = False
        self._control_u_last_change_at = 0.0
        self._control_u_last_send_at = 0.0
        self._control_u_pending_until = 0.0
        self._control_u_pending: dict[str, float] = {}
        self._control_u_send_period_s = 1.0 / 30.0
        self._go2_was_active = False
        # GO2 teleop is a deadman command, not a render-rate stream.  Keep
        # the command cadence below the UI frame rate so a 60/144 Hz display
        # cannot turn one held button into an equivalent DDS request storm.
        # A stop is emitted once per active interval, even if SPACE remains
        # held for several frames.
        self._go2_last_command_at = 0.0
        self._go2_send_period_s = 1.0 / 20.0
        self._go2_stop_sent = False
        self._go2_obstacles_avoid_enabled = False
        self._resolution_header_init_open = False
        self._glfw_window = None
        self._camera_window = None
        self._camera_visible = False
        self._camera_next_draw_at = 0.0
        self._camera_draw_period_s = 1.0 / 30.0
        self._main_imgui_context = None
        self._camera_imgui_context = None
        self._main_imgui_impl = None
        self._camera_imgui_impl = None
        self._ui_resolution_requested_scale = 1.0
        self._ui_resolution_scale = 1.0
        self._ui_resolution_base_w = _INITIAL_WINDOW_W
        self._ui_resolution_base_h = _INITIAL_WINDOW_H
        self._ui_style_base_scalars: dict[str, float] = {}
        self._ui_style_base_vectors: dict[str, tuple[float, float]] = {}
        self._pending_file_browse: Optional[tuple[str, str]] = None

    def _draw_endpoint_selector(self) -> None:
        now = time.monotonic()
        if now - self._endpoint_cache_at >= 0.5:
            try:
                self._endpoint_cache = list(self.service.available_endpoints)
                self._active_endpoint_cache = str(self.service.active_endpoint)
                self._endpoint_cache_at = now
            except Exception:
                pass
        endpoints = self._endpoint_cache
        active = self._active_endpoint_cache
        if not endpoints:
            imgui.text_disabled("TARGET: waiting for endpoint")
            return
        imgui.text("TARGET")
        for index, endpoint in enumerate(endpoints):
            if isinstance(endpoint, dict):
                endpoint_id = str(endpoint.get("endpoint_id", ""))
                role = str(endpoint.get("role", ""))
            else:
                endpoint_id = str(getattr(endpoint, "endpoint_id", ""))
                role = str(getattr(endpoint, "role", ""))
            if not endpoint_id:
                continue
            selected = endpoint_id == active
            label = f"{'* ' if selected else ''}{role}: {endpoint_id}##endpoint-{endpoint_id}"
            if imgui.button(label):
                if self._endpoint_select is not None:
                    self._endpoint_select(endpoint_id, role)
                else:
                    self.service.select_endpoint(endpoint_id)
            if index + 1 < len(endpoints):
                imgui.same_line()

    def _draw_sim_video(self) -> None:
        self._draw_endpoint_selector()
        if self._sim_view is None:
            return
        snapshot = self._sim_view.session.snapshot
        connected = len(snapshot.connected_streams)
        imgui.text(f"SIM CAMERA  {connected}/2 streams")
        if self._camera_visible:
            imgui.text_disabled("Sim Camera 창이 별도로 열려 있습니다.")
            return
        if imgui.button("Open Sim Camera##open-sim-camera"):
            self._show_camera_window()

    def _draw_camera_contents(self) -> None:
        if self._sim_view is None:
            imgui.text_disabled("Sim session is not configured")
            return
        self._draw_endpoint_selector()
        self._sim_view.draw()

    @staticmethod
    def _set_imgui_context(context: object | None) -> None:
        setter = getattr(imgui, "set_current_context", None)
        if context is not None and setter is not None:
            setter(context)

    def _show_camera_window(self) -> None:
        window = self._camera_window
        if window is None:
            return
        self._camera_visible = True
        self._camera_next_draw_at = 0.0
        try:
            glfw.show_window(window)
            glfw.focus_window(window)
        except Exception:
            pass

    def _hide_camera_window(self) -> None:
        window = self._camera_window
        if window is None:
            return
        self._camera_visible = False
        self._camera_next_draw_at = 0.0
        try:
            glfw.hide_window(window)
            glfw.set_window_should_close(window, glfw.FALSE)
        except Exception:
            pass

    def _draw_camera_window(self) -> None:
        window = self._camera_window
        context = self._camera_imgui_context
        if window is None or context is None or not self._camera_visible:
            return
        now = time.monotonic()
        if now < float(getattr(self, "_camera_next_draw_at", 0.0)):
            return
        self._camera_next_draw_at = now + max(
            0.01,
            float(getattr(self, "_camera_draw_period_s", 1.0 / 30.0)),
        )
        glfw.make_context_current(window)
        self._set_imgui_context(context)
        impl = self._camera_imgui_impl
        if impl is None:
            return
        impl.process_inputs()
        imgui.new_frame()
        # Keep the camera contents anchored to the native window in the same
        # way as the main control surface.  A normal ImGui child window has a
        # title bar and can be dragged inside the native window, which leaves
        # the video surface floating and makes camera mouse gestures compete
        # with window movement.  The native GLFW window remains resizable;
        # this ImGui surface simply follows its full client area.
        cond = getattr(imgui, "ALWAYS", 0)
        io = imgui.get_io()
        imgui.set_next_window_position(0.0, 0.0, cond)
        imgui.set_next_window_size(
            float(io.display_size.x),
            float(io.display_size.y),
            cond,
        )
        flags = getattr(imgui, "WINDOW_NO_TITLE_BAR", 0)
        opened = imgui.begin("Sim Camera###sim_camera_window", True, flags=flags)
        visible = opened[0] if isinstance(opened, tuple) else bool(opened)
        if visible:
            self._draw_camera_contents()
        imgui.end()
        imgui.render()
        impl.render(imgui.get_draw_data())
        glfw.swap_buffers(window)
        if isinstance(opened, tuple) and len(opened) > 1 and not opened[1]:
            self._hide_camera_window()

    def request_file_browse(self, *, kind: str, initial_path: str) -> None:
        self._pending_file_browse = (str(kind), str(initial_path))

    def _process_pending_file_browse(self) -> None:
        pending = self._pending_file_browse
        if pending is None:
            return
        self._pending_file_browse = None
        kind, initial_path = pending
        if kind == "sag":
            from elesim_ui.panels.sag import browse_sag_model_path

            selected = browse_sag_model_path(initial_path)
            if selected:
                self._sag_model_path_draft = str(selected)
            return
        if kind == "perception_detector":
            from elesim_ui.panels.perception import browse_detector_config_path

            selected = browse_detector_config_path(initial_path)
            if selected:
                self._perception_config_path_draft = str(selected)

    def _install_ui_font(self) -> None:
        io = imgui.get_io()
        fonts = getattr(io, "fonts", None)
        if fonts is None or not hasattr(fonts, "add_font_from_file_ttf"):
            return
        font_path = next((p for p in CONTENT_FONT_CANDIDATES if p.exists()), None)
        if font_path is None:
            return
        try:
            font = add_font_with_korean_ranges(fonts, font_path, float(FONT_SPEC.content_px))
            if font is not None:
                if hasattr(io, "font_default"):
                    io.font_default = font
                print(f"[ui] content font: {font_path} ({FONT_SPEC.content_px:.1f}px)")
            header_path = TITLE_FONT
            if header_path.exists():
                header_font = add_font_with_korean_ranges(fonts, header_path, float(FONT_SPEC.title_px))
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
            ("item_spacing", (8.0, 10.5)),
            ("frame_padding", (8.0, 6.0)),
            ("window_padding", (11.0, 13.5)),
            ("cell_padding", (7.0, 6.0)),
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
        self._capture_ui_style_base()
        self._apply_ui_resolution_scale()

    def _capture_ui_style_base(self) -> None:
        style = imgui.get_style()
        self._ui_style_base_scalars = {}
        for attr in (
            "window_rounding",
            "child_rounding",
            "frame_rounding",
            "grab_rounding",
            "popup_rounding",
            "scrollbar_rounding",
            "tab_rounding",
            "window_border_size",
            "child_border_size",
            "frame_border_size",
            "scrollbar_size",
            "grab_min_size",
        ):
            if hasattr(style, attr):
                try:
                    self._ui_style_base_scalars[attr] = float(getattr(style, attr))
                except Exception:
                    pass
        self._ui_style_base_vectors = {}
        for attr in ("item_spacing", "frame_padding", "window_padding", "cell_padding"):
            if hasattr(style, attr):
                try:
                    value = getattr(style, attr)
                    self._ui_style_base_vectors[attr] = (float(value.x), float(value.y))
                except Exception:
                    pass

    def _apply_ui_resolution_scale(self) -> None:
        scale = float(getattr(self, "_ui_resolution_scale", 1.0) or 1.0)
        try:
            io = imgui.get_io()
            if hasattr(io, "font_global_scale"):
                io.font_global_scale = scale
        except Exception:
            pass
        try:
            style = imgui.get_style()
            for attr, value in self._ui_style_base_scalars.items():
                if hasattr(style, attr):
                    setattr(style, attr, float(value) * scale)
            for attr, value in self._ui_style_base_vectors.items():
                if hasattr(style, attr):
                    target = getattr(style, attr)
                    target.x = float(value[0]) * scale
                    target.y = float(value[1]) * scale
        except Exception:
            pass

    def ui_resolution_presets(self) -> tuple[tuple[str, float], ...]:
        return _UI_RESOLUTION_PRESETS

    def ui_resolution_scale(self) -> float:
        return float(getattr(self, "_ui_resolution_scale", 1.0) or 1.0)

    def ui_resolution_requested_scale(self) -> float:
        return float(getattr(self, "_ui_resolution_requested_scale", 1.0) or 1.0)

    def _set_ui_resolution_effective_scale(self, scale: float) -> None:
        scale = max(0.1, float(scale))
        if abs(scale - self.ui_resolution_scale()) <= 1e-6:
            return
        self._ui_resolution_scale = scale
        self._apply_ui_resolution_scale()

    def _effective_ui_scale_from_window(self) -> float:
        requested = self.ui_resolution_requested_scale()
        window = getattr(self, "_glfw_window", None)
        if window is None:
            return requested
        base_w = max(1.0, float(getattr(self, "_ui_resolution_base_w", _INITIAL_WINDOW_W) or _INITIAL_WINDOW_W))
        base_h = max(1.0, float(getattr(self, "_ui_resolution_base_h", _INITIAL_WINDOW_H) or _INITIAL_WINDOW_H))
        try:
            size = glfw.get_window_size(window)
        except Exception:
            return requested
        try:
            actual_w = float(size[0])
            actual_h = float(size[1])
        except Exception:
            return requested
        if actual_w <= 0.0 or actual_h <= 0.0:
            return requested
        return max(0.1, min(float(requested), actual_w / base_w, actual_h / base_h))

    def _sync_ui_resolution_scale_to_window(self) -> None:
        self._set_ui_resolution_effective_scale(self._effective_ui_scale_from_window())

    def _lock_os_window_size(self) -> None:
        window = getattr(self, "_glfw_window", None)
        if window is None:
            return
        try:
            width, height = glfw.get_window_size(window)
            width = max(1, int(width))
            height = max(1, int(height))
            glfw.set_window_size_limits(window, width, height, width, height)
        except Exception:
            pass

    def set_ui_resolution_scale(self, scale: float) -> None:
        scale = float(scale)
        if abs(scale - self.ui_resolution_requested_scale()) <= 1e-6:
            return
        self._ui_resolution_requested_scale = scale
        self._set_ui_resolution_effective_scale(scale)
        window = getattr(self, "_glfw_window", None)
        if window is not None:
            try:
                glfw.set_window_size(
                    window,
                    max(1, int(round(float(self._ui_resolution_base_w) * scale))),
                    max(1, int(round(float(self._ui_resolution_base_h) * scale))),
                )
            except Exception:
                pass
            self._lock_os_window_size()
            self._sync_ui_resolution_scale_to_window()

    def stop(self) -> None:
        self._stop = True

    def sync_offset_drafts(self) -> None:
        if bool(getattr(self, "_offset_editing", False)):
            return
        linear_off, roll_off, s1_off, s2_off, rev = self.state.offset_values()
        if int(rev) == int(self._offset_revision_seen):
            return
        self._offset_linear_draft = float(linear_off)
        self._offset_roll_draft = float(roll_off)
        self._offset_s1_draft = float(s1_off)
        self._offset_s2_draft = float(s2_off)
        self._offset_revision_seen = int(rev)

    @staticmethod
    def _control_u_dict(value: object) -> dict[str, float]:
        """Return the display-space control values as a stable local draft."""

        return {
            "linear": float(getattr(value, "u_linear", 0.0)),
            "roll": float(getattr(value, "u_roll", 0.0)),
            "s1": float(getattr(value, "u_s1", 0.0)),
            "s2": float(getattr(value, "u_s2", 0.0)),
        }

    def sync_control_u_draft(self, current: object) -> dict[str, float]:
        """Synchronize the slider draft when it is safe to accept remote data.

        The operator snapshot is intentionally asynchronous.  While a slider
        is active, or while its latest command is still making the round trip,
        replacing the local value with that snapshot makes the thumb visibly
        stutter and feel behind the pointer.  Once the remote state catches up
        (or the short acknowledgement grace period expires), normal external
        updates such as IK are accepted again.
        """

        now = time.monotonic()
        remote = self._control_u_dict(current)
        draft = getattr(self, "_control_u_draft", None)
        if draft is None:
            draft = dict(remote)
            self._control_u_draft = draft
            return dict(draft)

        pending_until = float(getattr(self, "_control_u_pending_until", 0.0))
        if bool(getattr(self, "_control_u_dirty", False)) and not pending_until:
            pending_until = now + 1.0
            self._control_u_pending_until = pending_until
        matches_remote = all(
            abs(float(draft[key]) - float(remote[key])) <= 1e-3
            for key in draft
            if key in remote
        )
        if matches_remote:
            self._control_u_dirty = False
            self._control_u_pending_until = 0.0
            pending = getattr(self, "_control_u_pending", None)
            if isinstance(pending, dict):
                pending.clear()
            pending_until = 0.0

        if (
            not bool(getattr(self, "_control_u_dragging", False))
            and not bool(getattr(self, "_control_u_dirty", False))
            and now >= pending_until
        ):
            draft = dict(remote)
            self._control_u_draft = draft
        return dict(draft)

    def update_control_u_draft(self, partial: dict[str, float]) -> None:
        draft = getattr(self, "_control_u_draft", None)
        if draft is None:
            draft = {"linear": 0.0, "roll": 0.0, "s1": 0.0, "s2": 0.0}
            self._control_u_draft = draft
        pending = getattr(self, "_control_u_pending", None)
        if not isinstance(pending, dict):
            pending = {}
            self._control_u_pending = pending
        for key, value in partial.items():
            normalized = str(key).strip().lower()
            if normalized in draft:
                draft[normalized] = float(value)
                pending[normalized] = float(value)
        self._control_u_dirty = True
        self._control_u_last_change_at = time.monotonic()
        self._control_u_pending_until = self._control_u_last_change_at + 1.0

    def flush_control_u_draft(self, *, force: bool = False) -> None:
        """Send at most the latest 30 Hz slider value through the async proxy."""

        if not bool(getattr(self, "_control_u_dirty", False)):
            return
        now = time.monotonic()
        last_send = float(getattr(self, "_control_u_last_send_at", 0.0))
        period = max(0.01, float(getattr(self, "_control_u_send_period_s", 1.0 / 30.0)))
        if not force and now - last_send < period:
            return
        draft = getattr(self, "_control_u_draft", None)
        if not isinstance(draft, dict):
            return
        pending = dict(getattr(self, "_control_u_pending", {}))
        if not pending:
            self._control_u_dirty = False
            return
        # Preserve the service's partial-update semantics: a concurrent gaze
        # or IK update of another axis must not be overwritten by an older UI
        # snapshot.  Multiple changed axes are accumulated into this one
        # latest-only request.
        self.service.apply_partial_control_u(pending)
        self._control_u_pending.clear()
        self._control_u_dirty = False
        self._control_u_last_send_at = now
        self._control_u_pending_until = now + 1.0

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
        cond = getattr(imgui, "ALWAYS", 0)
        io = imgui.get_io()
        imgui.set_next_window_position(0.0, 0.0, cond)
        imgui.set_next_window_size(float(io.display_size.x), float(io.display_size.y), cond)
        self._ctrl_window_init = True
        window_flags = getattr(imgui, "WINDOW_NO_TITLE_BAR", 0)
        imgui.begin("Arm Control###arm_control_window", True, flags=window_flags)
        avail_w = max(1.0, float(imgui.get_content_region_available_width()))
        style = imgui.get_style()
        spacing_x = float(getattr(style.item_spacing, "x", 8.0))
        if avail_w >= scaled(self, _THREE_COLUMN_MIN_W):
            col_w = max(1.0, (avail_w - spacing_x * 2.0) / 3.0)
            first_w = max(1.0, col_w * _CONTROL_COLUMN_SCALE)
            second_w = first_w
            main_w = first_w + second_w + spacing_x
            imgui.begin_child("main_controls_block", main_w, 0.0, True)
            main_inner_w = max(1.0, float(imgui.get_content_region_available_width()))
            inner_col_w = max(1.0, (main_inner_w - spacing_x) * 0.5)
            first_item_w = max(scaled(self, 108.0), inner_col_w * 0.45)
            second_item_w = first_item_w
            imgui.begin_child("primary_controls_inner", inner_col_w, 0.0, False)
            try:
                self._draw_panel_stack(
                    (
                        draw_hardware_panel,
                        draw_control_4dof_panel,
                        draw_go2_panel,
                        draw_ik_panel,
                        draw_sag_panel,
                    ),
                    item_width=first_item_w,
                )
            finally:
                imgui.end_child()
            imgui.same_line()
            imgui.begin_child("motion_controls_inner", 0.0, 0.0, False)
            try:
                self._draw_panel_stack(
                    (
                        draw_perception_panel,
                    ),
                    item_width=second_item_w,
                )
            finally:
                imgui.end_child()
            imgui.end_child()
            imgui.same_line()
            imgui.begin_child("status_controls", 0.0, 0.0, True)
            status_w = max(1.0, float(imgui.get_content_region_available_width()))
            self._draw_panel_stack(
                (
                    draw_sim_video_panel,
                    draw_status_panel,
                    draw_resolution_panel,
                ),
                item_width=max(scaled(self, 108.0), status_w * 0.45),
            )
            imgui.end_child()
        elif avail_w >= scaled(self, _TWO_COLUMN_MIN_W):
            col_w = max(scaled(self, 300.0), (avail_w - spacing_x) * 0.5)
            imgui.begin_child("left_controls", col_w, 0.0, True)
            left_w = max(1.0, float(imgui.get_content_region_available_width()))
            self._draw_panel_stack(
                (
                    draw_hardware_panel,
                    draw_control_4dof_panel,
                    draw_go2_panel,
                    draw_ik_panel,
                    draw_sag_panel,
                ),
                item_width=max(scaled(self, 120.0), left_w * 0.45),
            )
            imgui.end_child()
            imgui.same_line()
            imgui.begin_child("right_controls", 0.0, 0.0, True)
            right_w = max(scaled(self, 300.0), float(imgui.get_content_region_available_width()))
            self._draw_panel_stack(
                (
                    draw_sim_video_panel,
                    draw_status_panel,
                    draw_resolution_panel,
                    draw_perception_panel,
                ),
                item_width=max(scaled(self, 120.0), right_w * 0.45),
            )
            imgui.end_child()
        else:
            self._draw_panel_stack(
                (
                    draw_hardware_panel,
                    draw_control_4dof_panel,
                    draw_go2_panel,
                    draw_ik_panel,
                    draw_sag_panel,
                    draw_sim_video_panel,
                    draw_status_panel,
                    draw_resolution_panel,
                    draw_perception_panel,
                ),
                item_width=max(scaled(self, 120.0), avail_w * 0.45),
            )
        imgui.end()

    def run(self) -> None:
        if not glfw.init():
            raise SystemExit("glfw.init() failed.")

        glfw.window_hint(glfw.RESIZABLE, False)
        # pyimgui's programmable pipeline renderer is most stable with an explicit 3.3 core context.
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        if sys.platform == "darwin":
            glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, glfw.TRUE)
        win_w = _INITIAL_WINDOW_W
        win_h = _INITIAL_WINDOW_H
        monitor = glfw.get_primary_monitor()
        if monitor is not None:
            mode = glfw.get_video_mode(monitor)
            if mode is not None:
                width = int(getattr(mode.size, "width", 0) or 0)
                height = int(getattr(mode.size, "height", 0) or 0)
                if width > 0 and height > 0:
                    max_w = int(width * 0.9)
                    max_h = int(height * 0.9)
                    win_w = min(_INITIAL_WINDOW_W, max_w, int(max_h * 16.0 / 10.0))
                    win_h = int(win_w * 10.0 / 16.0)
        window = glfw.create_window(win_w, win_h, "Arm Control", None, None)
        if not window:
            glfw.terminate()
            raise SystemExit("Failed to create GLFW window.")
        self._glfw_window = window
        file_dialog.set_glfw_window(window)
        self._ui_resolution_base_w = int(win_w)
        self._ui_resolution_base_h = int(win_h)
        self._lock_os_window_size()

        glfw.make_context_current(window)
        # Let the compositor pace the primary control window.  A fixed
        # 10 ms sleep after every swap otherwise stacks on top of vsync and
        # makes pointer/slider input feel one frame behind.
        try:
            glfw.swap_interval(1)
        except Exception:
            pass

        self._main_imgui_context = imgui.create_context()
        self._install_ui_font()
        self._install_ui_style()
        main_impl = GlfwRenderer(window)
        self._main_imgui_impl = main_impl

        # Keep both RGB streams in one independently resizable native window.
        # The contexts are separate, while the OpenGL share context keeps the
        # lifecycle explicit and avoids embedding video in the control layout.
        camera_impl = None
        if self._sim_view is not None:
            glfw.window_hint(glfw.RESIZABLE, glfw.TRUE)
            camera_window = glfw.create_window(
                960,
                720,
                "Sim Camera",
                None,
                window,
            )
            glfw.window_hint(glfw.RESIZABLE, glfw.FALSE)
            if camera_window:
                self._camera_window = camera_window
                self._camera_visible = True
                glfw.make_context_current(camera_window)
                # The camera is a secondary window.  Do not let its buffer
                # swap add a second vsync wait to the control window's frame;
                # the camera draw is capped independently above.
                try:
                    glfw.swap_interval(0)
                except Exception:
                    pass
                self._camera_imgui_context = imgui.create_context()
                self._set_imgui_context(self._camera_imgui_context)
                camera_impl = GlfwRenderer(camera_window)
                self._camera_imgui_impl = camera_impl
                # Camera controls intentionally retain the independent
                # context's default style; all stream textures belong to this
                # context.  Do not call _install_ui_style here because that
                # method stores the main-window scaling baseline on ``self``.
                glfw.make_context_current(window)
                self._set_imgui_context(self._main_imgui_context)

        try:
            while not glfw.window_should_close(window) and not self._stop:
                self.service.poll()
                self._host_state = self.service.current_host_state()
                self.sync_offset_drafts()
                glfw.poll_events()
                if self._camera_window is not None and glfw.window_should_close(self._camera_window):
                    self._hide_camera_window()
                self._process_pending_file_browse()
                glfw.make_context_current(window)
                self._set_imgui_context(self._main_imgui_context)
                main_impl.process_inputs()
                self._sync_ui_resolution_scale_to_window()

                imgui.new_frame()
                self._draw_controls_window()
                imgui.render()

                main_impl.render(imgui.get_draw_data())
                glfw.swap_buffers(window)
                self._draw_camera_window()
                # Keep a tiny fallback yield for drivers that ignore the swap
                # interval; normal composited paths block in swap_buffers.
                time.sleep(0.001)
        finally:
            if self._camera_window is not None:
                try:
                    glfw.make_context_current(self._camera_window)
                    self._set_imgui_context(self._camera_imgui_context)
                    if self._sim_view is not None:
                        self._sim_view.close()
                    if camera_impl is not None:
                        camera_impl.shutdown()
                    glfw.destroy_window(self._camera_window)
                except Exception:
                    pass
                self._camera_window = None
            glfw.make_context_current(window)
            self._set_imgui_context(self._main_imgui_context)
            main_impl.shutdown()
            glfw.terminate()
