from __future__ import annotations

import time
from typing import Optional

import glfw
import imgui
from imgui.integrations.glfw import GlfwRenderer

from engine.controller import ControlService, HostState, PanelState
from engine.config_loader import PerceptionConfig, PickConfig

from .panels import (
    draw_control_4dof_panel,
    draw_hardware_panel,
    draw_ik_panel,
    draw_perception_panel,
    draw_sag_panel,
)


class ControlPanel:
    """External ImGui window that draws and edits PanelState."""

    def __init__(
        self,
        state: PanelState,
        service: ControlService,
        *,
        use_hardware: bool = False,
        perception_cfg: PerceptionConfig | None = None,
        pick_cfg: PickConfig | None = None,
    ):
        self.state = state
        self.service = service
        self._use_hardware = bool(use_hardware)
        pc = perception_cfg or PerceptionConfig()
        pk = pick_cfg or PickConfig()
        self._stop = False
        self._hw_header_init_open = False
        self._ctrl_header_init_open = False
        self._ik_header_init_open = False
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
        self._repeat_button_deadlines: dict[str, float] = {}
        self._visual_pan_deg_draft = 10.0
        self._visual_tilt_deg_draft = 10.0

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

    def run_repeat_button(
        self,
        key: str,
        *,
        clicked: bool,
        active: bool,
        action,
        initial_delay_s: float = 0.35,
        repeat_period_s: float = 0.08,
    ) -> None:
        now = float(time.time())
        name = str(key)
        if clicked:
            action()
            self._repeat_button_deadlines[name] = now + float(max(initial_delay_s, 0.01))
            return
        if not active:
            self._repeat_button_deadlines.pop(name, None)
            return
        deadline = float(self._repeat_button_deadlines.get(name, now + float(max(initial_delay_s, 0.01))))
        if now >= deadline:
            action()
            self._repeat_button_deadlines[name] = now + float(max(repeat_period_s, 0.01))

    def _draw_controls_window(self) -> None:
        if not self._ctrl_window_init:
            cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
            io = imgui.get_io()
            imgui.set_next_window_position(0.0, 0.0, cond)
            imgui.set_next_window_size(float(io.display_size.x), float(io.display_size.y), cond)
            self._ctrl_window_init = True
        imgui.begin("###arm_control_window", True)
        draw_hardware_panel(self)
        if self._use_hardware and self.service.has_client():
            imgui.separator()
        draw_perception_panel(self)
        imgui.separator()
        draw_control_4dof_panel(self)
        imgui.separator()
        draw_ik_panel(self)
        imgui.separator()
        draw_sag_panel(self)
        imgui.end()

    def run(self) -> None:
        if not glfw.init():
            raise SystemExit("glfw.init() failed.")

        glfw.window_hint(glfw.RESIZABLE, True)
        win_w = 800
        win_h = 600
        monitor = glfw.get_primary_monitor()
        if monitor is not None:
            mode = glfw.get_video_mode(monitor)
            if mode is not None:
                width = int(getattr(mode.size, "width", 0) or 0)
                height = int(getattr(mode.size, "height", 0) or 0)
                if width > 0 and height > 0:
                    win_w = max(640, int(width * 0.4))
                    win_h = max(540, int(height * 0.5))
        window = glfw.create_window(win_w, win_h, "Arm Control", None, None)
        if not window:
            glfw.terminate()
            raise SystemExit("Failed to create GLFW window.")

        glfw.make_context_current(window)

        imgui.create_context()
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
