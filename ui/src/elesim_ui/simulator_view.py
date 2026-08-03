"""ImGui presentation for the UI-owned remote simulator session."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import imgui

try:
    from OpenGL import GL
except ImportError:
    GL = None  # type: ignore[assignment]


def _mouse_delta_xy(value: Any) -> tuple[float, float]:
    if hasattr(value, "x") and hasattr(value, "y"):
        return float(value.x), float(value.y)
    if isinstance(value, (tuple, list)) and len(value) >= 2:
        return float(value[0]), float(value[1])
    return 0.0, 0.0


@dataclass
class SimulatorViewState:
    main_stream: str = "observer"
    preview_stream: str = "hand_eye_preview"

    def swap_streams(self) -> None:
        self.main_stream, self.preview_stream = self.preview_stream, self.main_stream


class SimulatorView:
    """Render two named streams and issue simulation-session commands."""

    SPEEDS = (0.25, 0.5, 1.0, 2.0, 4.0)

    def __init__(self, session: Any) -> None:
        self.session = session
        self.state = SimulatorViewState()
        self._textures: dict[str, int] = {}

    def draw(self) -> None:
        snapshot = self.session.snapshot
        self._draw_toolbar(snapshot)
        if snapshot.last_error:
            imgui.text_colored(snapshot.last_error, 0.78, 0.18, 0.18)

        available = max(180.0, float(imgui.get_content_region_available_width()))
        main_clicked = self._draw_stream(
            self.state.main_stream,
            width=available,
            max_height=300.0,
            connected=self.state.main_stream in snapshot.connected_streams,
        )
        if main_clicked and self.state.main_stream != "observer":
            self.state.swap_streams()

        imgui.separator()
        preview_width = min(240.0, max(150.0, available * 0.42))
        preview_clicked = self._draw_stream(
            self.state.preview_stream,
            width=preview_width,
            max_height=150.0,
            connected=self.state.preview_stream in snapshot.connected_streams,
        )
        if preview_clicked:
            self.state.swap_streams()

    def _draw_toolbar(self, snapshot: Any) -> None:
        status = snapshot.status
        active = snapshot.active_sim_id or snapshot.requested_sim_id or "none"
        connected = len(snapshot.connected_streams)
        imgui.text(f"SIM {active} | video {connected}/2")

        paused = bool(status.paused) if status is not None else False
        if imgui.button(">##sim-run" if paused else "||##sim-pause"):
            self.session.send_command("resume" if paused else "pause")
        self._tooltip("Resume simulation" if paused else "Pause simulation")
        imgui.same_line()
        if imgui.button(">|##sim-step") and paused:
            self.session.send_command("step", {"count": 1})
        self._tooltip("Advance one physics step while paused")
        imgui.same_line()
        if imgui.button("R##sim-reset"):
            self.session.send_command("reset")
        self._tooltip("Reset simulation state")
        imgui.same_line()
        if imgui.button("V##sim-reset-view"):
            self.session.send_command("reset_view")
        self._tooltip("Reset observer camera")

        speed = float(status.speed) if status is not None else 1.0
        speed_index = min(
            range(len(self.SPEEDS)),
            key=lambda index: abs(self.SPEEDS[index] - speed),
        )
        changed, selected = imgui.combo(
            "Speed##sim-speed",
            speed_index,
            [f"{value:g}x" for value in self.SPEEDS],
        )
        if changed:
            self.session.send_command("set_speed", {"scale": self.SPEEDS[int(selected)]})

        debug_visible = bool(status.debug_visible) if status is not None else True
        changed, visible = imgui.checkbox("Debug markers##sim-debug", debug_visible)
        if changed:
            self.session.send_command("set_debug_visible", {"visible": bool(visible)})

        if status is not None:
            run_state = "paused" if status.paused else "running"
            imgui.text_disabled(
                f"epoch {status.epoch} | {run_state} | t={status.sim_time_s:.2f}s"
            )

    def _draw_stream(
        self,
        stream: str,
        *,
        width: float,
        max_height: float,
        connected: bool,
    ) -> bool:
        label = "OBSERVER" if stream == "observer" else "HAND-EYE"
        imgui.text(f"{label} {'LIVE' if connected else 'WAIT'}")
        frame = self.session.frame(stream)
        if (
            GL is None
            or frame is None
            or not hasattr(frame, "shape")
            or len(frame.shape) != 3
        ):
            imgui.text_disabled(f"{label} video waiting...")
            return False
        height, source_width = int(frame.shape[0]), int(frame.shape[1])
        if source_width <= 0 or height <= 0:
            return False
        texture = self._upload(stream, frame, source_width, height)
        draw_height = min(float(max_height), float(width) * height / source_width)
        imgui.image(
            texture,
            float(width),
            draw_height,
        )
        hovered = bool(imgui.is_item_hovered())
        if hovered and stream == "observer":
            self._handle_observer_input(width=float(width), height=draw_height)
        return hovered and bool(getattr(imgui, "is_item_clicked", lambda *_: False)(0))

    def _upload(self, stream: str, frame: Any, width: int, height: int) -> int:
        texture = self._textures.get(stream, 0)
        if not texture:
            texture = int(GL.glGenTextures(1))
            self._textures[stream] = texture
            GL.glBindTexture(GL.GL_TEXTURE_2D, texture)
            GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR)
            GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
        GL.glBindTexture(GL.GL_TEXTURE_2D, texture)
        GL.glPixelStorei(GL.GL_UNPACK_ALIGNMENT, 1)
        GL.glTexImage2D(
            GL.GL_TEXTURE_2D,
            0,
            GL.GL_RGB,
            width,
            height,
            0,
            GL.GL_BGR,
            GL.GL_UNSIGNED_BYTE,
            frame,
        )
        return texture

    def _handle_observer_input(self, *, width: float, height: float) -> None:
        io = imgui.get_io()
        raw_dx, raw_dy = _mouse_delta_xy(io.mouse_delta)
        dx = raw_dx / max(width, 1.0)
        dy = raw_dy / max(height, 1.0)
        if imgui.is_mouse_dragging(0) and abs(dx) + abs(dy) > 0.0:
            self.session.send_command("orbit", {"dx": dx, "dy": dy})
        elif imgui.is_mouse_dragging(1) and abs(dx) + abs(dy) > 0.0:
            self.session.send_command("pan", {"dx": dx, "dy": dy})
        wheel = float(getattr(io, "mouse_wheel", 0.0))
        if abs(wheel) > 1e-6:
            self.session.send_command("zoom", {"delta": -wheel * 0.08})

    @staticmethod
    def _tooltip(text: str) -> None:
        if bool(getattr(imgui, "is_item_hovered", lambda: False)()):
            setter = getattr(imgui, "set_tooltip", None)
            if setter is not None:
                setter(str(text))

    def close(self) -> None:
        if GL is None:
            self._textures.clear()
            return
        textures = tuple(self._textures.values())
        self._textures.clear()
        if textures:
            try:
                GL.glDeleteTextures(list(textures))
            except Exception:
                pass


__all__ = ["SimulatorView", "SimulatorViewState"]
