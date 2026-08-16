"""ImGui presentation for the UI-owned remote sim session."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import imgui

from .helpers import _color_u32, _draw_rect_filled, _draw_text, _xy

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


def _genesis_scroll_zoom_delta(clicks: float) -> float:
    """Protocol zoom unit matching Genesis 1.2.0's 0.90 wheel ratio."""

    value = float(clicks)
    if not math.isfinite(value):
        return 0.0
    return math.log(0.90) * value / 1.5


def _genesis_drag_zoom_delta(normalized_dy: float, *, height: float) -> float:
    """Protocol zoom unit for Genesis' right-button vertical drag."""

    value = float(normalized_dy)
    h = max(float(height), 1.0)
    multiplier = 2.0 - math.exp((value * h) / (0.5 * h))
    multiplier = min(20.0, max(0.08, multiplier))
    return math.log(multiplier) / 1.5


OBSERVER_ASPECT = 4.0 / 3.0
_PIP_MARGIN = 12.0
_PIP_WIDTH_FRACTION = 0.34
_PIP_MIN_WIDTH = 120.0
_PIP_MAX_WIDTH = 300.0


def _fit_aspect_size(
    width: float,
    max_height: float,
    aspect: float,
) -> tuple[float, float]:
    """Fit a display rectangle without ever stretching its contents."""

    available_width = max(float(width), 1.0)
    height_limit = max(float(max_height), 1.0)
    ratio = max(float(aspect), 1e-6)
    display_width = min(available_width, height_limit * ratio)
    return display_width, display_width / ratio


def _center_crop_uv(
    source_width: int,
    source_height: int,
    display_aspect: float,
) -> tuple[float, float, float, float]:
    """Return centered UVs that preserve source pixels in a fixed display ratio."""

    source_w = max(int(source_width), 1)
    source_h = max(int(source_height), 1)
    source_aspect = source_w / source_h
    target_aspect = max(float(display_aspect), 1e-6)
    if math.isclose(source_aspect, target_aspect, rel_tol=1e-6, abs_tol=1e-6):
        return 0.0, 0.0, 1.0, 1.0
    if source_aspect > target_aspect:
        visible_width = target_aspect / source_aspect
        left = (1.0 - visible_width) * 0.5
        return left, 0.0, 1.0 - left, 1.0
    visible_height = source_aspect / target_aspect
    top = (1.0 - visible_height) * 0.5
    return 0.0, top, 1.0, 1.0 - top


def _pip_rect(
    container: tuple[float, float, float, float],
    *,
    source_aspect: float,
    margin: float = _PIP_MARGIN,
) -> tuple[float, float, float, float]:
    """Place a bounded PIP rectangle at the container's upper-right corner."""

    x0, y0, x1, y1 = (float(value) for value in container)
    container_width = max(1.0, x1 - x0)
    container_height = max(1.0, y1 - y0)
    gap = max(
        0.0,
        min(float(margin), container_width * 0.05, container_height * 0.05),
    )
    pip_width = min(_PIP_MAX_WIDTH, container_width * _PIP_WIDTH_FRACTION)
    pip_width = max(_PIP_MIN_WIDTH, pip_width)
    pip_width = min(pip_width, max(1.0, container_width - gap * 2.0))
    ratio = max(float(source_aspect), 1e-6)
    pip_height = pip_width / ratio
    if pip_height > container_height - gap * 2.0:
        pip_height = max(1.0, container_height - gap * 2.0)
        pip_width = pip_height * ratio
    return x1 - gap - pip_width, y0 + gap, x1 - gap, y0 + gap + pip_height


@dataclass
class SimViewState:
    main_stream: str = "observer"
    preview_stream: str = "hand_eye_preview"

    def swap_streams(self) -> None:
        self.main_stream, self.preview_stream = self.preview_stream, self.main_stream


class SimView:
    """Render two named streams and issue simulation-session commands."""

    SPEEDS = (0.25, 0.5, 1.0, 2.0, 4.0)

    def __init__(self, session: Any) -> None:
        self.session = session
        self.state = SimViewState()
        self._textures: dict[str, int] = {}
        self._texture_versions: dict[str, int] = {}

    def draw(self) -> None:
        snapshot = self.session.snapshot
        self._draw_toolbar(snapshot)
        if snapshot.last_error:
            imgui.text_colored(snapshot.last_error, 0.78, 0.18, 0.18)

        available = max(180.0, float(imgui.get_content_region_available_width()))
        available_height = self._available_content_height(fallback=available * 0.75)
        observer_max_height = max(240.0, min(720.0, available_height))
        _, observer_rect = self._draw_stream(
            "observer",
            width=available,
            max_height=observer_max_height,
            connected="observer" in snapshot.connected_streams,
            display_aspect=OBSERVER_ASPECT,
            center=True,
        )
        if observer_rect is not None:
            self._draw_pip(
                observer_rect,
                connected="hand_eye_preview" in snapshot.connected_streams,
            )

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
        display_aspect: float | None = None,
        center: bool = False,
    ) -> tuple[bool, tuple[float, float, float, float] | None]:
        label = "OBSERVER" if stream == "observer" else "HAND-EYE"
        imgui.text(f"{label} {'LIVE' if connected else 'WAIT'}")
        version_getter = getattr(self.session, "frame_version", None)
        # Read the version before the frame pointer.  If a decode completes
        # between the two reads, the next draw will see the newer version and
        # upload it instead of accidentally marking the older pointer fresh.
        version = version_getter(stream) if callable(version_getter) else None
        frame = self.session.frame(stream)
        if (
            GL is None
            or frame is None
            or not hasattr(frame, "shape")
            or len(frame.shape) != 3
        ):
            imgui.text_disabled(f"{label} video waiting...")
            return False, None
        height, source_width = int(frame.shape[0]), int(frame.shape[1])
        if source_width <= 0 or height <= 0:
            return False, None
        target_aspect = (
            max(float(display_aspect), 1e-6)
            if display_aspect is not None
            else source_width / height
        )
        texture = self._upload(stream, frame, source_width, height, version=version)
        draw_width, draw_height = _fit_aspect_size(width, max_height, target_aspect)
        cursor_x, cursor_y = _xy(imgui.get_cursor_screen_pos())
        if center and draw_width < float(width):
            setter = getattr(imgui, "set_cursor_screen_pos", None)
            if callable(setter):
                setter((cursor_x + (float(width) - draw_width) * 0.5, cursor_y))
        uv0_x, uv0_y, uv1_x, uv1_y = _center_crop_uv(
            source_width,
            height,
            target_aspect,
        )
        imgui.image(
            texture,
            draw_width,
            draw_height,
            uv0=(uv0_x, uv0_y),
            uv1=(uv1_x, uv1_y),
        )
        rect_min = _xy(imgui.get_item_rect_min())
        rect_max = _xy(imgui.get_item_rect_max())
        rect = (rect_min[0], rect_min[1], rect_max[0], rect_max[1])
        hovered = bool(imgui.is_item_hovered())
        if hovered and stream == "observer":
            self._handle_observer_input(width=draw_width, height=draw_height)
        clicked = hovered and bool(getattr(imgui, "is_item_clicked", lambda *_: False)(0))
        return clicked, rect

    def _draw_pip(
        self,
        container: tuple[float, float, float, float],
        *,
        connected: bool,
    ) -> None:
        stream = "hand_eye_preview"
        version_getter = getattr(self.session, "frame_version", None)
        version = version_getter(stream) if callable(version_getter) else None
        frame = self.session.frame(stream)
        valid_frame = (
            GL is not None
            and frame is not None
            and hasattr(frame, "shape")
            and len(frame.shape) == 3
            and int(frame.shape[0]) > 0
            and int(frame.shape[1]) > 0
        )
        if valid_frame:
            source_height, source_width = int(frame.shape[0]), int(frame.shape[1])
            source_aspect = source_width / source_height
        else:
            source_width = source_height = 0
            source_aspect = OBSERVER_ASPECT
        rect = _pip_rect(container, source_aspect=source_aspect)
        x0, y0, x1, y1 = rect
        draw_list_getter = getattr(imgui, "get_window_draw_list", None)
        if not callable(draw_list_getter):
            return
        draw_list = draw_list_getter()
        shadow = _color_u32(0.0, 0.0, 0.0, 0.60)
        panel = _color_u32(0.03, 0.04, 0.06, 0.92)
        border = _color_u32(0.92, 0.94, 0.98, 0.90)
        _draw_rect_filled(
            draw_list,
            x0 - 4.0,
            y0 - 4.0,
            x1 + 4.0,
            y1 + 4.0,
            shadow,
            5.0,
        )
        _draw_rect_filled(draw_list, x0, y0, x1, y1, panel, 3.0)
        if valid_frame:
            texture = self._upload(
                stream,
                frame,
                source_width,
                source_height,
                version=version,
            )
            self._draw_list_image(draw_list, texture, rect)
        else:
            _draw_text(
                draw_list,
                x0 + 10.0,
                y0 + (y1 - y0) * 0.5 - 7.0,
                _color_u32(0.85, 0.87, 0.91, 1.0),
                "HAND-EYE WAIT",
            )
        label_height = min(24.0, max(18.0, (y1 - y0) * 0.16))
        _draw_rect_filled(
            draw_list,
            x0,
            y0,
            x1,
            y0 + label_height,
            _color_u32(0.02, 0.03, 0.05, 0.78),
            3.0,
        )
        _draw_text(
            draw_list,
            x0 + 8.0,
            y0 + max(2.0, (label_height - 13.0) * 0.5),
            _color_u32(0.96, 0.97, 1.0, 1.0),
            f"HAND-EYE {'LIVE' if connected else 'WAIT'}",
        )
        try:
            draw_list.add_rect(x0, y0, x1, y1, border, 3.0, 0)
        except TypeError:
            try:
                draw_list.add_rect((x0, y0), (x1, y1), border, 3.0)
            except TypeError:
                pass

    @staticmethod
    def _draw_list_image(
        draw_list: Any,
        texture: int,
        rect: tuple[float, float, float, float],
    ) -> None:
        x0, y0, x1, y1 = rect
        for args in (
            (texture, (x0, y0), (x1, y1), (0.0, 0.0), (1.0, 1.0)),
            (texture, x0, y0, x1, y1, (0.0, 0.0), (1.0, 1.0)),
        ):
            try:
                draw_list.add_image(*args)
                return
            except TypeError:
                continue

    @staticmethod
    def _available_content_height(*, fallback: float) -> float:
        getter = getattr(imgui, "get_content_region_available", None)
        if callable(getter):
            try:
                _, height = _xy(getter())
                if height > 0.0:
                    return float(height)
            except Exception:
                pass
        return max(1.0, float(fallback))

    def _upload(
        self,
        stream: str,
        frame: Any,
        width: int,
        height: int,
        *,
        version: int | None = None,
    ) -> int:
        texture = self._textures.get(stream, 0)
        if not texture:
            texture = int(GL.glGenTextures(1))
            self._textures[stream] = texture
            GL.glBindTexture(GL.GL_TEXTURE_2D, texture)
            GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR)
            GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
        elif version is not None and self._texture_versions.get(stream) == version:
            return texture
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
        if version is not None:
            self._texture_versions[stream] = version
        return texture

    def _handle_observer_input(self, *, width: float, height: float) -> None:
        io = imgui.get_io()
        raw_dx, raw_dy = _mouse_delta_xy(io.mouse_delta)
        dx = raw_dx / max(width, 1.0)
        dy = raw_dy / max(height, 1.0)
        if imgui.is_mouse_dragging(0) and abs(dx) + abs(dy) > 0.0:
            self.session.send_command("orbit", {"dx": dx, "dy": dy})
        # Genesis: middle-button drag pans; right-button drag zooms.  The
        # previous UI used right-button pan, which made the remote camera feel
        # unlike the native viewer.
        elif imgui.is_mouse_dragging(2) and abs(dx) + abs(dy) > 0.0:
            self.session.send_command("pan", {"dx": dx, "dy": dy})
        elif imgui.is_mouse_dragging(1) and abs(dy) > 0.0:
            self.session.send_command(
                "zoom",
                {"delta": _genesis_drag_zoom_delta(dy, height=height)},
            )
        wheel = float(getattr(io, "mouse_wheel", 0.0))
        if abs(wheel) > 1e-6:
            self.session.send_command("zoom", {"delta": _genesis_scroll_zoom_delta(wheel)})

    @staticmethod
    def _tooltip(text: str) -> None:
        if bool(getattr(imgui, "is_item_hovered", lambda: False)()):
            setter = getattr(imgui, "set_tooltip", None)
            if setter is not None:
                setter(str(text))

    def close(self) -> None:
        if GL is None:
            self._textures.clear()
            self._texture_versions.clear()
            return
        textures = tuple(self._textures.values())
        self._textures.clear()
        self._texture_versions.clear()
        if textures:
            try:
                GL.glDeleteTextures(list(textures))
            except Exception:
                pass


__all__ = [
    "SimView",
    "SimViewState",
    "_genesis_drag_zoom_delta",
    "_genesis_scroll_zoom_delta",
]
