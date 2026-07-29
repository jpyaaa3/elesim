#!/usr/bin/env python3
"""Pseudo-realtime ImGui viewer for Elesim structured trace logs."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable, Optional

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    import glfw
    import imgui
    from imgui.integrations.glfw import GlfwRenderer
except ModuleNotFoundError:
    glfw = None  # type: ignore[assignment]
    imgui = None  # type: ignore[assignment]
    GlfwRenderer = None  # type: ignore[assignment]

from elesim_ui.theme import CONTENT_FONT_CANDIDATES, FONT_SPEC, TITLE_FONT, add_font_with_korean_ranges


WINDOW_W = 1380
WINDOW_H = 820
LEFT_W = 330.0
MAX_RECORDS = 6000
INITIAL_READ_BYTES = 1_000_000
SERVICE_ORDER = ("all", "elesim-control", "elesim-host", "elesim-sim", "elesim-perception")
SERVICE_LABELS = {
    "all": "전체",
    "elesim-control": "CTRL",
    "elesim-host": "HOST",
    "elesim-sim": "SIM",
    "elesim-perception": "VISION",
}
SERVICE_SHORT = {
    "elesim-control": "CTRL",
    "elesim-host": "HOST",
    "elesim-sim": "SIM",
    "elesim-perception": "VISION",
}


@dataclass
class _FileCursor:
    offset: int = 0
    inode: int = 0


class TraceTailer:
    def __init__(self, log_dir: Path, *, initial_read_bytes: int = INITIAL_READ_BYTES) -> None:
        self.log_dir = Path(log_dir)
        self.initial_read_bytes = max(0, int(initial_read_bytes))
        self._cursors: dict[Path, _FileCursor] = {}

    @property
    def file_count(self) -> int:
        return len(self._cursors)

    def reset(self) -> None:
        self._cursors.clear()

    def _discover(self) -> list[Path]:
        if not self.log_dir.exists():
            return []
        return sorted(path for path in self.log_dir.glob("*.jsonl") if path.is_file())

    def poll(self) -> list[dict[str, Any]]:
        records: list[dict[str, Any]] = []
        for path in self._discover():
            try:
                stat = path.stat()
            except OSError:
                continue
            cursor = self._cursors.get(path)
            skip_partial = False
            if cursor is None or cursor.inode != int(stat.st_ino) or stat.st_size < cursor.offset:
                start = max(0, int(stat.st_size) - self.initial_read_bytes)
                cursor = _FileCursor(offset=start, inode=int(stat.st_ino))
                self._cursors[path] = cursor
                skip_partial = start > 0
            try:
                with path.open("r", encoding="utf-8", errors="replace") as handle:
                    handle.seek(cursor.offset)
                    if skip_partial:
                        handle.readline()
                    for line in handle:
                        text = line.strip()
                        if not text:
                            continue
                        try:
                            payload = json.loads(text)
                        except json.JSONDecodeError:
                            payload = {
                                "ts_unix_ns": time.time_ns(),
                                "service": path.stem,
                                "event": "logger.decode_error",
                                "error": text[:500],
                            }
                        if isinstance(payload, dict):
                            payload["_file"] = path.name
                            records.append(payload)
                    cursor.offset = handle.tell()
            except OSError:
                continue
        records.sort(key=lambda item: int(item.get("ts_unix_ns", 0) or 0))
        return records


def _clock_text(timestamp_ns: Any) -> str:
    try:
        timestamp = float(timestamp_ns) / 1_000_000_000.0
        return datetime.fromtimestamp(timestamp).strftime("%H:%M:%S.%f")[:-3]
    except Exception:
        return "--:--:--.---"


def _duration_text(value: Any) -> str:
    try:
        duration_ms = float(value)
    except (TypeError, ValueError):
        return ""
    if duration_ms >= 1000.0:
        return f"{duration_ms / 1000.0:.2f}s"
    return f"{duration_ms:.1f}ms"


def _compact_attributes(raw: Any) -> str:
    if not isinstance(raw, dict):
        return ""
    preferred = (
        "messaging.message.type",
        "elesim.message.seq",
        "elesim.message.source",
        "elesim.message.ok",
        "server.address",
        "port",
        "mode",
        "gaze_mode",
    )
    parts: list[str] = []
    for key in preferred:
        if key in raw:
            label = key.rsplit(".", 1)[-1]
            parts.append(f"{label}={raw[key]}")
    return " ".join(parts)


def record_is_error(record: dict[str, Any]) -> bool:
    event = str(record.get("event", "")).lower()
    error = str(record.get("error", "")).strip()
    return bool(error) or "error" in event or "failed" in event or "exception" in event


def format_record(record: dict[str, Any]) -> str:
    clock = _clock_text(record.get("ts_unix_ns"))
    service_raw = str(record.get("service", "unknown"))
    service = SERVICE_SHORT.get(service_raw, service_raw.replace("elesim-", "").upper()[:10])
    event = str(record.get("event", "event"))
    marker = "!" if record_is_error(record) else " "
    if event == "span.end":
        name = str(record.get("span", "span"))
        duration = _duration_text(record.get("duration_ms"))
        attrs = _compact_attributes(record.get("attributes"))
        error = str(record.get("error", "")).strip()
        suffix = " ".join(part for part in (duration, attrs, error) if part)
        return f"{clock} {service:<10} {marker} {name:<42} {suffix}".rstrip()
    if event == "span.event":
        parent = str(record.get("span", "span"))
        name = str(record.get("name", "event"))
        attrs = _compact_attributes(record.get("attributes"))
        return f"{clock} {service:<10} {marker} {parent} > {name} {attrs}".rstrip()
    details = _compact_attributes(record.get("attributes"))
    error = str(record.get("error", "")).strip()
    suffix = " ".join(part for part in (details, error) if part)
    return f"{clock} {service:<10} {marker} {event:<42} {suffix}".rstrip()


def filter_records(
    records: Iterable[dict[str, Any]],
    *,
    service: str,
    query: str,
    errors_only: bool,
) -> list[dict[str, Any]]:
    needle = str(query).strip().lower()
    filtered: list[dict[str, Any]] = []
    for record in records:
        if service != "all" and str(record.get("service", "")) != service:
            continue
        if errors_only and not record_is_error(record):
            continue
        if needle and needle not in json.dumps(record, ensure_ascii=False).lower():
            continue
        filtered.append(record)
    return filtered


class TraceViewer:
    def __init__(self, log_dir: Path) -> None:
        self.log_dir = Path(log_dir)
        self.tailer = TraceTailer(self.log_dir)
        self.records: list[dict[str, Any]] = []
        self.frozen_records: Optional[list[dict[str, Any]]] = None
        self.selected_service = "all"
        self.query = ""
        self.errors_only = False
        self.paused = False
        self.newest_first = True
        self.status = "로그 디렉터리 대기 중"
        self._last_poll = 0.0
        self._last_text = ""

    def _install_font(self) -> None:
        assert imgui is not None
        io = imgui.get_io()
        fonts = getattr(io, "fonts", None)
        if fonts is None:
            return
        font_path = next((path for path in CONTENT_FONT_CANDIDATES if path.exists()), None)
        if font_path is None:
            return
        font = add_font_with_korean_ranges(fonts, font_path, float(FONT_SPEC.content_px))
        if font is not None and hasattr(io, "font_default"):
            io.font_default = font
        if TITLE_FONT.exists():
            add_font_with_korean_ranges(fonts, TITLE_FONT, float(FONT_SPEC.title_px))

    @staticmethod
    def _install_style() -> None:
        assert imgui is not None
        style = imgui.get_style()
        style.window_rounding = 5.0
        style.child_rounding = 5.0
        style.frame_rounding = 4.0
        style.window_border_size = 1.0
        style.child_border_size = 1.0
        style.frame_padding = (8.0, 6.0)
        style.item_spacing = (8.0, 8.0)

    def _poll(self) -> None:
        now = time.monotonic()
        if now - self._last_poll < 0.2:
            return
        self._last_poll = now
        added = self.tailer.poll()
        if added:
            self.records.extend(added)
            if len(self.records) > MAX_RECORDS:
                del self.records[: len(self.records) - MAX_RECORDS]
        self.status = f"파일 {self.tailer.file_count}개 | 이벤트 {len(self.records)}개"

    def _visible_records(self) -> list[dict[str, Any]]:
        source = self.frozen_records if self.frozen_records is not None else self.records
        visible = filter_records(
            source,
            service=self.selected_service,
            query=self.query,
            errors_only=self.errors_only,
        )
        if self.newest_first:
            visible.reverse()
        return visible

    def _visible_text(self) -> str:
        visible = self._visible_records()
        self._last_text = "\n".join(format_record(record) for record in visible)
        return self._last_text or "표시할 로그가 없습니다. Elesim 프로세스를 실행하거나 필터를 확인하세요."

    @staticmethod
    def _readonly_multiline(identifier: str, text: str, height: float) -> None:
        assert imgui is not None
        flags = getattr(imgui, "INPUT_TEXT_READ_ONLY", getattr(imgui, "INPUT_TEXT_FLAGS_READ_ONLY", 1 << 14))
        width = max(100.0, float(imgui.get_content_region_available_width()))
        buf_size = max(4096, len(text) + 1024)
        imgui.push_item_width(width)
        try:
            try:
                imgui.input_text_multiline(identifier, text, buf_size, width, height, flags=flags)
            except TypeError:
                imgui.input_text_multiline(identifier, text, buf_size, width, height, flags)
        finally:
            imgui.pop_item_width()

    def _copy(self, text: str) -> None:
        assert imgui is not None
        setter = getattr(imgui, "set_clipboard_text", None)
        if callable(setter):
            setter(str(text))
            self.status = f"클립보드에 {len(text)}자 복사"

    def _draw_service_buttons(self) -> None:
        assert imgui is not None
        button_w = 142.0
        services = list(SERVICE_ORDER)
        extras = sorted(
            {str(record.get("service", "")) for record in self.records}
            - set(SERVICE_ORDER)
            - {""}
        )
        services.extend(extras)
        for index, service in enumerate(services):
            if index % 2:
                imgui.same_line()
            active = service == self.selected_service
            if active:
                imgui.push_style_color(imgui.COLOR_BUTTON, 0.13, 0.48, 0.82, 1.0)
                imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, 0.16, 0.55, 0.90, 1.0)
                imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, 0.10, 0.38, 0.68, 1.0)
            try:
                label = SERVICE_LABELS.get(service, service.replace("elesim-", "").upper())
                if imgui.button(f"{label}##service_{index}", button_w, 36.0):
                    self.selected_service = service
            finally:
                if active:
                    imgui.pop_style_color(3)

    def _draw_left(self) -> None:
        assert imgui is not None
        imgui.begin_child("trace_controls", LEFT_W, 0.0, True)
        try:
            imgui.text("TRACE VIEWER")
            imgui.separator()
            self._draw_service_buttons()
            imgui.spacing()
            imgui.separator()
            imgui.text("필터")
            imgui.push_item_width(-1.0)
            try:
                changed, query = imgui.input_text("##trace_query", self.query, 256)
                if changed:
                    self.query = query
            finally:
                imgui.pop_item_width()
            _, self.errors_only = imgui.checkbox("오류만", self.errors_only)
            imgui.same_line()
            _, self.newest_first = imgui.checkbox("최신순", self.newest_first)
            imgui.spacing()
            pause_label = "재개" if self.paused else "일시정지"
            if imgui.button(pause_label, 94.0, 34.0):
                self.paused = not self.paused
                self.frozen_records = list(self.records) if self.paused else None
            imgui.same_line()
            if imgui.button("새로 읽기", 94.0, 34.0):
                self.records.clear()
                self.frozen_records = None
                self.tailer.reset()
                self._poll()
            imgui.same_line()
            if imgui.button("비우기", 94.0, 34.0):
                self.records.clear()
                self.frozen_records = [] if self.paused else None
            imgui.spacing()
            if imgui.button("표시 로그 복사", 142.0, 36.0):
                self._copy(self._visible_text())
            imgui.same_line()
            if imgui.button("오류 복사", 142.0, 36.0):
                errors = filter_records(self.records, service=self.selected_service, query=self.query, errors_only=True)
                self._copy("\n".join(format_record(record) for record in errors))
            imgui.separator()
            imgui.text("상태")
            imgui.text_wrapped(self.status)
            imgui.text_wrapped(f"경로: {self.log_dir}")
            imgui.text_wrapped(f"표시: {len(self._visible_records())}개")
            imgui.spacing()
            imgui.separator()
            imgui.text("읽는 법")
            imgui.text_wrapped("! 표시는 오류입니다. span 이름 오른쪽에는 실행시간과 통신 타입, seq, source가 표시됩니다.")
            imgui.spacing()
            imgui.text_wrapped("오른쪽 로그는 드래그 후 Ctrl+C로 복사할 수 있습니다. 최신순에서는 새 이벤트가 맨 위에 추가됩니다.")
        finally:
            imgui.end_child()

    def _draw_log(self) -> None:
        assert imgui is not None
        imgui.begin_child("trace_log_panel", 0.0, 0.0, True)
        try:
            imgui.text("실시간 호출 로그")
            imgui.same_line()
            imgui.text_disabled("드래그 후 Ctrl+C")
            imgui.separator()
            available = imgui.get_content_region_available()
            height = float(available.y if hasattr(available, "y") else available[1])
            self._readonly_multiline("##trace_log_copyable", self._visible_text(), max(120.0, height))
        finally:
            imgui.end_child()

    def draw(self) -> None:
        assert imgui is not None
        self._poll()
        io = imgui.get_io()
        cond = getattr(imgui, "ALWAYS", 0)
        imgui.set_next_window_position(0.0, 0.0, cond)
        imgui.set_next_window_size(float(io.display_size.x), float(io.display_size.y), cond)
        imgui.begin("Elesim Trace Viewer###trace_viewer_root", True, flags=getattr(imgui, "WINDOW_NO_TITLE_BAR", 0))
        try:
            self._draw_left()
            imgui.same_line()
            self._draw_log()
        finally:
            imgui.end()

    def run(self) -> None:
        if glfw is None or imgui is None or GlfwRenderer is None:
            raise SystemExit("trace viewer에는 elesim-ui와 같은 glfw/imgui 의존성이 필요합니다.")
        if not glfw.init():
            raise SystemExit("glfw.init() failed.")
        glfw.window_hint(glfw.RESIZABLE, False)
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        window = glfw.create_window(WINDOW_W, WINDOW_H, "Elesim Trace Viewer", None, None)
        if not window:
            glfw.terminate()
            raise SystemExit("GLFW 창 생성에 실패했습니다.")
        glfw.make_context_current(window)
        imgui.create_context()
        self._install_font()
        self._install_style()
        renderer = GlfwRenderer(window)
        try:
            while not glfw.window_should_close(window):
                glfw.poll_events()
                renderer.process_inputs()
                imgui.new_frame()
                self.draw()
                imgui.render()
                renderer.render(imgui.get_draw_data())
                glfw.swap_buffers(window)
                time.sleep(0.02)
        finally:
            renderer.shutdown()
            glfw.terminate()


def main() -> int:
    parser = argparse.ArgumentParser(description="Elesim JSONL trace 로그 GUI viewer")
    parser.add_argument("--log-dir", default=str(ROOT / "logs/tracing"), help="trace JSONL 디렉터리")
    args = parser.parse_args()
    TraceViewer(Path(args.log_dir)).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
