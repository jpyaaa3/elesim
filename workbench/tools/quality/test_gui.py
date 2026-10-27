#!/usr/bin/env python3
"""Small ImGui runner for the repo's pytest suites."""

from __future__ import annotations

import argparse
import os
import queue
import re
import shlex
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Optional

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

from workbench.tools.quality.selections import SELECTIONS as TEST_GROUPS, Selection as TestCaseGroup


WINDOW_W = 1280
WINDOW_H = 760
LEFT_W = 350.0
DESCRIPTION_H = 320.0
ERROR_SUMMARY_H = 220.0
MAX_LOG_LINES = 4000
PYTEST_LOCATION_RE = re.compile(
    r"^(?:payload|model|workbench|installer)/.+\.py:\d+(?::|$)"
)


class TestGui:
    def __init__(self, *, runner: str = "local", dev_wrapper: str = ""):
        self.runner = str(runner)
        self.dev_wrapper = str(dev_wrapper)
        self.selected: Optional[TestCaseGroup] = None
        self.hovered: Optional[TestCaseGroup] = None
        self.log_lines: list[str] = ["준비됨. 테스트 버튼에 커서를 올리면 설명이 나오고, 클릭하면 실행합니다."]
        self.status = "대기"
        self.exit_code: Optional[int] = None
        self._queue: queue.Queue[str] = queue.Queue()
        self._proc: Optional[subprocess.Popen[str]] = None
        self._thread: Optional[threading.Thread] = None
        self._auto_scroll = False
        self._log_dirty = False

    def _install_font(self) -> None:
        from elesim_ui.theme import CONTENT_FONT_CANDIDATES, FONT_SPEC, TITLE_FONT, add_font_with_korean_ranges
        assert imgui is not None
        io = imgui.get_io()
        fonts = getattr(io, "fonts", None)
        if fonts is None or not hasattr(fonts, "add_font_from_file_ttf"):
            return
        font_path = next((p for p in CONTENT_FONT_CANDIDATES if p.exists()), None)
        if font_path is None:
            return
        try:
            font = add_font_with_korean_ranges(fonts, font_path, float(FONT_SPEC.content_px))
            if font is not None and hasattr(io, "font_default"):
                io.font_default = font
            if TITLE_FONT.exists():
                add_font_with_korean_ranges(fonts, TITLE_FONT, float(FONT_SPEC.title_px))
        except Exception as exc:
            self._append_log(f"[ui] font load skipped: {exc}")

    def _install_style(self) -> None:
        assert imgui is not None
        style = imgui.get_style()
        for attr, value in (
            ("window_rounding", 5.0),
            ("child_rounding", 5.0),
            ("frame_rounding", 4.0),
            ("scrollbar_rounding", 5.0),
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
            ("item_spacing", (8.0, 8.0)),
            ("frame_padding", (8.0, 6.0)),
            ("window_padding", (10.0, 10.0)),
        ):
            if hasattr(style, attr):
                try:
                    current = getattr(style, attr)
                    current.x = float(value[0])
                    current.y = float(value[1])
                except Exception:
                    pass

    def _append_log(self, text: str) -> None:
        for raw in str(text).splitlines() or [""]:
            self.log_lines.append(raw.rstrip())
        if len(self.log_lines) > MAX_LOG_LINES:
            del self.log_lines[: len(self.log_lines) - MAX_LOG_LINES]
        self._log_dirty = True

    def _error_summary_text(self) -> str:
        if self._proc is not None:
            return "테스트 실행 중입니다. 실패가 감지되면 여기에 요약 후보를 모읍니다."
        if self.exit_code == 0:
            return "실패 없음. 선택한 테스트 묶음이 통과했습니다."

        picked: list[str] = []
        in_failure_block = False
        in_short_summary = False
        for line in self.log_lines:
            raw = str(line).rstrip()
            text = raw.strip()
            if not text:
                continue
            lower = text.lower()
            is_section = text.startswith("=") and (
                "failures" in lower or "errors" in lower or "short test summary info" in lower
            )
            if is_section:
                picked.append(text)
                in_failure_block = "failures" in lower or "errors" in lower
                in_short_summary = "short test summary info" in lower
                continue
            if in_short_summary:
                if text.startswith("="):
                    picked.append(text)
                    in_short_summary = False
                    continue
                picked.append(text)
                continue
            if (
                text.startswith("FAILED ")
                or text.startswith("ERROR ")
                or text.startswith("E   ")
                or text.startswith("E ")
                or text.startswith("> ")
                or text.startswith("Traceback")
                or text.startswith("File ")
                or bool(PYTEST_LOCATION_RE.match(text))
                or "AssertionError" in text
                or "Exception" in text
                or "Error:" in text
            ):
                picked.append(text)
                continue
            if in_failure_block and text.startswith("_"):
                picked.append(text)

        if picked:
            return "\n".join(picked[-160:])
        if self.exit_code is None:
            return "아직 실행 결과가 없습니다."
        return "실패했지만 pytest 오류 요약 패턴을 찾지 못했습니다. 위 실행 로그를 확인하세요."

    def _readonly_multiline(self, identifier: str, text: str, height: float) -> None:
        assert imgui is not None
        input_multiline = getattr(imgui, "input_text_multiline", None)
        flags = getattr(
            imgui,
            "INPUT_TEXT_READ_ONLY",
            getattr(imgui, "INPUT_TEXT_FLAGS_READ_ONLY", 1 << 14),
        )
        width_getter = getattr(imgui, "get_content_region_available_width", None)
        width = max(100.0, float(width_getter()) if callable(width_getter) else 420.0)
        if callable(input_multiline):
            buf_size = max(1024, len(text) + 1024)
            imgui.push_item_width(width)
            try:
                try:
                    input_multiline(f"##{identifier}", text, buf_size, width, height, flags=flags)
                except TypeError:
                    try:
                        input_multiline(f"##{identifier}", text, buf_size, width, height, flags)
                    except TypeError:
                        input_multiline(f"##{identifier}", text, width, height, flags=flags)
            finally:
                imgui.pop_item_width()
            return

        imgui.begin_child(f"{identifier}_fallback", 0.0, height, True)
        try:
            text_unformatted = getattr(imgui, "text_unformatted", None)
            if callable(text_unformatted):
                text_unformatted(text)
            else:
                imgui.text_wrapped(text)
        finally:
            imgui.end_child()

    def _drain_queue(self) -> None:
        while True:
            try:
                item = self._queue.get_nowait()
            except queue.Empty:
                break
            if item.startswith("__EXIT__:"):
                try:
                    self.exit_code = int(item.split(":", 1)[1])
                except Exception:
                    self.exit_code = -1
                self.status = "통과" if self.exit_code == 0 else f"실패 ({self.exit_code})"
                self._proc = None
                continue
            self._append_log(item)

    def _command_for(self, group: TestCaseGroup) -> list[str]:
        command = ["python3", "-m", "workbench.tools.quality.check", "--selection", group.key]
        if self.runner == "dev":
            wrapper = Path(self.dev_wrapper).expanduser()
            if not wrapper.is_absolute() or wrapper.name != "elesim-dev" or not wrapper.is_file():
                raise ValueError("Supply the current installation's absolute bin/elesim-dev path")
            return [str(wrapper), *command]
        return [sys.executable, *command[1:]]

    def _run_group(self, group: TestCaseGroup) -> None:
        if self._proc is not None:
            self._append_log("[runner] A test run is already active. Stop it first.")
            return
        if not group.paths and not group.checks:
            self._append_log(f"[runner] No test files belong to {group.label}.")
            return
        self.selected = group
        self.exit_code = None
        self.status = f"{group.label} 실행 중"
        self._append_log("")
        env = os.environ.copy()

        def worker() -> None:
            code = -1
            try:
                for current in (self._command_for(group),):
                    self._queue.put(f"$ {shlex.join(current)}")
                    self._proc = subprocess.Popen(
                        current,
                        cwd=str(ROOT),
                        env=env,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        text=True,
                        bufsize=1,
                    )
                    assert self._proc.stdout is not None
                    for line in self._proc.stdout:
                        self._queue.put(line.rstrip("\n"))
                    code = int(self._proc.wait())
                    if code != 0:
                        break
            except Exception as exc:
                self._queue.put(f"[runner] Failed to start the test run: {exc}")
                code = -1
            finally:
                self._queue.put(f"__EXIT__:{code}")

        self._thread = threading.Thread(target=worker, daemon=True)
        self._thread.start()

    def _stop(self) -> None:
        proc = self._proc
        if proc is None:
            return
        self._append_log("[runner] 테스트 프로세스를 중단합니다...")
        try:
            proc.terminate()
        except Exception as exc:
            self._append_log(f"[runner] Failed to stop the test run: {exc}")

    def _draw_button_grid(self, width: float) -> None:
        assert imgui is not None
        self.hovered = None
        button_w = max(110.0, (float(width) - 8.0) * 0.5)
        for idx, group in enumerate(TEST_GROUPS):
            if idx % 2:
                imgui.same_line()
            active = self.selected is group
            pushed = 0
            if active:
                imgui.push_style_color(imgui.COLOR_BUTTON, 0.13, 0.48, 0.82, 1.0)
                imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, 0.16, 0.55, 0.90, 1.0)
                imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, 0.10, 0.38, 0.68, 1.0)
                imgui.push_style_color(imgui.COLOR_TEXT, 1.0, 1.0, 1.0, 1.0)
                pushed = 4
            try:
                if imgui.button(f"{group.label}##test_group_{idx}", button_w, 38.0):
                    self._run_group(group)
            finally:
                if pushed:
                    imgui.pop_style_color(pushed)
            if bool(getattr(imgui, "is_item_hovered", lambda: False)()):
                self.hovered = group

    def _draw_left(self, height: float) -> None:
        assert imgui is not None
        imgui.begin_child("test_left", LEFT_W, 0.0, True)
        try:
            imgui.text("테스트 묶음")
            imgui.separator()
            top_h = max(380.0, float(height) - DESCRIPTION_H)
            imgui.begin_child("test_buttons", 0.0, top_h, False)
            try:
                self._draw_button_grid(LEFT_W - 24.0)
                imgui.separator()
                if imgui.button("중단", 92.0, 34.0):
                    self._stop()
                imgui.same_line()
                if imgui.button("지우기", 92.0, 34.0):
                    self.log_lines = []
                imgui.same_line()
                _, self._auto_scroll = imgui.checkbox("따라가기", bool(self._auto_scroll))
                imgui.text(f"실행기: {self.runner}")
                imgui.text(f"상태: {self.status}")
            finally:
                imgui.end_child()

            imgui.begin_child("test_description", 0.0, 0.0, True)
            try:
                group = self.hovered or self.selected
                imgui.text("설명")
                imgui.separator()
                if group is None:
                    imgui.text_wrapped("테스트 버튼에 커서를 올리면 무엇을 확인하는지, runtime/perception/GO2/archive 중 어디를 건드리는지 볼 수 있습니다.")
                else:
                    imgui.text(group.label)
                    imgui.separator()
                    imgui.text_wrapped(group.description)
                    imgui.spacing()
                    imgui.text_wrapped("파일:")
                    for path in group.paths[:8]:
                        imgui.text_wrapped(f"- {path}")
                    if len(group.paths) > 8:
                        imgui.text_wrapped(f"... 외 {len(group.paths) - 8}개")
            finally:
                imgui.end_child()
        finally:
            imgui.end_child()

    def _draw_log(self) -> None:
        assert imgui is not None
        imgui.begin_child("test_right", 0.0, 0.0, True)
        try:
            imgui.text("실행 로그")
            imgui.same_line()
            if self.exit_code is not None:
                if self.exit_code == 0:
                    imgui.text_colored("통과", 0.1, 0.55, 0.18)
                else:
                    imgui.text_colored(f"실패 {self.exit_code}", 0.75, 0.12, 0.12)
            imgui.separator()
            available_getter = getattr(imgui, "get_content_region_available", None)
            if callable(available_getter):
                available = available_getter()
                if hasattr(available, "y"):
                    available_h = max(1.0, float(available.y))
                else:
                    available_h = max(1.0, float(available[1]))
            else:
                available_h = max(1.0, float(WINDOW_H) - 80.0)
            summary_h = min(ERROR_SUMMARY_H, max(150.0, available_h * 0.36))
            log_h = max(170.0, available_h - summary_h - 38.0)
            imgui.begin_child("test_log_scroll", 0.0, log_h, True)
            try:
                io = imgui.get_io()
                if bool(getattr(imgui, "is_window_hovered", lambda: False)()) and abs(float(getattr(io, "mouse_wheel", 0.0))) > 0.0:
                    self._auto_scroll = False
                text_unformatted = getattr(imgui, "text_unformatted", None)
                for line in self.log_lines:
                    if callable(text_unformatted):
                        text_unformatted(line)
                    else:
                        imgui.text(line[:2000])
                if self._auto_scroll and self._log_dirty:
                    set_scroll_here = getattr(imgui, "set_scroll_here_y", None)
                    if callable(set_scroll_here):
                        set_scroll_here(1.0)
                self._log_dirty = False
            finally:
                imgui.end_child()

            imgui.spacing()
            imgui.text("오류 원인 / 복사용")
            imgui.same_line()
            imgui.text_disabled("드래그 후 Ctrl+C")
            summary_box_h = max(90.0, summary_h - 32.0)
            self._readonly_multiline("test_error_summary", self._error_summary_text(), summary_box_h)
        finally:
            imgui.end_child()

    def draw(self) -> None:
        assert imgui is not None
        self._drain_queue()
        cond = getattr(imgui, "ALWAYS", 0)
        io = imgui.get_io()
        imgui.set_next_window_position(0.0, 0.0, cond)
        imgui.set_next_window_size(float(io.display_size.x), float(io.display_size.y), cond)
        flags = getattr(imgui, "WINDOW_NO_TITLE_BAR", 0)
        imgui.begin("테스트 러너###test_runner_root", True, flags=flags)
        try:
            available_getter = getattr(imgui, "get_content_region_available", None)
            if callable(available_getter):
                available = available_getter()
                if hasattr(available, "y"):
                    avail_h = max(1.0, float(available.y))
                else:
                    avail_h = max(1.0, float(available[1]))
            else:
                avail_h = max(1.0, float(getattr(io.display_size, "y", WINDOW_H)) - 24.0)
            self._draw_left(avail_h)
            imgui.same_line()
            self._draw_log()
        finally:
            imgui.end()

    def run(self) -> None:
        if glfw is None or imgui is None or GlfwRenderer is None:
            raise SystemExit(
                "The test GUI requires glfw and imgui. Install the elesim-ui "
                "dependencies."
            )
        if not glfw.init():
            raise SystemExit("glfw.init() failed.")
        glfw.window_hint(glfw.RESIZABLE, False)
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        if sys.platform == "darwin":
            glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, glfw.TRUE)
        window = glfw.create_window(WINDOW_W, WINDOW_H, "테스트 러너", None, None)
        if not window:
            glfw.terminate()
            raise SystemExit("Failed to create the GLFW window.")
        glfw.make_context_current(window)
        imgui.create_context()
        self._install_font()
        self._install_style()
        impl = GlfwRenderer(window)
        try:
            while not glfw.window_should_close(window):
                glfw.poll_events()
                impl.process_inputs()
                imgui.new_frame()
                self.draw()
                imgui.render()
                impl.render(imgui.get_draw_data())
                glfw.swap_buffers(window)
                time.sleep(0.01)
        finally:
            self._stop()
            impl.shutdown()
            glfw.terminate()


def main() -> int:
    ap = argparse.ArgumentParser(description="repo pytest 묶음을 실행하는 GUI 러너입니다.")
    ap.add_argument(
        "--runner",
        choices=("local", "dev"),
        default="local",
        help="Run the quality matrix locally or through the installed development wrapper",
    )
    ap.add_argument(
        "--dev-wrapper", default="",
        help="Absolute current-installation bin/elesim-dev path; required for --runner=dev",
    )
    args = ap.parse_args()
    TestGui(runner=args.runner, dev_wrapper=args.dev_wrapper).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
