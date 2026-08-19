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
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

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


TEST_ROOTS = (
    ROOT / "packages/protocol/tests",
    ROOT / "pilot/tests",
    ROOT / "robot/tests",
    ROOT / "sim/tests",
    ROOT / "ui/tests",
    ROOT / "model/builder/tests",
    ROOT / "misc/research/analysis/tests",
    ROOT / "misc/research/debug/tests",
    ROOT / "misc/research/experiments/tests",
)
SOURCE_ROOTS = (
    ROOT / "packages/protocol/src",
    ROOT / "pilot/src",
    ROOT / "robot/src",
    ROOT / "sim/src",
    ROOT / "ui/src",
    ROOT / "model/builder/src",
)
WINDOW_W = 1280
WINDOW_H = 760
LEFT_W = 350.0
DESCRIPTION_H = 320.0
ERROR_SUMMARY_H = 220.0
MAX_LOG_LINES = 4000
PYTEST_LOCATION_RE = re.compile(
    r"^(?:packages|pilot|robot|sim|ui|model|misc|installer)/.+\.py:\d+(?::|$)"
)
DEVELOPER_PROJECT = "elesim-runtime-dev"
DEVELOPER_CONTAINER = "elesim-dev"


class DockerOwnerConflict(RuntimeError):
    """A fixed EleSim container name belongs to another Compose context."""


@dataclass(frozen=True)
class TestCaseGroup:
    label: str
    paths: tuple[str, ...]
    description: str


def _existing(paths: Sequence[str]) -> tuple[str, ...]:
    resolved: list[str] = []
    for raw in paths:
        path = raw
        if raw.startswith("tests/"):
            path = "pilot/tests/regression/" + raw[len("tests/"):]
        if (ROOT / path).exists():
            resolved.append(path)
    return tuple(resolved)


def _all_tests() -> tuple[str, ...]:
    return tuple(
        str(p.relative_to(ROOT))
        for test_root in TEST_ROOTS
        for p in sorted(test_root.rglob("test_*.py"))
        if p.is_file()
    )


_ALL_TESTS = _all_tests()


def _under(prefix: str) -> tuple[str, ...]:
    if prefix.startswith("tests/"):
        prefix = "pilot/tests/regression/" + prefix[len("tests/"):]
    return tuple(p for p in _ALL_TESTS if p.startswith(prefix))


TEST_GROUPS = (
    TestCaseGroup(
        "안정",
        _ALL_TESTS,
        "현재 engine 계약 기준으로 통과가 확인된 테스트 묶음입니다. 큰 리팩터 중 새 회귀가 생겼는지 빠르게 확인할 때 씁니다.",
    ),
    TestCaseGroup(
        "전체",
        _ALL_TESTS,
        "tests/ 아래 모든 테스트를 실행합니다. 현재 코드 상태에서 전체 회귀가 없는지 확인합니다.",
    ),
    TestCaseGroup(
        "픽 전체",
        _under("tests/scenarios/pick/"),
        "Pick 디버그 흐름 전체입니다. 설정, Look, Aim, equal-sag, Grasp/LJI, extend-ready, 수렴, E2E, 타이밍 순서로 실행합니다.",
    ),
    TestCaseGroup(
        "00 준비",
        _existing(
            (
                "tests/scenarios/pick/test_00_config.py",
                "tests/scenarios/pick/test_01_ownership.py",
                "tests/scenarios/pick/test_02_pilot_lease.py",
            )
        ),
        "움직이기 전 사전 점검입니다. 실제 적용 설정, 컨트롤러 소유권, lease 아래에서 허용되는 명령 source를 확인합니다.",
    ),
    TestCaseGroup(
        "10 룩",
        _existing(
            (
                "tests/scenarios/pick/test_10_look.py",
                "tests/scenarios/pick/test_11_look_recover.py",
                "tests/scenarios/pick/test_12_auto_ready_dir.py",
                "tests/scenarios/pick/test_13_ready_align.py",
                "tests/scenarios/pick/test_14_feasible_ready.py",
                "tests/scenarios/pick/test_15_view_pregrasp.py",
            )
        ),
        "Look/Ready 기하 점검입니다. view pose, 복구, 선호 방향, ready 정렬, feasible ready, pregrasp view를 확인합니다.",
    ),
    TestCaseGroup(
        "20 에임",
        _existing(
            (
                "tests/scenarios/pick/test_20_aim.py",
                "tests/scenarios/pick/test_21_aim_drift.py",
                "tests/scenarios/pick/test_22_uv.py",
                "tests/scenarios/pick/test_23_uv_sign.py",
            )
        ),
        "Aim과 이미지 공간 조향 점검입니다. aim 수렴, drift 처리, UV Jacobian 방향, 부호 규약을 확인합니다.",
    ),
    TestCaseGroup(
        "30 새그",
        _existing(
            (
                "tests/scenarios/pick/test_30_equal_sag.py",
                "tests/scenarios/pick/test_31_sag_drift.py",
                "tests/scenarios/pick/test_32_ready_pose.py",
            )
        ),
        "equal-sag gate 점검입니다. sag drift 입력 품질과 ready pose 보정 동작을 확인합니다.",
    ),
    TestCaseGroup(
        "40 그래스프",
        _existing(
            (
                "tests/scenarios/pick/test_40_guided_grasp.py",
                "tests/scenarios/pick/test_41_grasp_after_aim.py",
                "tests/scenarios/pick/test_42_grasp_trajectory.py",
                "tests/scenarios/pick/test_43_lji.py",
                "tests/scenarios/pick/test_50_extend_ready.py",
            )
        ),
        "guided grasp와 LJI 점검입니다. 그래스프 단계에서 LJI가 어떤 gate를 통과해야 하는지 확인합니다.",
    ),
    TestCaseGroup(
        "60 세션",
        _existing(
            (
                "tests/scenarios/pick/test_60_convergence.py",
                "tests/scenarios/pick/test_70_e2e.py",
                "tests/scenarios/pick/test_80_timing.py",
            )
        ),
        "Pick 세션 단위 점검입니다. 수렴, mock 기반 E2E 흐름, 타이밍 동작을 확인합니다.",
    ),
    TestCaseGroup(
        "비전 체인",
        _under("tests/scenarios/vision/"),
        "화면을 띄우지 않는 비전 점검입니다. 설정, hand-eye, mock world, remote config, lifecycle, detector/tracker, sim-camera 계약 순서로 봅니다.",
    ),
    TestCaseGroup(
        "게이즈 체인",
        _under("tests/scenarios/gaze/"),
        "정지 안정화부터 preview, baseline 비교, 보행 trial까지 gaze 동작을 시간 순서로 확인합니다.",
    ),
    TestCaseGroup(
        "GO2 체인",
        _under("tests/scenarios/go2/"),
        "GO2 bring-up 점검입니다. 환경, lowstate, bridge, mirror mode, locomotion, payload, MPC import guard 순서로 확인합니다.",
    ),
    TestCaseGroup(
        "회귀",
        _under("tests/regressions/"),
        "실제 로그나 디버깅 과정에서 이미 한 번 잡은 증상을 고정합니다. 시간 순서보다는 특정 사고 재발 방지용입니다.",
    ),
    TestCaseGroup(
        "계약",
        _under("tests/contracts/"),
        "리팩터 중 유용한 subsystem 계약 테스트입니다. 시간 순서의 로봇 디버그 시나리오는 아니고 부품별 불변식을 확인합니다.",
    ),
    TestCaseGroup(
        "보관",
        _under("tests/contracts/archive/"),
        "예전 visual-servo 참고 테스트입니다. 일상적인 runtime 신뢰도 판단보다는 과거 동작을 추적할 때 씁니다.",
    ),
    TestCaseGroup(
        "LJI",
        _existing(
            (
                "tests/scenarios/pick/test_43_lji.py",
                "tests/contracts/vision/test_lji_invariants.py",
                "tests/regressions/pick/test_lji_sign_flip_case.py",
            )
        ),
        "Local Image Jacobian 묶음입니다. 그래스프 gate, 수학 불변식, 과거 sign-flip 회귀를 함께 확인합니다.",
    ),
)

TEST_GROUPS += (
    TestCaseGroup("프로토콜", _under("packages/protocol/tests/"), "v6 DDS 계약, peer discovery와 target-owned authority를 확인합니다."),
    TestCaseGroup("로봇", _under("robot/tests/"), "Jetson I/O 경계, q 명령과 local safety를 확인합니다."),
    TestCaseGroup("시뮬레이터", _under("sim/tests/"), "Genesis adapter, model bundle과 virtual endpoint 계약을 확인합니다."),
    TestCaseGroup("UI", _under("ui/tests/"), "UI가 pilot 구현 없이 operator protocol만 사용하는지 확인합니다."),
    TestCaseGroup("모델 빌더", _under("model/builder/tests/"), "blueprint와 URDF 생성 계약을 확인합니다."),
    TestCaseGroup("개발 도구", _under("misc/tools/"), "분석기, 로그뷰어와 실험 CLI의 순수 helper를 확인합니다."),
)


class TestGui:
    def __init__(self, *, runner: str, docker_compose: str):
        self.runner = str(runner)
        self.docker_compose = str(docker_compose)
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
        paths = list(group.paths)
        if self.runner == "docker":
            source_path = ":".join(str(path.relative_to(ROOT)) for path in SOURCE_ROOTS)
            inner = (
                f"cd {shlex.quote(str(ROOT))} && "
                f"PYTHONPATH={shlex.quote(source_path)} "
                f"python3 -m pytest {shlex.join(paths)}"
            )
            return [
                "docker",
                "compose",
                "-f",
                self.docker_compose,
                "exec",
                "-T",
                "dev",
                "/usr/local/bin/elesim-dev-env",
                "bash",
                "-lc",
                inner,
            ]
        return [sys.executable, "-m", "pytest", *paths]

    def _docker_up_command(self) -> list[str]:
        return [
            "docker",
            "compose",
            "-f",
            self.docker_compose,
            "up",
            "-d",
            "--build",
            "dev",
        ]

    def _require_docker_owner(self) -> None:
        expected_compose = str(Path(self.docker_compose).expanduser().resolve())
        result = subprocess.run(
            [
                "docker",
                "container",
                "inspect",
                "--format",
                (
                    '{{ index .Config.Labels "com.docker.compose.project" }}|'
                    '{{ index .Config.Labels "com.docker.compose.project.config_files" }}'
                ),
                DEVELOPER_CONTAINER,
            ],
            cwd=str(ROOT),
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            return
        actual_project, separator, actual_compose = result.stdout.strip().partition("|")
        if (
            separator
            and actual_project == DEVELOPER_PROJECT
            and actual_compose == expected_compose
        ):
            return
        raise DockerOwnerConflict(
            "EleSim 고정 컨테이너 이름 충돌: "
            f"{DEVELOPER_CONTAINER}\n"
            f"  기존 소유자: project={actual_project} compose={actual_compose}\n"
            f"  현재 설치: project={DEVELOPER_PROJECT} compose={expected_compose}\n"
            "기존 설치의 elesim-down으로 종료·제거한 뒤 다시 실행하십시오."
        )

    def _run_group(self, group: TestCaseGroup) -> None:
        if self._proc is not None:
            self._append_log("[runner] 이미 실행 중입니다. 먼저 중단하세요.")
            return
        if not group.paths:
            self._append_log(f"[runner] {group.label}에 해당하는 테스트 파일이 없습니다.")
            return
        self.selected = group
        self.exit_code = None
        self.status = f"{group.label} 실행 중"
        command = self._command_for(group)
        commands = (
            (self._docker_up_command(), command)
            if self.runner == "docker"
            else (command,)
        )
        self._append_log("")
        env = os.environ.copy()
        source_path = os.pathsep.join(str(path) for path in SOURCE_ROOTS)
        env["PYTHONPATH"] = source_path + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")

        def worker() -> None:
            code = -1
            try:
                if self.runner == "docker":
                    self._require_docker_owner()
                for current in commands:
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
            except DockerOwnerConflict as exc:
                self._queue.put(f"[runner] {exc}")
                code = 73
            except Exception as exc:
                self._queue.put(f"[runner] 실행 시작 실패: {exc}")
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
            self._append_log(f"[runner] 중단 실패: {exc}")

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
                "테스트 GUI에는 glfw와 imgui가 필요합니다. elesim-ui 의존성을 설치하세요."
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
            raise SystemExit("GLFW 창 생성에 실패했습니다.")
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
        choices=("local", "docker"),
        default="local",
        help="pytest를 로컬 또는 EleSim Developer 컨테이너로 실행합니다",
    )
    ap.add_argument(
        "--docker-compose",
        default=str(ROOT / ".elesim/development/compose.yaml"),
        help="--runner=docker일 때 사용할 EleSim Developer compose 파일",
    )
    args = ap.parse_args()
    TestGui(runner=args.runner, docker_compose=args.docker_compose).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
