"""One selection catalog for the quality CLI and its optional GUI."""
from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
from typing import Sequence

from .check import CHECKS, ROOT, Check, select_checks

TEST_ROOTS = (ROOT / "workbench/tests",)


@dataclass(frozen=True)
class Selection:
    label: str
    paths: tuple[str, ...]
    description: str
    key: str = ""
    checks: tuple[str, ...] = ()


def _existing(paths: Sequence[str]) -> tuple[str, ...]:
    return tuple(path for path in paths if (ROOT / path).exists())


def _all_tests() -> tuple[str, ...]:
    return tuple(
        str(p.relative_to(ROOT))
        for test_root in TEST_ROOTS
        for p in sorted(test_root.rglob("test_*.py"))
        if p.is_file()
    )


_ALL_TESTS = _all_tests()


def _under(prefix: str) -> tuple[str, ...]:
    return tuple(p for p in _ALL_TESTS if p.startswith(prefix))


WORKFLOWS = (
    Selection(
        "픽 전체",
        _under("workbench/tests/apps/pilot/scenarios/pick/"),
        "Pick 디버그 흐름 전체입니다. 설정, Look, Aim, 명목 운동학, Grasp/LJI, extend-ready, 수렴, E2E, 타이밍 순서로 실행합니다.",
    ),
    Selection(
        "00 준비",
        _existing(
            (
                "workbench/tests/apps/pilot/scenarios/pick/test_00_config.py",
            )
        ),
        "움직이기 전 사전 점검입니다. 실제 적용 설정, 컨트롤러 소유권, lease 아래에서 허용되는 명령 source를 확인합니다.",
    ),
    Selection(
        "10 룩",
        _existing(
            (
                "workbench/tests/apps/pilot/scenarios/pick/test_10_look.py",
                "workbench/tests/apps/pilot/scenarios/pick/test_11_look_recover.py",
                "workbench/tests/apps/pilot/scenarios/pick/test_12_auto_ready_dir.py",
                "workbench/tests/apps/pilot/scenarios/pick/test_13_ready_align.py",
                "workbench/tests/apps/pilot/scenarios/pick/test_14_feasible_ready.py",
                "workbench/tests/apps/pilot/scenarios/pick/test_15_view_pregrasp.py",
            )
        ),
        "Look/Ready 기하 점검입니다. view pose, 복구, 선호 방향, ready 정렬, feasible ready, pregrasp view를 확인합니다.",
    ),
    Selection(
        "20 에임",
        _existing(
            (
                "workbench/tests/apps/pilot/scenarios/pick/test_20_aim.py",
                "workbench/tests/apps/pilot/scenarios/pick/test_21_aim_drift.py",
                "workbench/tests/apps/pilot/scenarios/pick/test_22_uv.py",
                "workbench/tests/apps/pilot/scenarios/pick/test_23_uv_sign.py",
            )
        ),
        "Aim과 이미지 공간 조향 점검입니다. aim 수렴, drift 처리, UV Jacobian 방향, 부호 규약을 확인합니다.",
    ),
    Selection(
        "30 명목 운동학",
        _existing(
            (
                "workbench/tests/apps/pilot/scenarios/pick/test_30_nominal_kinematics.py",
                "workbench/tests/apps/pilot/scenarios/pick/test_32_ready_pose.py",
            )
        ),
        "명목 운동학과 ready pose 계산을 확인합니다.",
    ),
    Selection(
        "40 그래스프",
        _existing(
            (
                "workbench/tests/apps/pilot/scenarios/pick/test_40_guided_grasp.py",
                "workbench/tests/apps/pilot/scenarios/pick/test_41_grasp_after_aim.py",
                "workbench/tests/apps/pilot/scenarios/pick/test_42_grasp_trajectory.py",
                "workbench/tests/apps/pilot/scenarios/pick/test_43_lji.py",
                "workbench/tests/apps/pilot/scenarios/pick/test_50_extend_ready.py",
            )
        ),
        "guided grasp와 LJI 점검입니다. 그래스프 단계에서 LJI가 어떤 gate를 통과해야 하는지 확인합니다.",
    ),
    Selection(
        "60 세션",
        _existing(
            (
                "workbench/tests/apps/pilot/scenarios/pick/test_60_convergence.py",
                "workbench/tests/apps/pilot/scenarios/pick/test_70_e2e.py",
                "workbench/tests/apps/pilot/scenarios/pick/test_80_timing.py",
            )
        ),
        "Pick 세션 단위 점검입니다. 수렴, mock 기반 E2E 흐름, 타이밍 동작을 확인합니다.",
    ),
    Selection(
        "비전 체인",
        _under("workbench/tests/apps/pilot/scenarios/vision/"),
        "화면을 띄우지 않는 비전 점검입니다. 설정, hand-eye, mock world, remote config, lifecycle, detector/tracker, sim-camera 계약 순서로 봅니다.",
    ),
    Selection(
        "게이즈 체인",
        _under("workbench/tests/apps/pilot/scenarios/gaze/"),
        "정지 안정화부터 preview, baseline 비교, 보행 trial까지 gaze 동작을 시간 순서로 확인합니다.",
    ),
    Selection(
        "GO2 체인",
        _under("workbench/tests/apps/robot/scenarios/go2/")
        + _under("workbench/tests/apps/sim/scenarios/go2/"),
        "GO2 bring-up 점검입니다. 환경, lowstate, bridge, mirror mode, locomotion, payload, MPC import guard 순서로 확인합니다.",
    ),
    Selection(
        "회귀",
        _under("workbench/tests/apps/pilot/regressions/"),
        "실제 로그나 디버깅 과정에서 이미 한 번 잡은 증상을 고정합니다. 시간 순서보다는 특정 사고 재발 방지용입니다.",
    ),
    Selection(
        "계약",
        _under("workbench/tests/apps/pilot/contracts/")
        + _under("workbench/tests/apps/sim/contracts/"),
        "리팩터 중 유용한 subsystem 계약 테스트입니다. 시간 순서의 로봇 디버그 시나리오는 아니고 부품별 불변식을 확인합니다.",
    ),
    Selection(
        "Visual-servo 회귀",
        _under("workbench/tests/apps/pilot/regressions/visual_servo/"),
        "과거 visual-servo 동작과 feasible-look 조건이 다시 깨지지 않는지 확인합니다.",
    ),
    Selection(
        "LJI",
        _existing(
            (
                "workbench/tests/apps/pilot/scenarios/pick/test_43_lji.py",
                "workbench/tests/apps/pilot/contracts/vision/test_lji_invariants.py",
                "workbench/tests/apps/pilot/regressions/pick/test_lji_sign_flip_case.py",
            )
        ),
        "Local Image Jacobian 묶음입니다. 그래스프 gate, 수학 불변식, 과거 sign-flip 회귀를 함께 확인합니다.",
    ),
)


SELECTIONS = (
    Selection("필수", (), "Required software gates, including real DDS smoke commands.",
              "required", tuple(c.name for c in select_checks(group="required", names=()))),
    Selection("확장", (), "Workbench tools, research utilities and critical mutations.",
              "extended", tuple(c.name for c in select_checks(group="extended", names=()))),
    Selection("전체", (), "All catalog checks with their own source roots.",
              "all", tuple(c.name for c in CHECKS)),
    *(Selection(c.name, c.paths, c.description, c.name, (c.name,)) for c in CHECKS),
    *(replace(s, key=f"workflow-{i}") for i, s in enumerate(WORKFLOWS)),
)


def selection_checks(key: str) -> tuple[Check, ...]:
    selection = next((s for s in SELECTIONS if s.key == key), None)
    if selection is None:
        raise ValueError(f"unknown selection: {key}")
    if selection.checks:
        return select_checks(group="all", names=selection.checks)
    # Workflow buttons remain useful, but each deployment keeps its own gate.
    selected = []
    remaining = set(selection.paths)
    for check in CHECKS:
        if check.module != "pytest":
            continue
        paths = tuple(p for p in selection.paths if any(
            p == root or p.startswith(root + "/") for root in check.paths))
        if paths:
            selected.append(replace(check, paths=paths))
            remaining.difference_update(paths)
    if remaining or not selected:
        raise ValueError(f"selection has paths outside the quality matrix: {sorted(remaining)}")
    return tuple(selected)
