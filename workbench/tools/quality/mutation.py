#!/usr/bin/env python3
"""Run focused, dependency-free mutation checks for critical invariants."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence


ROOT = Path(__file__).resolve().parents[3]


@dataclass(frozen=True)
class MutationCase:
    name: str
    source_root: str
    source_file: str
    original: str
    mutant: str
    tests: tuple[str, ...]
    python_paths: tuple[str, ...] = ()


CASES: tuple[MutationCase, ...] = (
    MutationCase(
        name="protocol-version-mismatch",
        source_root="payload/runtime/common/protocol",
        source_file="elesim_protocol/messages.py",
        original="or self.version != PROTOCOL_VERSION:",
        mutant="or False:",
        tests=("workbench/tests/protocol/test_protocol_v6.py",),
    ),
    MutationCase(
        name="target-authority-stale-sequence",
        source_root="payload/runtime/common/protocol",
        source_file="elesim_protocol/authority.py",
        original="if fence.sequence <= self._lease.last_command_sequence:",
        mutant="if False:",
        tests=("workbench/tests/protocol/test_peer_authority.py",),
    ),
    MutationCase(
        name="sim-operator-session-lease",
        source_root="payload/runtime/docker/sim/app",
        source_file="elesim_sim/endpoint.py",
        original="or message.lease_id != self.simulation_session_id",
        mutant="or False",
        tests=("workbench/tests/apps/sim/test_endpoint.py",),
        python_paths=("payload/runtime/common/protocol",),
    ),
    MutationCase(
        name="robot-stale-sequence",
        source_root="payload/runtime/native/robot/app",
        source_file="elesim_robot/runtime.py",
        original="if envelope.seq <= self.last_seq:",
        mutant="if False:",
        tests=("workbench/tests/apps/robot/scenarios/go2/test_12_agent_bridge.py",),
        python_paths=("payload/runtime/common/protocol",),
    ),
    MutationCase(
        name="robot-go2-deadman",
        source_root="payload/runtime/native/robot/app",
        source_file="elesim_robot/runtime.py",
        original="if now - self._go2_command_at <= float(self.safety.command_deadman_s):",
        mutant="if True:",
        tests=("workbench/tests/apps/robot/test_runtime.py",),
        python_paths=("payload/runtime/common/protocol",),
    ),
    MutationCase(
        name="uv-control-direction",
        source_root="payload/runtime/docker/pilot/app",
        source_file="elesim_pilot/vision/visual_servoing/uv_jacobian.py",
        original="delta = -float(gain) * (pinv @ err)",
        mutant="delta = +float(gain) * (pinv @ err)",
        tests=("workbench/tests/apps/pilot/properties/test_uv_jacobian_properties.py",),
        python_paths=("payload/runtime/common/protocol",),
    ),
    MutationCase(
        name="lji-gain-direction",
        source_root="payload/runtime/docker/pilot/app",
        source_file="elesim_pilot/vision/visual_servoing/local_image_jacobian.py",
        original="s_stack = gains[active] * ordered_s[active]",
        mutant="s_stack = ordered_s[active] / gains[active]",
        tests=("workbench/tests/apps/pilot/properties/test_lji_properties.py",),
        python_paths=("payload/runtime/common/protocol",),
    ),
    MutationCase(
        name="equal-sag-finite-input",
        source_root="payload/runtime/docker/pilot/app",
        source_file="elesim_pilot/vision/visual_servoing/equal_sag_probe.py",
        original="if not np.all(np.isfinite(drift)) or not np.all(np.isfinite(j)):",
        mutant="if False:",
        tests=("workbench/tests/apps/pilot/properties/test_equal_sag_properties.py",),
        python_paths=("payload/runtime/common/protocol",),
    ),
)


class MutationError(RuntimeError):
    """A mutation was invalid or survived its focused tests."""


def apply_mutation(path: Path, original: str, mutant: str) -> None:
    text = path.read_text(encoding="utf-8")
    count = text.count(original)
    if count != 1:
        raise MutationError(
            f"mutation anchor must occur exactly once in {path}, found {count}: {original!r}"
        )
    path.write_text(text.replace(original, mutant, 1), encoding="utf-8")


def validate_cases(cases: Sequence[MutationCase] = CASES) -> None:
    names: set[str] = set()
    for case in cases:
        if case.name in names:
            raise MutationError(f"duplicate mutation name: {case.name}")
        names.add(case.name)
        path = ROOT / case.source_root / case.source_file
        if not path.is_file():
            raise MutationError(f"mutation source is missing: {path}")
        count = path.read_text(encoding="utf-8").count(case.original)
        if count != 1:
            raise MutationError(
                f"mutation anchor drifted for {case.name}: expected once, found {count}"
            )


def run_case(case: MutationCase, *, python: str = sys.executable) -> None:
    source = ROOT / case.source_root
    with tempfile.TemporaryDirectory(prefix=f"elesim-mutant-{case.name}-") as td:
        mutant_source = Path(td) / "src"
        shutil.copytree(
            source,
            mutant_source,
            ignore=shutil.ignore_patterns("__pycache__", "*.pyc", "*.pyo", "*.egg-info"),
        )
        apply_mutation(mutant_source / case.source_file, case.original, case.mutant)
        python_path = [str(mutant_source)]
        python_path.extend(str((ROOT / path).resolve()) for path in case.python_paths)
        env = os.environ.copy()
        env["PYTHONPATH"] = os.pathsep.join(python_path)
        env["PYTHONDONTWRITEBYTECODE"] = "1"
        completed = subprocess.run(
            (python, "-m", "pytest", *case.tests, "-q"),
            cwd=ROOT,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=120,
            check=False,
        )
    if completed.returncode == 0:
        raise MutationError(f"mutation survived: {case.name}")
    if completed.returncode != 1:
        raise MutationError(
            f"mutation did not produce an assertion failure: {case.name}\n"
            f"{completed.stdout.rstrip()}"
        )


def main() -> int:
    validate_cases()
    for case in CASES:
        run_case(case)
        print(f"mutation killed: {case.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
