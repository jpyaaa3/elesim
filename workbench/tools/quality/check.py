#!/usr/bin/env python3
"""Run EleSim's software-only verification matrix.

Each check gets only the source roots that its deployment is allowed to import.
This makes the command useful both as a local test runner and as a lightweight
architecture boundary check.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


ROOT = Path(__file__).resolve().parents[3]
PROTOCOL_SRC = "payload/runtime/common/protocol"


@dataclass(frozen=True)
class Check:
    name: str
    paths: tuple[str, ...]
    python_paths: tuple[str, ...]
    group: str = "required"
    module: str = "pytest"
    description: str = ""

    def command(self, python: str = sys.executable) -> tuple[str, ...]:
        if self.module == "pytest":
            return (python, "-m", "pytest", *self.paths)
        return (python, *self.paths)


CHECKS: tuple[Check, ...] = (
    Check(
        "protocol",
        ("workbench/tests/protocol",),
        (PROTOCOL_SRC,),
        description="Protocol v6 DDS contracts, peer authority and media sessions",
    ),
    Check(
        "robot",
        ("workbench/tests/apps/robot",),
        (PROTOCOL_SRC, "payload/runtime/native/robot/app"),
        description="Physical I/O boundary and local safety behavior",
    ),
    Check(
        "pilot",
        ("workbench/tests/apps/pilot",),
        (PROTOCOL_SRC, "payload/runtime/docker/pilot/app"),
        description="Vision, IK, gaze and Pick control behavior",
    ),
    Check(
        "sim",
        ("workbench/tests/apps/sim",),
        (PROTOCOL_SRC, "payload/runtime/docker/sim/app"),
        description="Genesis endpoint and virtual-robot behavior",
    ),
    Check(
        "ui",
        ("workbench/tests/apps/ui",),
        (PROTOCOL_SRC, "payload/runtime/docker/ui/app"),
        description="Operator API and presentation state behavior",
    ),
    Check(
        "model-builder",
        ("workbench/tests/model_builder", "workbench/tests/tools/release"),
        (PROTOCOL_SRC, "payload/runtime/docker/pilot/app", "model/builder/src"),
        description="Blueprint, bundle and URDF generation",
    ),
    Check(
        "topology",
        ("workbench/tests/system/smoke_topology.py",),
        (PROTOCOL_SRC,),
        module="script",
        description="Router-free four-process DDS topology smoke test",
    ),
    Check(
        "dds-rgbd",
        ("workbench/tests/system/test_dds_rgbd.py",),
        (PROTOCOL_SRC, "payload/runtime/docker/pilot/app", "payload/runtime/docker/sim/app"),
        description="Typed latest-frame DDS RGBD contract and peer fencing",
    ),
    Check(
        "webrtc-media",
        ("workbench/tests/system/test_webrtc_media.py",),
        (PROTOCOL_SRC, "payload/runtime/docker/sim/app", "payload/runtime/docker/ui/app"),
        description="Two independent encoded Sim WebRTC streams",
    ),
    Check(
        "quality-tools",
        ("workbench/tests/tools/quality",),
        (PROTOCOL_SRC,),
        group="extended",
        description="Repository quality-gate behavior and readability budgets",
    ),
    Check(
        "setup-tools",
        ("workbench/tests/setup",),
        (PROTOCOL_SRC, "payload/runtime/docker/tools/app"),
        description="Installer profiles, generated configs and network diagnostics",
    ),
    Check(
        "critical-mutations",
        ("workbench/tools/quality/mutation.py",),
        (PROTOCOL_SRC,),
        group="extended",
        module="script",
        description="Focused mutation checks for safety and control invariants",
    ),
    Check(
        "analysis-tools",
        ("workbench/tests/research/analysis",),
        (PROTOCOL_SRC, "payload/runtime/docker/pilot/app", "model/builder/src"),
        group="extended",
        description="Offline analysis helpers",
    ),
    Check(
        "debug-tools",
        ("workbench/tests/research/debug",),
        (PROTOCOL_SRC, "payload/runtime/docker/pilot/app", "payload/runtime/docker/ui/app"),
        group="extended",
        description="Manual debugger support code",
    ),
    Check(
        "experiment-tools",
        ("workbench/tests/research/experiments",),
        (PROTOCOL_SRC, "payload/runtime/docker/pilot/app", "model/builder/src"),
        group="extended",
        description="Repeatable experiment orchestration",
    ),
)


def checks_by_name() -> dict[str, Check]:
    return {check.name: check for check in CHECKS}


def select_checks(*, group: str, names: Sequence[str]) -> tuple[Check, ...]:
    indexed = checks_by_name()
    if names:
        unknown = sorted(set(names) - set(indexed))
        if unknown:
            raise ValueError(f"unknown checks: {', '.join(unknown)}")
        return tuple(indexed[name] for name in names)
    if group == "all":
        return CHECKS
    if group == "required":
        return tuple(check for check in CHECKS if check.group == "required")
    if group == "extended":
        return tuple(check for check in CHECKS if check.group == "extended")
    raise ValueError(f"unknown group: {group}")


def python_path(check: Check, inherited: str = "") -> str:
    entries = [str((ROOT / relative).resolve()) for relative in check.python_paths]
    entries.extend(part for part in inherited.split(os.pathsep) if part)
    return os.pathsep.join(dict.fromkeys(entries))


def run_checks(checks: Iterable[Check], *, fail_fast: bool = False) -> int:
    failures: list[str] = []
    for check in checks:
        env = os.environ.copy()
        env["PYTHONPATH"] = python_path(check, env.get("PYTHONPATH", ""))
        command = check.command()
        print(f"\n== {check.name}: {check.description} ==", flush=True)
        print("$ " + " ".join(command), flush=True)
        completed = subprocess.run(command, cwd=ROOT, env=env, check=False)
        if completed.returncode:
            failures.append(check.name)
            if fail_fast:
                break
    if failures:
        print("\nFailed checks: " + ", ".join(failures), file=sys.stderr)
        return 1
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--group",
        choices=("required", "extended", "all"),
        default="required",
        help="required is the documented release gate; all also runs tooling tests",
    )
    parser.add_argument("--check", action="append", default=[], help="run one named check; repeatable")
    parser.add_argument("--list", action="store_true", help="list checks without running them")
    parser.add_argument("--fail-fast", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        selected = select_checks(group=args.group, names=args.check)
    except ValueError as exc:
        _parser().error(str(exc))
    if args.list:
        for check in selected:
            print(f"{check.name:18} {check.group:8} {check.description}")
        return 0
    return run_checks(selected, fail_fast=bool(args.fail_fast))


if __name__ == "__main__":
    raise SystemExit(main())
