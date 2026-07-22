#!/usr/bin/env python3
"""Generate repeatable stdlib line-execution reports for one deployment."""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence


ROOT = Path(__file__).resolve().parents[3]


@dataclass(frozen=True)
class CoverageProfile:
    tests: str
    python_paths: tuple[str, ...]


PROFILES: dict[str, CoverageProfile] = {
    "protocol": CoverageProfile("packages/protocol/tests", ("packages/protocol/src",)),
    "router": CoverageProfile(
        "router/tests",
        ("packages/protocol/src", "router/src"),
    ),
    "robot": CoverageProfile(
        "robot/tests",
        ("packages/protocol/src", "robot/src"),
    ),
    "controller": CoverageProfile(
        "controller/tests",
        ("packages/protocol/src", "controller/src"),
    ),
    "simulator": CoverageProfile(
        "simulator/tests",
        ("packages/protocol/src", "simulator/src"),
    ),
    "ui": CoverageProfile(
        "ui/tests",
        ("packages/protocol/src", "ui/src"),
    ),
}
SUMMARY_LINE = re.compile(r"^\s*(\d+)\s+(\d+)%\s+(elesim_[A-Za-z0-9_.]+)\s+")


def parse_project_summary(output: str) -> tuple[tuple[str, int, int], ...]:
    records: list[tuple[str, int, int]] = []
    for line in output.splitlines():
        match = SUMMARY_LINE.match(line)
        if match:
            records.append((match.group(3), int(match.group(1)), int(match.group(2))))
    return tuple(records)


def trace_command(
    profile: CoverageProfile,
    output_dir: Path,
    *,
    python: str = sys.executable,
) -> tuple[str, ...]:
    return (
        python,
        "-m",
        "trace",
        "--count",
        "--missing",
        "--summary",
        f"--coverdir={output_dir}",
        "--ignore-dir=/usr:/opt",
        "--module",
        "pytest",
        profile.tests,
        "-q",
    )


def run_profile(
    role: str,
    *,
    output_root: Path,
    python: str = sys.executable,
) -> tuple[tuple[str, int, int], ...]:
    profile = PROFILES[role]
    output_dir = output_root / role
    shutil.rmtree(output_dir, ignore_errors=True)
    output_dir.mkdir(parents=True)
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join(
        str((ROOT / path).resolve()) for path in profile.python_paths
    )
    completed = subprocess.run(
        trace_command(profile, output_dir, python=python),
        cwd=ROOT,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=600,
        check=False,
    )
    print(completed.stdout, end="")
    if completed.returncode:
        raise RuntimeError(f"coverage test run failed for {role}")
    records = parse_project_summary(completed.stdout)
    if not records:
        raise RuntimeError(f"coverage report contained no Elesim modules for {role}")
    return records


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("role", choices=(*PROFILES, "all"))
    parser.add_argument("--output", default="/tmp/elesim-line-coverage")
    args = parser.parse_args(argv)
    roles = tuple(PROFILES) if args.role == "all" else (args.role,)
    for role in roles:
        run_profile(role, output_root=Path(args.output).resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
