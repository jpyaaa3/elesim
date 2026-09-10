from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

from elesim_setup.operation_lock import render_lock_preamble


def _wrapper(tmp_path: Path) -> Path:
    maintenance = tmp_path / "maintenance"
    package = maintenance / "elesim_setup"
    package.mkdir(parents=True)
    source = Path(__file__).parents[3] / "payload/runtime/docker/setup/app/elesim_setup/operation_lock.py"
    (package / "__init__.py").write_text("\n", encoding="utf-8")
    (package / "operation_lock.py").write_bytes(source.read_bytes())
    lock = tmp_path / "instances/.locks/install.lock"
    script = "\n".join(
        (
            "#!/usr/bin/env bash",
            "set -euo pipefail",
            *render_lock_preamble(lock_path=lock, maintenance_root=maintenance),
            'printf "start-%s\\n" "$1"',
            "sleep 0.15",
            'printf "end-%s\\n" "$1"',
            "",
        )
    )
    path = tmp_path / "wrapper"
    path.write_text(script, encoding="utf-8")
    path.chmod(0o755)
    return path


def test_scoped_wrapper_serializes_contenders(tmp_path: Path):
    wrapper = _wrapper(tmp_path)
    first = subprocess.Popen([str(wrapper), "a"], stdout=subprocess.PIPE, text=True)
    time.sleep(0.03)
    second = subprocess.run([str(wrapper), "b"], capture_output=True, text=True, check=True)
    first_output = first.communicate(timeout=3)[0]
    assert first_output.splitlines() == ["start-a", "end-a"]
    assert second.stdout.splitlines() == ["start-b", "end-b"]


@pytest.mark.parametrize("kind", ["symlink", "fifo"])
def test_lock_rejects_non_regular_or_link(tmp_path: Path, kind: str):
    wrapper = _wrapper(tmp_path)
    lock = tmp_path / "instances/.locks/install.lock"
    lock.parent.mkdir(parents=True, exist_ok=True)
    if kind == "symlink":
        target = tmp_path / "elsewhere"
        target.write_text("x", encoding="utf-8")
        lock.symlink_to(target)
    else:
        os.mkfifo(lock)
    result = subprocess.run([str(wrapper), "x"], capture_output=True, text=True)
    assert result.returncode == 73
    assert "operation lock" in result.stderr.lower() or "regular" in result.stderr.lower()


def test_wrapper_syntax(tmp_path: Path):
    wrapper = _wrapper(tmp_path)
    result = subprocess.run(["bash", "-n", str(wrapper)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
