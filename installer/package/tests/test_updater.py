from __future__ import annotations

import subprocess
from pathlib import Path

from elesim_setup.updater import render_update_wrapper


def test_general_update_wrapper_fetches_regenerates_and_builds_incrementally(
    monkeypatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setenv("ELESIM_REPOSITORY", "lab/elesim")
    monkeypatch.setenv("ELESIM_REF", "refactoring")
    script = render_update_wrapper(
        edition="general",
        prefix=tmp_path / "install",
        state_path=tmp_path / "install/install-state.json",
        compose=tmp_path / "install/containers/compose.yaml",
        build_services=("pilot", "ui", "tools"),
        preamble="printf guard-ok\\n",
    )

    assert "raw.githubusercontent.com/${repository}/${ref}" in script
    assert "update --edition general" in script
    assert "build pilot ui tools" in script
    assert "docker compose down" not in script
    assert "docker image rm" not in script
    assert subprocess.run(
        ("bash", "-n"),
        input=script,
        text=True,
        capture_output=True,
        check=False,
    ).returncode == 0


def test_developer_update_wrapper_requires_clean_fast_forward_checkout(
    monkeypatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setenv("ELESIM_REF", "refactoring")
    script = render_update_wrapper(
        edition="developer",
        prefix=tmp_path / "workspace",
        state_path=tmp_path / "workspace/.elesim/development/install-state.json",
        compose=tmp_path / "workspace/.elesim/development/compose.yaml",
        build_services=("dev",),
    )

    assert "diff --quiet" in script
    assert "diff --cached --quiet" in script
    assert 'fetch --prune origin "$ref"' in script
    assert "merge --ff-only FETCH_HEAD" in script
    assert "update --edition developer" in script
    assert "build dev" in script
