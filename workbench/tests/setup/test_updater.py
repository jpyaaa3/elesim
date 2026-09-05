from __future__ import annotations

import os
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
        prefix=tmp_path / "install",
        state_path=tmp_path / "install/install-state.json",
        compose=tmp_path / "install/containers/compose.yaml",
        build_services=("pilot", "ui", "tools"),
        preamble="printf guard-ok\\n",
        install_uuid="01234567-89ab-cdef-0123-456789abcdef",
        owned_images=(
            "elesim/pilot:local",
            "elesim/ui:local",
            "elesim/tools:local",
        ),
    )

    assert "raw.githubusercontent.com/${repository}/${ref}" in script
    assert "--state" in script and " update" in script
    assert "--edition" not in script
    assert "build pilot ui tools" in script
    assert "recorded_repository=lab/elesim" in script
    assert "recorded_ref=refactoring" in script
    assert "source=%s@%s" in script
    assert "docker compose down" not in script
    assert "docker image inspect" in script
    assert "docker image rm \"$elesim_image_id\"" in script
    assert "docker image prune" not in script
    assert "ancestor=$elesim_image_id" in script
    assert 'filter "label=io.elesim.install_uuid=$elesim_expected_install_uuid"' in script
    assert subprocess.run(
        ("bash", "-n"),
        input=script,
        text=True,
        capture_output=True,
        check=False,
    ).returncode == 0


def test_explicit_update_source_is_recorded_and_runtime_override_remains_available(
    monkeypatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setenv("ELESIM_REPOSITORY", "wrong/current")
    monkeypatch.setenv("ELESIM_REF", "wrong-ref")

    script = render_update_wrapper(
        prefix=tmp_path / "install",
        state_path=tmp_path / "install/install-state.json",
        repository="lab/elesim",
        ref="refactoring",
    )

    assert "recorded_repository=lab/elesim" in script
    assert "recorded_ref=refactoring" in script
    assert 'repository="${ELESIM_REPOSITORY:-$recorded_repository}"' in script
    assert 'ref="${ELESIM_REF:-$recorded_ref}"' in script
    assert '"$repository" == *[[:space:]]*' in script
    assert subprocess.run(
        ("bash", "-n"),
        input=script,
        text=True,
        capture_output=True,
        check=False,
    ).returncode == 0


def test_update_wrapper_rejects_a_different_install_owner(tmp_path: Path) -> None:
    script = render_update_wrapper(
        prefix=tmp_path / "install",
        state_path=tmp_path / "install/install-state.json",
        runtime_uid=0 if os.getuid() != 0 else 1,
    )

    result = subprocess.run(
        ("bash", "-c", script, "elesim-update"),
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 77
    assert "expected UID" in result.stderr


def test_update_wrapper_requires_install_identity_for_owned_image_cleanup(
    tmp_path: Path,
) -> None:
    try:
        render_update_wrapper(
            prefix=tmp_path / "install",
            state_path=tmp_path / "install/install-state.json",
            compose=tmp_path / "install/containers/compose.yaml",
            build_services=("sim",),
            owned_images=("elesim/sim:local",),
        )
    except ValueError as exc:
        assert "install_uuid" in str(exc)
    else:
        raise AssertionError("owned image cleanup must require an install UUID")


def test_general_update_can_pull_explicit_infrastructure_then_builds(
    monkeypatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setenv("ELESIM_REF", "refactoring")
    compose = tmp_path / "install/containers/compose.yaml"
    compose_wrapper = tmp_path / "bin/elesim-compose"

    script = render_update_wrapper(
        prefix=tmp_path / "install",
        state_path=tmp_path / "install/install-state.json",
        compose=compose,
        compose_wrapper=compose_wrapper,
        build_services=("sim", "tools"),
        pull_services=("tailscale",),
    )

    assert f"{compose_wrapper} -f {compose} pull tailscale" in script
    assert f"{compose_wrapper} --progress plain -f {compose} build sim tools" in script
    assert "pull sim" not in script
    assert " up " not in script
    assert "elesim-connections or run elesim-tailscale login" in script
    assert subprocess.run(
        ("bash", "-n"),
        input=script,
        text=True,
        capture_output=True,
        check=False,
    ).returncode == 0
