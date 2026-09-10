from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

from elesim_setup.updater import render_release_wrapper, render_update_wrapper


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
    assert "bootstrap-source-revision" not in script
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


def test_update_wrapper_accepts_matching_immutable_image_and_rejects_foreign(
    tmp_path: Path,
) -> None:
    install_uuid = "01234567-89ab-cdef-0123-456789abcdef"
    image = "elesim/pilot:0123456789abcdef0123456789abcdef-" + "a" * 64
    script = render_update_wrapper(
        prefix=tmp_path / "install",
        state_path=tmp_path / "install/install-state.json",
        compose=tmp_path / "install/containers/compose.yaml",
        install_uuid=install_uuid,
        owned_images=(image,),
    )
    assert image in script
    with pytest.raises(ValueError, match="current-install"):
        render_update_wrapper(
            prefix=tmp_path / "install",
            state_path=tmp_path / "install/install-state.json",
            install_uuid=install_uuid,
            owned_images=(
                "elesim/pilot:fedcba9876543210fedcba9876543210-" + "a" * 64,
            ),
        )


def test_general_update_does_not_pull_infrastructure_before_building(
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
    )

    assert f"{compose_wrapper} -f {compose} pull" not in script
    assert f"{compose_wrapper} --progress plain -f {compose} build sim tools" in script
    assert " up " not in script
    assert subprocess.run(
        ("bash", "-n"),
        input=script,
        text=True,
        capture_output=True,
        check=False,
    ).returncode == 0


def test_scoped_release_wrapper_publishes_only_after_complete_build(
    tmp_path: Path,
) -> None:
    install_uuid = "01234567-89ab-cdef-0123-456789abcdef"
    images = tuple(
        f"elesim/{role}:{install_uuid.replace('-', '')}-{'a' * 64}"
        for role in ("pilot", "sim", "ui")
    )
    script = render_release_wrapper(
        prefix=tmp_path / "install",
        state_path=tmp_path / "install/install-state.json",
        compose=tmp_path / "install/containers/compose.yaml",
        compose_wrapper=tmp_path / "bin/elesim-compose",
        build_services=("pilot", "sim", "ui", "tools"),
        source_revision="git-" + "b" * 40,
        runtime_snapshot=tmp_path / "install/containers/runtime-snapshot",
        install_uuid=install_uuid,
        release_images=images,
    )

    assert "curl -fsSL" not in script
    assert "run --rm --no-deps --no-build tools elesim-setup" in script
    assert "release publish" in script
    assert "mktemp" in script and "umask 077" in script
    assert script.index("build pilot sim ui tools") < script.index("release publish")
    assert "run elesim-up to apply" not in script
    assert "registered instances remain pinned" in script
    assert script.index("release publish") < script.index("immutable release published")
    assert subprocess.run(
        ("bash", "-n"),
        input=script,
        text=True,
        capture_output=True,
        check=False,
    ).returncode == 0


def test_scoped_update_reads_only_fresh_authenticated_revision_handoff(
    tmp_path: Path,
) -> None:
    install_uuid = "01234567-89ab-cdef-0123-456789abcdef"
    script = render_update_wrapper(
        prefix=tmp_path / "install",
        state_path=tmp_path / "install/install-state.json",
        compose=tmp_path / "install/containers/compose.yaml",
        build_services=("pilot", "tools"),
        install_uuid=install_uuid,
        runtime_snapshot=tmp_path / "install/containers/runtime-snapshot",
        publish_roles=("pilot",),
    )

    assert "rm -f -- \"$release_revision_file\"" in script
    assert "ELESIM_SCOPED_UPDATE=1" in script
    assert "ELESIM_SOURCE_REVISION_FILE=\"$release_revision_file\"" in script
    assert "authenticated source revision handoff is missing" in script
    assert "authenticated source revision handoff is invalid" in script
    # The outer wrapper must not accept a stale or forged ambient value after
    # bootstrap exits; only its exact private handoff is consulted.
    assert "ELESIM_SOURCE_REVISION:-" not in script
    assert "docker image rm" not in script
    assert subprocess.run(
        ("bash", "-n"),
        input=script,
        text=True,
        capture_output=True,
        check=False,
    ).returncode == 0


def test_release_wrapper_requires_an_embedded_revision(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="embed its authenticated"):
        render_release_wrapper(
            prefix=tmp_path / "install",
            state_path=tmp_path / "install/install-state.json",
            compose=tmp_path / "install/containers/compose.yaml",
            compose_wrapper=None,
            build_services=("pilot", "tools"),
            source_revision=None,
            runtime_snapshot=tmp_path / "install/containers/runtime-snapshot",
            install_uuid="01234567-89ab-cdef-0123-456789abcdef",
            release_images=(
                "elesim/pilot:0123456789abcdef0123456789abcdef-" + "a" * 64,
            ),
        )
