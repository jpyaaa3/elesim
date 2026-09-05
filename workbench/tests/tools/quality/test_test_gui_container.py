from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]


def test_test_gui_uses_the_generated_persistent_developer_container() -> None:
    source = (ROOT / "workbench/tools/quality/test_gui.py").read_text(encoding="utf-8")
    external_compose_root = "/home/user" + "/docker"
    legacy_runner = "u" + "rop"

    assert external_compose_root not in source
    assert legacy_runner not in source
    assert '~/.local/share/elesim/containers/compose.yaml' in source
    assert 'DEVELOPER_PROJECT = "elesim-runtime"' in source
    assert '"--profile",' in source
    assert '"developer",' in source
    assert '"exec",' in source
    assert '"dev",' in source
    assert '"/usr/local/bin/elesim-dev-env",' in source
    assert '"run",' not in source
    assert '"up",' in source
    assert '"--build",' in source
    assert '"com.docker.compose.project"' in source
    assert '"com.docker.compose.project.config_files"' in source
    assert "code = 73" in source


def test_developer_tooling_has_no_standalone_compose_project() -> None:
    retired_project = "elesim-runtime" + "-dev"
    implementation_roots = (
        ROOT / "installer",
        ROOT / "payload/runtime/docker",
        ROOT / "workbench/tools",
    )

    offenders = []
    for root in implementation_roots:
        for path in root.rglob("*"):
            if path.suffix not in {".py", ".sh", ".json", ".yaml", ".yml", ".md"}:
                continue
            if retired_project in path.read_text(encoding="utf-8"):
                offenders.append(str(path.relative_to(ROOT)))

    assert offenders == []


def test_developer_environment_fingerprints_interfaces_and_project_metadata() -> None:
    source = (ROOT / "payload/runtime/docker/development/dev-env.sh").read_text(encoding="utf-8")

    assert 'fingerprint_file="$state_root/dev-env.fingerprint"' in source
    assert '"$interfaces/msg" "$interfaces/srv" "$interfaces/action"' in source
    assert 'fingerprint_inputs+=("$project/pyproject.toml")' in source
    assert '[[ "$input_fingerprint" != "$stored_fingerprint" ]]' in source
