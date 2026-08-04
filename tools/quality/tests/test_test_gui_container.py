from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_test_gui_uses_the_generated_persistent_developer_container() -> None:
    source = (ROOT / "tools/quality/test_gui.py").read_text(encoding="utf-8")
    external_compose_root = "/home/user" + "/docker"
    legacy_runner = "u" + "rop"

    assert external_compose_root not in source
    assert legacy_runner not in source
    assert 'ROOT / ".elesim/development/compose.yaml"' in source
    assert '"exec",' in source
    assert '"dev",' in source
    assert '"/usr/local/bin/elesim-dev-env",' in source
    assert '"run",' not in source
    assert '"up",' in source
    assert '"--build",' in source
    assert '"com.docker.compose.project"' in source
    assert '"com.docker.compose.project.config_files"' in source
    assert "code = 73" in source


def test_developer_environment_fingerprints_interfaces_and_project_metadata() -> None:
    source = (ROOT / "environment/development/dev-env.sh").read_text(encoding="utf-8")

    assert 'fingerprint_file="$state_root/dev-env.fingerprint"' in source
    assert '"$interfaces/msg" "$interfaces/srv" "$interfaces/action"' in source
    assert 'fingerprint_inputs+=("$project/pyproject.toml")' in source
    assert '[[ "$input_fingerprint" != "$stored_fingerprint" ]]' in source
