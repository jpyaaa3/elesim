from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]


def test_test_gui_uses_the_same_catalog_and_current_install_wrapper(tmp_path) -> None:
    from workbench.tools.quality.selections import SELECTIONS, selection_checks
    from workbench.tools.quality.test_gui import TestGui
    from workbench.tools.quality.check import select_checks
    required = next(s for s in SELECTIONS if s.key == "required")
    assert selection_checks("required") == select_checks(group="required", names=())
    assert len({s.key for s in SELECTIONS}) == len(SELECTIONS)
    assert "안정" not in {s.label for s in SELECTIONS}
    local = TestGui()._command_for(required)
    assert local[1:] == ["-m", "workbench.tools.quality.check", "--selection", "required"]
    wrapper = tmp_path / "bin/elesim-dev"
    wrapper.parent.mkdir()
    wrapper.touch()
    command = TestGui(runner="dev", dev_wrapper=str(wrapper))._command_for(required)
    assert command == [str(wrapper), "python3", *local[1:]]


def test_workflow_selections_keep_role_boundaries_and_complete_path_coverage():
    from workbench.tools.quality.selections import SELECTIONS, selection_checks
    for selection in SELECTIONS:
        checks = selection_checks(selection.key)
        assert checks
        if selection.key.startswith("workflow-"):
            assert {p for c in checks for p in c.paths} == set(selection.paths)


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
    source = (ROOT / "payload/runtime/docker/dev/dev-env.sh").read_text(encoding="utf-8")

    assert 'fingerprint_file="$state_root/dev-env.fingerprint"' in source
    assert '"$interfaces/msg" "$interfaces/srv" "$interfaces/action"' in source
    assert 'fingerprint_inputs+=("$project/pyproject.toml")' in source
    assert 'runtime_contract_inputs=(' in source
    assert '"$workspace/payload/runtime/docker/sim/requirements.lock"' in source
    assert '"$workspace/payload/runtime/docker/shared/Dockerfile.app"' in source
    assert '[[ "$input_fingerprint" != "$stored_fingerprint" ]]' in source


def test_developer_runtime_parity_runs_isolated_release_gate() -> None:
    source = (ROOT / "payload/runtime/docker/dev/dev-env.sh").read_text(encoding="utf-8")

    assert '[[ "${1:-}" == "--runtime-parity" ]]' in source
    assert 'python3 "$build_tool"' in source
    assert 'python3 "$verify_tool" "$workspace/dist/releases"' in source
    parity = (ROOT / "workbench/tools/release/runtime_parity.py").read_text(encoding="utf-8")
    assert "ELESIM_RUNTIME_ROLES" in parity
    assert "importlib.metadata.version" in parity
