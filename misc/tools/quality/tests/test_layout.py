"""Guard the boundary between deployable code and repository-only material."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
ROLES = ("pilot", "ui", "robot", "sim")
RUNTIME_SOURCES = tuple(
    ROOT / relative
    for relative in (
        "payload/runtime/docker/pilot/app",
        "payload/runtime/docker/ui/app",
        "payload/runtime/native/robot/app",
        "payload/runtime/docker/sim/app",
    )
) + (ROOT / "payload/runtime/common/protocol",)


def test_payload_is_the_only_deployable_source_layout() -> None:
    assert (ROOT / "payload/runtime/common/protocol/elesim_protocol").is_dir()
    assert (ROOT / "payload/runtime/common/elesim_interfaces/package.xml").is_file()
    projects = {
        "pilot": ROOT / "payload/runtime/docker/pilot/app",
        "sim": ROOT / "payload/runtime/docker/sim/app",
        "ui": ROOT / "payload/runtime/docker/ui/app",
        "robot": ROOT / "payload/runtime/native/robot/app",
    }
    for role, project in projects.items():
        assert (project / "pyproject.toml").is_file()
        assert (project / f"elesim_{role}").is_dir()
        assert not (project / "src").exists()
        assert not (ROOT / role).exists()
    assert (ROOT / "payload/config/pilot/config.yaml").is_file()
    assert (ROOT / "payload/data/models/assemblies/zed-mini/bundle.json").is_file()
    assert (ROOT / "payload/runtime/docker/tools/app/pyproject.toml").is_file()
    assert (ROOT / "payload/runtime/docker/tools/app/elesim_setup").is_dir()
    assert (ROOT / "tests/setup").is_dir()
    assert not (ROOT / "installer/package").exists()
    assert not (ROOT / "payload/roles").exists()
    assert not (ROOT / "packages").exists()


def test_runtime_sources_do_not_depend_on_research_material() -> None:
    for source_root in RUNTIME_SOURCES:
        for source in source_root.rglob("*.py"):
            text = source.read_text(encoding="utf-8")
            assert "from misc.research" not in text, source
            assert "import misc.research" not in text, source


def test_test_only_helpers_are_not_packaged_as_runtime_code() -> None:
    assert not (ROOT / "payload/runtime/docker/pilot/app/elesim_pilot/observability/pick_replay.py").exists()
    assert not (ROOT / "payload/runtime/docker/sim/app/elesim_sim/experiment/sim_target_visibility.py").exists()
