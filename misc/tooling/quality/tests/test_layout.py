"""Guard the boundary between deployable code and repository-only material."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
RUNTIME_SOURCES = tuple(
    ROOT / role / "src" for role in ("controller", "ui", "robot", "simulator")
) + (ROOT / "packages/protocol/src",)


def test_runtime_sources_do_not_depend_on_research_material() -> None:
    for source_root in RUNTIME_SOURCES:
        for source in source_root.rglob("*.py"):
            assert "misc.research" not in source.read_text(encoding="utf-8"), source


def test_test_only_helpers_are_not_packaged_as_runtime_code() -> None:
    assert not (ROOT / "controller/src/elesim_controller/observability/pick_replay.py").exists()
    assert not (ROOT / "simulator/src/elesim_simulator/experiment/sim_target_visibility.py").exists()
