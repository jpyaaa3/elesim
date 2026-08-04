"""Guard the boundary between deployable code and repository-only material."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
RUNTIME_SOURCES = tuple(
    ROOT / directory / "src"
    for directory in ("pilot", "ui", "robot", "sim")
) + (ROOT / "packages/protocol/src",)


def test_runtime_sources_do_not_depend_on_research_material() -> None:
    for source_root in RUNTIME_SOURCES:
        for source in source_root.rglob("*.py"):
            text = source.read_text(encoding="utf-8")
            assert "from research" not in text, source
            assert "import research" not in text, source


def test_test_only_helpers_are_not_packaged_as_runtime_code() -> None:
    assert not (ROOT / "pilot/src/elesim_pilot/observability/pick_replay.py").exists()
    assert not (ROOT / "sim/src/elesim_sim/experiment/sim_target_visibility.py").exists()
