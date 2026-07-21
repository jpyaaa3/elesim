from __future__ import annotations

from pathlib import Path

import pytest

from tooling.quality.mutation import CASES, MutationError, apply_mutation, validate_cases


def test_every_critical_mutation_has_a_unique_live_anchor() -> None:
    validate_cases()
    assert len({case.name for case in CASES}) == len(CASES)


def test_apply_mutation_changes_exactly_one_anchor(tmp_path: Path) -> None:
    source = tmp_path / "sample.py"
    source.write_text("enabled = True\n", encoding="utf-8")

    apply_mutation(source, "True", "False")

    assert source.read_text(encoding="utf-8") == "enabled = False\n"


@pytest.mark.parametrize("content", ("value = 1\n", "value = True or True\n"))
def test_apply_mutation_rejects_missing_or_ambiguous_anchor(
    tmp_path: Path,
    content: str,
) -> None:
    source = tmp_path / "sample.py"
    source.write_text(content, encoding="utf-8")

    with pytest.raises(MutationError, match="exactly once"):
        apply_mutation(source, "True", "False")
