from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
RELEASE_PROJECTS = ("controller", "ui", "robot", "simulator")
MAX_CLASS_LINES = 1000
MAX_FUNCTION_LINES = 900


def _oversized_definitions() -> list[str]:
    failures: list[str] = []
    source_files = (
        path
        for project in RELEASE_PROJECTS
        for path in (ROOT / project / "src").rglob("*.py")
    )
    for path in sorted(source_files):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if not isinstance(node, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            if node.end_lineno is None:
                continue
            line_count = int(node.end_lineno - node.lineno + 1)
            limit = MAX_CLASS_LINES if isinstance(node, ast.ClassDef) else MAX_FUNCTION_LINES
            if line_count > limit:
                relative = path.relative_to(ROOT)
                failures.append(
                    f"{relative}:{node.lineno} {node.name} is {line_count} lines (limit {limit})"
                )
    return failures


def test_deployment_definitions_stay_within_human_review_budget() -> None:
    failures = _oversized_definitions()
    assert not failures, "oversized production definitions:\n" + "\n".join(failures)
