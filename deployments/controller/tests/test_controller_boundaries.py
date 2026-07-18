from __future__ import annotations

import ast
from pathlib import Path


def test_controller_has_no_monolith_builder_or_sibling_imports() -> None:
    root = Path(__file__).parents[1] / "src"
    forbidden = ("engine", "apps", "builders", "elesim_robot", "elesim_simulator", "elesim_ui")
    violations: list[str] = []
    for path in root.rglob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            names = []
            if isinstance(node, ast.Import):
                names = [item.name for item in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module:
                names = [node.module]
            for name in names:
                if name in forbidden or name.startswith(tuple(item + "." for item in forbidden)):
                    violations.append(f"{path.relative_to(root)}:{node.lineno}: {name}")
    assert not violations, "\n".join(violations)
