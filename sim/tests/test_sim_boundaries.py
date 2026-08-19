from __future__ import annotations

import ast
from pathlib import Path


FORBIDDEN_IMPLEMENTATION_PATHS = (
    "elesim_sim/gaze",
    "elesim_sim/pick",
    "elesim_sim/robot/arm/dynamixel.py",
    "elesim_sim/robot/arm/ik.py",
    "elesim_sim/robot/arm/iklib",
    "elesim_sim/robot/go2/hardware",
    "elesim_sim/vision/perception",
    "elesim_sim/vision/pick",
    "elesim_sim/vision/visual_servoing",
)


def test_sim_has_no_monolith_builder_or_sibling_imports() -> None:
    root = Path(__file__).parents[1] / "src"
    forbidden = ("engine", "apps", "builders", "elesim_pilot", "elesim_robot", "elesim_ui")
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


def test_sim_contains_only_sim_owned_implementations() -> None:
    root = Path(__file__).parents[1] / "src"
    present = []
    for path in FORBIDDEN_IMPLEMENTATION_PATHS:
        candidate = root / path
        if candidate.is_file() or (candidate.is_dir() and any(candidate.rglob("*.py"))):
            present.append(path)
    assert not present, "sim contains copied foreign-role code:\n" + "\n".join(present)
