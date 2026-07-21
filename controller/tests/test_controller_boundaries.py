from __future__ import annotations

import ast
from pathlib import Path


FORBIDDEN_IMPLEMENTATION_PATHS = (
    "elesim_controller/robot/arm/dynamixel.py",
    "elesim_controller/robot/go2",
    "elesim_controller/simulation",
    "elesim_controller/vision/sim_camera/mount.py",
    "elesim_controller/vision/sim_camera/publisher.py",
    "elesim_controller/vision/sim_camera/remote_control.py",
)


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


def test_controller_contains_only_controller_owned_implementations() -> None:
    root = Path(__file__).parents[1] / "src"
    present = []
    for path in FORBIDDEN_IMPLEMENTATION_PATHS:
        candidate = root / path
        if candidate.is_file() or (candidate.is_dir() and any(candidate.rglob("*.py"))):
            present.append(path)
    assert not present, "controller contains copied foreign-role code:\n" + "\n".join(present)


def test_controller_does_not_recreate_legacy_runtime_directories() -> None:
    root = Path(__file__).parents[1] / "src"
    legacy_fragments = ("engine/", "configs/", "crafts/")
    violations: list[str] = []
    for path in root.rglob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Constant) or not isinstance(node.value, str):
                continue
            fragment = next(
                (item for item in legacy_fragments if item in node.value),
                "",
            )
            if fragment:
                violations.append(
                    f"{path.relative_to(root)}:{node.lineno}: legacy path {fragment!r}"
                )
    assert not violations, "\n".join(violations)
