from __future__ import annotations

import ast
from pathlib import Path


FORBIDDEN_IMPLEMENTATION_PATHS = (
    "elesim_pilot/robot/arm/dynamixel.py",
    "elesim_pilot/robot/go2",
    "elesim_pilot/simulation",
    "elesim_pilot/vision/sim_camera/mount.py",
    "elesim_pilot/vision/sim_camera/publisher.py",
    "elesim_pilot/vision/sim_camera/remote_control.py",
)


REPO_ROOT = next(parent for parent in Path(__file__).resolve().parents if (parent / "payload").is_dir())
PILOT_ROOT = REPO_ROOT / "payload" / "runtime" / "docker" / "pilot" / "app"


def test_pilot_has_no_monolith_builder_or_sibling_imports() -> None:
    root = PILOT_ROOT
    forbidden = ("engine", "apps", "builders", "elesim_robot", "elesim_sim", "elesim_ui")
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


def test_pilot_contains_only_pilot_owned_implementations() -> None:
    root = PILOT_ROOT
    present = []
    for path in FORBIDDEN_IMPLEMENTATION_PATHS:
        candidate = root / path
        if candidate.is_file() or (candidate.is_dir() and any(candidate.rglob("*.py"))):
            present.append(path)
    assert not present, "pilot contains copied foreign-role code:\n" + "\n".join(present)


def test_pilot_does_not_recreate_legacy_runtime_directories() -> None:
    root = PILOT_ROOT
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
