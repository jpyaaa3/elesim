from __future__ import annotations

import ast
from pathlib import Path


REPO_ROOT = next(parent for parent in Path(__file__).resolve().parents if (parent / "payload").is_dir())
ROBOT_ROOT = REPO_ROOT / "payload" / "runtime" / "native" / "robot" / "app" / "elesim_robot"


def test_robot_has_no_monolith_or_sibling_imports() -> None:
    root = ROBOT_ROOT
    forbidden = ("engine", "apps", "builders", "elesim_pilot", "elesim_sim", "elesim_ui")
    violations: list[str] = []
    for path in root.rglob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            names: list[str] = []
            if isinstance(node, ast.Import):
                names = [item.name for item in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module:
                names = [node.module]
            for name in names:
                if name in forbidden or name.startswith(tuple(item + "." for item in forbidden)):
                    violations.append(f"{path.relative_to(root)}:{node.lineno}: {name}")
    assert not violations, "\n".join(violations)


def test_only_the_dedicated_daemon_imports_the_unitree_ros2_bridge() -> None:
    root = ROBOT_ROOT
    importers: list[str] = []
    for path in root.rglob("*.py"):
        if path.name == "unitree_ros2_bridge.py":
            continue
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module == (
                "elesim_robot.go2.unitree_ros2_bridge"
            ):
                importers.append(str(path.relative_to(root)))
            elif isinstance(node, ast.Import):
                for item in node.names:
                    if item.name == "elesim_robot.go2.unitree_ros2_bridge":
                        importers.append(str(path.relative_to(root)))
    assert importers == ["go2/unitree_bridge_daemon.py"]
