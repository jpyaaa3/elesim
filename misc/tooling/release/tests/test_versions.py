from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
VERSION = "0.3.0"
PROJECTS = {
    "packages/protocol/pyproject.toml": "elesim-protocol",
    "controller/pyproject.toml": "elesim-controller",
    "ui/pyproject.toml": "elesim-ui",
    "simulator/pyproject.toml": "elesim-simulator",
    "robot/pyproject.toml": "elesim-robot",
    "misc/tooling/setup/pyproject.toml": "elesim-setup",
    "misc/tooling/model_builder/pyproject.toml": "elesim-model-builder",
}
INTERNAL_DEPENDENCIES = {
    "packages/protocol/pyproject.toml": (),
    "controller/pyproject.toml": ("elesim-protocol==0.3.0",),
    "ui/pyproject.toml": ("elesim-protocol==0.3.0",),
    "simulator/pyproject.toml": ("elesim-protocol==0.3.0",),
    "robot/pyproject.toml": ("elesim-protocol==0.3.0",),
    "misc/tooling/setup/pyproject.toml": ("elesim-protocol==0.3.0",),
    "misc/tooling/model_builder/pyproject.toml": (
        "elesim-protocol==0.3.0",
        "elesim-controller==0.3.0",
    ),
}
EXPORTED_VERSIONS = (
    "packages/protocol/src/elesim_protocol/__init__.py",
    "misc/tooling/setup/src/elesim_setup/__init__.py",
)


def _project_metadata(text: str) -> tuple[str, str]:
    project = text.split("[project]\n", maxsplit=1)[1].split("\n[", maxsplit=1)[0]
    name = re.search(r'^name = "([^"]+)"$', project, flags=re.MULTILINE)
    version = re.search(r'^version = "([^"]+)"$', project, flags=re.MULTILINE)
    assert name is not None
    assert version is not None
    return name.group(1), version.group(1)


def _dependency_block(text: str) -> str:
    lines: list[str] = []
    collecting = False
    for line in text.splitlines():
        stripped = line.strip()
        if not collecting and stripped.startswith("dependencies ="):
            collecting = True
            lines.append(line.split("=", maxsplit=1)[1])
            if stripped.endswith("]"):
                break
        elif collecting:
            lines.append(line)
            if stripped == "]":
                break
    return "\n".join(lines)


def test_all_project_versions_are_coordinated() -> None:
    for relative, expected_name in PROJECTS.items():
        text = (ROOT / relative).read_text(encoding="utf-8")
        assert _project_metadata(text) == (expected_name, VERSION)


def test_all_internal_dependencies_use_the_coordinated_exact_version() -> None:
    internal_names = "|".join(re.escape(name) for name in PROJECTS.values())
    pattern = re.compile(rf'"((?:{internal_names})[^"]*)"')

    for relative, expected_dependencies in INTERNAL_DEPENDENCIES.items():
        text = (ROOT / relative).read_text(encoding="utf-8")
        actual_dependencies = tuple(pattern.findall(_dependency_block(text)))
        assert actual_dependencies == expected_dependencies


def test_exported_versions_match_project_versions() -> None:
    pattern = re.compile(r'^__version__ = "([^"]+)"$', flags=re.MULTILINE)

    for relative in EXPORTED_VERSIONS:
        text = (ROOT / relative).read_text(encoding="utf-8")
        match = pattern.search(text)
        assert match is not None
        assert match.group(1) == VERSION
