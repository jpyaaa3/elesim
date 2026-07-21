#!/usr/bin/env python3
"""Verify generated deployment contexts without relying on the source tree."""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import tempfile
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence


@dataclass(frozen=True)
class RoleSpec:
    package: str
    main_module: str
    entrypoint: str


ROLE_SPECS: Mapping[str, RoleSpec] = {
    "router": RoleSpec("elesim_router", "elesim_router.main", "elesim-router"),
    "controller": RoleSpec(
        "elesim_controller", "elesim_controller.main", "elesim-controller"
    ),
    "ui": RoleSpec("elesim_ui", "elesim_ui.main", "elesim-ui"),
    "robot": RoleSpec("elesim_robot", "elesim_robot.main", "elesim-robot"),
    "simulator": RoleSpec(
        "elesim_simulator", "elesim_simulator.main", "elesim-simulator"
    ),
}
WHEEL_ENV_KEYS = frozenset(("PROTOCOL_WHEEL", "APP_WHEEL"))
WHEEL_NAME = re.compile(r"^[A-Za-z0-9_.+-]+\.whl$")


class ReleaseVerificationError(RuntimeError):
    """A generated deployment context is incomplete or not isolated."""


def read_wheel_environment(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise ReleaseVerificationError(f"cannot read {path}: {exc}") from exc
    for line_number, raw_line in enumerate(lines, start=1):
        line = raw_line.strip()
        if not line:
            continue
        if "=" not in line:
            raise ReleaseVerificationError(f"invalid {path.name} line {line_number}")
        key, value = (part.strip() for part in line.split("=", 1))
        if key not in WHEEL_ENV_KEYS or key in values:
            raise ReleaseVerificationError(f"unexpected {path.name} key: {key!r}")
        if not WHEEL_NAME.fullmatch(value) or Path(value).name != value:
            raise ReleaseVerificationError(f"unsafe wheel name for {key}: {value!r}")
        values[key] = value
    if set(values) != WHEEL_ENV_KEYS:
        missing = ", ".join(sorted(WHEEL_ENV_KEYS - set(values)))
        raise ReleaseVerificationError(f"missing wheel environment keys: {missing}")
    return values


def _elesim_roots(wheel: Path) -> set[str]:
    try:
        with zipfile.ZipFile(wheel) as archive:
            names = archive.namelist()
    except (OSError, zipfile.BadZipFile) as exc:
        raise ReleaseVerificationError(f"invalid wheel archive {wheel}: {exc}") from exc
    roots: set[str] = set()
    for name in names:
        root = name.split("/", 1)[0]
        if root.startswith("elesim_") and not root.endswith((".dist-info", ".data")):
            roots.add(root)
    return roots


def assert_wheel_boundary(wheel: Path, expected_package: str) -> None:
    roots = _elesim_roots(wheel)
    if roots != {expected_package}:
        found = ", ".join(sorted(roots)) or "<none>"
        raise ReleaseVerificationError(
            f"{wheel.name} must contain only {expected_package}; found {found}"
        )


def _require_path(path: Path, *, kind: str = "file") -> None:
    exists = path.is_dir() if kind == "directory" else path.is_file()
    if not exists:
        raise ReleaseVerificationError(f"missing release {kind}: {path}")


def _release_wheels(release: Path, role: str) -> tuple[Path, Path]:
    values = read_wheel_environment(release / "WHEELS.env")
    wheel_dir = release / "wheels"
    protocol_wheel = wheel_dir / values["PROTOCOL_WHEEL"]
    app_wheel = wheel_dir / values["APP_WHEEL"]
    _require_path(protocol_wheel)
    _require_path(app_wheel)
    assert_wheel_boundary(protocol_wheel, "elesim_protocol")
    assert_wheel_boundary(app_wheel, ROLE_SPECS[role].package)
    return protocol_wheel, app_wheel


def verify_release_layout(release: Path, role: str) -> tuple[Path, Path]:
    if role not in ROLE_SPECS:
        raise ReleaseVerificationError(f"unknown release role: {role}")
    _require_path(release, kind="directory")
    _require_path(release / "config", kind="directory")
    _require_path(release / "config/default.yaml")
    _require_path(release / "requirements.lock")
    if role in ("controller", "simulator"):
        _require_path(release / "config/runtime.yaml")
    if role == "controller":
        _require_path(release / "config/arm_model.json")
    if role == "robot":
        _require_path(release / "install.sh")
        _require_path(release / "systemd/elesim-robot.service")
    else:
        _require_path(release / "Dockerfile")
    if role == "simulator":
        _require_path(release / "model/bundles/default/bundle.json")
    return _release_wheels(release, role)


def _probe_source() -> str:
    return r'''
import importlib
import importlib.util
import pathlib
import sys

target = pathlib.Path(sys.argv[1]).resolve()
release = pathlib.Path(sys.argv[2]).resolve()
role = sys.argv[3]
owned_package = sys.argv[4]
main_module = sys.argv[5]
entrypoint = sys.argv[6]

def require_installed(name):
    module = importlib.import_module(name)
    origin = pathlib.Path(module.__file__).resolve()
    try:
        origin.relative_to(target)
    except ValueError as exc:
        raise AssertionError(f"{name} imported outside isolated target: {origin}") from exc

require_installed("elesim_protocol")
require_installed(owned_package)

siblings = {
    "elesim_router", "elesim_controller", "elesim_ui", "elesim_robot", "elesim_simulator"
} - {owned_package}
visible = sorted(name for name in siblings if importlib.util.find_spec(name) is not None)
if visible:
    raise AssertionError(f"sibling applications visible in isolated install: {visible}")

config = release / "config/default.yaml"
if role == "controller":
    from elesim_controller.config import load_app_config, load_runtime_role_config
    from elesim_controller.robot.arm.iklib.solver import load_solver_context
    app_config = load_app_config(str(config))
    load_runtime_role_config(str(release / "config/runtime.yaml"))
    _model_config, model_context = load_solver_context(str(config))
    if "limit" not in model_context:
        raise AssertionError("controller arm model has no joint limits")
    detector_config = app_config.perception_config.resolved_detector_config_path()
    if app_config.perception_config.detector_config and not detector_config.is_file():
        raise AssertionError(f"controller detector config is missing: {detector_config}")
elif role == "ui":
    from elesim_ui.config import load_config
    load_config(str(config))
elif role == "robot":
    from elesim_robot.config import load_config
    load_config(str(config))
elif role == "simulator":
    from elesim_simulator.config import load_app_config, load_runtime_role_config
    from elesim_simulator.model_bundle import validate_model_bundle
    load_app_config(str(config))
    load_runtime_role_config(str(release / "config/runtime.yaml"))
    validate_model_bundle(release / "model/bundles/default")

module = importlib.import_module(main_module)
origin = pathlib.Path(module.__file__).resolve()
origin.relative_to(target)
sys.argv = [entrypoint, "--help"]
module.main()
'''


def _run_checked(command: Sequence[str], *, cwd: Path, env: Mapping[str, str]) -> None:
    completed = subprocess.run(
        tuple(command),
        cwd=cwd,
        env=dict(env),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=120,
        check=False,
    )
    if completed.returncode:
        rendered = " ".join(command)
        raise ReleaseVerificationError(
            f"release probe failed ({rendered}):\n{completed.stdout.rstrip()}"
        )


def verify_release_context(
    release: str | os.PathLike[str],
    role: str,
    *,
    python: str = sys.executable,
) -> None:
    release_path = Path(release).resolve()
    protocol_wheel, app_wheel = verify_release_layout(release_path, role)
    spec = ROLE_SPECS[role]
    with tempfile.TemporaryDirectory(prefix=f"elesim-{role}-install-") as td:
        target = Path(td) / "site"
        target.mkdir()
        clean_env = os.environ.copy()
        clean_env["PYTHONNOUSERSITE"] = "1"
        clean_env.pop("PYTHONPATH", None)
        _run_checked(
            (
                python,
                "-m",
                "pip",
                "install",
                "--disable-pip-version-check",
                "--no-compile",
                "--no-deps",
                "--target",
                str(target),
                str(protocol_wheel),
                str(app_wheel),
            ),
            cwd=release_path,
            env=clean_env,
        )
        probe_env = clean_env.copy()
        probe_env["PYTHONPATH"] = str(target)
        _run_checked(
            (
                python,
                "-c",
                _probe_source(),
                str(target),
                str(release_path),
                role,
                spec.package,
                spec.main_module,
                spec.entrypoint,
            ),
            cwd=release_path,
            env=probe_env,
        )


def verify_release_tree(
    release_root: str | os.PathLike[str],
    *,
    python: str = sys.executable,
) -> None:
    root = Path(release_root).resolve()
    for role in ROLE_SPECS:
        verify_release_context(root / role, role, python=python)
        print(f"verified isolated release: {root / role}")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("release_root", nargs="?", default="dist/releases")
    parser.add_argument("--role", choices=tuple(ROLE_SPECS))
    args = parser.parse_args(argv)
    root = Path(args.release_root).resolve()
    if args.role:
        verify_release_context(root / args.role, args.role)
    else:
        verify_release_tree(root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
