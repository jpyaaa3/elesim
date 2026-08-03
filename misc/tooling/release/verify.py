#!/usr/bin/env python3
"""Verify generated deployment contexts without relying on the source tree."""

from __future__ import annotations

import argparse
import configparser
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
COMMON_ROLE_ENTRIES = frozenset(
    ("WHEELS.env", "config", "interfaces", "requirements.lock", "wheels")
)
ROBOT_SYSTEMD_UNITS = frozenset(
    ("elesim-robot.service", "elesim-unitree-bridge.service")
)
ROBOT_WHEEL_MODULES = frozenset(
    (
        "elesim_robot/go2/unitree_bridge_daemon.py",
        "elesim_robot/go2/unitree_ipc.py",
        "elesim_robot/go2/unitree_ipc_protocol.py",
    )
)
ROBOT_CONSOLE_SCRIPTS = {
    "elesim-robot": "elesim_robot.main:main",
    "elesim-unitree-bridge": "elesim_robot.go2.unitree_bridge_daemon:main",
}


class ReleaseVerificationError(RuntimeError):
    """A generated deployment context is incomplete or not isolated."""


def expected_release_entries(role: str) -> frozenset[str]:
    """Return the complete top-level manifest for one deployable role.

    The only runtime material shared by every role is the protocol wheel and
    ROSIDL interface source, held below ``wheels`` and ``interfaces``.  Every
    other entry is either role-owned or deliberately absent.
    """
    if role not in ROLE_SPECS:
        raise ReleaseVerificationError(f"unknown release role: {role}")
    entries = set(COMMON_ROLE_ENTRIES)
    if role == "robot":
        entries.update(("install.sh", "systemd"))
    else:
        entries.add("Dockerfile")
    if role == "simulator":
        entries.add("model")
    return frozenset(entries)


def assert_release_entries(release: Path, role: str) -> None:
    expected = expected_release_entries(role)
    actual = frozenset(path.name for path in release.iterdir())
    if actual != expected:
        missing = ", ".join(sorted(expected - actual)) or "<none>"
        unexpected = ", ".join(sorted(actual - expected)) or "<none>"
        raise ReleaseVerificationError(
            f"unexpected release manifest for {role}: "
            f"missing={missing}; unexpected={unexpected}"
        )


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


def assert_robot_wheel_runtime(wheel: Path) -> None:
    """Require the local Unitree bridge implementation and both executables."""
    try:
        with zipfile.ZipFile(wheel) as archive:
            members = frozenset(archive.namelist())
            entrypoint_files = tuple(
                name
                for name in members
                if name.endswith(".dist-info/entry_points.txt")
            )
            if len(entrypoint_files) != 1:
                raise ReleaseVerificationError(
                    f"{wheel.name} has {len(entrypoint_files)} entry_points.txt files"
                )
            entrypoints = archive.read(entrypoint_files[0]).decode("utf-8")
    except ReleaseVerificationError:
        raise
    except (OSError, UnicodeDecodeError, zipfile.BadZipFile) as exc:
        raise ReleaseVerificationError(
            f"cannot inspect Robot runtime wheel {wheel}: {exc}"
        ) from exc

    missing_modules = ROBOT_WHEEL_MODULES - members
    if missing_modules:
        raise ReleaseVerificationError(
            "Robot wheel is missing Unitree bridge modules: "
            + ", ".join(sorted(missing_modules))
        )

    parser = configparser.ConfigParser(interpolation=None)
    parser.optionxform = str
    try:
        parser.read_string(entrypoints)
    except configparser.Error as exc:
        raise ReleaseVerificationError(
            f"invalid Robot wheel entry points: {exc}"
        ) from exc
    console_scripts = (
        dict(parser.items("console_scripts"))
        if parser.has_section("console_scripts")
        else {}
    )
    for name, target in ROBOT_CONSOLE_SCRIPTS.items():
        if console_scripts.get(name, "").strip() != target:
            raise ReleaseVerificationError(
                f"Robot wheel console script {name!r} must target {target!r}"
            )


def assert_robot_systemd_units(systemd: Path) -> None:
    _require_path(systemd, kind="directory")
    actual = frozenset(path.name for path in systemd.iterdir())
    if actual != ROBOT_SYSTEMD_UNITS:
        missing = ", ".join(sorted(ROBOT_SYSTEMD_UNITS - actual)) or "<none>"
        unexpected = ", ".join(sorted(actual - ROBOT_SYSTEMD_UNITS)) or "<none>"
        raise ReleaseVerificationError(
            "unexpected Robot systemd manifest: "
            f"missing={missing}; unexpected={unexpected}"
        )
    for unit in ROBOT_SYSTEMD_UNITS:
        _require_path(systemd / unit)


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
    if role == "robot":
        assert_robot_wheel_runtime(app_wheel)
    return protocol_wheel, app_wheel


def verify_release_layout(release: Path, role: str) -> tuple[Path, Path]:
    if role not in ROLE_SPECS:
        raise ReleaseVerificationError(f"unknown release role: {role}")
    _require_path(release, kind="directory")
    assert_release_entries(release, role)
    _require_path(release / "config", kind="directory")
    _require_path(release / "config/default.yaml")
    _require_path(release / "requirements.lock")
    if role in ("controller", "simulator"):
        _require_path(release / "config/runtime.yaml")
    if role == "controller":
        _require_path(release / "config/arm_model.json")
    if role == "robot":
        _require_path(release / "install.sh")
        assert_robot_systemd_units(release / "systemd")
    else:
        _require_path(release / "Dockerfile")
    if role == "simulator":
        _require_path(release / "model/bundles/default/bundle.json")
    _require_path(release / "interfaces/elesim_interfaces", kind="directory")
    _require_path(release / "interfaces/elesim_interfaces/package.xml")
    _require_path(release / "interfaces/elesim_interfaces/CMakeLists.txt")
    _require_path(release / "interfaces/elesim_interfaces/msg/RgbdFrame.msg")
    _require_path(
        release / "interfaces/elesim_interfaces/srv/OpenSimulationSession.srv"
    )
    return _release_wheels(release, role)


def _probe_source() -> str:
    return r'''
import importlib
import importlib.util
import pathlib
import sys
import sysconfig

target = pathlib.Path(sys.argv[1]).resolve()
release = pathlib.Path(sys.argv[2]).resolve()
role = sys.argv[3]
owned_package = sys.argv[4]
main_module = sys.argv[5]
entrypoint = sys.argv[6]

# The developer container intentionally installs all Elesim projects editable
# into one venv.  ``PYTHONNOUSERSITE`` does not hide that venv site directory,
# so a normal interpreter would report every sibling application as visible
# even though the release target itself is isolated.  ``-S`` skips venv site
# initialization; add only the interpreter's base site directories explicitly
# so third-party runtime dependencies remain available while editable .pth
# files cannot reintroduce source-tree applications.
for key in ("purelib", "platlib"):
    base_site = sysconfig.get_path(key)
    if base_site and base_site not in sys.path:
        sys.path.append(base_site)

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
    "elesim_controller", "elesim_ui", "elesim_robot", "elesim_simulator"
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
                "-S",
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
