#!/usr/bin/env python3
"""Verify generated deployment contexts without relying on the source tree."""

from __future__ import annotations

import argparse
import configparser
import os
import re
import stat
import subprocess
import sys
import tempfile
import zipfile
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Mapping, Sequence


@dataclass(frozen=True)
class RoleSpec:
    package: str
    main_module: str
    entrypoint: str


ROLE_SPECS: Mapping[str, RoleSpec] = {
    "pilot": RoleSpec(
        "elesim_pilot", "elesim_pilot.main", "elesim-pilot"
    ),
    "ui": RoleSpec("elesim_ui", "elesim_ui.main", "elesim-ui"),
    "robot": RoleSpec("elesim_robot", "elesim_robot.main", "elesim-robot"),
    "sim": RoleSpec(
        "elesim_sim", "elesim_sim.main", "elesim-sim"
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
SOURCE_ONLY_WHEEL_COMPONENTS = frozenset(
    ("tests", "fixtures", "__pycache__", ".pytest_cache")
)
EXPECTED_INFRA_FILES = {
    "containers": frozenset(
        (
            "Dockerfile.app",
            "Dockerfile.tools",
            "README.md",
            "robotpkg.asc",
            "tools-entrypoint",
        )
    ),
    "development": frozenset(
        (
            "Dockerfile",
            "README.md",
            "requirements.lock",
            "entrypoint.sh",
            "dev-env.sh",
        )
    ),
}
SETUP_PYTHON_MODULES = frozenset(
    f"src/elesim_setup/{name}.py"
    for name in (
        "__init__",
        "_security_storage",
        "capabilities",
        "cli",
        "configuration",
        "connection_gui",
        "connection_manager",
        "connections",
        "container_installer",
        "credentials",
        "developer",
        "doctor",
        "gui",
        "host_helper",
        "host_proxy",
        "installer",
        "manager_lifecycle",
        "network",
        "ownership",
        "profiles",
        "request",
        "secure_deployment",
        "security_authority",
        "security_policy",
        "security_provisioning",
        "security_views",
        "service",
        "shell",
        "state",
        "uninstall",
        "updater",
    )
)
REQUIRED_SETUP_PACKAGE_FILES = (
    *sorted(SETUP_PYTHON_MODULES),
    "src/elesim_setup/web/index.html",
    "src/elesim_setup/web/app.js",
    "src/elesim_setup/web/style.css",
    "src/elesim_setup/web/i18n.json",
    "src/elesim_setup/web/icon.svg",
    "src/elesim_setup/web/fonts/NotoSansCJKkr-Regular.otf",
    "src/elesim_setup/connection_web/index.html",
    "src/elesim_setup/connection_web/app.js",
    "src/elesim_setup/connection_web/style.css",
    "src/elesim_setup/connection_web/i18n.json",
    "src/elesim_setup/connection_web/icon.svg",
)
PUBLIC_CONFIG_TEMPLATES = {
    "pilot": "runtime.public.example.yaml",
    "sim": "runtime.public.example.yaml",
    "ui": "public.example.yaml",
    "robot": "public.example.yaml",
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
    if role == "sim":
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


def _wheel_members(wheel: Path) -> tuple[zipfile.ZipInfo, ...]:
    try:
        with zipfile.ZipFile(wheel) as archive:
            return tuple(archive.infolist())
    except (OSError, zipfile.BadZipFile) as exc:
        raise ReleaseVerificationError(f"invalid wheel archive {wheel}: {exc}") from exc


def assert_wheel_boundary(wheel: Path, expected_package: str) -> None:
    entries = _wheel_members(wheel)
    names = tuple(entry.filename for entry in entries)
    invalid: list[str] = []
    seen: set[str] = set()
    for entry in entries:
        name = entry.filename
        path = PurePosixPath(name)
        unix_mode = (entry.external_attr >> 16) & 0xFFFF
        file_type = stat.S_IFMT(unix_mode)
        allowed_file_types = (
            (0, stat.S_IFDIR)
            if entry.is_dir()
            else (0, stat.S_IFREG)
        )
        normalized_name = path.as_posix() + ("/" if entry.is_dir() else "")
        if (
            not name
            or "\\" in name
            or "\x00" in name
            or path.is_absolute()
            or not path.parts
            or ".." in path.parts
            or normalized_name != name
            or name.rstrip("/") in seen
            or file_type not in allowed_file_types
        ):
            invalid.append(name or "<empty>")
        seen.add(name.rstrip("/"))
    if invalid:
        raise ReleaseVerificationError(
            f"{wheel.name} contains unsafe wheel members: "
            + ", ".join(sorted(invalid)[:5])
        )
    source_only = sorted(
        name
        for name in names
        if name.endswith(".pyc")
        or SOURCE_ONLY_WHEEL_COMPONENTS.intersection(PurePosixPath(name).parts)
    )
    if source_only:
        rendered = ", ".join(source_only[:5])
        raise ReleaseVerificationError(
            f"{wheel.name} contains source-only wheel members: {rendered}"
        )
    roots = {PurePosixPath(name).parts[0] for name in names}
    metadata_roots = {
        root
        for root in roots
        if root.startswith(f"{expected_package}-")
        and root.endswith(".dist-info")
    }
    unexpected = roots - {expected_package} - metadata_roots
    dist_info = {root for root in metadata_roots if root.endswith(".dist-info")}
    if unexpected or expected_package not in roots or len(dist_info) != 1:
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
    for ancestor in path.parents:
        try:
            ancestor_mode = ancestor.lstat().st_mode
        except OSError as exc:
            raise ReleaseVerificationError(
                f"missing release path ancestor: {ancestor}"
            ) from exc
        if stat.S_ISLNK(ancestor_mode):
            raise ReleaseVerificationError(
                f"release path ancestor must not be a symlink: {ancestor}"
            )
    try:
        mode = path.lstat().st_mode
    except OSError as exc:
        raise ReleaseVerificationError(f"missing release {kind}: {path}") from exc
    if stat.S_ISLNK(mode):
        raise ReleaseVerificationError(f"release path must not be a symlink: {path}")
    exists = stat.S_ISDIR(mode) if kind == "directory" else stat.S_ISREG(mode)
    if not exists:
        raise ReleaseVerificationError(f"missing release {kind}: {path}")


def _assert_regular_tree(root: Path) -> None:
    """Reject links and special files anywhere below a generated boundary."""

    _require_path(root, kind="directory")
    for current, directories, files in os.walk(root, followlinks=False):
        for name, expected_kind in (
            *((name, "directory") for name in directories),
            *((name, "file") for name in files),
        ):
            _require_path(Path(current) / name, kind=expected_kind)


_ROSIDL_SOURCE_RE = re.compile(
    r'"((?:msg/[A-Za-z][A-Za-z0-9_]*\.msg|'
    r'srv/[A-Za-z][A-Za-z0-9_]*\.srv|'
    r'action/[A-Za-z][A-Za-z0-9_]*\.action))"'
)


def assert_rosidl_source_manifest(interface_root: Path) -> None:
    """Require every ROSIDL source declared by CMake and no undeclared source."""

    cmake = interface_root / "CMakeLists.txt"
    _require_path(cmake)
    try:
        declared_values = _ROSIDL_SOURCE_RE.findall(cmake.read_text(encoding="utf-8"))
    except (OSError, UnicodeError) as exc:
        raise ReleaseVerificationError(f"cannot read ROSIDL manifest: {cmake}") from exc
    declared = frozenset(declared_values)
    if not declared or len(declared) != len(declared_values):
        raise ReleaseVerificationError(
            f"ROSIDL CMake manifest is empty or contains duplicates: {cmake}"
        )
    actual: set[str] = set()
    for directory, suffix in (("msg", ".msg"), ("srv", ".srv"), ("action", ".action")):
        source_dir = interface_root / directory
        _require_path(source_dir, kind="directory")
        for source in source_dir.iterdir():
            _require_path(source)
            if source.suffix != suffix:
                raise ReleaseVerificationError(
                    f"unexpected ROSIDL source member: {source}"
                )
            actual.add(source.relative_to(interface_root).as_posix())
    if actual != declared:
        missing = sorted(declared - actual)
        unexpected = sorted(actual - declared)
        raise ReleaseVerificationError(
            "ROSIDL source manifest mismatch: "
            f"missing={missing!r}; unexpected={unexpected!r}"
        )


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
    _assert_regular_tree(release)
    assert_release_entries(release, role)
    _require_path(release / "config", kind="directory")
    _require_path(release / "config/default.yaml")
    public_template = release / "config" / PUBLIC_CONFIG_TEMPLATES[role]
    if os.path.lexists(public_template):
        raise ReleaseVerificationError(
            f"source-only public config template is present: {public_template}"
        )
    _require_path(release / "requirements.lock")
    if role in ("pilot", "sim"):
        _require_path(release / "config/runtime.yaml")
    if role == "pilot":
        _require_path(release / "config/arm_model.json")
    if role == "robot":
        _require_path(release / "install.sh")
        assert_robot_systemd_units(release / "systemd")
    else:
        _require_path(release / "Dockerfile")
    if role == "sim":
        _require_path(release / "model/bundles/default/bundle.json")
    interfaces = release / "interfaces/elesim_interfaces"
    _require_path(interfaces, kind="directory")
    _require_path(interfaces / "package.xml")
    assert_rosidl_source_manifest(interfaces)
    return _release_wheels(release, role)


def verify_infrastructure_layout(release_root: Path) -> None:
    """Verify the exact setup source boundary shipped beside role releases."""

    infra = release_root / "infra"
    _assert_regular_tree(infra)
    expected_infra = {"containers", "development", "setup"}
    actual_infra = {path.name for path in infra.iterdir()}
    if actual_infra != expected_infra:
        raise ReleaseVerificationError(
            "unexpected infra manifest: "
            f"missing={sorted(expected_infra - actual_infra)!r}; "
            f"unexpected={sorted(actual_infra - expected_infra)!r}"
        )
    for directory, expected_files in EXPECTED_INFRA_FILES.items():
        root = infra / directory
        _require_path(root, kind="directory")
        actual_files = frozenset(path.name for path in root.iterdir())
        if actual_files != expected_files:
            raise ReleaseVerificationError(
                f"unexpected infra {directory} manifest: "
                f"missing={sorted(expected_files - actual_files)!r}; "
                f"unexpected={sorted(actual_files - expected_files)!r}"
            )
        for name in expected_files:
            _require_path(root / name)
    setup = infra / "setup"
    _require_path(setup, kind="directory")
    expected_setup = {
        "bootstrap.py",
        "install.sh",
        "bootstrap-contract.json",
        "package",
    }
    actual_setup = {path.name for path in setup.iterdir()}
    if actual_setup != expected_setup:
        raise ReleaseVerificationError(
            "unexpected infra setup manifest: "
            f"missing={sorted(expected_setup - actual_setup)!r}; "
            f"unexpected={sorted(actual_setup - expected_setup)!r}"
        )
    for name in ("bootstrap.py", "install.sh", "bootstrap-contract.json"):
        _require_path(setup / name)
    package = setup / "package"
    _require_path(package, kind="directory")
    expected_package = {"pyproject.toml", "requirements.lock", "src"}
    actual_package = {path.name for path in package.iterdir()}
    if actual_package != expected_package:
        raise ReleaseVerificationError(
            "unexpected infra setup package manifest: "
            f"missing={sorted(expected_package - actual_package)!r}; "
            f"unexpected={sorted(actual_package - expected_package)!r}"
        )
    _require_path(package / "pyproject.toml")
    _require_path(package / "requirements.lock")
    _require_path(package / "src", kind="directory")
    _require_path(package / "src/elesim_setup", kind="directory")
    actual_python = frozenset(
        path.relative_to(package).as_posix()
        for path in (package / "src/elesim_setup").rglob("*.py")
    )
    if actual_python != SETUP_PYTHON_MODULES:
        raise ReleaseVerificationError(
            "unexpected setup Python module manifest: "
            f"missing={sorted(SETUP_PYTHON_MODULES - actual_python)!r}; "
            f"unexpected={sorted(actual_python - SETUP_PYTHON_MODULES)!r}"
        )
    for relative in REQUIRED_SETUP_PACKAGE_FILES:
        _require_path(package / relative)
    source_only = sorted(
        str(path.relative_to(package))
        for path in package.rglob("*")
        if path.name.endswith(".pyc")
        or SOURCE_ONLY_WHEEL_COMPONENTS.intersection(path.relative_to(package).parts)
        or path.name.endswith(".egg-info")
    )
    if source_only:
        raise ReleaseVerificationError(
            "infra setup package contains source-only members: "
            + ", ".join(source_only[:5])
        )


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

# The developer container intentionally installs all EleSim projects editable
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
    "elesim_pilot", "elesim_ui", "elesim_robot", "elesim_sim"
} - {owned_package}
visible = sorted(name for name in siblings if importlib.util.find_spec(name) is not None)
if visible:
    raise AssertionError(f"sibling applications visible in isolated install: {visible}")

config = release / "config/default.yaml"
if role == "pilot":
    from elesim_pilot.config import load_app_config, load_runtime_role_config
    from elesim_pilot.robot.arm.iklib.solver import load_solver_context
    app_config = load_app_config(str(config))
    load_runtime_role_config(str(release / "config/runtime.yaml"))
    _model_config, model_context = load_solver_context(str(config))
    if "limit" not in model_context:
        raise AssertionError("pilot arm model has no joint limits")
    detector_config = app_config.perception_config.resolved_detector_config_path()
    if app_config.perception_config.detector_config and not detector_config.is_file():
        raise AssertionError(f"pilot detector config is missing: {detector_config}")
elif role == "ui":
    from elesim_ui.config import load_config
    load_config(str(config))
elif role == "robot":
    from elesim_robot.config import load_config
    load_config(str(config))
elif role == "sim":
    from elesim_sim.config import load_app_config, load_runtime_role_config
    from elesim_sim.model_bundle import validate_model_bundle
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
    release_path = Path(release).expanduser().absolute()
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
        probe_env = _probe_environment(clean_env, target)
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


def _probe_environment(
    clean_env: Mapping[str, str], target: Path
) -> dict[str, str]:
    result = dict(clean_env)
    result["PYTHONPATH"] = str(target)
    # Verification must be read-only with respect to the release tree. The
    # persistent developer environment enables tracing globally, while
    # Pilot/Sim resolve a relative trace directory from their working tree.
    result["ELESIM_TRACE"] = "0"
    return result


def verify_release_tree(
    release_root: str | os.PathLike[str],
    *,
    python: str = sys.executable,
) -> None:
    root = Path(release_root).expanduser().absolute()
    _assert_regular_tree(root)
    expected = {*ROLE_SPECS, "infra"}
    actual = {path.name for path in root.iterdir()}
    if actual != expected:
        raise ReleaseVerificationError(
            "unexpected release root manifest: "
            f"missing={sorted(expected - actual)!r}; "
            f"unexpected={sorted(actual - expected)!r}"
        )
    verify_infrastructure_layout(root)
    for role in ROLE_SPECS:
        release = root / role
        verify_release_context(release, role, python=python)
        print(f"verified isolated release: {release}")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("release_root", nargs="?", default="dist/releases")
    parser.add_argument("--role", choices=tuple(ROLE_SPECS))
    args = parser.parse_args(argv)
    root = Path(args.release_root).expanduser().absolute()
    if args.role:
        verify_release_context(root / args.role, args.role)
    else:
        verify_release_tree(root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
