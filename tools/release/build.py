#!/usr/bin/env python3
from __future__ import annotations

import argparse
import email.parser
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.release.verify import ROBOT_SYSTEMD_UNITS, verify_release_tree


RELEASE_PROJECTS = ("pilot", "ui", "robot", "sim")


def build_wheel(project: Path, wheel_dir: Path) -> Path:
    before = set(wheel_dir.glob("*.whl"))
    try:
        subprocess.run(
            [
                sys.executable,
                "-m",
                "pip",
                "wheel",
                "--no-build-isolation",
                "--no-deps",
                "--wheel-dir",
                str(wheel_dir),
                str(project),
            ],
            check=True,
        )
    finally:
        shutil.rmtree(project / "build", ignore_errors=True)
        for parent in (project, project / "src"):
            for metadata in parent.glob("*.egg-info"):
                shutil.rmtree(metadata, ignore_errors=True)
    created = set(wheel_dir.glob("*.whl")) - before
    if len(created) != 1:
        prefix = project.name.replace("-", "_")
        candidates = sorted(wheel_dir.glob(f"*{prefix}*.whl"))
        if not candidates:
            raise RuntimeError(f"could not identify wheel built from {project}")
        wheel = candidates[-1]
    else:
        wheel = created.pop()
    _validate_wheel_metadata(wheel, project)
    return wheel


def _validate_wheel_metadata(wheel: Path, project: Path) -> None:
    try:
        with zipfile.ZipFile(wheel) as archive:
            metadata_files = [
                name
                for name in archive.namelist()
                if name.endswith(".dist-info/METADATA")
            ]
            if len(metadata_files) != 1:
                raise RuntimeError(
                    f"wheel from {project} has {len(metadata_files)} METADATA files"
                )
            metadata = email.parser.BytesParser().parsebytes(
                archive.read(metadata_files[0])
            )
    except zipfile.BadZipFile as exc:
        raise RuntimeError(f"invalid wheel built from {project}: {wheel}") from exc
    name = str(metadata.get("Name", "")).strip()
    version = str(metadata.get("Version", "")).strip()
    if not name or name.upper() == "UNKNOWN" or not version or version == "0.0.0":
        raise RuntimeError(
            f"build backend did not read project metadata for {project}: "
            f"name={name!r} version={version!r}"
        )


def copy_tree(source: Path, destination: Path, *, ignore=None) -> None:
    if source.exists():
        shutil.copytree(source, destination, dirs_exist_ok=True, ignore=ignore)


def copy_sim_bundle(model_root: Path, release: Path) -> None:
    source = model_root / "bundles/default"
    if not (source / "bundle.json").is_file():
        raise FileNotFoundError(f"validated sim bundle is missing: {source}")
    copy_tree(source, release / "model/bundles/default")


def copy_interfaces(source: Path, release: Path) -> None:
    required = (
        source / "package.xml",
        source / "CMakeLists.txt",
        source / "msg/RgbdFrame.msg",
    )
    missing = [path for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError(
            "ROS interface package is incomplete: "
            + ", ".join(str(path) for path in missing)
        )
    copy_tree(
        source,
        release / "interfaces/elesim_interfaces",
        ignore=shutil.ignore_patterns(
            "__pycache__",
            "*.pyc",
            ".pytest_cache",
            "build",
            "install",
            "log",
        ),
    )


def copy_robot_runtime(project: Path, release: Path) -> None:
    """Copy the complete, exact standalone Robot service surface."""
    install_script = project / "install.sh"
    if not install_script.is_file():
        raise FileNotFoundError(f"Robot install script is missing: {install_script}")
    units = project / "systemd"
    missing = [
        units / name
        for name in ROBOT_SYSTEMD_UNITS
        if not (units / name).is_file()
    ]
    if missing:
        raise FileNotFoundError(
            "Robot systemd service set is incomplete: "
            + ", ".join(str(path) for path in missing)
        )
    shutil.copy2(install_script, release / "install.sh")
    destination = release / "systemd"
    destination.mkdir(parents=True, exist_ok=True)
    for name in sorted(ROBOT_SYSTEMD_UNITS):
        shutil.copy2(units / name, destination / name)


def copy_infrastructure(source: Path, release_root: Path) -> None:
    destination = release_root / "infra"
    destination.mkdir(parents=True, exist_ok=True)
    copy_tree(source / "development", destination / "development")
    setup_destination = destination / "setup"
    setup_destination.mkdir(parents=True, exist_ok=True)
    shutil.copy2(
        source.parent / "installer/bootstrap/bootstrap.py",
        setup_destination / "bootstrap.py",
    )
    shutil.copy2(
        source.parent / "installer/bootstrap/bootstrap.sh",
        setup_destination / "bootstrap.sh",
    )
    shutil.copy2(
        source.parent / "installer/bootstrap/bootstrap-contract.json",
        setup_destination / "bootstrap-contract.json",
    )
    copy_tree(source / "containers", destination / "containers")
    copy_tree(
        source.parent / "installer/package",
        setup_destination / "package",
        ignore=shutil.ignore_patterns(
            "__pycache__",
            "*.pyc",
            ".pytest_cache",
            "build",
            "*.egg-info",
        ),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Build isolated Elesim release contexts")
    parser.add_argument("--output", default="dist/releases")
    parser.add_argument(
        "--no-verify",
        action="store_true",
        help="skip isolated wheel install and entrypoint probes",
    )
    args = parser.parse_args()
    output = (ROOT / args.output).resolve()
    wheel_dir = output.parent / "wheels"
    if output.exists():
        shutil.rmtree(output)
    if wheel_dir.exists():
        shutil.rmtree(wheel_dir)
    output.mkdir(parents=True)
    wheel_dir.mkdir(parents=True)

    protocol_wheel = build_wheel(ROOT / "packages/protocol", wheel_dir)
    for role in RELEASE_PROJECTS:
        project = ROOT / role
        app_wheel = build_wheel(project, wheel_dir)
        release = output / role
        wheels = release / "wheels"
        wheels.mkdir(parents=True)
        shutil.copy2(protocol_wheel, wheels / protocol_wheel.name)
        shutil.copy2(app_wheel, wheels / app_wheel.name)
        copy_tree(project / "config", release / "config")
        if (project / "requirements.lock").is_file():
            shutil.copy2(project / "requirements.lock", release / "requirements.lock")
        if (project / "Dockerfile").is_file():
            shutil.copy2(project / "Dockerfile", release / "Dockerfile")
        if role == "robot":
            copy_robot_runtime(project, release)
        if role == "sim":
            copy_sim_bundle(ROOT / "model", release)
        copy_interfaces(ROOT / "packages/elesim_interfaces", release)
        (release / "WHEELS.env").write_text(
            f"PROTOCOL_WHEEL={protocol_wheel.name}\nAPP_WHEEL={app_wheel.name}\n",
            encoding="utf-8",
        )
        print(release)

    copy_infrastructure(ROOT / "environment", output)

    if not args.no_verify:
        verify_release_tree(output)


if __name__ == "__main__":
    main()
