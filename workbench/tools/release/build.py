#!/usr/bin/env python3
from __future__ import annotations

import argparse
import email.parser
import json
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from workbench.tools.release.verify import (
    PUBLIC_CONFIG_TEMPLATES,
    ROBOT_SYSTEMD_UNITS,
    assert_rosidl_source_manifest,
    verify_release_tree,
)


RELEASE_PROJECTS = ("pilot", "ui", "robot", "sim")


def role_runtime(role: str) -> Path:
    if role == "robot":
        return ROOT / "payload/runtime/native/robot"
    return ROOT / "payload/runtime/docker" / role


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
        for metadata in project.glob("*.egg-info"):
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
    for name in ("zed-mini", "d435"):
        source = model_root / name
        if not (source / "bundle.json").is_file():
            raise FileNotFoundError(f"validated sim bundle is missing: {source}")
        copy_tree(source, release / f"data/models/assemblies/{name}")


def copy_role_data(role: str, release: Path) -> None:
    if role in {"pilot", "sim"}:
        copy_tree(ROOT / "payload/data/calibration", release / "data/calibration")
    elif role == "ui":
        copy_tree(
            ROOT / "payload/data/calibration/arm",
            release / "data/calibration/arm",
        )
    if role == "pilot":
        copy_tree(ROOT / "payload/data/models/arm", release / "data/models/arm")
        copy_tree(
            ROOT / "payload/data/models/perception",
            release / "data/models/perception",
        )
        copy_tree(ROOT / "payload/data/policies", release / "data/policies")
    elif role == "sim":
        copy_sim_bundle(ROOT / "payload/data/models/assemblies", release)
        copy_tree(ROOT / "payload/data/models/objects", release / "data/models/objects")


def bind_release_data_paths(release: Path, role: str) -> None:
    if role not in {"pilot", "sim"}:
        return
    config_path = release / "config/config.yaml"
    raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    hand_eye = raw["simulation"]["cameras"]["hand_eye"]
    hand_eye["config"] = "../data/calibration/cameras/zed_mini.hand_eye.json"
    if role == "sim":
        raw["simulation"]["assembly"]["build_dir"] = "../data/models/assemblies/zed-mini"
    config_path.write_text(
        yaml.safe_dump(raw, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    detector = release / "config/perception/detector.yolo.example.json"
    if role == "pilot" and detector.is_file():
        payload = json.loads(detector.read_text(encoding="utf-8"))
        payload["model"] = "../../data/models/perception/yolov8n-seg.pt"
        detector.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def copy_role_config(source: Path, release: Path, role: str) -> None:
    try:
        excluded = PUBLIC_CONFIG_TEMPLATES[role]
    except KeyError as exc:
        raise ValueError(f"unsupported release role: {role}") from exc
    if (source / "config").is_dir():
        source = source / "config"
    source_root = source.resolve()
    destination = release / "config"
    excluded_destination = destination / excluded
    if excluded_destination.is_symlink() or excluded_destination.is_file():
        excluded_destination.unlink()
    elif excluded_destination.exists():
        raise ValueError(
            f"public config template destination must not be a directory: {excluded_destination}"
        )

    def ignore(directory: str, names: list[str]) -> set[str]:
        if Path(directory).resolve() == source_root and excluded in names:
            return {excluded}
        return set()

    copy_tree(source, destination, ignore=ignore)


def copy_interfaces(source: Path, release: Path) -> None:
    required = (source / "package.xml", source / "CMakeLists.txt")
    missing = [path for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError(
            "ROS interface package is incomplete: "
            + ", ".join(str(path) for path in missing)
        )
    assert_rosidl_source_manifest(source)
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


def copy_infrastructure(repository: Path, release_root: Path) -> None:
    destination = release_root / "infra"
    destination.mkdir(parents=True, exist_ok=True)
    docker_payload = repository / "payload/runtime/docker"
    copy_tree(docker_payload / "development", destination / "development")
    containers = destination / "containers"
    containers.mkdir(parents=True, exist_ok=True)
    shutil.copy2(docker_payload / "shared/Dockerfile.app", containers / "Dockerfile.app")
    shutil.copy2(docker_payload / "shared/robotpkg.asc", containers / "robotpkg.asc")
    shutil.copy2(docker_payload / "tools/Dockerfile", containers / "Dockerfile.tools")
    shutil.copy2(docker_payload / "tools/tools-entrypoint", containers / "tools-entrypoint")
    shutil.copy2(docker_payload / "README.md", containers / "README.md")
    setup_destination = destination / "setup"
    setup_destination.mkdir(parents=True, exist_ok=True)
    shutil.copy2(
        repository / "installer/bootstrap/bootstrap.py",
        setup_destination / "bootstrap.py",
    )
    shutil.copy2(
        repository / "installer/bootstrap/install.sh",
        setup_destination / "install.sh",
    )
    shutil.copy2(
        repository / "installer/bootstrap/bootstrap-contract.json",
        setup_destination / "bootstrap-contract.json",
    )
    setup_project = repository / "payload/runtime/docker/tools/app"
    package_destination = setup_destination / "package"
    if package_destination.exists():
        shutil.rmtree(package_destination)
    copy_tree(
        setup_project,
        package_destination,
        ignore=shutil.ignore_patterns(
            "__pycache__",
            "*.pyc",
            "*.egg-info",
            "build",
            "dist",
        ),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Build isolated EleSim release contexts")
    parser.add_argument("--output", default="dist/releases")
    args = parser.parse_args()
    output = (ROOT / args.output).resolve()
    wheel_dir = output.parent / "wheels"
    if output.exists():
        shutil.rmtree(output)
    if wheel_dir.exists():
        shutil.rmtree(wheel_dir)
    output.mkdir(parents=True)
    wheel_dir.mkdir(parents=True)

    protocol_wheel = build_wheel(ROOT / "payload/runtime/common/protocol", wheel_dir)
    for role in RELEASE_PROJECTS:
        runtime = role_runtime(role)
        project = runtime / "app"
        app_wheel = build_wheel(project, wheel_dir)
        release = output / role
        wheels = release / "wheels"
        wheels.mkdir(parents=True)
        shutil.copy2(protocol_wheel, wheels / protocol_wheel.name)
        shutil.copy2(app_wheel, wheels / app_wheel.name)
        copy_role_config(ROOT / "payload/config" / role, release, role)
        copy_role_data(role, release)
        bind_release_data_paths(release, role)
        if (runtime / "requirements.lock").is_file():
            shutil.copy2(runtime / "requirements.lock", release / "requirements.lock")
        if (runtime / "Dockerfile.release").is_file():
            shutil.copy2(runtime / "Dockerfile.release", release / "Dockerfile")
        if role == "robot":
            copy_robot_runtime(runtime, release)
        copy_interfaces(ROOT / "payload/runtime/common/elesim_interfaces", release)
        (release / "WHEELS.env").write_text(
            f"PROTOCOL_WHEEL={protocol_wheel.name}\nAPP_WHEEL={app_wheel.name}\n",
            encoding="utf-8",
        )
        print(release)

    copy_infrastructure(ROOT, output)
    verify_release_tree(output)


if __name__ == "__main__":
    main()
