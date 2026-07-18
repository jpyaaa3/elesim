#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DEPLOYMENTS = ("router", "controller", "ui", "robot", "simulator")


def build_wheel(project: Path, wheel_dir: Path) -> Path:
    before = set(wheel_dir.glob("*.whl"))
    try:
        subprocess.run(
            [sys.executable, "-m", "pip", "wheel", "--no-deps", "--wheel-dir", str(wheel_dir), str(project)],
            check=True,
        )
    finally:
        shutil.rmtree(project / "build", ignore_errors=True)
        for metadata in (project / "src").glob("*.egg-info"):
            shutil.rmtree(metadata, ignore_errors=True)
    created = set(wheel_dir.glob("*.whl")) - before
    if len(created) != 1:
        prefix = project.name.replace("-", "_")
        candidates = sorted(wheel_dir.glob(f"*{prefix}*.whl"))
        if not candidates:
            raise RuntimeError(f"could not identify wheel built from {project}")
        return candidates[-1]
    return created.pop()


def copy_tree(source: Path, destination: Path) -> None:
    if source.exists():
        shutil.copytree(source, destination, dirs_exist_ok=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="Build isolated Elesim deployment contexts")
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

    protocol_wheel = build_wheel(ROOT / "packages/protocol", wheel_dir)
    for name in DEPLOYMENTS:
        project = ROOT / "deployments" / name
        app_wheel = build_wheel(project, wheel_dir)
        release = output / name
        wheels = release / "wheels"
        wheels.mkdir(parents=True)
        shutil.copy2(protocol_wheel, wheels / protocol_wheel.name)
        shutil.copy2(app_wheel, wheels / app_wheel.name)
        copy_tree(project / "config", release / "config")
        if (project / "requirements.lock").is_file():
            shutil.copy2(project / "requirements.lock", release / "requirements.lock")
        if (project / "Dockerfile").is_file():
            shutil.copy2(project / "Dockerfile", release / "Dockerfile")
        if name == "robot":
            copy_tree(project / "systemd", release / "systemd")
            shutil.copy2(project / "install.sh", release / "install.sh")
        if name == "simulator":
            copy_tree(ROOT / "model", release / "model")
        (release / "WHEELS.env").write_text(
            f"PROTOCOL_WHEEL={protocol_wheel.name}\nAPP_WHEEL={app_wheel.name}\n",
            encoding="utf-8",
        )
        print(release)


if __name__ == "__main__":
    main()
