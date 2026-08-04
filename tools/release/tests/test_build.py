from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path

import pytest

from tools.release.build import (
    copy_interfaces,
    copy_robot_runtime,
    copy_sim_bundle,
)


ROOT = Path(__file__).resolve().parents[3]


def test_sim_release_contains_only_the_immutable_bundle() -> None:
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        source = root / "model"
        bundle = source / "bundles/default"
        bundle.mkdir(parents=True)
        (bundle / "bundle.json").write_text("{}", encoding="utf-8")
        source_assets = source / "source/assets"
        source_assets.mkdir(parents=True)
        (source_assets / "must-not-ship.obj").write_text("source", encoding="utf-8")
        release = root / "release"

        copy_sim_bundle(source, release)

        assert (release / "model/bundles/default/bundle.json").is_file()
        assert not (release / "model/source").exists()


def test_release_copies_ros_interface_source_without_colcon_artifacts(
    tmp_path: Path,
) -> None:
    source = tmp_path / "elesim_interfaces"
    for relative in ("package.xml", "CMakeLists.txt", "msg/RgbdFrame.msg"):
        path = source / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(relative, encoding="utf-8")
    (source / "build").mkdir()
    (source / "build/generated").write_text("no", encoding="utf-8")
    release = tmp_path / "release"

    copy_interfaces(source, release)

    copied = release / "interfaces/elesim_interfaces"
    assert (copied / "package.xml").is_file()
    assert (copied / "msg/RgbdFrame.msg").is_file()
    assert not (copied / "build").exists()


def test_robot_release_copies_exactly_both_service_units(tmp_path: Path) -> None:
    project = tmp_path / "robot"
    (project / "systemd").mkdir(parents=True)
    (project / "install.sh").write_text("#!/bin/bash\n", encoding="utf-8")
    for name in ("elesim-robot.service", "elesim-unitree-bridge.service"):
        (project / "systemd" / name).write_text(name, encoding="utf-8")
    (project / "systemd/unrelated.service").write_text("no", encoding="utf-8")
    release = tmp_path / "release"
    release.mkdir()

    copy_robot_runtime(project, release)

    assert {path.name for path in (release / "systemd").iterdir()} == {
        "elesim-robot.service",
        "elesim-unitree-bridge.service",
    }


def test_robot_release_refuses_an_incomplete_service_set(tmp_path: Path) -> None:
    project = tmp_path / "robot"
    (project / "systemd").mkdir(parents=True)
    (project / "install.sh").write_text("#!/bin/bash\n", encoding="utf-8")
    (project / "systemd/elesim-robot.service").write_text("unit", encoding="utf-8")

    with pytest.raises(FileNotFoundError, match="elesim-unitree-bridge.service"):
        copy_robot_runtime(project, tmp_path / "release")


def test_robot_install_script_only_reports_privileged_follow_up() -> None:
    script = ROOT / "robot/install.sh"
    text = script.read_text(encoding="utf-8")

    completed = subprocess.run(
        ("bash", "-n", str(script)),
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr
    assert "elesim-unitree-bridge.service" in text
    assert "elesim-robot.service" in text
    assert "venv/bin/elesim-unitree-bridge" in text
    assert "No account, group, /etc configuration, or systemd state was changed." in text
    assert "sudo " not in text
    assert "systemctl " not in text
    assert "useradd " not in text
    assert "groupadd " not in text
