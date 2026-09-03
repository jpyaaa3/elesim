from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path

import pytest

from misc.tools.release.build import (
    copy_interfaces,
    copy_robot_runtime,
    copy_role_data,
    copy_role_config,
    copy_sim_bundle,
)
from misc.tools.release.verify import ReleaseVerificationError


ROOT = Path(__file__).resolve().parents[4]


@pytest.mark.parametrize(
    ("role", "template"),
    (
        ("pilot", "runtime.public.example.yaml"),
        ("sim", "runtime.public.example.yaml"),
        ("ui", "public.example.yaml"),
        ("robot", "public.example.yaml"),
    ),
)
def test_release_config_excludes_only_public_template(
    tmp_path: Path,
    role: str,
    template: str,
) -> None:
    project = tmp_path / role
    config = project / "config"
    perception = config / "perception"
    perception.mkdir(parents=True)
    config_name = "config.yaml" if role in ("pilot", "sim") else "default.yaml"
    (config / config_name).write_text("runtime: true\n", encoding="utf-8")
    (config / template).write_text("public: true\n", encoding="utf-8")
    yolo = perception / "detector.yolo.example.json"
    yolo.write_text("{}\n", encoding="utf-8")
    release = tmp_path / "release"
    stale_template = release / "config" / template
    stale_template.parent.mkdir(parents=True)
    stale_template.write_text("stale: true\n", encoding="utf-8")

    copy_role_config(project, release, role)

    assert (release / "config" / config_name).is_file()
    assert not (release / "config" / template).exists()
    assert (release / "config/perception/detector.yolo.example.json").is_file()


def test_sim_release_keeps_the_validated_mock_object_catalog(tmp_path: Path) -> None:
    release = tmp_path / "release"

    copy_role_data("sim", release)

    assert (release / "data/models/objects/demo_box.obj").read_bytes() == (
        ROOT / "payload/data/models/objects/demo_box.obj"
    ).read_bytes()


def test_sim_release_contains_only_the_immutable_bundle() -> None:
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        source = root / "model"
        bundle = source / "zed-mini"
        bundle.mkdir(parents=True)
        (bundle / "bundle.json").write_text("{}", encoding="utf-8")
        d435 = source / "d435"
        d435.mkdir(parents=True)
        (d435 / "bundle.json").write_text("{}", encoding="utf-8")
        extra_assets = source / "extras/assets"
        extra_assets.mkdir(parents=True)
        (extra_assets / "must-not-ship.obj").write_text("extra", encoding="utf-8")
        release = root / "release"

        copy_sim_bundle(source, release)

        assert (release / "data/models/assemblies/zed-mini/bundle.json").is_file()
        assert (release / "data/models/assemblies/d435/bundle.json").is_file()
        assert not (release / "model/extras").exists()


def test_release_copies_ros_interface_source_without_colcon_artifacts(
    tmp_path: Path,
) -> None:
    source = tmp_path / "elesim_interfaces"
    for relative in (
        "package.xml",
        "msg/RgbdFrame.msg",
        "srv/Extra.srv",
        "action/Extra.action",
    ):
        path = source / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(relative, encoding="utf-8")
    (source / "CMakeLists.txt").write_text(
        "rosidl_generate_interfaces(${PROJECT_NAME}\n"
        '  "msg/RgbdFrame.msg"\n'
        '  "srv/Extra.srv"\n'
        '  "action/Extra.action"\n'
        ")\n",
        encoding="utf-8",
    )
    (source / "build").mkdir()
    (source / "build/generated").write_text("no", encoding="utf-8")
    release = tmp_path / "release"

    copy_interfaces(source, release)

    copied = release / "interfaces/elesim_interfaces"
    assert (copied / "package.xml").is_file()
    assert (copied / "msg/RgbdFrame.msg").is_file()
    assert not (copied / "build").exists()


@pytest.mark.parametrize(
    "missing",
    ("msg/RgbdFrame.msg", "srv/Extra.srv", "action/Extra.action"),
)
def test_release_refuses_missing_cmake_declared_rosidl_source(
    tmp_path: Path,
    missing: str,
) -> None:
    source = tmp_path / "elesim_interfaces"
    (source / "package.xml").parent.mkdir(parents=True)
    (source / "package.xml").write_text("", encoding="utf-8")
    (source / "CMakeLists.txt").write_text(
        "rosidl_generate_interfaces(${PROJECT_NAME}\n"
        '  "msg/RgbdFrame.msg"\n'
        '  "srv/Extra.srv"\n'
        '  "action/Extra.action"\n'
        ")\n",
        encoding="utf-8",
    )
    for relative in ("msg/RgbdFrame.msg", "srv/Extra.srv", "action/Extra.action"):
        path = source / relative
        path.parent.mkdir(exist_ok=True)
        path.write_text("", encoding="utf-8")
    (source / missing).unlink()

    with pytest.raises(ReleaseVerificationError, match=Path(missing).name):
        copy_interfaces(source, tmp_path / "release")


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
    script = ROOT / "payload/runtime/native/robot/install.sh"
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
