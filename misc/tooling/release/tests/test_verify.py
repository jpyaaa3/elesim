from __future__ import annotations

import zipfile
from pathlib import Path

import pytest

from misc.tooling.release.verify import (
    ReleaseVerificationError,
    assert_release_entries,
    assert_robot_systemd_units,
    assert_robot_wheel_runtime,
    assert_wheel_boundary,
    expected_release_entries,
    read_wheel_environment,
    verify_release_layout,
)


def _wheel(path: Path, *members: str) -> Path:
    with zipfile.ZipFile(path, "w") as archive:
        for member in members:
            archive.writestr(member, "")
    return path


def test_wheel_boundary_accepts_only_the_owned_package(tmp_path: Path) -> None:
    wheel = _wheel(
        tmp_path / "controller.whl",
        "elesim_controller/__init__.py",
        "elesim_controller/main.py",
        "elesim_controller-0.3.0.dist-info/METADATA",
    )

    assert_wheel_boundary(wheel, "elesim_controller")


def test_role_release_manifest_declares_only_contractual_shared_material(
    tmp_path: Path,
) -> None:
    release = tmp_path / "controller"
    release.mkdir()
    for name in expected_release_entries("controller"):
        path = release / name
        if "." in name:
            path.write_text("", encoding="utf-8")
        else:
            path.mkdir()

    assert_release_entries(release, "controller")
    (release / "tests").mkdir()

    with pytest.raises(ReleaseVerificationError, match="unexpected=tests"):
        assert_release_entries(release, "controller")


def test_wheel_boundary_rejects_a_sibling_deployment(tmp_path: Path) -> None:
    wheel = _wheel(
        tmp_path / "controller.whl",
        "elesim_controller/__init__.py",
        "elesim_robot/runtime.py",
    )

    with pytest.raises(ReleaseVerificationError, match="elesim_robot"):
        assert_wheel_boundary(wheel, "elesim_controller")


def test_wheel_boundary_rejects_missing_owned_package(tmp_path: Path) -> None:
    wheel = _wheel(tmp_path / "controller.whl", "README.txt")

    with pytest.raises(ReleaseVerificationError, match="elesim_controller"):
        assert_wheel_boundary(wheel, "elesim_controller")


def test_wheel_environment_is_data_not_shell_code(tmp_path: Path) -> None:
    env_file = tmp_path / "WHEELS.env"
    env_file.write_text(
        "PROTOCOL_WHEEL=elesim_protocol-0.3.0.whl\n"
        "APP_WHEEL=elesim_controller-0.3.0.whl\n",
        encoding="utf-8",
    )

    assert read_wheel_environment(env_file) == {
        "PROTOCOL_WHEEL": "elesim_protocol-0.3.0.whl",
        "APP_WHEEL": "elesim_controller-0.3.0.whl",
    }


@pytest.mark.parametrize(
    "content",
    [
        "PROTOCOL_WHEEL=a.whl\nAPP_WHEEL=../../b.whl\n",
        "PROTOCOL_WHEEL=$(touch nope).whl\nAPP_WHEEL=b.whl\n",
        "PROTOCOL_WHEEL=a.whl\nUNKNOWN=b.whl\n",
        "PROTOCOL_WHEEL=a.whl\n",
    ],
)
def test_wheel_environment_rejects_unsafe_or_incomplete_values(
    tmp_path: Path,
    content: str,
) -> None:
    env_file = tmp_path / "WHEELS.env"
    env_file.write_text(content, encoding="utf-8")

    with pytest.raises(ReleaseVerificationError):
        read_wheel_environment(env_file)


def test_controller_release_requires_the_generated_arm_model(tmp_path: Path) -> None:
    release = tmp_path / "controller"
    (release / "config").mkdir(parents=True)
    (release / "wheels").mkdir()
    (release / "interfaces").mkdir()
    for relative in (
        "config/default.yaml",
        "config/runtime.yaml",
        "requirements.lock",
        "Dockerfile",
    ):
        (release / relative).write_text("", encoding="utf-8")
    protocol = _wheel(release / "wheels/protocol.whl", "elesim_protocol/__init__.py")
    application = _wheel(release / "wheels/controller.whl", "elesim_controller/__init__.py")
    (release / "WHEELS.env").write_text(
        f"PROTOCOL_WHEEL={protocol.name}\nAPP_WHEEL={application.name}\n",
        encoding="utf-8",
    )

    with pytest.raises(ReleaseVerificationError, match="arm_model.json"):
        verify_release_layout(release, "controller")


def _robot_wheel(path: Path, *, bridge_entrypoint: bool = True) -> Path:
    scripts = "elesim-robot = elesim_robot.main:main\n"
    if bridge_entrypoint:
        scripts += (
            "elesim-unitree-bridge = "
            "elesim_robot.go2.unitree_bridge_daemon:main\n"
        )
    with zipfile.ZipFile(path, "w") as archive:
        for member in (
            "elesim_robot/__init__.py",
            "elesim_robot/main.py",
            "elesim_robot/go2/unitree_bridge_daemon.py",
            "elesim_robot/go2/unitree_ipc.py",
            "elesim_robot/go2/unitree_ipc_protocol.py",
        ):
            archive.writestr(member, "")
        archive.writestr(
            "elesim_robot-0.3.0.dist-info/entry_points.txt",
            "[console_scripts]\n" + scripts,
        )
    return path


def test_robot_wheel_requires_bridge_modules_and_both_entrypoints(
    tmp_path: Path,
) -> None:
    assert_robot_wheel_runtime(_robot_wheel(tmp_path / "robot.whl"))

    without_entrypoint = _robot_wheel(
        tmp_path / "without-entrypoint.whl", bridge_entrypoint=False
    )
    with pytest.raises(ReleaseVerificationError, match="elesim-unitree-bridge"):
        assert_robot_wheel_runtime(without_entrypoint)


def test_robot_systemd_manifest_is_exactly_two_units(tmp_path: Path) -> None:
    systemd = tmp_path / "systemd"
    systemd.mkdir()
    (systemd / "elesim-robot.service").write_text("unit", encoding="utf-8")

    with pytest.raises(ReleaseVerificationError, match="elesim-unitree-bridge"):
        assert_robot_systemd_units(systemd)

    (systemd / "elesim-unitree-bridge.service").write_text(
        "unit", encoding="utf-8"
    )
    assert_robot_systemd_units(systemd)
