from __future__ import annotations

import zipfile
import stat
from pathlib import Path

import pytest

from misc.tools.release.verify import (
    EXPECTED_INFRA_FILES,
    REQUIRED_SETUP_PACKAGE_FILES,
    _probe_environment,
    ReleaseVerificationError,
    assert_release_entries,
    assert_rosidl_source_manifest,
    assert_robot_systemd_units,
    assert_robot_wheel_runtime,
    assert_wheel_boundary,
    expected_release_entries,
    read_wheel_environment,
    verify_infrastructure_layout,
    verify_release_layout,
)


def _wheel(path: Path, *members: str) -> Path:
    with zipfile.ZipFile(path, "w") as archive:
        for member in members:
            archive.writestr(member, "")
    return path


def _minimal_infrastructure(root: Path) -> Path:
    infra = root / "infra"
    for directory, files in EXPECTED_INFRA_FILES.items():
        target = infra / directory
        target.mkdir(parents=True)
        for name in files:
            (target / name).write_text("", encoding="utf-8")
    setup = infra / "setup"
    setup.mkdir()
    for name in ("bootstrap.py", "install.sh", "bootstrap-contract.json"):
        (setup / name).write_text("", encoding="utf-8")
    package = setup / "package"
    (package / "src").mkdir(parents=True)
    (package / "pyproject.toml").write_text("", encoding="utf-8")
    (package / "requirements.lock").write_text("", encoding="utf-8")
    for relative in REQUIRED_SETUP_PACKAGE_FILES:
        path = package / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("", encoding="utf-8")
    return package


def test_wheel_boundary_accepts_only_the_owned_package(tmp_path: Path) -> None:
    wheel = _wheel(
        tmp_path / "pilot.whl",
        "elesim_pilot/__init__.py",
        "elesim_pilot/main.py",
        "elesim_pilot-0.3.0.dist-info/METADATA",
    )

    assert_wheel_boundary(wheel, "elesim_pilot")


@pytest.mark.parametrize(
    "member",
    (
        "foreign_runtime/__init__.py",
        "../escape.py",
        "/absolute.py",
        "a//b.py",
        ".",
        "./",
    ),
)
def test_wheel_boundary_rejects_foreign_or_unsafe_members(
    tmp_path: Path, member: str
) -> None:
    wheel = _wheel(
        tmp_path / "pilot.whl",
        "elesim_pilot/__init__.py",
        "elesim_pilot-0.3.0.dist-info/METADATA",
        member,
    )

    with pytest.raises(ReleaseVerificationError):
        assert_wheel_boundary(wheel, "elesim_pilot")


def test_wheel_boundary_rejects_symlink_members(tmp_path: Path) -> None:
    wheel = tmp_path / "pilot.whl"
    with zipfile.ZipFile(wheel, "w") as archive:
        archive.writestr("elesim_pilot/__init__.py", "")
        archive.writestr("elesim_pilot-0.3.0.dist-info/METADATA", "")
        link = zipfile.ZipInfo("elesim_pilot/config_link")
        link.create_system = 3
        link.external_attr = (stat.S_IFLNK | 0o777) << 16
        archive.writestr(link, "../../outside")

    with pytest.raises(ReleaseVerificationError, match="unsafe wheel members"):
        assert_wheel_boundary(wheel, "elesim_pilot")


def test_wheel_boundary_rejects_directory_named_symlink_member(tmp_path: Path) -> None:
    wheel = tmp_path / "pilot.whl"
    with zipfile.ZipFile(wheel, "w") as archive:
        archive.writestr("elesim_pilot/__init__.py", "")
        archive.writestr("elesim_pilot-0.3.0.dist-info/METADATA", "")
        link = zipfile.ZipInfo("elesim_pilot/config_link/")
        link.create_system = 3
        link.external_attr = (stat.S_IFLNK | 0o777) << 16
        archive.writestr(link, "")

    with pytest.raises(ReleaseVerificationError, match="unsafe wheel members"):
        assert_wheel_boundary(wheel, "elesim_pilot")


def test_wheel_boundary_rejects_data_purelib_package_escape(tmp_path: Path) -> None:
    wheel = _wheel(
        tmp_path / "pilot.whl",
        "elesim_pilot/__init__.py",
        "elesim_pilot-0.3.0.dist-info/METADATA",
        "elesim_pilot-0.3.0.data/purelib/elesim_robot/__init__.py",
    )

    with pytest.raises(ReleaseVerificationError, match="only elesim_pilot"):
        assert_wheel_boundary(wheel, "elesim_pilot")


@pytest.mark.parametrize(
    "source_only_member",
    (
        "tests/test_runtime.py",
        "elesim_pilot/fixtures/runtime.json",
        "elesim_pilot/__pycache__/main.cpython-310.pyc",
        ".pytest_cache/CACHEDIR.TAG",
        "elesim_pilot/cache/state.pyc",
    ),
)
def test_wheel_boundary_rejects_source_only_members(
    tmp_path: Path,
    source_only_member: str,
) -> None:
    wheel = _wheel(
        tmp_path / "pilot.whl",
        "elesim_pilot/__init__.py",
        "elesim_pilot-0.3.0.dist-info/METADATA",
        source_only_member,
    )

    with pytest.raises(ReleaseVerificationError, match="source-only wheel members"):
        assert_wheel_boundary(wheel, "elesim_pilot")


def test_release_probe_disables_inherited_runtime_tracing(tmp_path: Path) -> None:
    environment = _probe_environment(
        {"ELESIM_TRACE": "1", "PYTHONPATH": "foreign"}, tmp_path / "site"
    )

    assert environment["ELESIM_TRACE"] == "0"
    assert environment["PYTHONPATH"] == str(tmp_path / "site")


def test_role_release_manifest_declares_only_contractual_shared_material(
    tmp_path: Path,
) -> None:
    release = tmp_path / "pilot"
    release.mkdir()
    for name in expected_release_entries("pilot"):
        path = release / name
        if "." in name:
            path.write_text("", encoding="utf-8")
        else:
            path.mkdir()

    assert_release_entries(release, "pilot")
    (release / "tests").mkdir()

    with pytest.raises(ReleaseVerificationError, match="unexpected=tests"):
        assert_release_entries(release, "pilot")


def test_wheel_boundary_rejects_a_sibling_deployment(tmp_path: Path) -> None:
    wheel = _wheel(
        tmp_path / "pilot.whl",
        "elesim_pilot/__init__.py",
        "elesim_pilot-0.3.0.dist-info/METADATA",
        "elesim_robot/runtime.py",
    )

    with pytest.raises(ReleaseVerificationError, match="elesim_robot"):
        assert_wheel_boundary(wheel, "elesim_pilot")


def test_wheel_boundary_rejects_missing_owned_package(tmp_path: Path) -> None:
    wheel = _wheel(tmp_path / "pilot.whl", "README.txt")

    with pytest.raises(ReleaseVerificationError, match="elesim_pilot"):
        assert_wheel_boundary(wheel, "elesim_pilot")


def test_release_layout_rejects_public_config_template(
    tmp_path: Path,
) -> None:
    release = tmp_path / "pilot"
    release.mkdir()
    for name in expected_release_entries("pilot"):
        path = release / name
        path.mkdir() if "." not in name else path.write_text("", encoding="utf-8")
    config = release / "config"
    (config / "default.yaml").write_text("{}\n", encoding="utf-8")
    (config / "runtime.yaml").write_text("{}\n", encoding="utf-8")
    (config / "arm_model.json").write_text("{}\n", encoding="utf-8")
    (config / "runtime.public.example.yaml").write_text("{}\n", encoding="utf-8")

    with pytest.raises(ReleaseVerificationError, match="public config template"):
        verify_release_layout(release, "pilot")


def test_infrastructure_layout_rejects_source_only_package_material(
    tmp_path: Path,
) -> None:
    package = _minimal_infrastructure(tmp_path)
    (package / "src/elesim_setup/tests").mkdir()

    with pytest.raises(ReleaseVerificationError, match="source-only members"):
        verify_infrastructure_layout(tmp_path)


def test_infrastructure_layout_rejects_symlinked_required_path_ancestor(
    tmp_path: Path,
) -> None:
    package = _minimal_infrastructure(tmp_path)
    web = package / "src/elesim_setup/web"
    outside = tmp_path / "outside-web"
    web.rename(outside)
    web.symlink_to(outside, target_is_directory=True)

    assert (web / "index.html").is_file()
    with pytest.raises(ReleaseVerificationError, match="symlink"):
        verify_infrastructure_layout(tmp_path)


@pytest.mark.parametrize(
    "relative",
    (
        "src/elesim_setup/cli.py",
        "src/elesim_setup/network.py",
        "src/elesim_setup/connections.py",
        "src/elesim_setup/uninstall.py",
        "src/elesim_setup/host_proxy.py",
        "src/elesim_setup/ownership.py",
        "src/elesim_setup/shell.py",
    ),
)
def test_infrastructure_requires_every_setup_console_target(
    tmp_path: Path,
    relative: str,
) -> None:
    package = _minimal_infrastructure(tmp_path)
    target = package / relative
    target.unlink()

    with pytest.raises(ReleaseVerificationError, match=target.name):
        verify_infrastructure_layout(tmp_path)


def test_infrastructure_rejects_arbitrary_symlink_member(tmp_path: Path) -> None:
    package = _minimal_infrastructure(tmp_path)
    outside = tmp_path / "outside.py"
    outside.write_text("outside = True\n", encoding="utf-8")
    (package / "src/elesim_setup/not_required.py").symlink_to(outside)

    with pytest.raises(ReleaseVerificationError, match="symlink"):
        verify_infrastructure_layout(tmp_path)


def test_infrastructure_rejects_unowned_setup_python_module(tmp_path: Path) -> None:
    package = _minimal_infrastructure(tmp_path)
    (package / "src/elesim_setup/dummy.py").write_text("", encoding="utf-8")

    with pytest.raises(ReleaseVerificationError, match="unexpected setup Python"):
        verify_infrastructure_layout(tmp_path)


def test_infrastructure_rejects_nested_setup_python_package(
    tmp_path: Path,
) -> None:
    package = _minimal_infrastructure(tmp_path)
    rogue = package / "src/elesim_setup/rogue"
    rogue.mkdir()
    (rogue / "__init__.py").write_text("", encoding="utf-8")
    (rogue / "payload.py").write_text("", encoding="utf-8")

    with pytest.raises(ReleaseVerificationError, match="unexpected setup Python"):
        verify_infrastructure_layout(tmp_path)


def test_release_layout_rejects_symlinked_role_root(tmp_path: Path) -> None:
    outside = tmp_path / "outside-role"
    outside.mkdir()
    release = tmp_path / "pilot"
    release.symlink_to(outside, target_is_directory=True)

    with pytest.raises(ReleaseVerificationError, match="symlink"):
        verify_release_layout(release, "pilot")


@pytest.mark.parametrize(
    "missing",
    ("msg/Extra.msg", "srv/Extra.srv", "action/Extra.action"),
)
def test_rosidl_manifest_requires_every_cmake_declared_source(
    tmp_path: Path,
    missing: str,
) -> None:
    interfaces = tmp_path / "interfaces"
    cmake = interfaces / "CMakeLists.txt"
    cmake.parent.mkdir()
    cmake.write_text(
        "rosidl_generate_interfaces(${PROJECT_NAME}\n"
        '  "msg/Extra.msg"\n'
        '  "srv/Extra.srv"\n'
        '  "action/Extra.action"\n'
        ")\n",
        encoding="utf-8",
    )
    (interfaces / "package.xml").write_text("", encoding="utf-8")
    for relative in ("msg/Extra.msg", "srv/Extra.srv", "action/Extra.action"):
        source = interfaces / relative
        source.parent.mkdir(exist_ok=True)
        source.write_text("", encoding="utf-8")
    (interfaces / missing).unlink()

    with pytest.raises(ReleaseVerificationError, match=Path(missing).name):
        assert_rosidl_source_manifest(interfaces)


def test_infrastructure_layout_rejects_empty_or_wrongly_typed_material(
    tmp_path: Path,
) -> None:
    infra = tmp_path / "infra"
    (infra / "containers").mkdir(parents=True)
    (infra / "development").mkdir()
    setup = infra / "setup"
    setup.mkdir()
    for name in ("bootstrap.py", "install.sh", "bootstrap-contract.json"):
        (setup / name).write_text("", encoding="utf-8")
    package = setup / "package"
    (package / "src").mkdir(parents=True)
    (package / "pyproject.toml").write_text("", encoding="utf-8")
    (package / "requirements.lock").write_text("", encoding="utf-8")

    with pytest.raises(ReleaseVerificationError, match="Dockerfile.app"):
        verify_infrastructure_layout(tmp_path)

    for name in EXPECTED_INFRA_FILES["containers"]:
        (infra / "containers" / name).write_text("", encoding="utf-8")
    for name in EXPECTED_INFRA_FILES["development"]:
        (infra / "development" / name).write_text("", encoding="utf-8")

    (package / "src/elesim_setup").mkdir()
    with pytest.raises(ReleaseVerificationError, match="__init__.py"):
        verify_infrastructure_layout(tmp_path)


def test_infrastructure_layout_accepts_complete_minimal_boundary(
    tmp_path: Path,
) -> None:
    _minimal_infrastructure(tmp_path)

    verify_infrastructure_layout(tmp_path)


def test_wheel_environment_is_data_not_shell_code(tmp_path: Path) -> None:
    env_file = tmp_path / "WHEELS.env"
    env_file.write_text(
        "PROTOCOL_WHEEL=elesim_protocol-0.3.0.whl\n"
        "APP_WHEEL=elesim_pilot-0.3.0.whl\n",
        encoding="utf-8",
    )

    assert read_wheel_environment(env_file) == {
        "PROTOCOL_WHEEL": "elesim_protocol-0.3.0.whl",
        "APP_WHEEL": "elesim_pilot-0.3.0.whl",
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


def test_pilot_release_requires_the_generated_arm_model(tmp_path: Path) -> None:
    release = tmp_path / "pilot"
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
    application = _wheel(release / "wheels/pilot.whl", "elesim_pilot/__init__.py")
    (release / "WHEELS.env").write_text(
        f"PROTOCOL_WHEEL={protocol.name}\nAPP_WHEEL={application.name}\n",
        encoding="utf-8",
    )

    with pytest.raises(ReleaseVerificationError, match="arm_model.json"):
        verify_release_layout(release, "pilot")


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
