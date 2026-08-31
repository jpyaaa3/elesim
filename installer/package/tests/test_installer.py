from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest
import yaml

from elesim_setup import installer as installer_module
from elesim_setup.configuration import role_directory
from elesim_setup.installer import (
    NATIVE_RUNTIME_LOG_BYTES,
    Installer,
    _native_down_wrapper,
    _native_logs_wrapper,
    _ensure_python_pip,
    build_install_plan,
    preflight_notes,
)
from elesim_setup.ownership import OwnershipManifest, sha256_file
from elesim_setup.state import DdsSettings


def test_native_install_plan_is_robot_only(local_state) -> None:
    state = local_state(roles=("robot",), dds=DdsSettings(interface="tailscale0"))

    actions = build_install_plan(state)

    assert [action.title for action in actions] == [
        "도구",
        "robot",
        "DDS",
        "명령",
        "상태",
    ]


def test_native_venv_pip_repair_reports_missing_ensurepip(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def fake_run(command, **_kwargs):
        if "ensurepip" in command:
            return subprocess.CompletedProcess(
                command, 1, stderr="No module named ensurepip"
            )
        return subprocess.CompletedProcess(command, 1, stderr="No module named pip")

    monkeypatch.setattr(installer_module.subprocess, "run", fake_run)
    with pytest.raises(RuntimeError, match="python3-venv|ensurepip"):
        _ensure_python_pip(tmp_path / "venv/bin/python")


def test_native_venvs_pin_setuptools_for_ros_colcon() -> None:
    source = (
        Path(__file__).resolve().parents[1]
        / "src/elesim_setup/installer.py"
    ).read_text(encoding="utf-8")

    assert source.count('"setuptools>=68,<80"') == 2
    assert source.count('"packaging>=24.2,<26"') == 2


def test_robot_lock_covers_system_pynacl_dependency() -> None:
    requirements = (
        Path(__file__).resolve().parents[3] / "robot/requirements.lock"
    ).read_text(encoding="utf-8")

    assert "cffi==2.1.1" in requirements


def test_ros_interface_build_isolates_python_metadata_from_host(
    local_state,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    state = local_state(roles=("robot",), dds=DdsSettings(interface="tailscale0"))
    installer = Installer(state, dry_run=True)
    captured: dict[str, object] = {}

    def fake_run(command, *, env=None):
        captured["command"] = tuple(command)
        captured["env"] = dict(env or {})

    monkeypatch.setattr(installer, "_run", fake_run)
    installer._install_ros_interfaces()

    environment = captured["env"]
    assert isinstance(environment, dict)
    assert environment["PYTHONNOUSERSITE"] == "1"
    site_packages = (
        Path(installer_module.sys.prefix)
        / "lib"
        / f"python{installer_module.sys.version_info.major}.{installer_module.sys.version_info.minor}"
        / "site-packages"
    )
    if site_packages.is_dir():
        assert str(site_packages) in str(environment.get("PYTHONPATH", ""))


def test_robot_dry_run_validates_and_reports_without_writing(local_state) -> None:
    state = local_state(roles=("robot",), dds=DdsSettings(interface="tailscale0"))
    logs: list[str] = []

    Installer(state, dry_run=True, log=logs.append).run()

    assert any("DRY-RUN" in line for line in logs)
    assert not state.prefix_path.exists()


def test_robot_wrapper_and_unit_use_only_generated_install_paths(
    local_state,
    monkeypatch,
    tmp_path,
) -> None:
    robot_home = tmp_path / "robot-home"
    unitree_workspace = robot_home / "unitree_ros2"
    monkeypatch.setenv("ELESIM_HOST_USER", "robot-operator")
    monkeypatch.setenv("ELESIM_OPERATOR_HOME", str(robot_home))
    monkeypatch.setenv("ELESIM_UNITREE_ROS2_WS", str(unitree_workspace))
    state = local_state(roles=("robot",), dds=DdsSettings(interface="tailscale0"))
    installer = Installer(state, dry_run=True)
    monkeypatch.setattr(
        installer,
        "_ensure_venv",
        lambda path, **_kwargs: Path(path) / "bin/python",
    )
    monkeypatch.setattr(installer, "_pip", lambda *_args: None)

    installer._install_role("robot")
    role_root = role_directory(state, "robot")
    assert (role_root / "config/default.yaml").is_file()
    assert not (role_root / "config/public.example.yaml").exists()
    assert not (role_root / "systemd").exists()
    assert not (role_root / "install.sh").exists()

    installer._write_wrappers()
    status_wrapper = (state.bin_path / "elesim-status").read_text(
        encoding="utf-8"
    )
    assert "elesim-robot.service" in status_wrapper
    assert "elesim-unitree-bridge.service" in status_wrapper
    robot_service, bridge_service = installer._write_robot_service_units()
    rendered = robot_service.read_text(encoding="utf-8")
    bridge_rendered = bridge_service.read_text(encoding="utf-8")

    assert robot_service.stat().st_mode & 0o777 == 0o644
    assert bridge_service.stat().st_mode & 0o777 == 0o644
    assert "User=robot-operator" in rendered
    assert "SupplementaryGroups=elesim-unitree" in rendered
    assert f'Environment="HOME={robot_home}"' in rendered
    # systemd does not strip quotes from path-valued settings: a quoted value
    # is read as a relative path starting with '"' and the unit is rejected
    # with LoadState=bad-setting.
    assert f"WorkingDirectory={role_root}\n" in rendered
    assert 'WorkingDirectory="' not in rendered
    assert 'WorkingDirectory="' not in bridge_rendered
    assert f'ExecStart="{state.bin_path / "elesim-robot"}"' in rendered
    assert "BindsTo=elesim-unitree-bridge.service" in rendered
    marker = state.prefix_path / "security/provisioning-required"
    assert f"ConditionPathExists=!{marker}\n" in rendered
    assert 'ConditionPathExists=!"' not in rendered
    assert 'ConditionPathExists=!"' not in bridge_rendered
    assert "RestartPreventExitStatus=78" in rendered
    assert "/etc/elesim/robot.yaml" not in rendered
    assert "/opt/elesim-robot" not in rendered
    assert "venv/bin/elesim-robot" not in rendered

    wrapper = (state.bin_path / "elesim-robot").read_text(encoding="utf-8")
    assert str(role_root / "config/installed.yaml") in wrapper
    assert "source /opt/ros/humble/setup.bash" in wrapper
    assert "UNITREE_ROS2_WS" not in wrapper
    bridge_wrapper = (state.bin_path / "elesim-unitree-bridge").read_text(
        encoding="utf-8"
    )
    assert str(role_root / "venv/bin/elesim-unitree-bridge") in bridge_wrapper
    assert "ROS_SECURITY_ENABLE" not in bridge_wrapper
    assert "CYCLONEDDS_URI" not in bridge_wrapper
    assert "User=elesim-unitree" in bridge_rendered
    assert "Group=elesim-unitree" in bridge_rendered
    assert "RuntimeDirectory=elesim-unitree" in bridge_rendered
    assert "RuntimeDirectoryMode=0750" in bridge_rendered
    assert "PartOf=elesim-robot.service" in bridge_rendered
    assert f'ExecStart="{state.bin_path / "elesim-unitree-bridge"}"' in bridge_rendered


def test_robot_registration_is_manual_and_pending_does_not_start_service(
    local_state,
) -> None:
    state = local_state(
        roles=("robot",),
        dds=DdsSettings(
            security_profile="sros2",
            security_provisioning="managed",
        ),
    )
    logs: list[str] = []
    installer = Installer(state, dry_run=True, log=logs.append)
    installer._write_wrappers()
    services = installer._write_robot_service_units()

    installer._log_robot_service_registration(services)
    rendered = "\n".join(logs)

    assert (
        f"sudo install -m 0644 -- {services[0]} "
        "/etc/systemd/system/elesim-robot.service"
    ) in rendered
    assert (
        f"sudo install -m 0644 -- {services[1]} "
        "/etc/systemd/system/elesim-unitree-bridge.service"
    ) in rendered
    assert "groupadd --force --system elesim-unitree" in rendered
    assert "usermod --append --groups elesim-unitree" in rendered
    assert "setfacl" in rendered
    assert "sudo systemctl daemon-reload" in rendered
    assert "sudo systemctl enable elesim-robot.service" in rendered
    assert "enable --now" not in rendered
    assert "provisioning" in rendered


def test_native_install_records_host_uninstaller_and_exact_systemd_hashes(
    local_state,
    monkeypatch,
    tmp_path: Path,
) -> None:
    robot_home = tmp_path / "robot-home"
    unitree_workspace = robot_home / "unitree_ros2"
    (unitree_workspace / "install").mkdir(parents=True)
    (unitree_workspace / "install/setup.bash").write_text(
        "# test overlay\n",
        encoding="utf-8",
    )
    monkeypatch.setenv("ELESIM_HOST_USER", "robot-owner")
    monkeypatch.setenv("ELESIM_OPERATOR_HOME", str(robot_home))
    monkeypatch.setenv("ELESIM_UNITREE_ROS2_WS", str(unitree_workspace))
    state = local_state(roles=("robot",), dds=DdsSettings(interface="tailscale0"))
    installer = Installer(state, log=lambda _line: None)
    monkeypatch.setattr(
        installer,
        "_install_ros_interfaces",
        lambda: (state.prefix_path / "ros").mkdir(parents=True),
    )
    monkeypatch.setattr(
        installer,
        "_install_tools",
        lambda: (state.prefix_path / "tools").mkdir(parents=True),
    )
    monkeypatch.setattr(
        installer,
        "_install_role",
        lambda role: (state.prefix_path / "roles" / role).mkdir(parents=True),
    )
    monkeypatch.setattr(
        installer_module,
        "generate_role_configs",
        lambda *_args, **_kwargs: {},
    )

    installer.run()

    manifest_path = state.prefix_path / "install-ownership.json"
    manifest = OwnershipManifest.load(manifest_path)
    assert manifest.path == manifest_path
    assert manifest.docker is None
    units = {unit.name: unit for unit in manifest.systemd_units}
    assert set(units) == {
        "elesim-robot.service",
        "elesim-unitree-bridge.service",
    }
    for name, ownership in units.items():
        generated = state.prefix_path / "roles/robot/systemd" / name
        assert generated.is_file()
        assert ownership.destination == f"/etc/systemd/system/{name}"
        assert ownership.sha256 == sha256_file(generated)

    wrapper = (state.bin_path / "elesim-uninstall").read_text(encoding="utf-8")
    assert "exec python3 -B -S -m elesim_setup.uninstall" in wrapper
    assert f"--manifest {manifest_path}" in wrapper
    assert "export PYTHONNOUSERSITE=1" in wrapper
    update_wrapper = (state.bin_path / "elesim-update").read_text(encoding="utf-8")
    assert "update --edition general" in update_wrapper
    assert "docker compose" not in update_wrapper
    assert "docker compose" not in wrapper


def test_robot_generated_config_pins_host_ipc_identity_and_workspace(
    local_state,
    monkeypatch,
    tmp_path,
) -> None:
    from conftest import copy_role_configs
    from elesim_setup.configuration import generate_role_configs

    robot_home = tmp_path / "robot-home"
    unitree_workspace = robot_home / "unitree_ros2"
    monkeypatch.setenv("ELESIM_HOST_USER", "robot-owner")
    monkeypatch.setenv("ELESIM_OPERATOR_HOME", str(robot_home))
    monkeypatch.setenv("ELESIM_UNITREE_ROS2_WS", str(unitree_workspace))
    state = local_state(
        roles=("robot",),
        dds=DdsSettings(interface="tailscale0"),
    )
    installer = Installer(state, dry_run=True)
    copy_role_configs(state)

    path = generate_role_configs(
        state,
        robot_host=installer.robot_host.config_settings,
    )["robot"]
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))

    assert payload["go2"]["ipc_robot_user"] == "robot-owner"
    assert payload["go2"]["ipc_bridge_user"] == "elesim-unitree"
    assert payload["go2"]["ros_workspace"] == str(unitree_workspace)
    assert payload["go2"]["network_interface"] == "eth0"
    assert payload["go2"]["ros_domain_id"] == 1


def test_robot_install_rejects_overlapping_elesim_and_unitree_networks(
    local_state,
    monkeypatch,
) -> None:
    monkeypatch.setenv("ELESIM_UNITREE_INTERFACE", "eth0")
    monkeypatch.setenv("ELESIM_UNITREE_DOMAIN_ID", "1")

    same_interface = local_state(
        roles=("robot",),
        dds=DdsSettings(interface="eth0", domain_id=0),
    )
    with pytest.raises(ValueError, match="interface"):
        Installer(same_interface, dry_run=True).run()

    same_domain = local_state(
        roles=("robot",),
        dds=DdsSettings(interface="tailscale0", domain_id=1),
    )
    with pytest.raises(ValueError, match="domain"):
        Installer(same_domain, dry_run=True).run()


def test_preflight_separates_container_and_robot_requirements() -> None:
    container_notes = " ".join(
        preflight_notes(("sim", "ui"), install_mode="container")
    )
    robot_notes = " ".join(preflight_notes(("robot",), install_mode="native"))

    assert "Docker" in container_notes
    assert "GPU" in container_notes
    assert "X11" in container_notes
    assert "ROS2" in robot_notes
    assert "sudo" in robot_notes


def _write_native_wrapper(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    path.chmod(0o755)


def _fake_native_sudo(path: Path) -> None:
    sudo = path / "sudo"
    sudo.write_text(
        "#!/usr/bin/env bash\n"
        "set -u\n"
        "if [[ ${1:-} == -n ]]; then shift; fi\n"
        "command=${1:-}\n"
        "shift || true\n"
        "case $command in\n"
        "  journalctl)\n"
        "    arguments=\" $* \"\n"
        "    if [[ $arguments == *' --follow '* ]]; then\n"
        "      printf 'native follow\\n'\n"
        "      exit 0\n"
        "    fi\n"
        "    bytes=${ELESIM_JOURNAL_BYTES:-0}\n"
        "    if (( bytes > 0 )); then\n"
        "      head -c \"$bytes\" /dev/zero | tr '\\000' x\n"
        "    else\n"
        "      printf 'saved native journal\\n'\n"
        "    fi\n"
        "    exit \"${ELESIM_JOURNAL_STATUS:-0}\"\n"
        "    ;;\n"
        "  systemctl)\n"
        "    if [[ ${1:-} == stop && -n ${ELESIM_STOP_MARKER:-} ]]; then\n"
        "      : >\"$ELESIM_STOP_MARKER\"\n"
        "    fi\n"
        "    exit \"${ELESIM_SYSTEMCTL_STATUS:-0}\"\n"
        "    ;;\n"
        "esac\n"
        "exit 65\n",
        encoding="utf-8",
    )
    sudo.chmod(0o755)


def test_native_logs_follow_and_save_bounded_private_journald_archive(
    tmp_path: Path,
) -> None:
    logs_root = tmp_path / "install/logs"
    wrapper = tmp_path / "bin/elesim-logs"
    _write_native_wrapper(
        wrapper,
        _native_logs_wrapper(logs_root=logs_root, archive_enabled=True),
    )
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    _fake_native_sudo(fake_bin)
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"

    follow = subprocess.run(
        (wrapper,),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert follow.returncode == 0
    assert follow.stdout == "native follow\n"
    assert not logs_root.exists()

    unsupported = subprocess.run(
        (wrapper, "--since", "today"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert unsupported.returncode == 64
    assert "elesim-logs [--save]" in unsupported.stderr

    environment["ELESIM_JOURNAL_BYTES"] = str(NATIVE_RUNTIME_LOG_BYTES + 4096)
    saved = subprocess.run(
        (wrapper, "--save"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert saved.returncode == 0
    run = next((logs_root / "runs").iterdir())
    destination = run / "robot.log"
    assert destination.stat().st_size == NATIVE_RUNTIME_LOG_BYTES
    assert logs_root.stat().st_mode & 0o777 == 0o700
    assert (logs_root / "runs").stat().st_mode & 0o777 == 0o700
    assert run.stat().st_mode & 0o777 == 0o700
    assert destination.stat().st_mode & 0o777 == 0o600

    environment.pop("ELESIM_JOURNAL_BYTES")
    for _index in range(5):
        subsequent = subprocess.run(
            (wrapper, "--save"),
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
        assert subsequent.returncode == 0
    retained = tuple((logs_root / "runs").iterdir())
    assert len(retained) == 5
    assert all((candidate / "robot.log").is_file() for candidate in retained)


def test_native_down_stops_robot_after_archive_failure_and_returns_nonzero(
    tmp_path: Path,
) -> None:
    logs_root = tmp_path / "install/logs"
    wrapper = tmp_path / "bin/elesim-down"
    _write_native_wrapper(
        wrapper,
        _native_down_wrapper(logs_root=logs_root, archive_enabled=True),
    )
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    _fake_native_sudo(fake_bin)
    marker = tmp_path / "stop-called"
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "ELESIM_JOURNAL_STATUS": "19",
            "ELESIM_STOP_MARKER": str(marker),
        }
    )

    result = subprocess.run(
        (wrapper,),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 74
    assert marker.is_file()
    assert "저장 실패" in result.stderr


def test_native_log_archive_rejects_a_symlinked_install_ancestor_before_write(
    tmp_path: Path,
) -> None:
    install_root = tmp_path / "install"
    logs_root = install_root / "logs"
    install_root.mkdir()
    wrapper = tmp_path / "bin/elesim-logs"
    _write_native_wrapper(
        wrapper,
        _native_logs_wrapper(logs_root=logs_root, archive_enabled=True),
    )
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    _fake_native_sudo(fake_bin)
    original = tmp_path / "original-install"
    outside = tmp_path / "outside"
    outside.mkdir()
    install_root.rename(original)
    install_root.symlink_to(outside, target_is_directory=True)
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"

    result = subprocess.run(
        (wrapper, "--save"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 74
    assert "symlink" in result.stderr
    assert not (outside / "logs").exists()
