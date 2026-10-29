"""Regressions for the October Jetson installation failures; no devices needed."""
from dataclasses import replace
import json
import os
from pathlib import Path
import struct

import pytest
import yaml

from elesim_setup import configuration, installer, network
from elesim_setup.capabilities import require_free_space
from elesim_setup.state import ComputeSettings, DdsSettings, InstallState, RobotHostSettings
from elesim_setup.container_installer import ContainerInstaller
from conftest import copy_role_configs


def test_native_reconfigure_migrates_host_facts_and_preserves_motor_calibration(local_state):
    state = local_state(roles=("robot",), dds=DdsSettings(domain_id=1, interface="tailscale0"))
    copy_role_configs(state)
    host = RobotHostSettings("operator", "elesim-unitree", Path("/srv/unitree"), "eno1", 0)
    written = configuration.generate_role_configs(state, robot_host=host)["robot"]
    installed = yaml.safe_load(written.read_text())
    installed["arm"]["current_limit_ma"] = 2100
    installed["mapping"]["linear_u_limit"] = 240.0
    written.write_text(yaml.safe_dump(installed))
    state.save()
    network._apply_configuration_transaction(state.state_path, replace(state, dds=replace(state.dds, domain_id=2)))
    after = yaml.safe_load(written.read_text())
    persisted = InstallState.load(state.state_path)
    assert persisted.robot_host == host
    assert after["go2"]["network_interface"] == "eno1"
    assert after["go2"]["ros_workspace"] == "/srv/unitree"
    assert after["go2"]["ros_domain_id"] == 0
    assert after["dds"]["domain_id"] == 2
    assert after["arm"]["current_limit_ma"] == 2100
    assert after["mapping"]["linear_u_limit"] == 240.0


def test_robot_host_roundtrip_and_v11_read_migration(local_state):
    state = replace(local_state(roles=("robot",)), robot_host=RobotHostSettings(
        "operator", "bridge", Path("/srv/ros2"), "eno1", 1,
    ))
    assert InstallState.from_dict(json.loads(json.dumps(state.to_dict()))) == state
    legacy = state.to_dict()
    legacy["schema_version"] = 11
    legacy.pop("robot_host")
    assert InstallState.from_dict(legacy).robot_host is None


def test_fresh_domains_are_separate_without_rewriting_legacy_graph(local_state):
    assert DdsSettings().domain_id == 1
    assert RobotHostSettings("robot", "bridge", Path("/srv/unitree")).unitree_domain_id == 0
    raw = local_state(roles=("ui",)).to_dict()
    raw["schema_version"] = 11
    raw["dds"]["domain_id"] = 0
    assert InstallState.from_dict(raw).dds.domain_id == 0
    del raw["dds"]["domain_id"]
    assert InstallState.from_dict(raw).dds.domain_id == 0


def test_explicit_native_host_override_updates_only_selected_fields(local_state, monkeypatch):
    saved = RobotHostSettings("operator", "bridge", Path("/srv/old"), "eno1", 1)
    state = replace(local_state(roles=("robot",)), robot_host=saved)
    for key in ("ELESIM_UNITREE_ROS2_WS", "UNITREE_ROS2_WS", "ELESIM_UNITREE_INTERFACE", "ELESIM_UNITREE_DOMAIN_ID"):
        monkeypatch.delenv(key, raising=False)
    assert installer.Installer(state, dry_run=True).state.robot_host == saved
    monkeypatch.setenv("ELESIM_UNITREE_DOMAIN_ID", "0")
    updated = installer.Installer(state, dry_run=True).state.robot_host
    assert updated == replace(saved, unitree_domain_id=0)


@pytest.mark.parametrize("operation", ["rewrite", "failed-copy", "rollback"])
def test_atomic_config_rewrite_preserves_named_service_account_acl(tmp_path, monkeypatch, operation):
    path = tmp_path / "installed.yaml"
    path.write_text("old")
    # Linux access ACL: owner rw, named service UID read, group none, mask read.
    acl = struct.pack("<I", 2) + b"".join(
        struct.pack("<HHI", tag, permission, identity)
        for tag, permission, identity in (
            (1, 6, 0xFFFFFFFF), (2, 4, 65534), (4, 0, 0xFFFFFFFF),
            (16, 4, 0xFFFFFFFF), (32, 0, 0xFFFFFFFF),
        )
    )
    try:
        os.setxattr(path, "system.posix_acl_access", acl)
    except OSError as exc:
        if exc.errno in {95, 38}:
            pytest.skip("filesystem does not implement POSIX ACLs")
        raise
    expected = os.getxattr(path, "system.posix_acl_access")
    if operation == "failed-copy":
        monkeypatch.setattr(configuration.shutil, "copystat", lambda *args: None)
        with pytest.raises(OSError):
            configuration._atomic_text(path, "new")
    elif operation == "rollback":
        from elesim_setup.install_transaction import RefreshControlRollback
        manifest = tmp_path / "manifest.json"
        manifest.write_text("unchanged")
        with pytest.raises(RuntimeError, match="generation failed"):
            with RefreshControlRollback(manifest=manifest, paths=[path]):
                configuration._atomic_text(path, "new")
                os.removexattr(path, "system.posix_acl_access")
                raise RuntimeError("generation failed")
    else:
        configuration._atomic_text(path, "new")
    assert path.read_text() == ("new" if operation == "rewrite" else "old")
    assert os.getxattr(path, "system.posix_acl_access") == expected
    assert path.stat().st_mode & 0o777 == 0o640


def test_native_venv_uses_base_interpreter_instead_of_private_setup_cache(tmp_path, monkeypatch):
    value = object.__new__(installer.Installer)
    value.log = lambda _: None
    commands = []
    value._run = commands.append
    monkeypatch.setattr(installer.sys, "executable", "/private/setup/cache/python")
    monkeypatch.setattr(installer.sys, "_base_executable", "/usr/bin/python3")
    monkeypatch.setattr(installer, "_ensure_python_pip", lambda _: None)
    value._ensure_venv(tmp_path / "robot-venv", system_site_packages=True)
    assert commands == [(str(Path("/usr/bin/python3").resolve()), "-m", "venv",
                         "--system-site-packages", str(tmp_path / "robot-venv"))]


def test_space_check_fails_before_writing_destination(tmp_path, monkeypatch):
    monkeypatch.setattr("elesim_setup.capabilities.shutil.disk_usage", lambda _: type("Usage", (), {"free": 10})())
    destination = tmp_path / "not-created/install"
    with pytest.raises(OSError, match="Insufficient free space.*backup"):
        require_free_space(destination, required_bytes=100, purpose="backup")
    assert not destination.exists()


def test_ui_field_gpu_choice_migrates_and_explicit_instance_choice_wins(local_state):
    state = local_state(roles=("ui",), compute=ComputeSettings("cpu"))
    policy = state.prefix_path / "maintenance/ui-rendering.json"
    policy.parent.mkdir(parents=True)
    policy.write_text('{"backend":"nvidia"}')
    state.save()
    migrated = InstallState.load(state.state_path)
    assert migrated.ui_compute == ComputeSettings("inherit")
    migrated.save()
    policy.unlink()
    assert InstallState.load(state.state_path).ui_compute == ComputeSettings("inherit")
    value = ContainerInstaller(migrated)
    value._image_fingerprints["elesim/ui:local"] = "a" * 64
    gpu = value._role_service("ui")
    assert gpu["user"] == f"{os.getuid()}:{os.getgid()}"
    assert gpu["environment"]["LIBGL_ALWAYS_SOFTWARE"] == "0"
    assert gpu["environment"]["__GLX_VENDOR_LIBRARY_NAME"] == "nvidia"
    cpu = value._role_service("ui", compute=ComputeSettings("cpu"))
    assert cpu["environment"]["LIBGL_ALWAYS_SOFTWARE"] == "1"
    assert "__GLX_VENDOR_LIBRARY_NAME" not in cpu["environment"]
    selected = value._role_service("ui", compute=ComputeSettings("specific", "GPU-render"))
    reservation = selected["deploy"]["resources"]["reservations"]["devices"][0]
    assert tuple(reservation["device_ids"]) == ("GPU-render",)
    assert "CUDA_VISIBLE_DEVICES" not in selected["environment"]


def test_ui_policy_rejects_dangling_symlink(local_state):
    state = local_state(roles=("ui",))
    policy = state.prefix_path / "maintenance/ui-rendering.json"
    policy.parent.mkdir(parents=True)
    policy.symlink_to(policy.parent / "missing.json")
    with pytest.raises(ValueError, match="non-symlinked"):
        ContainerInstaller(state)
