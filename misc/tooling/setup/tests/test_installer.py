from __future__ import annotations

import pytest

from elesim_setup import installer as installer_module
from elesim_setup.installer import Installer, build_install_plan, preflight_notes
from elesim_setup.state import ComputeSettings


def test_install_plan_exposes_tools_roles_network_wrappers_and_state(local_state) -> None:
    state = local_state(roles=("ui",))
    actions = build_install_plan(state)
    titles = [action.title for action in actions]
    assert titles == ["도구", "ui", "DDS", "명령", "상태"]


def test_dry_run_validates_and_reports_without_writing(local_state) -> None:
    state = local_state(roles=("ui",))
    logs: list[str] = []

    Installer(state, dry_run=True, log=logs.append).run()

    assert any("DRY-RUN" in line for line in logs)
    assert not state.prefix_path.exists()


def test_role_commands_point_only_to_owned_install_directory(local_state) -> None:
    state = local_state(
        profile="local-sim",
        roles=("simulator", "controller", "ui", "robot"),
    )
    installer = Installer(state, dry_run=True)
    for role in state.roles:
        executable, arguments = installer._role_command(role)
        assert str(state.prefix_path / "roles" / role) in str(executable)
        rendered = " ".join(arguments)
        assert all(
            str(state.prefix_path / "roles" / sibling) not in rendered
            for sibling in state.roles
            if sibling != role
        )
    _sim_executable, sim_args = installer._role_command("simulator")
    assert any(value.endswith("app.installed.yaml") for value in sim_args)


def test_compute_wrappers_inherit_pin_or_disable_cuda(local_state) -> None:
    inherited = Installer(
        local_state(roles=("controller", "simulator")),
        dry_run=True,
    )
    assert inherited._role_environment("controller") == {}
    assert inherited._role_environment("simulator") == {}

    pinned = Installer(
        local_state(
            roles=("controller", "simulator"),
            compute=ComputeSettings(gpu_mode="specific", gpu_device="GPU-lab-a"),
        ),
        dry_run=True,
    )
    assert pinned._role_environment("controller") == {
        "CUDA_VISIBLE_DEVICES": "GPU-lab-a"
    }
    assert pinned._role_environment("simulator") == {
        "CUDA_VISIBLE_DEVICES": "GPU-lab-a"
    }

    cpu = Installer(
        local_state(
            roles=("controller", "simulator"),
            compute=ComputeSettings(gpu_mode="cpu"),
        ),
        dry_run=True,
    )
    assert cpu._role_environment("controller") == {"CUDA_VISIBLE_DEVICES": ""}
    assert cpu._role_environment("simulator") == {"CUDA_VISIBLE_DEVICES": ""}


def test_written_wrapper_preserves_external_cuda_unless_policy_overrides_it(local_state) -> None:
    inherited_state = local_state(roles=("controller",))
    Installer(inherited_state, dry_run=True)._write_wrappers()
    inherited = (inherited_state.bin_path / "elesim-controller").read_text(encoding="utf-8")
    assert "CUDA_VISIBLE_DEVICES" not in inherited

    pinned_state = local_state(
        roles=("controller",),
        compute=ComputeSettings(gpu_mode="specific", gpu_device="1"),
    )
    Installer(pinned_state, dry_run=True)._write_wrappers()
    pinned = (pinned_state.bin_path / "elesim-controller").read_text(encoding="utf-8")
    assert "export CUDA_VISIBLE_DEVICES=1" in pinned
    assert "source /opt/ros/humble/setup.bash" in pinned
    assert "ROS_DOMAIN_ID=0" in pinned


def test_preflight_calls_out_external_system_requirements() -> None:
    notes = " ".join(preflight_notes(("simulator", "ui", "robot")))
    assert "GPU" in notes
    assert "OpenGL" in notes
    assert "ROS2" in notes
    assert "sudo" in notes


def test_simulator_preflight_reports_missing_external_git(local_state, monkeypatch) -> None:
    state = local_state(roles=("simulator",))
    monkeypatch.setattr(installer_module.shutil, "which", lambda _name: None)
    with pytest.raises(RuntimeError, match="git"):
        Installer(state, dry_run=True).run()
