"""Installed role inventory is independent of the current graph assignment."""

from dataclasses import replace
import json
import os
from pathlib import Path
import subprocess

import pytest
import yaml

from elesim_setup.configuration import generate_role_configs, generated_config_path
from elesim_setup.container_installer import _runtime_up_wrapper
from elesim_setup.network import _apply_configuration_transaction, _configure_from_args, _parser
from elesim_setup.security_views import _install_prepared_views
from elesim_setup.state import InstallState
from workbench.tests.setup.conftest import copy_role_configs


def test_assignment_round_trip_and_legacy_inventory(local_state):
    state = replace(local_state(roles=("pilot", "sim", "ui")), assigned_roles=("sim", "ui"))
    loaded = InstallState.from_dict(state.to_dict())
    assert loaded.roles == ("pilot", "sim", "ui")
    assert loaded.runtime_roles == ("sim", "ui")
    legacy = state.to_dict()
    legacy["schema_version"] = 10
    legacy.pop("assigned_roles")
    assert InstallState.from_dict(legacy).runtime_roles == state.roles


@pytest.mark.parametrize("assigned", [(), ("robot",), ("pilot", "pilot"), ("unknown",)])
def test_assignment_rejects_invalid_inventory_selection(local_state, assigned):
    with pytest.raises(ValueError, match="assigned_roles"):
        replace(local_state(roles=("pilot", "sim", "ui")), assigned_roles=assigned).validate()


def test_configure_subset_preserves_spare_files_and_can_reassign(local_state, monkeypatch):
    state = local_state(roles=("pilot", "sim", "ui"))
    copy_role_configs(state)
    generate_role_configs(state)
    state.save()
    spare = generated_config_path(state, "pilot")
    original = spare.read_bytes()
    compose = state.prefix_path / "containers/compose.yaml"
    compose.parent.mkdir(parents=True)
    compose.write_text(yaml.safe_dump({"services": {
        role: {"environment": {"untouched": role}} for role in state.roles
    }}))
    args = _parser().parse_args([
        "configure", "--non-interactive", "--assigned-role", "sim", "--assigned-role", "ui",
        "--dds-system-id", "other_graph", "--pilot-id", "pilot-remote", "--sim-id", "sim-remote",
    ])
    updated = _configure_from_args(state, args)
    written = _apply_configuration_transaction(state.state_path, updated)
    assert set(written) == {"sim", "ui"}
    assert spare.read_bytes() == original
    assert InstallState.load(state.state_path).roles == state.roles
    ui = yaml.safe_load(generated_config_path(state, "ui").read_text())
    assert ui["runtime"]["pilot_id"] == "pilot-remote"
    assert ui["runtime"]["sim_id"] == "sim-remote"
    services = yaml.safe_load(compose.read_text())["services"]
    assert services["pilot"]["environment"] == {"untouched": "pilot"}
    assert services["sim"]["environment"]["ELESIM_SYSTEM_ID"] == "other_graph"

    # A failed reassignment restores both the previous state and selected files.
    before_state = state.state_path.read_bytes()
    before_compose = compose.read_bytes()
    reassigned = replace(updated, assigned_roles=("pilot",))
    import elesim_setup.network as network
    real_generate = network.generate_role_configs
    def fail_after_write(value):
        real_generate(value)
        raise RuntimeError("injected write failure")
    monkeypatch.setattr(network, "generate_role_configs", fail_after_write)
    with pytest.raises(RuntimeError, match="injected"):
        _apply_configuration_transaction(state.state_path, reassigned)
    assert state.state_path.read_bytes() == before_state
    assert compose.read_bytes() == before_compose
    assert spare.read_bytes() == original
    monkeypatch.setattr(network, "generate_role_configs", real_generate)
    assert set(_apply_configuration_transaction(state.state_path, reassigned)) == {"pilot"}
    assert InstallState.load(state.state_path).runtime_roles == ("pilot",)


def test_up_reads_current_assignment_and_refuses_spare_roles(tmp_path: Path):
    state = tmp_path / "state.json"
    state.write_text(json.dumps({"assigned_roles": ["pilot"], "dds": {"security_profile": "trusted-network"}}))
    compose = tmp_path / "compose"
    log = tmp_path / "calls"
    compose.write_text('#!/bin/sh\nprintf "%s\\n" "$*" >> "$CALLS"\n')
    compose.chmod(0o755)
    wrapper = tmp_path / "up"
    wrapper.write_text(_runtime_up_wrapper(
        compose=tmp_path / "compose.yaml", compose_wrapper=compose,
        guard="", launch_guard="", has_sim=True,
        runtime_roles=("pilot", "sim", "ui"), state_path=state,
    ))
    wrapper.chmod(0o755)
    env = dict(os.environ, CALLS=str(log))
    result = subprocess.run([wrapper], env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert log.read_text().splitlines()[-1].endswith("--remove-orphans pilot")
    before = log.read_bytes()
    result = subprocess.run([wrapper, "sim"], env=env, capture_output=True, text=True)
    assert result.returncode == 64
    assert "not assigned" in result.stderr
    assert log.read_bytes() == before
    state.write_text(json.dumps({"assigned_roles": ["ui"], "dds": {"security_profile": "trusted-network"}}))
    result = subprocess.run([wrapper], env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert log.read_text().splitlines()[-1].endswith("--remove-orphans ui")


def test_security_view_refresh_clears_inactive_role(tmp_path: Path):
    destinations = {role: tmp_path / "apps" / role for role in ("pilot", "sim")}
    for role, destination in destinations.items():
        (destination / "public").mkdir(parents=True)
        (destination / "enclaves").mkdir()
        (destination / "enclaves/key.pem").write_text(f"old-{role}")
    prepared = tmp_path / "prepared"
    (prepared / "sim/public").mkdir(parents=True)
    (prepared / "sim/enclaves").mkdir()
    (prepared / "sim/enclaves/key.pem").write_text("new-sim")

    _install_prepared_views(
        destinations=destinations,
        prepared=prepared,
        backups=tmp_path / "backups",
    )

    assert not (destinations["pilot"] / "public").exists()
    assert not (destinations["pilot"] / "enclaves").exists()
    assert (destinations["sim"] / "enclaves/key.pem").read_text() == "new-sim"
