from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from elesim_setup.configuration import generate_instance_configs
from elesim_setup.instances import InstanceEndpoint, InstanceState
from elesim_setup.state import DdsSettings, InstallState, NetworkSettings


def _install(tmp_path: Path) -> InstallState:
    state = InstallState(
        profile="custom", roles=("pilot", "sim", "ui"),
        prefix=str(tmp_path / "prefix"), bin_dir=str(tmp_path / "bin"),
        source_root=str(tmp_path), network=NetworkSettings(), dds=DdsSettings(),
    )
    for role, source in (("pilot", "runtime.yaml"), ("sim", "runtime.yaml"), ("ui", "default.yaml")):
        directory = state.prefix_path / "apps" / role / "config"
        directory.mkdir(parents=True, exist_ok=True)
        (directory / source).write_text(yaml.safe_dump({"runtime": {"role": role}}))
        if role == "sim":
            (directory / "config.yaml").write_text("simulation: {}\n")
    return state


def _instance(system: str, domain: int, suffix: str) -> InstanceState:
    return InstanceState(system, "a" * 64, (
        InstanceEndpoint("pilot", f"pilot-{suffix}"),
        InstanceEndpoint("sim", f"sim-{suffix}"),
        InstanceEndpoint("ui", f"ui-{suffix}"),
    ), domain)


def test_two_systems_are_private_and_legacy_is_preserved(tmp_path: Path):
    state = _install(tmp_path)
    legacy = (state.prefix_path / "apps" / "pilot" / "config" / "runtime.yaml").read_bytes()
    first = _instance("alpha", 11, "a")
    second = _instance("beta", 12, "b")
    generate_instance_configs(state, first)
    generate_instance_configs(state, second)
    assert (state.prefix_path / "apps" / "pilot" / "config" / "runtime.yaml").read_bytes() == legacy
    for item in (first, second):
        target = state.prefix_path / "instances" / item.system_id / "endpoints" / f"pilot-{item.system_id[0]}" / "config"
        assert (target / "runtime.installed.yaml").is_file()
        payload = yaml.safe_load((target / "runtime.installed.yaml").read_text())
        assert payload["runtime"]["endpoint_id"] == f"pilot-{item.system_id[0]}"
        assert payload["dds"]["system_id"] == item.system_id


def test_subset_and_symlink_destination_are_safe(tmp_path: Path):
    state = _install(tmp_path)
    instance = InstanceState("alpha", "a" * 64, (InstanceEndpoint("pilot", "pilot-a"),), 1)
    generate_instance_configs(state, instance)
    assert (state.prefix_path / "instances/alpha/endpoints/pilot-a/config/runtime.installed.yaml").is_file()
    outside = tmp_path / "outside"
    outside.mkdir()
    endpoint = state.prefix_path / "instances/beta/endpoints/pilot-b"
    endpoint.parent.mkdir(parents=True, exist_ok=True)
    endpoint.symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError, match="symlink"):
        generate_instance_configs(state, InstanceState("beta", "b" * 64, (InstanceEndpoint("pilot", "pilot-b"),), 2))
    assert not list(outside.iterdir())


def test_assigned_roles_do_not_hide_installed_capabilities(tmp_path: Path):
    state = _install(tmp_path)
    state = InstallState(**{**state.__dict__, "assigned_roles": ("pilot",)})
    value = InstanceState("alpha", "a" * 64, (InstanceEndpoint("sim", "sim-a"),), 2)
    generate_instance_configs(state, value)
    assert (state.prefix_path / "instances/alpha/endpoints/sim-a/config/runtime.installed.yaml").is_file()


def test_non_trusted_security_and_duplicate_roles_rejected(tmp_path: Path):
    state = _install(tmp_path)
    with pytest.raises(ValueError, match="unique"):
        InstanceState("alpha", "a" * 64, (
            InstanceEndpoint("pilot", "pilot-a"),
            InstanceEndpoint("pilot", "pilot-b"),
        ), 1)
    secure = InstallState(**{**state.__dict__, "dds": DdsSettings(security_profile="sros2", security_provisioning="external", keystore="/tmp/k", enclave="/e")})
    secure_instance = replace(_instance("secure", 2, "s"), security_profile="sros2")
    with pytest.raises(ValueError, match="managed"):
        generate_instance_configs(secure, secure_instance)


def test_ui_and_sim_generated_paths_are_local(tmp_path: Path):
    state = _install(tmp_path)
    value = _instance("alpha", 3, "a")
    generate_instance_configs(state, value)
    for role, endpoint in (("sim", "sim-a"), ("ui", "ui-a")):
        config = state.prefix_path / "instances/alpha/endpoints" / endpoint / "config"
        generated = yaml.safe_load((config / ("runtime.installed.yaml" if role == "sim" else "installed.yaml")).read_text())
        assert generated["dds"]["system_id"] == "alpha"
        if role == "sim":
            assert (config / "config.yaml").is_file()
            assert yaml.safe_load((config / "app.installed.yaml").read_text())["extends"] == "config.yaml"
        else:
            assert generated["runtime"]["endpoint_id"] == endpoint
        assert (config / "cyclonedds.xml").is_file()


def test_instance_dds_settings_are_owned_by_instance(tmp_path: Path):
    state = _install(tmp_path)
    first = InstanceState(
        "alpha", "a" * 64, (InstanceEndpoint("pilot", "alpha-p"),), 11,
        discovery_mode="static", static_peers=("10.0.0.2",), interface="tailscale0",
    )
    second = InstanceState(
        "beta", "b" * 64, (InstanceEndpoint("pilot", "beta-p"),), 12,
        interface="eth0",
    )
    generate_instance_configs(state, first)
    changed = InstallState(**{**state.__dict__, "dds": DdsSettings(interface="wlan0", domain_id=99)})
    generate_instance_configs(changed, second)
    alpha = yaml.safe_load((state.prefix_path / "instances/alpha/endpoints/alpha-p/config/runtime.installed.yaml").read_text())
    beta = yaml.safe_load((state.prefix_path / "instances/beta/endpoints/beta-p/config/runtime.installed.yaml").read_text())
    assert alpha["dds"]["domain_id"] == 11
    assert alpha["dds"]["discovery_mode"] == "static"
    assert alpha["dds"]["static_peers"] == ["10.0.0.2"]
    assert alpha["dds"]["network_interface"] == "tailscale0"
    assert beta["dds"]["domain_id"] == 12
    assert beta["dds"]["network_interface"] == "eth0"


def test_remote_graph_endpoint_ids_reach_local_role_configs(tmp_path: Path):
    state = _install(tmp_path)
    pilot_host = InstanceState(
        "alpha",
        "a" * 64,
        (InstanceEndpoint("pilot", "pilot-1"),),
        11,
        pilot_id="pilot-1",
        sim_id="sim-2",
        ui_id="ui-3",
    )
    ui_host = InstanceState(
        "alpha",
        "a" * 64,
        (InstanceEndpoint("ui", "ui-3"),),
        11,
        pilot_id="pilot-1",
        sim_id="sim-2",
        ui_id="ui-3",
    )

    generate_instance_configs(state, pilot_host)
    generate_instance_configs(state, ui_host)

    pilot = yaml.safe_load(
        (state.prefix_path / "instances/alpha/endpoints/pilot-1/config/runtime.installed.yaml").read_text()
    )
    ui = yaml.safe_load(
        (state.prefix_path / "instances/alpha/endpoints/ui-3/config/installed.yaml").read_text()
    )
    assert pilot["runtime"]["active_target"] == "sim-2"
    assert ui["runtime"] == {
        "role": "ui",
        "endpoint_id": "ui-3",
        "pilot_id": "pilot-1",
        "sim_id": "sim-2",
    }
