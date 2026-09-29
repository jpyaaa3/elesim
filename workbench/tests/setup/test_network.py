from __future__ import annotations

from pathlib import Path
import json
from types import SimpleNamespace

import pytest

from elesim_setup.container_installer import refresh_compose_dds_environment
from elesim_setup.network import _snapshot


def test_doctor_json_keeps_probe_prints_on_stderr(local_state, monkeypatch, capsys):
    from elesim_setup import network

    monkeypatch.setattr(network.InstallState, "load", lambda path: local_state())
    class Doctor:
        def __init__(self, *args, **kwargs):
            print("probe initialization")

        def run(self):
            print("probe progress")
            return SimpleNamespace(ok=True, to_dict=lambda: {"ok": True, "results": []})

    monkeypatch.setattr(network, "NetworkDoctor", Doctor)
    assert network.main(["doctor", "--json"]) == 0
    output = capsys.readouterr()
    assert json.loads(output.out) == {"ok": True, "results": []}
    assert "probe initialization" in output.err
    assert "probe progress" in output.err


def test_compose_refresh_rejects_broken_symlink(local_state) -> None:
    state = local_state(roles=("pilot",))
    compose = state.prefix_path / "containers/compose.yaml"
    compose.parent.mkdir(parents=True)
    compose.symlink_to(compose.parent / "missing.yaml")

    with pytest.raises(ValueError, match="regular file"):
        refresh_compose_dds_environment(state)


def test_configuration_snapshot_rejects_broken_symlink(tmp_path: Path) -> None:
    linked = tmp_path / "config.yaml"
    linked.symlink_to(tmp_path / "missing.yaml")

    with pytest.raises(ValueError, match="regular file"):
        _snapshot(linked)


def test_runtime_preflight_returns_state_and_checks_runtime_namespace(
    local_state, monkeypatch, capsys
) -> None:
    from elesim_setup import network

    state = local_state(roles=("pilot",))
    checks = []
    monkeypatch.setattr(network.InstallState, "load", lambda _path: state)
    monkeypatch.setattr(
        network,
        "require_runtime_network_namespace",
        lambda received, **kwargs: checks.append((received, kwargs)),
    )

    assert network.main(
        [
            "runtime-preflight",
            "--dds-interface",
            "eth0",
            "--dds-address",
            "10.0.0.10",
            "--dds-peer",
            "10.0.0.20",
        ]
    ) == 0

    output = capsys.readouterr()
    payload = json.loads(output.out)
    assert payload == {
        "schema_version": 1,
        "install_state": state.to_dict(),
        "instance_state": None,
        "namespace": {
            "interface": "eth0",
            "address": "10.0.0.10",
            "peers": ["10.0.0.20"],
        },
    }
    assert checks == [
        (
            state,
            {
                "interface": "eth0",
                "address": "10.0.0.10",
                "peers": ["10.0.0.20"],
            },
        )
    ]
    assert output.err == ""


def test_sidecar_runtime_preflight_defers_install_tree_check(
    local_state, monkeypatch, capsys, tmp_path: Path
) -> None:
    from elesim_setup import network
    from elesim_setup.state import ContainerNetworkSettings

    state = local_state(
        roles=("pilot",),
        container_network=ContainerNetworkSettings(
            mode="tailscale-sidecar",
            docker_context="default",
            docker_engine_id="engine-id",
            tailscale_hostname="pilot-node",
            tailscale_state_dir=str(tmp_path / "tailscale"),
        ),
    )
    monkeypatch.setattr(network.InstallState, "load", lambda _path: state)
    monkeypatch.setattr(network, "require_runtime_network_namespace", lambda *_a, **_k: None)
    monkeypatch.setattr(
        network,
        "require_generated_dds_configuration",
        lambda _state: pytest.fail("sidecar runtime tools must not read the install tree"),
    )

    assert network.main(
        [
            "runtime-preflight",
            "--dds-interface",
            "tailscale0",
            "--dds-address",
            "100.64.0.10",
            "--check-configuration",
        ]
    ) == 0

    assert json.loads(capsys.readouterr().out)["install_state"] == state.to_dict()


@pytest.mark.parametrize("security_profile", ["trusted-network", "sros2"])
def test_scoped_runtime_preflight_reads_only_validated_instance(
    local_state, security_profile: str
) -> None:
    from elesim_setup.instances import InstanceEndpoint, InstanceState
    from elesim_setup import network

    state = local_state(roles=("pilot",))
    system = "lab"
    root = state.prefix_path / "instances" / system
    (root / "bin").mkdir(parents=True)
    (state.prefix_path / "instances").mkdir(exist_ok=True)
    (state.prefix_path / "bin").mkdir(parents=True, exist_ok=True)
    (state.bin_path).mkdir(parents=True, exist_ok=True)
    (state.prefix_path / "containers").mkdir(parents=True, exist_ok=True)
    (state.prefix_path / "bin" / "elesim-instance").write_text("#!/bin/sh\n")
    (state.prefix_path / "bin" / "elesim-instance").chmod(0o755)
    for action in ("up", "down", "status", "logs"):
        wrapper = root / "bin" / action
        wrapper.write_text("#!/bin/sh\n")
        wrapper.chmod(0o755)
    (state.prefix_path / "containers" / "compose.instances.yaml").write_text(
        "services: {}\n"
    )
    instance = InstanceState(
        system_id=system,
        release_key="a" * 64,
        endpoints=(InstanceEndpoint("pilot", "pilot-main"),),
        domain_id=0,
        security_profile=security_profile,
        security_generation="g1" if security_profile == "sros2" else "",
    )
    (root / "state.json").write_text(json.dumps(instance.to_dict()))
    if security_profile == "sros2":
        generation = root / "security" / "generations" / "g1"
        generation.mkdir(parents=True)
        (generation / "manifest.json").write_text(json.dumps({"system_id": system}))
        (root / "security" / "current").symlink_to("generations/g1")

    result = network._scoped_runtime_instance_state(state, system)

    assert result["system_id"] == system
    assert result["security_profile"] == security_profile
