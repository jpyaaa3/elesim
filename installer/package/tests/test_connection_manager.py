from __future__ import annotations

import json
from pathlib import Path

import pytest

from elesim_setup.connection_manager import (
    ConnectionTopology,
    DdsEndpoint,
    ManagedHost,
    PreflightHost,
    PreflightSshEndpoint,
    RoleAssignment,
    SshEndpoint,
    TwoHostPreflight,
)


FINGERPRINT = "SHA256:" + "A" * 43


def _ssh(host: str, *, port: int = 2222) -> SshEndpoint:
    return SshEndpoint(
        host=host,
        port=port,
        user="elesim",
        identity_file="~/.ssh/elesim_ed25519",
        pinned_fingerprint=FINGERPRINT,
    )


def _topology() -> ConnectionTopology:
    return ConnectionTopology(
        system_id="lab_arm",
        security_profile="sros2",
        hosts=(
            ManagedHost(
                host_id="laptop",
                display_name="Operator laptop",
                local=True,
                dds=DdsEndpoint("100.64.0.10", "tailscale0"),
                ssh=None,
                assignments=(
                    RoleAssignment("pilot", "pilot-main"),
                    RoleAssignment("ui", "ui-main"),
                ),
            ),
            ManagedHost(
                host_id="compute",
                display_name="Compute server",
                local=False,
                dds=DdsEndpoint("100.64.0.20", "tailscale0"),
                ssh=_ssh("server.example"),
                assignments=(RoleAssignment("sim", "sim-main"),),
            ),
            ManagedHost(
                host_id="robot",
                display_name="Robot Jetson",
                local=False,
                dds=DdsEndpoint("100.64.0.30", "tailscale0"),
                ssh=_ssh("jetson.example", port=2201),
                assignments=(RoleAssignment("robot", "robot-main"),),
                install_mode="native",
                jetson=True,
                install_root="/opt/elesim-robot",
                bin_dir="/usr/local/bin",
                lifecycle="systemd",
            ),
        ),
    ).validate()


def _preflight() -> TwoHostPreflight:
    return TwoHostPreflight(
        hosts=(
            PreflightHost(
                host_id="laptop",
                display_name="Operator laptop",
                local=True,
                dds=DdsEndpoint("100.64.0.10", "tailscale0"),
                ssh=None,
            ),
            PreflightHost(
                host_id="compute",
                display_name="Compute host",
                local=False,
                dds=DdsEndpoint("100.64.0.20", "tailscale0"),
                ssh=PreflightSshEndpoint("100.64.0.20", 22, "elesim"),
            ),
        ),
    ).validate()


def _simulation_topology() -> ConnectionTopology:
    return ConnectionTopology(
        system_id="lab_sim",
        security_profile="trusted-network",
        topology_mode="simulation-only",
        hosts=(
            ManagedHost(
                host_id="sim-laptop",
                display_name="Simulation laptop",
                local=True,
                dds=DdsEndpoint("100.64.0.40", "tailscale0"),
                ssh=None,
                assignments=(
                    RoleAssignment("pilot", "pilot-main"),
                    RoleAssignment("sim", "sim-main"),
                    RoleAssignment("ui", "ui-main"),
                ),
            ),
        ),
    ).validate()


def test_two_host_preflight_keeps_mutable_tailscale_and_ssh_endpoints_distinct() -> None:
    preflight = _preflight()
    restored = TwoHostPreflight.from_dict(preflight.to_dict())

    assert restored == preflight
    assert restored.hosts[0].dds.address == "100.64.0.10"
    assert restored.hosts[1].dds.interface == "tailscale0"
    assert restored.hosts[1].ssh is not None
    assert restored.hosts[1].ssh.port == 22
    assert restored.discovery_peers("laptop") == ("100.64.0.20",)


def test_dds_endpoint_tailscale_provenance_roundtrips_without_a_port() -> None:
    endpoint = DdsEndpoint("100.64.0.10", "tailscale0", "tailscale")

    restored = DdsEndpoint.from_dict(endpoint.to_dict())

    assert restored == endpoint
    assert "port" not in endpoint.to_dict()
    with pytest.raises(ValueError, match="tailscale0"):
        DdsEndpoint.from_dict(
            {"address": "100.64.0.10", "interface": "eth0", "address_source": "tailscale"}
        )


def test_two_host_preflight_does_not_accept_http_port_in_dds_address() -> None:
    raw = _preflight().to_dict()
    raw["hosts"][0]["dds"]["address"] = "100.64.0.10:8080"

    with pytest.raises(ValueError, match="port"):
        TwoHostPreflight.from_dict(raw)


def test_two_host_preflight_is_not_a_robot_deployment_topology() -> None:
    raw = _preflight().to_dict()
    raw["hosts"].append(raw["hosts"][1].copy())

    with pytest.raises(ValueError, match="exactly two"):
        TwoHostPreflight.from_dict(raw)


def test_connection_topology_roundtrip_keeps_dds_and_ssh_distinct() -> None:
    topology = _topology()
    raw = topology.to_dict()

    restored = ConnectionTopology.from_dict(raw)

    assert restored == topology
    assert restored.host("compute").dds.address == "100.64.0.20"
    assert restored.host("compute").ssh is not None
    assert restored.host("compute").ssh.host == "server.example"
    assert restored.host("compute").ssh.port == 2222


def test_simulation_only_topology_accepts_one_host_without_robot() -> None:
    topology = _simulation_topology()
    raw = topology.to_dict()

    assert raw["schema_version"] == 3
    assert raw["topology_mode"] == "simulation-only"
    restored = ConnectionTopology.from_dict(raw)

    assert restored == topology
    assert restored.hosts[0].roles == ("pilot", "sim", "ui")


def test_legacy_schema_v1_is_loaded_as_full_and_normalized() -> None:
    raw = _topology().to_dict()
    raw["schema_version"] = 1
    raw.pop("topology_mode")

    restored = ConnectionTopology.from_dict(raw)

    assert restored.schema_version == 3
    assert restored.topology_mode == "full"
    assert restored.to_dict()["topology_mode"] == "full"


def test_simulation_only_rejects_robot_and_jetson_hosts() -> None:
    raw = _simulation_topology().to_dict()
    raw["hosts"][0]["assignments"].append(
        {"role": "robot", "endpoint_id": "robot-go2"}
    )
    with pytest.raises(ValueError, match="simulation-only|Robot"):
        ConnectionTopology.from_dict(raw)

    raw = _simulation_topology().to_dict()
    raw["hosts"][0].update(
        {"jetson": True, "install_mode": "native", "lifecycle": "systemd"}
    )
    with pytest.raises(ValueError, match="Jetson|container/Compose"):
        ConnectionTopology.from_dict(raw)


def test_dds_and_ssh_may_share_a_hostname_without_sharing_port_semantics() -> None:
    raw = _topology().to_dict()
    raw["hosts"][1]["dds"]["address"] = "server.example"

    restored = ConnectionTopology.from_dict(raw)

    assert restored.host("compute").dds.address == "server.example"
    assert restored.host("compute").ssh is not None
    assert restored.host("compute").ssh.port == 2222


def test_static_discovery_peers_are_generated_from_other_active_hosts() -> None:
    raw = _topology().to_dict()
    raw["dds_graph"]["discovery_mode"] = "static"
    topology = ConnectionTopology.from_dict(raw)

    assert topology.discovery_peers("compute") == (
        "100.64.0.10",
        "100.64.0.30",
    )


def test_duplicate_dds_addresses_are_rejected() -> None:
    raw = _topology().to_dict()
    raw["hosts"][1]["dds"]["address"] = raw["hosts"][0]["dds"]["address"]

    with pytest.raises(ValueError, match="DDS addresses"):
        ConnectionTopology.from_dict(raw)


@pytest.mark.parametrize(
    "address",
    ("127.0.0.1", "::1", "224.0.0.1", "ff02::1"),
)
def test_dds_addresses_reject_loopback_and_multicast(address: str) -> None:
    raw = _topology().to_dict()
    raw["hosts"][1]["dds"]["address"] = address

    with pytest.raises(ValueError, match="loopback|multicast"):
        ConnectionTopology.from_dict(raw)


@pytest.mark.parametrize("host", ("127.0.0.1", "::1"))
def test_ssh_management_endpoint_may_still_use_loopback(host: str) -> None:
    endpoint = _ssh(host)

    assert endpoint.validate() is endpoint


def test_endpoint_ids_reject_canonical_dash_underscore_collisions() -> None:
    raw = _topology().to_dict()
    raw["hosts"][0]["assignments"][1]["endpoint_id"] = "pilot_main"

    with pytest.raises(ValueError, match="canonicalization"):
        ConnectionTopology.from_dict(raw)


def test_endpoint_ids_reject_ros_key_truncation_collisions() -> None:
    raw = _topology().to_dict()
    common = "a" + "x" * 62
    raw["hosts"][0]["assignments"][0]["endpoint_id"] = common + "-one"
    raw["hosts"][0]["assignments"][1]["endpoint_id"] = common + "-two"

    with pytest.raises(ValueError, match="canonicalization"):
        ConnectionTopology.from_dict(raw)


@pytest.mark.parametrize("role", ["pilot", "sim", "ui", "robot"])
def test_every_role_must_be_assigned_exactly_once(role: str) -> None:
    raw = _topology().to_dict()
    for host in raw["hosts"]:
        host["assignments"] = [
            item for item in host["assignments"] if item["role"] != role
        ]

    with pytest.raises(ValueError, match="at least one role|exactly once"):
        ConnectionTopology.from_dict(raw)


def test_duplicate_role_assignment_is_rejected() -> None:
    raw = _topology().to_dict()
    raw["hosts"][1]["assignments"].append(
        {"role": "pilot", "endpoint_id": "pilot-other"}
    )

    with pytest.raises(ValueError, match="exactly once"):
        ConnectionTopology.from_dict(raw)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda host: host["assignments"].append(
            {"role": "ui", "endpoint_id": "ui-on-robot"}
        ), "only role"),
        (lambda host: host.update({"jetson": False}), "Jetson"),
        (lambda host: host.update({"install_mode": "container"}), "native"),
    ],
)
def test_robot_is_alone_and_native_on_jetson(mutation, message: str) -> None:
    raw = _topology().to_dict()
    mutation(raw["hosts"][2])

    with pytest.raises(ValueError, match=message):
        ConnectionTopology.from_dict(raw)


def test_exactly_one_host_is_local_and_remote_hosts_require_ssh() -> None:
    raw = _topology().to_dict()
    raw["hosts"][1]["local"] = True
    raw["hosts"][1]["ssh"] = None
    with pytest.raises(ValueError, match="exactly one"):
        ConnectionTopology.from_dict(raw)

    raw = _topology().to_dict()
    raw["hosts"][1]["ssh"] = None
    with pytest.raises(ValueError, match="requires an explicit SSH"):
        ConnectionTopology.from_dict(raw)


def test_non_robot_hosts_are_container_compose_only() -> None:
    raw = _topology().to_dict()
    raw["hosts"][1].update({"install_mode": "native", "lifecycle": "systemd"})

    with pytest.raises(ValueError, match="container/Compose"):
        ConnectionTopology.from_dict(raw)


def test_robot_cannot_be_the_local_authority_host() -> None:
    raw = _topology().to_dict()
    raw["hosts"][0]["local"] = False
    raw["hosts"][0]["ssh"] = _ssh("laptop.example").to_dict()
    raw["hosts"][2]["local"] = True
    raw["hosts"][2]["ssh"] = None

    with pytest.raises(ValueError, match="authority host"):
        ConnectionTopology.from_dict(raw)


def test_connection_state_rejects_passwords_and_key_contents() -> None:
    raw = _topology().to_dict()
    raw["hosts"][1]["ssh"]["password"] = "do-not-store"
    with pytest.raises(ValueError, match="secret material"):
        ConnectionTopology.from_dict(raw)

    raw = _topology().to_dict()
    raw["hosts"][1]["ssh"]["identity_file"] = "-----BEGIN PRIVATE KEY-----\n"
    with pytest.raises(ValueError, match="path, not key contents"):
        ConnectionTopology.from_dict(raw)


def test_connection_state_is_atomically_persisted_mode_0600(tmp_path: Path) -> None:
    destination = tmp_path / "connections.json"
    topology = _topology()

    topology.save(destination)
    first = destination.read_bytes()
    topology.save(destination)

    assert destination.stat().st_mode & 0o777 == 0o600
    assert destination.read_bytes() == first
    assert ConnectionTopology.load(destination) == topology
    assert json.loads(first)["hosts"][1]["ssh"]["identity_file"].endswith(
        "elesim_ed25519"
    )
    assert not list(tmp_path.glob(".connections.json.*"))


def test_connection_state_refuses_symlinks(tmp_path: Path) -> None:
    real = tmp_path / "real.json"
    real.write_text("{}", encoding="utf-8")
    link = tmp_path / "state.json"
    link.symlink_to(real)

    with pytest.raises(ValueError, match="symlink"):
        _topology().save(link)
    with pytest.raises(ValueError, match="symlink"):
        ConnectionTopology.load(link)


@pytest.mark.parametrize("field", ["install_root", "bin_dir"])
def test_host_paths_reject_filesystem_root(field: str) -> None:
    raw = _topology().to_dict()
    raw["hosts"][0][field] = "/"

    with pytest.raises(ValueError, match="contained absolute POSIX path"):
        ConnectionTopology.from_dict(raw)
