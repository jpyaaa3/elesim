from __future__ import annotations

import json
from pathlib import Path

import pytest

from elesim_setup.connection_manager import (
    ConnectionTopology,
    DdsEndpoint,
    DdsGraphSettings,
    DeploymentUnit,
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
        dds_graph=DdsGraphSettings(discovery_mode="static"),
        hosts=(
            ManagedHost(
                host_id="laptop",
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
                local=False,
                dds=DdsEndpoint("100.64.0.20", "tailscale0"),
                ssh=_ssh("server.example"),
                assignments=(RoleAssignment("sim", "sim-main"),),
            ),
            ManagedHost(
                host_id="robot",
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
                local=True,
                dds=DdsEndpoint("100.64.0.10", "tailscale0"),
                ssh=None,
            ),
            PreflightHost(
                host_id="compute",
                local=False,
                dds=DdsEndpoint("100.64.0.20", "tailscale0"),
                ssh=PreflightSshEndpoint("compute-management.example", 22, "elesim"),
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


def test_two_host_preflight_preserves_independent_ssh_destination() -> None:
    preflight = _preflight()
    restored = TwoHostPreflight.from_dict(preflight.to_dict())

    assert restored == preflight
    assert restored.hosts[0].dds.address == "100.64.0.10"
    assert restored.hosts[1].dds.interface == "tailscale0"
    assert restored.hosts[1].ssh is not None
    assert restored.hosts[1].ssh.host == "compute-management.example"
    assert restored.hosts[1].ssh.port == 22
    assert restored.discovery_peers("laptop") == ("100.64.0.20",)
    assert all("display_name" not in host for host in preflight.to_dict()["hosts"])


def test_preflight_schema_v1_migrates_the_legacy_shared_ssh_address() -> None:
    raw = _preflight().to_dict()
    raw["schema_version"] = 1
    raw["hosts"][1]["ssh"]["host"] = "stale-management.example"

    restored = TwoHostPreflight.from_dict(raw)

    assert restored.schema_version == 2
    assert restored.hosts[1].ssh is not None
    assert restored.hosts[1].ssh.host == restored.hosts[1].dds.address


def test_dds_endpoint_tailscale_provenance_roundtrips_without_a_port() -> None:
    endpoint = DdsEndpoint("100.64.0.10", "tailscale0", "tailscale")

    restored = DdsEndpoint.from_dict(endpoint.to_dict())

    assert restored == endpoint
    assert "port" not in endpoint.to_dict()
    with pytest.raises(ValueError, match="tailscale\\*"):
        DdsEndpoint.from_dict(
            {"address": "100.64.0.10", "interface": "eth0", "address_source": "tailscale"}
        )

    assert DdsEndpoint("100.64.0.11", "tailscale1", "tailscale").validate().interface == "tailscale1"


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


def test_connection_topology_roundtrip_preserves_independent_ssh_destination() -> None:
    topology = _topology()
    raw = topology.to_dict()

    restored = ConnectionTopology.from_dict(raw)

    assert restored == topology
    assert all("display_name" not in host for host in raw["hosts"])
    assert not hasattr(restored.host("compute"), "display_name")
    assert restored.host("compute").dds.address == "100.64.0.20"
    assert restored.host("compute").ssh is not None
    assert restored.host("compute").ssh.host == "server.example"
    assert restored.host("compute").ssh.port == 2222


def test_connection_topology_rejects_symlinked_state_ancestors(tmp_path: Path) -> None:
    real_root = tmp_path / "real-state"
    real_root.mkdir()
    linked_root = tmp_path / "linked-state"
    linked_root.symlink_to(real_root, target_is_directory=True)
    topology_path = linked_root / "connections" / "topology.json"

    with pytest.raises(ValueError, match="symlink"):
        _topology().save(topology_path)
    with pytest.raises(ValueError, match="symlink"):
        ConnectionTopology.load(topology_path)


def test_legacy_coturn_fields_are_discarded_on_topology_roundtrip() -> None:
    raw = _topology().to_dict()
    raw["hosts"][1]["coturn"] = {
        "url": "turn:100.64.0.20:3478?transport=udp",
        "public_host": "100.64.0.20",
        "realm": "elesim.local",
        "auth_file": "/opt/elesim/secrets/turn.secret",
    }

    restored = ConnectionTopology.from_dict(raw)

    assert "coturn" not in restored.to_dict()["hosts"][1]


def test_jetson_is_an_equal_host_with_shared_paths_and_distinct_lifecycles() -> None:
    host = ManagedHost(
        host_id="jetson",
        local=False,
        dds=DdsEndpoint("100.64.0.31", "tailscale0"),
        ssh=_ssh("jetson.example", port=2201),
        jetson=True,
        units=(
            DeploymentUnit(
                "runtime",
                (RoleAssignment("pilot", "pilot-main"), RoleAssignment("ui", "ui-main")),
                install_root="/opt/elesim-runtime",
            ),
            DeploymentUnit(
                "robot-native",
                (RoleAssignment("robot", "robot-main"),),
                install_mode="native",
                install_root="/opt/elesim-robot",
                lifecycle="systemd",
            ),
        ),
    ).validate()

    restored = ManagedHost.from_dict(host.to_dict())

    assert restored == host
    assert restored.runtime_units[0].install_root == "/opt/elesim-runtime"
    assert restored.robot_units[0].install_root == "/opt/elesim-runtime"
    assert restored.robot_units[0].lifecycle == "systemd"
    assert restored.roles == ("pilot", "ui", "robot")


def test_robot_unit_can_share_the_host_installation_path_with_runtime() -> None:
    host = ManagedHost(
        host_id="jetson",
        local=False,
        dds=DdsEndpoint("100.64.0.31", "tailscale0"),
        ssh=_ssh("jetson.example"),
        jetson=True,
        units=(
            DeploymentUnit(
                "runtime",
                (RoleAssignment("pilot", "pilot-main"),),
                install_root="/opt/elesim",
            ),
            DeploymentUnit(
                "robot-native",
                (RoleAssignment("robot", "robot-main"),),
                install_mode="native",
                install_root="/opt/elesim",
                lifecycle="systemd",
            ),
        ),
    ).validate()
    assert host.runtime_units[0].install_root == host.robot_units[0].install_root


def test_mixed_units_share_the_host_command_path() -> None:
    host = ManagedHost(
        host_id="jetson",
        local=False,
        dds=DdsEndpoint("100.64.0.31", "tailscale0"),
        ssh=_ssh("jetson.example"),
        jetson=True,
        units=(
            DeploymentUnit(
                "runtime",
                (RoleAssignment("pilot", "pilot-main"),),
                install_root="/opt/elesim-runtime",
                bin_dir="/opt/shared/bin",
            ),
            DeploymentUnit(
                "robot-native",
                (RoleAssignment("robot", "robot-main"),),
                install_mode="native",
                install_root="/opt/elesim-robot",
                bin_dir="/opt/shared/bin",
                lifecycle="systemd",
            ),
        ),
    ).validate()
    assert host.runtime_units[0].bin_dir == host.robot_units[0].bin_dir


def test_legacy_mixed_unit_paths_are_normalized_to_the_runtime_path() -> None:
    host = ManagedHost(
        host_id="jetson",
        local=False,
        dds=DdsEndpoint("100.64.0.31", "tailscale0"),
        ssh=_ssh("jetson.example"),
        jetson=True,
        units=(
            DeploymentUnit(
                "runtime",
                (RoleAssignment("pilot", "pilot-main"),),
                install_root="/opt/elesim",
                bin_dir="/opt/elesim/bin",
            ),
            DeploymentUnit(
                "robot-native",
                (RoleAssignment("robot", "robot-main"),),
                install_mode="native",
                install_root="/opt/elesim/robot",
                bin_dir="/opt/elesim/robot/bin",
                lifecycle="systemd",
            ),
        ),
    ).validate()
    assert host.robot_units[0].install_root == "/opt/elesim"
    assert host.robot_units[0].bin_dir == "/opt/elesim/bin"


def test_current_sim_image_is_not_advertised_for_jetson_units() -> None:
    with pytest.raises(ValueError, match="amd64-only|ARM64"):
        ManagedHost(
            host_id="jetson",
            local=False,
            dds=DdsEndpoint("100.64.0.31", "tailscale0"),
            ssh=_ssh("jetson.example"),
            jetson=True,
            units=(
                DeploymentUnit(
                    "runtime",
                    (RoleAssignment("sim", "sim-main"),),
                    install_root="/opt/elesim-runtime",
                ),
                DeploymentUnit(
                    "robot-native",
                    (RoleAssignment("robot", "robot-main"),),
                    install_mode="native",
                    install_root="/opt/elesim-robot",
                    lifecycle="systemd",
                ),
            ),
        ).validate()


def test_jetson_requires_the_mandatory_robot_unit() -> None:
    with pytest.raises(ValueError, match="mandatory native Robot"):
        ManagedHost(
            host_id="jetson",
            local=False,
            dds=DdsEndpoint("100.64.0.31", "tailscale0"),
            ssh=_ssh("jetson.example"),
            jetson=True,
            assignments=(RoleAssignment("pilot", "pilot-main"),),
        ).validate()


def test_multicast_rejects_multi_host_tailscale_address_even_when_interface_is_eth0() -> None:
    topology = _topology()
    raw = topology.to_dict()
    raw["dds_graph"]["discovery_mode"] = "multicast"
    raw["hosts"][0]["dds"] = {
        "address": "100.64.0.10",
        "interface": "eth0",
    }
    raw["hosts"][1]["dds"] = {
        "address": "100.64.0.20",
        "interface": "eth0",
    }
    raw["hosts"][2]["dds"] = {
        "address": "100.64.0.30",
        "interface": "eth0",
    }

    with pytest.raises(ValueError, match="multicast DDS discovery"):
        ConnectionTopology.from_dict(raw)


def test_tailscale_ssh_endpoint_is_keyless_and_defaults_old_files_to_openssh() -> None:
    fingerprint = "SHA256:" + "B" * 43
    endpoint = SshEndpoint(
        "100.64.0.20", 22, "operator", "", fingerprint, auth_mode="tailscale"
    )

    assert endpoint.validate() is endpoint
    assert endpoint.uses_tailscale_ssh is True
    assert endpoint.uses_agent is False
    restored = SshEndpoint.from_dict(endpoint.to_dict())
    assert restored == endpoint
    assert SshEndpoint.from_dict(
        {
            "host": "server.example",
            "port": 2222,
            "user": "operator",
            "identity_file": "",
            "pinned_fingerprint": FINGERPRINT,
        }
    ).auth_mode == "openssh"


@pytest.mark.parametrize(
    "changes",
    [
        {"port": 2222},
        {"identity_file": "~/.ssh/id_ed25519"},
        {"auth_mode": "unknown"},
    ],
)
def test_tailscale_ssh_rejects_non_keyless_or_nonstandard_settings(changes) -> None:
    values = {
        "host": "100.64.0.20",
        "port": 22,
        "user": "operator",
        "identity_file": "",
        "pinned_fingerprint": FINGERPRINT,
        "auth_mode": "tailscale",
    }
    values.update(changes)

    with pytest.raises(ValueError, match="Tailscale|auth_mode"):
        SshEndpoint.from_dict(values)


def test_simulation_only_topology_accepts_one_host_without_robot() -> None:
    topology = _simulation_topology()
    raw = topology.to_dict()

    assert raw["schema_version"] == 4
    assert raw["topology_mode"] == "simulation-only"
    restored = ConnectionTopology.from_dict(raw)

    assert restored == topology
    assert restored.hosts[0].roles == ("pilot", "sim", "ui")


def test_legacy_schema_v1_is_loaded_as_full_and_normalized() -> None:
    raw = _topology().to_dict()
    raw["schema_version"] = 1
    raw.pop("topology_mode")

    restored = ConnectionTopology.from_dict(raw)

    assert restored.schema_version == 4
    assert restored.topology_mode == "full"
    assert restored.to_dict()["topology_mode"] == "full"
    assert restored.host("compute").ssh is not None
    assert restored.host("compute").ssh.host == "100.64.0.20"


@pytest.mark.parametrize("legacy_version", [2, 3])
def test_legacy_schema_v2_v3_keep_shared_address_semantics(
    legacy_version: int,
) -> None:
    raw = _topology().to_dict()
    raw["schema_version"] = legacy_version
    raw["hosts"][1]["ssh"]["host"] = "stale-management.example"

    restored = ConnectionTopology.from_dict(raw)

    assert restored.schema_version == 4
    assert restored.host("compute").ssh is not None
    assert restored.host("compute").ssh.host == restored.host("compute").dds.address


def test_simulation_only_rejects_robot_and_jetson_hosts() -> None:
    raw = _simulation_topology().to_dict()
    raw["hosts"][0]["assignments"].append(
        {"role": "robot", "endpoint_id": "robot-go2"}
    )
    with pytest.raises(ValueError, match="simulation-only|Robot"):
        ConnectionTopology.from_dict(raw)

    raw = _simulation_topology().to_dict()
    raw["hosts"][0].update(
        {"jetson": True}
    )
    with pytest.raises(ValueError, match="Jetson|container/Compose"):
        ConnectionTopology.from_dict(raw)


def test_changing_dds_address_does_not_change_ssh_destination_in_v4() -> None:
    raw = _topology().to_dict()
    raw["hosts"][1]["dds"]["address"] = "runtime-sidecar.example"

    restored = ConnectionTopology.from_dict(raw)

    assert restored.host("compute").dds.address == "runtime-sidecar.example"
    assert restored.host("compute").ssh is not None
    assert restored.host("compute").ssh.host == "server.example"
    assert restored.host("compute").ssh.port == 2222


def test_static_discovery_peers_are_generated_from_other_active_hosts() -> None:
    raw = _topology().to_dict()
    raw["dds_graph"]["discovery_mode"] = "static"
    topology = ConnectionTopology.from_dict(raw)

    assert topology.discovery_peers("compute") == (
        "100.64.0.10",
        "100.64.0.30",
    )


def test_static_discovery_peers_seed_co_located_roles() -> None:
    topology = _topology()

    assert topology.discovery_peers("laptop") == (
        "100.64.0.10",
        "100.64.0.20",
        "100.64.0.30",
    )


def test_static_discovery_peers_seed_one_host_simulation_roles() -> None:
    raw = _simulation_topology().to_dict()
    raw["dds_graph"]["discovery_mode"] = "static"
    topology = ConnectionTopology.from_dict(raw)

    assert topology.discovery_peers("sim-laptop") == ("100.64.0.40",)


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

    with pytest.raises(
        ValueError,
        match="at least one role|exactly once|Jetson host",
    ):
        ConnectionTopology.from_dict(raw)


def test_duplicate_role_assignment_is_rejected() -> None:
    raw = _topology().to_dict()
    raw["hosts"][1]["assignments"].append(
        {"role": "pilot", "endpoint_id": "pilot-other"}
    )

    with pytest.raises(
        ValueError,
        match="one role more than once|every role must be assigned exactly once",
    ):
        ConnectionTopology.from_dict(raw)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda host: host["assignments"].append(
            {"role": "ui", "endpoint_id": "ui-on-robot"}
        ), "only Robot"),
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
