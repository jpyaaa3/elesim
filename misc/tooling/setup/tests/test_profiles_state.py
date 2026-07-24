from __future__ import annotations

import json
from dataclasses import replace

import pytest

from elesim_setup.profiles import normalize_roles, roles_for_profile
from elesim_setup.state import (
    DdsSettings,
    InstallState,
    NetworkSettings,
    STATE_SCHEMA_VERSION,
    TurnSettings,
)


def test_profiles_are_router_free() -> None:
    assert roles_for_profile("local-sim") == ("simulator", "controller", "ui")
    assert roles_for_profile("laptop") == ("controller", "ui")
    assert roles_for_profile("compute") == ("simulator",)
    assert normalize_roles(("ui", "simulator", "ui")) == ("simulator", "ui")
    with pytest.raises(ValueError, match="router"):
        normalize_roles(("router",))


def test_state_round_trip_persists_dds_v5(local_state, tmp_path) -> None:
    state = local_state(
        roles=("controller", "ui"),
        dds=DdsSettings(
            system_id="lab_a",
            domain_id=42,
            discovery_mode="static",
            static_peers=("192.0.2.10", "sim.example.com"),
            interface="eth1",
            security_profile="sros2",
            keystore=str(tmp_path / "sros2"),
            enclave="/lab_a",
        ),
    )
    path = state.save()
    loaded = InstallState.load(path)

    assert loaded.schema_version == STATE_SCHEMA_VERSION == 5
    assert loaded.dds == state.dds
    assert loaded.to_dict()["dds"]["static_peers"] == [
        "192.0.2.10",
        "sim.example.com",
    ]
    assert path.stat().st_mode & 0o777 == 0o600


def test_v3_migration_never_invents_static_peer(local_state) -> None:
    raw = local_state(roles=("controller",)).to_dict()
    raw["schema_version"] = 3
    raw.pop("dds")
    raw["network"].update(
        {
            "router_host": "203.0.113.10",
            "advertise_host": "198.51.100.20",
            "router_port": 5558,
            "rgbd_port": 5568,
        }
    )
    raw["security"] = {"mode": "loopback", "credentials_root": ""}

    migrated = InstallState.from_dict(raw)

    assert migrated.dds.discovery_mode == "multicast"
    assert migrated.dds.static_peers == ()
    assert "router" not in migrated.roles
    assert "203.0.113.10" not in json.dumps(migrated.to_dict())


def test_v3_curve_migration_fails_closed_until_sros2_is_configured(
    local_state,
) -> None:
    raw = local_state(roles=("controller",)).to_dict()
    raw["schema_version"] = 3
    raw.pop("dds")
    raw["security"] = {
        "mode": "curve",
        "credentials_root": "/tmp/legacy-curve",
    }

    migrated = InstallState.from_dict(raw)

    assert migrated.dds.security_profile == "sros2"
    assert migrated.dds.keystore == ""
    with pytest.raises(ValueError, match="자동 변환"):
        migrated.require_runnable_dds()


def test_v4_external_turn_migration_requires_simulator_credentials(
    local_state,
) -> None:
    raw = local_state(
        roles=("simulator",),
        install_mode="container",
        network=NetworkSettings(
            turn_urls=("turn:relay.example.com:3478?transport=udp",),
        ),
        turn=TurnSettings(mode="external"),
    ).to_dict()
    raw["schema_version"] = 4
    raw["turn"].pop("credential_file")

    migrated = InstallState.from_dict(raw)

    assert migrated.schema_version == STATE_SCHEMA_VERSION
    assert migrated.turn.credential_file == ""
    with pytest.raises(ValueError, match="credential"):
        migrated.require_runnable_dds()


def test_external_turn_credentials_are_simulator_only(local_state, tmp_path) -> None:
    network = NetworkSettings(
        turn_urls=("turn:relay.example.com:3478?transport=udp",),
    )
    turn = TurnSettings(
        mode="external",
        credential_file=str(tmp_path / "turn.credentials.json"),
    )

    local_state(
        roles=("simulator",),
        network=network,
        turn=turn,
        install_mode="container",
    ).require_runnable_dds()
    with pytest.raises(ValueError, match="Simulator"):
        local_state(
            roles=("controller", "ui"),
            network=network,
            turn=turn,
            install_mode="container",
        ).validate()


def test_static_discovery_requires_explicit_peers_at_runtime(local_state) -> None:
    state = local_state(
        dds=DdsSettings(discovery_mode="static"),
    )
    state.validate()
    with pytest.raises(ValueError, match="peer"):
        state.require_runnable_dds()


def test_managed_turn_is_owned_by_simulator_and_requires_sros2(
    local_state,
    tmp_path,
) -> None:
    dds = DdsSettings(
        security_profile="sros2",
        keystore=str(tmp_path / "sros2"),
        enclave="/elesim",
    )
    turn = TurnSettings(
        mode="managed",
        realm="elesim.local",
        public_host="turn.example.com",
        secret_file=str(tmp_path / "turn.secret"),
    )
    network = NetworkSettings(
        turn_urls=("turn:turn.example.com:3478?transport=udp",),
    )
    local_state(
        roles=("simulator",),
        dds=dds,
        turn=turn,
        network=network,
        install_mode="container",
    ).validate()
    with pytest.raises(ValueError, match="sros2"):
        local_state(
            roles=("simulator",),
            turn=turn,
            network=network,
            install_mode="container",
        ).validate()

    with pytest.raises(ValueError, match="Simulator"):
        local_state(
            roles=("controller",),
            dds=dds,
            turn=turn,
            network=network,
        ).validate()


@pytest.mark.parametrize("domain_id", (-1, 233))
def test_dds_domain_range_is_bounded(domain_id: int) -> None:
    with pytest.raises(ValueError, match="0..232"):
        DdsSettings(domain_id=domain_id).validate()


def test_trusted_network_cannot_smuggle_sros2_paths() -> None:
    with pytest.raises(ValueError, match="trusted-network"):
        DdsSettings(keystore="/tmp/keys", enclave="/elesim").validate()


def test_multicast_rejects_static_peers() -> None:
    with pytest.raises(ValueError, match="multicast"):
        DdsSettings(static_peers=("192.0.2.1",)).validate()
