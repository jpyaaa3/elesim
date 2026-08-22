from __future__ import annotations

import json
from dataclasses import replace

import pytest

from elesim_setup.profiles import normalize_roles, roles_for_profile
from elesim_setup.state import (
    ContainerNetworkSettings,
    DdsSettings,
    InstallState,
    NetworkSettings,
    RuntimeTextLogSettings,
    STATE_SCHEMA_VERSION,
    TurnSettings,
)


def test_profiles_are_router_free() -> None:
    assert roles_for_profile("local-sim") == ("sim", "pilot", "ui")
    assert roles_for_profile("laptop") == ("pilot", "ui")
    assert roles_for_profile("compute") == ("sim",)
    assert normalize_roles(("ui", "sim", "ui")) == ("sim", "ui")
    with pytest.raises(ValueError, match="router"):
        normalize_roles(("router",))


def test_install_mode_is_centralized_by_runtime_topology(local_state) -> None:
    assert local_state(roles=("pilot", "ui")).validate().install_mode == (
        "container"
    )
    assert local_state(roles=("robot",)).validate().install_mode == "native"

    with pytest.raises(ValueError, match="Docker/Compose"):
        local_state(
            roles=("sim",),
            install_mode="native",
        ).validate()
    with pytest.raises(ValueError, match="generic Ubuntu"):
        local_state(
            roles=("robot",),
            install_mode="container",
        ).validate()
    with pytest.raises(ValueError, match="Robot 단독"):
        local_state(
            roles=("pilot", "robot"),
            install_mode="native",
        ).validate()


def test_state_round_trip_persists_dds_v9_and_runtime_text_logs(
    local_state, tmp_path
) -> None:
    state = local_state(
        roles=("pilot", "ui"),
        dds=DdsSettings(
            system_id="lab_a",
            domain_id=42,
            discovery_mode="static",
            static_peers=("192.0.2.10", "sim.example.com"),
            interface="eth1",
            security_profile="sros2",
            security_provisioning="external",
            keystore=str(tmp_path / "sros2"),
            enclave="/lab_a",
        ),
    )
    path = state.save()
    loaded = InstallState.load(path)

    assert loaded.schema_version == STATE_SCHEMA_VERSION == 9
    assert loaded.dds == state.dds
    assert loaded.runtime_text_logs == RuntimeTextLogSettings(enabled=True)
    assert loaded.to_dict()["runtime_text_logs"] == {"enabled": True}
    assert loaded.to_dict()["dds"]["static_peers"] == [
        "192.0.2.10",
        "sim.example.com",
    ]
    assert path.stat().st_mode & 0o777 == 0o600


def test_state_round_trip_persists_update_source_identity(local_state) -> None:
    state = local_state(
        source_repository="lab/elesim",
        source_ref="refactoring",
    )

    restored = InstallState.from_dict(state.to_dict())

    assert restored.source_repository == "lab/elesim"
    assert restored.source_ref == "refactoring"


def test_legacy_state_defaults_update_source_identity(local_state) -> None:
    raw = local_state().to_dict()
    raw.pop("source_repository")
    raw.pop("source_ref")

    restored = InstallState.from_dict(raw)

    assert restored.source_repository == "jpyaaa3/elesim"
    assert restored.source_ref == "main"


def test_state_v9_round_trip_persists_tailscale_sidecar_binding(local_state) -> None:
    prefix = local_state().prefix_path
    settings = ContainerNetworkSettings(
        mode="tailscale-sidecar",
        docker_context="desktop-linux",
        docker_engine_id="desktop-engine-id",
        tailscale_hostname="elesim-0123456789ab",
        tailscale_state_dir=str(prefix / "secrets/tailscale"),
    )
    state = local_state(container_network=settings)

    restored = InstallState.from_dict(state.to_dict())

    assert restored.container_network == settings
    assert "auth" not in json.dumps(restored.to_dict()).lower()


def test_v8_migration_preserves_legacy_direct_host_network(local_state) -> None:
    raw = local_state().to_dict()
    raw["schema_version"] = 8
    raw.pop("container_network")

    restored = InstallState.from_dict(raw)

    assert restored.container_network == ContainerNetworkSettings()


def test_legacy_controller_simulator_state_is_normalized_to_pilot_sim(local_state) -> None:
    raw = local_state(roles=("pilot", "sim")).to_dict()
    raw["schema_version"] = 7
    raw["roles"] = ["controller", "simulator"]
    network = raw["network"]
    network["controller_id"] = network.pop("pilot_id")
    network["simulator_id"] = network.pop("sim_id")

    migrated = InstallState.from_dict(raw)
    emitted = migrated.to_dict()

    assert set(migrated.roles) == {"pilot", "sim"}
    assert migrated.network.pilot_id.startswith("pilot-")
    assert migrated.network.sim_id.startswith("sim-")
    assert set(emitted["roles"]) == {"pilot", "sim"}
    assert "controller_id" not in emitted["network"]
    assert "simulator_id" not in emitted["network"]


@pytest.mark.parametrize("source_schema", range(1, 7))
def test_pre_v7_state_migration_keeps_runtime_text_archive_disabled(
    local_state,
    source_schema: int,
) -> None:
    raw = local_state(roles=("pilot",)).to_dict()
    raw["schema_version"] = source_schema

    migrated = InstallState.from_dict(raw)

    assert migrated.schema_version == STATE_SCHEMA_VERSION
    assert migrated.runtime_text_logs.enabled is False


def test_runtime_text_log_setting_requires_a_boolean(local_state) -> None:
    invalid = replace(
        local_state(),
        runtime_text_logs=RuntimeTextLogSettings(enabled="yes"),  # type: ignore[arg-type]
    )

    with pytest.raises(ValueError, match="boolean"):
        invalid.validate()


def test_state_round_trip_persists_all_runtime_endpoint_ids(local_state) -> None:
    state = local_state(
        roles=("pilot", "ui"),
        network=NetworkSettings(
            sim_id="sim-west",
            pilot_id="pilot-west",
            ui_id="ui-west",
            robot_id="robot-west",
        ),
    )

    restored = InstallState.from_dict(state.to_dict())

    assert restored.network == state.network


def test_pre_v6_state_gets_stable_ui_and_robot_endpoint_defaults(local_state) -> None:
    raw = local_state(roles=("pilot",)).to_dict()
    raw["schema_version"] = 5
    raw["network"].pop("ui_id")
    raw["network"].pop("robot_id")

    restored = InstallState.from_dict(raw)

    assert restored.network.ui_id == "ui-main"
    assert restored.network.robot_id == "robot-go2"


def test_v3_migration_never_invents_static_peer(local_state) -> None:
    raw = local_state(roles=("pilot",)).to_dict()
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
    raw = local_state(roles=("pilot",)).to_dict()
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


def test_v4_external_turn_migration_requires_sim_credentials(
    local_state,
) -> None:
    raw = local_state(
        roles=("sim",),
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


def test_v5_sros2_migrates_to_external_provisioning(local_state, tmp_path) -> None:
    raw = local_state(
        roles=("pilot",),
        dds=DdsSettings(
            security_profile="sros2",
            security_provisioning="external",
            keystore=str(tmp_path / "sros2"),
            enclave="/elesim",
        ),
    ).to_dict()
    raw["schema_version"] = 5
    raw["dds"].pop("security_provisioning")
    raw["dds"].pop("security_generation")
    raw["dds"].pop("security_bundle")

    migrated = InstallState.from_dict(raw)

    assert migrated.dds.security_provisioning == "external"
    assert migrated.dds.security_generation == ""
    assert migrated.dds.security_bundle == ""


def test_managed_sros2_requires_matching_versioned_bundle(tmp_path) -> None:
    bundle = tmp_path / "security/current"
    DdsSettings(
        security_profile="sros2",
        security_provisioning="managed",
        security_generation="gen-3",
        security_bundle=str(bundle),
        keystore=str(bundle),
        enclave="/elesim/lab",
    ).validate()

    with pytest.raises(ValueError, match="generation/bundle"):
        DdsSettings(
            security_profile="sros2",
            security_provisioning="managed",
            keystore=str(bundle),
            enclave="/elesim/lab",
        ).validate()


def test_pending_managed_sros2_is_installable_but_not_runnable(local_state) -> None:
    dds = DdsSettings(
        security_profile="sros2",
        security_provisioning="managed",
    ).validate()
    state = local_state(dds=dds)

    assert dds.managed_security_pending is True
    assert state.require_installable_dds() is state
    with pytest.raises(ValueError, match="elesim-connections"):
        state.require_runnable_dds()


def test_external_turn_credentials_are_sim_only(local_state, tmp_path) -> None:
    network = NetworkSettings(
        turn_urls=("turn:relay.example.com:3478?transport=udp",),
    )
    turn = TurnSettings(
        mode="external",
        credential_file=str(tmp_path / "turn.credentials.json"),
    )

    local_state(
        roles=("sim",),
        network=network,
        turn=turn,
        install_mode="container",
    ).require_runnable_dds()
    with pytest.raises(ValueError, match="Sim"):
        local_state(
            roles=("pilot", "ui"),
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


def test_managed_turn_is_owned_by_sim_and_requires_sros2(
    local_state,
    tmp_path,
) -> None:
    dds = DdsSettings(
        security_profile="sros2",
        security_provisioning="external",
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
        roles=("sim",),
        dds=dds,
        turn=turn,
        network=network,
        install_mode="container",
    ).validate()
    with pytest.raises(ValueError, match="sros2"):
        local_state(
            roles=("sim",),
            turn=turn,
            network=network,
            install_mode="container",
        ).validate()

    with pytest.raises(ValueError, match="Sim"):
        local_state(
            roles=("pilot",),
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
