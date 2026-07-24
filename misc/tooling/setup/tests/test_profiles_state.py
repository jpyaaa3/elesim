from __future__ import annotations

import json
from pathlib import Path

import pytest

from elesim_setup.profiles import ROLE_ORDER, normalize_roles, roles_for_profile
from elesim_setup.state import (
    ComputeSettings,
    InstallState,
    NetworkSettings,
    SecuritySettings,
    TurnSettings,
)


def test_profiles_select_only_the_expected_deployments() -> None:
    assert roles_for_profile("local-sim") == ("router", "simulator", "controller", "ui")
    assert roles_for_profile("laptop") == ("controller", "ui")
    assert roles_for_profile("compute") == ("router", "simulator")
    assert roles_for_profile("robot") == ("robot",)


def test_custom_roles_are_deduplicated_in_stable_order() -> None:
    assert normalize_roles(("ui", "router", "ui")) == ("router", "ui")
    assert normalize_roles(ROLE_ORDER) == ROLE_ORDER


@pytest.mark.parametrize("roles", [(), ("missing",)])
def test_invalid_role_selection_is_rejected(roles: tuple[str, ...]) -> None:
    with pytest.raises(ValueError):
        normalize_roles(roles)


def test_state_round_trip_contains_paths_but_no_secret_material(local_state, tmp_path: Path) -> None:
    credentials = tmp_path / "credentials"
    state = local_state(
        roles=("controller", "ui"),
        security=SecuritySettings(mode="curve", credentials_root=str(credentials)),
        network=NetworkSettings(router_host="server.local", advertise_host="laptop.local"),
        compute=ComputeSettings(gpu_mode="specific", gpu_device="GPU-lab-a"),
    )
    path = tmp_path / "state.json"
    state.save(path)

    loaded = InstallState.load(path)
    assert loaded == state
    raw = path.read_text(encoding="utf-8")
    assert str(credentials.resolve()) in raw
    assert "secret_key" not in raw
    assert "TURN_STATIC_AUTH_SECRET" not in raw
    assert path.stat().st_mode & 0o777 == 0o600


@pytest.mark.parametrize(
    ("compute", "valid"),
    (
        (ComputeSettings(), True),
        (ComputeSettings(gpu_mode="specific", gpu_device="1"), True),
        (ComputeSettings(gpu_mode="specific", gpu_device="GPU-deadbeef"), True),
        (ComputeSettings(gpu_mode="cpu"), True),
        (ComputeSettings(gpu_mode="specific"), False),
        (ComputeSettings(gpu_mode="specific", gpu_device="0,1"), False),
        (ComputeSettings(gpu_mode="specific", gpu_device="-1"), False),
        (ComputeSettings(gpu_mode="cpu", gpu_device="0"), False),
        (ComputeSettings(gpu_mode="unknown"), False),
    ),
)
def test_compute_policy_requires_exactly_one_device_only_when_pinned(
    compute: ComputeSettings,
    valid: bool,
) -> None:
    if valid:
        assert compute.validate() is compute
    else:
        with pytest.raises(ValueError):
            compute.validate()


def test_loopback_mode_rejects_remote_router(local_state) -> None:
    state = local_state(
        network=NetworkSettings(router_host="192.0.2.1", advertise_host="192.0.2.2")
    )
    with pytest.raises(ValueError, match="loopback"):
        state.validate()


def test_unknown_state_schema_is_rejected(local_state) -> None:
    raw = local_state().to_dict()
    raw["schema_version"] = 99
    with pytest.raises(ValueError, match="schema"):
        InstallState.from_dict(raw)


def test_legacy_native_state_is_migrated_without_changing_its_install_kind(
    local_state,
) -> None:
    raw = local_state().to_dict()
    raw["schema_version"] = 1
    raw.pop("install_mode", None)

    migrated = InstallState.from_dict(raw)

    assert migrated.install_mode == "native"
    assert migrated.schema_version == 3


def test_schema_v2_turn_urls_are_migrated_as_external_turn(local_state) -> None:
    raw = local_state(
        roles=("router",),
        network=NetworkSettings(
            router_host="relay.example.com",
            advertise_host="relay.example.com",
            turn_urls=("turn:relay.example.com:3478?transport=udp",),
        ),
        security=SecuritySettings(
            mode="curve",
            credentials_root="/tmp/credentials",
        ),
    ).to_dict()
    raw["schema_version"] = 2
    raw.pop("turn", None)

    migrated = InstallState.from_dict(raw)

    assert migrated.turn.mode == "external"
    assert migrated.turn.managed is False


def test_managed_turn_requires_router_curve_and_identity(local_state) -> None:
    turn = TurnSettings(
        mode="managed",
        realm="sim.example.com",
        public_host="203.0.113.10",
    )
    network = NetworkSettings(
        router_host="203.0.113.10",
        advertise_host="203.0.113.10",
        turn_urls=("turn:203.0.113.10:3478?transport=udp",),
    )
    security = SecuritySettings(mode="curve", credentials_root="/tmp/credentials")

    assert local_state(
        roles=("router",),
        network=network,
        security=security,
        turn=turn,
        install_mode="container",
    ).validate()

    with pytest.raises(ValueError, match="Router"):
        local_state(
            roles=("simulator",),
            network=network,
            security=security,
            turn=turn,
            install_mode="container",
        ).validate()

    with pytest.raises(ValueError, match="realm"):
        TurnSettings(mode="managed", public_host="203.0.113.10").validate()


def test_container_mode_rejects_generic_robot_image(local_state) -> None:
    with pytest.raises(ValueError, match="Robot.*Jetson"):
        local_state(roles=("robot",), install_mode="container").validate()


def test_router_turn_configuration_requires_curve(local_state) -> None:
    state = local_state(
        roles=("router",),
        network=NetworkSettings(
            router_host="192.0.2.10",
            advertise_host="192.0.2.10",
            turn_urls=("turn:192.0.2.10:3478?transport=udp",),
        ),
        security=SecuritySettings(mode="insecure-lan"),
        turn=TurnSettings(mode="external"),
    )
    with pytest.raises(ValueError, match="TURN.*CURVE"):
        state.validate()


@pytest.mark.parametrize(
    "network",
    [
        NetworkSettings(router_host="tcp://server:5558"),
        NetworkSettings(router_host="server:5558"),
        NetworkSettings(advertise_host="0.0.0.0"),
        NetworkSettings(controller_id="bad id"),
        NetworkSettings(turn_urls=("https://relay.example.com",)),
    ],
)
def test_network_state_rejects_values_that_cannot_be_advertised(
    local_state,
    network: NetworkSettings,
) -> None:
    with pytest.raises(ValueError):
        local_state(network=network).validate()
