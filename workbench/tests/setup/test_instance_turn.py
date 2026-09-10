from __future__ import annotations

from pathlib import Path

import pytest

from elesim_setup.container_installer import ContainerInstaller
from elesim_setup.instance_identity import service_key
from elesim_setup.instance_runtime import InstanceRuntime
from elesim_setup.instances import (
    InstanceEndpoint,
    InstanceState,
    scoped_turn_settings,
    turn_service_key,
)
from elesim_setup.state import (
    ContainerNetworkSettings,
    DdsSettings,
    NetworkSettings,
    TurnSettings,
)


INSTALL = "01234567-89ab-cdef-0123-456789abcdef"
KEY = "a" * 64


def _turn(system: str, secret: Path, **ports: int | None) -> TurnSettings:
    return TurnSettings(
        mode="managed",
        realm=system,
        public_host=f"{system}.example.test",
        secret_file=str(secret),
        **ports,
    )


def _instance(system: str, turn: TurnSettings) -> InstanceState:
    return InstanceState(
        system,
        KEY,
        (
            InstanceEndpoint("sim", f"{system}-sim"),
            InstanceEndpoint("pilot", f"{system}-pilot"),
        ),
        10 if system == "alpha" else 11,
        security_profile="sros2",
        security_generation="g1",
        turn=turn,
    )


def test_scoped_turn_allocator_is_deterministic_and_emits_scoped_service(tmp_path):
    alpha = _instance("alpha", _turn("alpha", tmp_path / "alpha.secret"))
    beta = _instance("beta", _turn("beta", tmp_path / "beta.secret"))
    assert scoped_turn_settings(alpha) == scoped_turn_settings(alpha)
    assert scoped_turn_settings(alpha).effective_listen_port != scoped_turn_settings(beta).effective_listen_port
    assert scoped_turn_settings(alpha).effective_relay_min_port != scoped_turn_settings(beta).effective_relay_min_port
    for system in (alpha, beta):
        allocated = scoped_turn_settings(system)
        assert not (
            allocated.effective_relay_min_port <= 49200
            and allocated.effective_relay_max_port >= 49160
        )
    assert turn_service_key("alpha") == service_key("alpha", "coturn")


def test_scoped_turn_allocator_includes_install_identity_for_host_networks(tmp_path):
    alpha = _instance("alpha", _turn("alpha", tmp_path / "alpha.secret"))
    first = scoped_turn_settings(alpha, INSTALL)
    second = scoped_turn_settings(
        alpha, "fedcba98-7654-3210-fedc-ba9876543210"
    )
    assert first != second
    assert first.effective_listen_port != second.effective_listen_port
    assert first.effective_relay_min_port != second.effective_relay_min_port


def test_scoped_turn_allocator_rejects_noncanonical_install_identity(tmp_path):
    alpha = _instance("alpha", _turn("alpha", tmp_path / "alpha.secret"))
    with pytest.raises(ValueError, match="canonical"):
        scoped_turn_settings(alpha, INSTALL.upper())


def test_scoped_turn_explicit_listener_collision_fails_closed(local_state, tmp_path):
    state = local_state(
        roles=("sim",),
        dds=DdsSettings(security_profile="sros2", security_provisioning="managed"),
        container_network=ContainerNetworkSettings(
            mode="direct-host", docker_context="default", docker_engine_id="engine"
        ),
    )
    runtime = InstanceRuntime(state, INSTALL)
    runtime._load_release = lambda instance: object()  # type: ignore[method-assign]
    same = {"listen_port": 50001, "relay_min_port": 42000, "relay_max_port": 42039}
    alpha = _instance("alpha", _turn("alpha", tmp_path / "alpha.secret", **same))
    beta = _instance("beta", _turn("beta", tmp_path / "beta.secret", **same))
    with pytest.raises(ValueError, match="collides"):
        runtime._validate_release_set({"alpha": alpha, "beta": beta})


def test_scoped_turn_ignores_inert_legacy_install_ranges(local_state, tmp_path):
    state = local_state(
        roles=("sim",),
        dds=DdsSettings(security_profile="sros2", security_provisioning="managed"),
        network=NetworkSettings(turn_urls=("turn:legacy.example.test:3478",)),
        container_network=ContainerNetworkSettings(
            mode="direct-host", docker_context="default", docker_engine_id="engine"
        ),
        turn=_turn("legacy", tmp_path / "legacy.secret"),
    )
    runtime = InstanceRuntime(state, INSTALL)
    runtime._load_release = lambda instance: object()  # type: ignore[method-assign]

    # These are deliberately the historical install-global defaults.  They
    # are not an active service in the scoped instance namespace and must not
    # reject the first registered instance.
    instance = _instance(
        "alpha",
        _turn(
            "alpha",
            tmp_path / "alpha.secret",
            listen_port=3478,
            relay_min_port=49160,
            relay_max_port=49200,
        ),
    )
    validated = runtime._validate_release_set({"alpha": instance})
    assert set(validated) == {"alpha"}


def test_scoped_coturn_uses_system_identity_and_ports(local_state, tmp_path):
    state = local_state(
        roles=("sim",),
        dds=DdsSettings(
            system_id="alpha",
            security_profile="sros2",
            security_provisioning="managed",
        ),
    )
    installer = ContainerInstaller(state, state_path=state.state_path, dry_run=True)
    installer._install_uuid = INSTALL
    turn = _turn("alpha", tmp_path / "alpha.secret", listen_port=50001, relay_min_port=42000, relay_max_port=42039)
    key = turn_service_key("alpha")
    service = installer._coturn_service(
        instance_scoped=True,
        service_key=key,
        sim_service_key=service_key("alpha", "alpha-sim"),
        turn=turn,
    )
    assert service["container_name"].startswith(
        f"elesim-{INSTALL.replace('-', '')}-"
    )
    assert service["labels"]["io.elesim.system_id"] == "alpha"
    assert service["labels"]["io.elesim.service_kind"] == "coturn"
    command = service["command"][0]
    assert "--listening-port=50001" in command
    assert "--min-port=42000 --max-port=42039" in command
    assert service["depends_on"] == (service_key("alpha", "alpha-sim"),)
