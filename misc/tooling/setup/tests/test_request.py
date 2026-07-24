from __future__ import annotations

from pathlib import Path

import pytest

from elesim_setup.capabilities import HostCapabilities
from elesim_setup.request import SetupRequest


def _capabilities(*, jetson: bool = False) -> HostCapabilities:
    return HostCapabilities(
        architecture="aarch64" if jetson else "x86_64",
        os_id="ubuntu",
        os_version="22.04",
        jetson=jetson,
        robot_installable=jetson,
        developer_installable=not jetson,
        display_available=True,
        ssh_agent=True,
        gpu_devices=(),
    )


def _payload(tmp_path: Path) -> dict[str, object]:
    return {
        "language": "ko",
        "edition": "general",
        "roles": ["simulator", "controller", "ui"],
        "prefix": str(tmp_path / "install"),
        "bin_dir": str(tmp_path / "install/bin"),
        "source_root": str(tmp_path / "source"),
        "gpu_mode": "inherit",
        "dds_system_id": "elesim",
        "dds_domain_id": 11,
        "dds_rmw_implementation": "rmw_cyclonedds_cpp",
        "dds_discovery_mode": "multicast",
        "dds_static_peers": "",
        "dds_interface": "",
        "dds_security_profile": "trusted-network",
        "dds_keystore": "",
        "dds_enclave": "",
        "turn_mode": "none",
        "turn_url": "",
        "register_path": False,
        "jaeger": False,
    }


def test_general_request_translates_to_router_free_state(tmp_path: Path) -> None:
    request = SetupRequest.from_dict(_payload(tmp_path)).validate(_capabilities())
    state = request.to_install_state()

    assert state.install_mode == "container"
    assert state.roles == ("simulator", "controller", "ui")
    assert state.dds.domain_id == 11
    assert state.turn.mode == "none"


def test_robot_is_native_only_exclusive_and_requires_jetson(tmp_path: Path) -> None:
    payload = _payload(tmp_path)
    payload["roles"] = ["robot"]
    request = SetupRequest.from_dict(payload)

    assert (
        request.validate(_capabilities(jetson=True))
        .to_install_state()
        .install_mode
        == "native"
    )
    with pytest.raises(ValueError, match="Jetson"):
        request.validate(_capabilities())

    payload["roles"] = ["simulator", "robot"]
    with pytest.raises(ValueError, match="단독"):
        SetupRequest.from_dict(payload).validate(_capabilities(jetson=True))


def test_developer_mode_requires_supported_host_and_has_no_runtime_roles(
    tmp_path: Path,
) -> None:
    payload = _payload(tmp_path)
    payload.update({"edition": "developer", "roles": [], "jaeger": True})
    request = SetupRequest.from_dict(payload)

    assert request.validate(_capabilities()).jaeger is True
    with pytest.raises(ValueError, match="amd64"):
        request.validate(_capabilities(jetson=True))
    with pytest.raises(ValueError, match="runtime InstallState"):
        request.to_install_state()


def test_sros2_requires_keystore_and_enclave(tmp_path: Path) -> None:
    payload = _payload(tmp_path)
    payload["dds_security_profile"] = "sros2"

    with pytest.raises(ValueError, match="keystore"):
        SetupRequest.from_dict(payload).validate(_capabilities())

    payload["dds_keystore"] = str(tmp_path / "sros2")
    payload["dds_enclave"] = "/elesim"
    request = SetupRequest.from_dict(payload).validate(_capabilities())
    assert request.dds.security_profile == "sros2"


def test_static_discovery_requires_explicit_peer(tmp_path: Path) -> None:
    payload = _payload(tmp_path)
    payload["dds_discovery_mode"] = "static"

    with pytest.raises(ValueError, match="peer"):
        SetupRequest.from_dict(payload).validate(_capabilities())

    payload["dds_static_peers"] = "192.0.2.10, sim.example.com"
    request = SetupRequest.from_dict(payload).validate(_capabilities())
    assert request.dds.static_peers == ("192.0.2.10", "sim.example.com")


def test_managed_turn_is_simulator_owned_and_requires_sros2(tmp_path: Path) -> None:
    payload = _payload(tmp_path)
    payload.update(
        {
            "turn_mode": "managed",
            "turn_url": "turn:203.0.113.10:3478?transport=udp",
            "turn_realm": "sim.example.com",
            "turn_public_host": "203.0.113.10",
            "dds_security_profile": "sros2",
            "dds_keystore": str(tmp_path / "keystore"),
            "dds_enclave": "/elesim",
        }
    )

    request = SetupRequest.from_dict(payload).validate(_capabilities())
    assert request.turn.secret_file == str(
        (tmp_path / "install/secrets/turn.secret").resolve()
    )

    payload["roles"] = ["controller"]
    with pytest.raises(ValueError, match="Simulator"):
        SetupRequest.from_dict(payload).validate(_capabilities())

    payload["roles"] = ["simulator"]
    payload["dds_security_profile"] = "trusted-network"
    payload["dds_keystore"] = ""
    payload["dds_enclave"] = ""
    with pytest.raises(ValueError, match="sros2"):
        SetupRequest.from_dict(payload).validate(_capabilities())


def test_external_turn_credentials_are_required_only_on_simulator_host(
    tmp_path: Path,
) -> None:
    payload = _payload(tmp_path)
    payload.update(
        {
            "turn_mode": "external",
            "turn_url": "turn:relay.example.com:3478?transport=udp",
        }
    )
    with pytest.raises(ValueError, match="credential"):
        SetupRequest.from_dict(payload).validate(_capabilities())

    credentials = tmp_path / "turn.credentials.json"
    payload["turn_credential_file"] = str(credentials)
    request = SetupRequest.from_dict(payload).validate(_capabilities())
    assert request.turn.credential_path == credentials.resolve()

    payload["roles"] = ["controller", "ui"]
    payload["turn_credential_file"] = ""
    request = SetupRequest.from_dict(payload).validate(_capabilities())
    assert request.turn.credential_path is None


def test_ssh_port_is_preserved_but_does_not_become_a_dds_setting(
    tmp_path: Path,
) -> None:
    payload = _payload(tmp_path)
    payload["ssh"] = {
        "host": "server.example",
        "port": 2222,
        "user": "operator",
        "remote_root": "/srv/elesim",
    }

    request = SetupRequest.from_dict(payload).validate(_capabilities())

    assert request.ssh.port == 2222
    assert request.dds.static_peers == ()
    assert "2222" not in str(request.to_install_state().to_dict()["dds"])


def test_required_paths_and_role_list_are_not_coerced(
    tmp_path: Path,
) -> None:
    payload = _payload(tmp_path)
    payload["prefix"] = ""
    with pytest.raises(ValueError, match="prefix"):
        SetupRequest.from_dict(payload)

    payload = _payload(tmp_path)
    payload["roles"] = "simulator"
    with pytest.raises(ValueError, match="목록"):
        SetupRequest.from_dict(payload)
