from __future__ import annotations

from dataclasses import replace
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
        "roles": ["router", "simulator", "controller", "ui"],
        "prefix": str(tmp_path / "install"),
        "bin_dir": str(tmp_path / "install/bin"),
        "source_root": str(tmp_path / "source"),
        "gpu_mode": "inherit",
        "router_host": "127.0.0.1",
        "advertise_host": "127.0.0.1",
        "router_port": 5558,
        "rgbd_port": 5568,
        "security_mode": "loopback",
        "credentials_root": "",
        "credential_source": "unused",
        "turn_mode": "none",
        "turn_url": "",
        "turn_realm": "",
        "turn_public_host": "",
        "register_path": False,
        "jaeger": False,
    }


def test_general_request_translates_to_existing_install_state(tmp_path: Path) -> None:
    request = SetupRequest.from_dict(_payload(tmp_path)).validate(_capabilities())
    state = request.to_install_state()

    assert state.install_mode == "container"
    assert state.roles == ("router", "simulator", "controller", "ui")
    assert state.turn.mode == "none"


def test_robot_is_native_only_exclusive_and_requires_jetson(tmp_path: Path) -> None:
    payload = _payload(tmp_path)
    payload["roles"] = ["robot"]
    request = SetupRequest.from_dict(payload)

    assert request.validate(_capabilities(jetson=True)).to_install_state().install_mode == "native"
    with pytest.raises(ValueError, match="Jetson"):
        request.validate(_capabilities())

    payload["roles"] = ["router", "robot"]
    with pytest.raises(ValueError, match="단독"):
        SetupRequest.from_dict(payload).validate(_capabilities(jetson=True))


def test_developer_mode_requires_supported_host_and_forces_full_workspace(
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


def test_remote_curve_requires_a_credential_source(tmp_path: Path) -> None:
    payload = _payload(tmp_path)
    payload.update(
        {
            "roles": ["controller", "ui"],
            "router_host": "sim.example.com",
            "advertise_host": "laptop.example.com",
            "security_mode": "curve",
            "credentials_root": str(tmp_path / "secrets"),
        }
    )

    with pytest.raises(ValueError, match="credential"):
        SetupRequest.from_dict(payload).validate(_capabilities())

    payload["credential_source"] = "ssh"
    payload["ssh"] = {
        "host": "sim.example.com",
        "port": 2222,
        "user": "operator",
        "remote_root": "/srv/elesim/secrets",
        "identity_file": "",
        "accepted_fingerprint": "SHA256:test",
    }
    assert SetupRequest.from_dict(payload).validate(_capabilities())


def test_managed_turn_is_router_curve_only(tmp_path: Path) -> None:
    payload = _payload(tmp_path)
    payload.update(
        {
            "router_host": "203.0.113.10",
            "advertise_host": "203.0.113.10",
            "security_mode": "curve",
            "credentials_root": str(tmp_path / "secrets"),
            "credential_source": "generate",
            "turn_mode": "managed",
            "turn_url": "turn:203.0.113.10:3478?transport=udp",
            "turn_realm": "sim.example.com",
            "turn_public_host": "203.0.113.10",
        }
    )

    assert SetupRequest.from_dict(payload).validate(_capabilities())

    payload["roles"] = ["controller"]
    with pytest.raises(ValueError, match="Router"):
        SetupRequest.from_dict(payload).validate(_capabilities())


def test_required_paths_and_role_list_are_not_coerced_from_empty_values(
    tmp_path: Path,
) -> None:
    payload = _payload(tmp_path)
    payload["prefix"] = ""
    with pytest.raises(ValueError, match="prefix"):
        SetupRequest.from_dict(payload)

    payload = _payload(tmp_path)
    payload["roles"] = "router"
    with pytest.raises(ValueError, match="목록"):
        SetupRequest.from_dict(payload)
