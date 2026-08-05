from __future__ import annotations

from pathlib import Path

import pytest

from elesim_setup.connection_manager import (
    ConnectionTopology,
    DdsEndpoint,
    ManagedHost,
    RoleAssignment,
    SshEndpoint,
)
from elesim_setup.connection_gui import ConnectionJobCancelled
from elesim_setup.connections import ConnectionDeploymentRunner


FINGERPRINT = "SHA256:" + "A" * 43


def _topology(tmp_path: Path, *, security_profile: str) -> ConnectionTopology:
    prefix = tmp_path / "install"
    return ConnectionTopology(
        system_id="lab",
        security_profile=security_profile,
        hosts=(
            ManagedHost(
                host_id="operator",
                display_name="COM1",
                local=True,
                dds=DdsEndpoint("10.0.0.10", "eth0"),
                ssh=None,
                assignments=(
                    RoleAssignment("pilot", "pilot-main"),
                    RoleAssignment("sim", "sim-main"),
                    RoleAssignment("ui", "ui-main"),
                ),
                install_root=str(prefix),
                bin_dir=str(prefix / "bin"),
            ),
            ManagedHost(
                host_id="jetson",
                display_name="Robot",
                local=False,
                dds=DdsEndpoint("10.0.0.20", "eth0"),
                ssh=SshEndpoint(
                    "robot.example",
                    2222,
                    "robot",
                    "",
                    FINGERPRINT,
                ),
                assignments=(RoleAssignment("robot", "robot-go2"),),
                install_mode="native",
                jetson=True,
                install_root="/opt/elesim",
                bin_dir="/usr/local/bin",
                lifecycle="systemd",
            ),
        ),
    ).validate()


def test_trusted_network_runner_applies_bundle_free_topology(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    topology = _topology(tmp_path, security_profile="trusted-network")
    events: list[str] = []
    applied: list[object] = []

    class FakeRollout:
        def __init__(self, received, operations) -> None:
            assert received is topology
            assert set(operations) == {"operator", "jetson"}

        def apply(self, *, progress) -> None:
            applied.append(progress)
            progress("verify", "operator")

    monkeypatch.setattr(
        ConnectionDeploymentRunner,
        "_operations",
        staticmethod(lambda _topology: {"operator": object(), "jetson": object()}),
    )
    monkeypatch.setattr("elesim_setup.connections.TopologyRollout", FakeRollout)

    runner = ConnectionDeploymentRunner(
        tmp_path / "authority",
        local_install_root=tmp_path / "install",
    )
    runner(topology, "deploy", events.append)

    assert applied
    assert "verify: COM1 (operator)" in events


def test_trusted_network_runner_ignores_cancel_after_rollout_commit(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    topology = _topology(tmp_path, security_profile="trusted-network")

    class FakeRollout:
        def __init__(self, _received, _operations) -> None:
            pass

        def apply(self, *, progress) -> None:
            progress("verify", "operator")

    monkeypatch.setattr(
        ConnectionDeploymentRunner,
        "_operations",
        staticmethod(lambda _topology: {"operator": object(), "jetson": object()}),
    )
    monkeypatch.setattr("elesim_setup.connections.TopologyRollout", FakeRollout)
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority",
        local_install_root=tmp_path / "install",
    )
    events: list[str] = []

    def cancel_on_completion(message: str) -> None:
        if "검증이 끝났습니다" in message:
            raise ConnectionJobCancelled("too late")
        events.append(message)

    runner(topology, "deploy", cancel_on_completion)

    assert events[-1] == "verify: COM1 (operator)"


def test_trusted_network_rejects_security_actions(tmp_path: Path) -> None:
    topology = _topology(tmp_path, security_profile="trusted-network")
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority",
        local_install_root=tmp_path / "install",
    )

    with pytest.raises(ValueError, match="deploy"):
        runner(topology, "rotate", lambda _message: None)


def test_sros2_provision_rejects_an_existing_active_generation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    topology = _topology(tmp_path, security_profile="sros2")

    class FakeAuthority:
        def __init__(self, path: Path) -> None:
            assert path == (tmp_path / "authority/lab").resolve()

        @staticmethod
        def active() -> object:
            return object()

    monkeypatch.setattr("elesim_setup.connections.Sros2Authority", FakeAuthority)
    monkeypatch.setattr(
        ConnectionDeploymentRunner,
        "_operations",
        staticmethod(lambda _topology: {"operator": object(), "jetson": object()}),
    )
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority",
        local_install_root=tmp_path / "install",
    )

    with pytest.raises(ValueError, match="provision/deploy"):
        runner(topology, "provision", lambda _message: None)


def test_runner_rejects_local_install_root_mismatch(tmp_path: Path) -> None:
    topology = _topology(tmp_path, security_profile="trusted-network")
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority",
        local_install_root=tmp_path / "other-install",
    )

    with pytest.raises(ValueError, match="install_root"):
        runner(topology, "deploy", lambda _message: None)


def test_runner_validates_tilde_identity_against_operator_home(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    operator_home = tmp_path / "host-home"
    identity = operator_home / ".ssh/id_ed25519"
    identity.parent.mkdir(parents=True)
    identity.write_text("not-a-real-test-key", encoding="utf-8")
    identity.chmod(0o600)
    monkeypatch.setenv("HOME", str(tmp_path / "generated-container-home"))
    monkeypatch.setenv("ELESIM_OPERATOR_HOME", str(operator_home))
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority",
        local_install_root=tmp_path / "install",
    )

    for configured in ("~/.ssh/id_ed25519", str(identity)):
        raw = _topology(tmp_path, security_profile="trusted-network").to_dict()
        raw["hosts"][1]["ssh"]["identity_file"] = configured
        topology = ConnectionTopology.from_dict(raw)

        runner._validate_management_host(topology)


def test_runner_does_not_require_a_private_file_for_tailscale_ssh(
    tmp_path: Path,
) -> None:
    raw = _topology(tmp_path, security_profile="trusted-network").to_dict()
    raw["hosts"][1]["ssh"].update(
        {"port": 22, "identity_file": "", "auth_mode": "tailscale"}
    )
    topology = ConnectionTopology.from_dict(raw)
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority",
        local_install_root=tmp_path / "install",
    )

    runner._validate_management_host(topology)
