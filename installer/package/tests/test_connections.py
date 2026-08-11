from __future__ import annotations

import json
from dataclasses import replace
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
from elesim_setup.secure_deployment import RuntimeLaunchOptions


FINGERPRINT = "SHA256:" + "A" * 43


class _NoopNetworkPreparation:
    @staticmethod
    def prepare_runtime_network(_host, _output):
        return None


def _topology(tmp_path: Path, *, security_profile: str) -> ConnectionTopology:
    prefix = tmp_path / "install"
    return ConnectionTopology(
        system_id="lab",
        security_profile=security_profile,
        hosts=(
            ManagedHost(
                host_id="operator",
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
        staticmethod(
            lambda _topology: {
                "operator": _NoopNetworkPreparation(),
                "jetson": _NoopNetworkPreparation(),
            }
        ),
    )
    monkeypatch.setattr("elesim_setup.connections.TopologyRollout", FakeRollout)

    runner = ConnectionDeploymentRunner(
        tmp_path / "authority",
        local_install_root=tmp_path / "install",
    )
    runner(topology, "deploy", events.append)

    assert applied
    assert "verify: operator" in events


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
        staticmethod(
            lambda _topology: {
                "operator": _NoopNetworkPreparation(),
                "jetson": _NoopNetworkPreparation(),
            }
        ),
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

    assert events[-1] == "verify: operator"


def test_trusted_network_rejects_security_actions_before_host_operations(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    topology = _topology(tmp_path, security_profile="trusted-network")
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority",
        local_install_root=tmp_path / "install",
    )
    monkeypatch.setattr(
        ConnectionDeploymentRunner,
        "_operations",
        staticmethod(
            lambda _topology: (_ for _ in ()).throw(
                AssertionError("host operations must not be created")
            )
        ),
    )

    with pytest.raises(ValueError, match="deploy"):
        runner(topology, "rotate", lambda _message: None)

    with pytest.raises(ValueError, match="지원하지 않는 연결 작업"):
        runner(topology, "unknown", lambda _message: None)


def test_runtime_start_builds_every_host_before_launching_any_host(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    topology = _topology(tmp_path, security_profile="trusted-network")
    events: list[str] = []
    logs: list[str] = []
    received_options: list[RuntimeLaunchOptions | None] = []

    class Operations(_NoopNetworkPreparation):
        def __init__(self, host_id: str) -> None:
            self.host_id = host_id

        def build(self, _host, output) -> None:
            events.append(f"build:{self.host_id}")
            output("stdout", f"{self.host_id}-step-1\n")
            output("stderr", f"{self.host_id}-step-2\n")

        def preflight(self, _host):
            events.append(f"preflight:{self.host_id}")

            class Capabilities:
                @staticmethod
                def require_for(_managed_host) -> None:
                    return None

            return Capabilities()

        def runtime_network_check(self, _host) -> None:
            events.append(f"network-check:{self.host_id}")

        def launch(self, _host, runtime_options=None) -> None:
            received_options.append(runtime_options)
            events.append(f"launch:{self.host_id}")

        @staticmethod
        def runtime_doctor(_host, _expected_peer_ids, *, timeout_s):
            assert timeout_s == 8
            return {"ok": True, "results": []}

        def stop(self, _host) -> None:
            events.append(f"stop:{self.host_id}")

    monkeypatch.setattr(
        ConnectionDeploymentRunner,
        "_operations",
        staticmethod(
            lambda graph: {
                host.host_id: Operations(host.host_id) for host in graph.hosts
            }
        ),
    )
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority",
        local_install_root=tmp_path / "install",
    )

    options = RuntimeLaunchOptions(True, "1", True)
    runner.set_runtime_launch_options(options)
    runner(topology, "start", logs.append)

    assert events == [
        "network-check:operator",
        "preflight:operator",
        "network-check:jetson",
        "preflight:jetson",
        "build:operator",
        "build:jetson",
        "launch:operator",
        "launch:jetson",
    ]
    assert "build operator [stdout] operator-step-1" in logs
    assert "build operator [stderr] operator-step-2" in logs
    assert "build 완료: operator" in logs
    assert "build 완료: jetson" in logs
    assert received_options == [options, options]


def test_runtime_start_reports_dds_readiness_after_launch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    topology = _topology(tmp_path, security_profile="trusted-network")
    events: list[str] = []
    logs: list[str] = []

    class Operations(_NoopNetworkPreparation):
        def __init__(self, host_id: str) -> None:
            self.host_id = host_id

        def build(self, _host, _output) -> None:
            return None

        def preflight(self, _host):
            class Capabilities:
                @staticmethod
                def require_for(_managed_host) -> None:
                    return None

            return Capabilities()

        def runtime_network_check(self, _host) -> None:
            return None

        def launch(self, _host) -> None:
            events.append(f"launch:{self.host_id}")

        def runtime_doctor(self, _host, expected_peer_ids, *, timeout_s):
            events.append(
                f"doctor:{self.host_id}:{','.join(expected_peer_ids)}:{timeout_s:g}"
            )
            return {"ok": True, "results": []}

        def stop(self, _host) -> None:
            return None

    monkeypatch.setattr(
        ConnectionDeploymentRunner,
        "_operations",
        staticmethod(
            lambda graph: {
                host.host_id: Operations(host.host_id) for host in graph.hosts
            }
        ),
    )
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority",
        local_install_root=tmp_path / "install",
    )

    runner(topology, "start", logs.append)

    assert events == [
        "launch:operator",
        "launch:jetson",
        "doctor:operator:pilot-main,robot-go2,sim-main,ui-main:8",
        "doctor:jetson:pilot-main,robot-go2,sim-main,ui-main:8",
    ]
    assert any("DDS endpoint 준비 상태" in message for message in logs)
    assert any("endpoint descriptor/heartbeat 확인" in message for message in logs)


def test_runtime_readiness_fails_on_malformed_results_payload(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    topology = _topology(tmp_path, security_profile="trusted-network")
    logs: list[str] = []

    class Operations(_NoopNetworkPreparation):
        def __init__(self, host_id: str) -> None:
            self.host_id = host_id

        def build(self, _host, _output) -> None:
            return None

        def preflight(self, _host):
            class Capabilities:
                @staticmethod
                def require_for(_managed_host) -> None:
                    return None

            return Capabilities()

        def runtime_network_check(self, _host) -> None:
            return None

        def launch(self, _host) -> None:
            return None

        def runtime_doctor(self, _host, _expected_peer_ids, *, timeout_s):
            assert timeout_s == 8
            return {"ok": False, "results": None}

        def stop(self, _host) -> None:
            return None

    monkeypatch.setattr(
        ConnectionDeploymentRunner,
        "_operations",
        staticmethod(
            lambda graph: {
                host.host_id: Operations(host.host_id) for host in graph.hosts
            }
        ),
    )
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority",
        local_install_root=tmp_path / "install",
    )

    with pytest.raises(RuntimeError, match="DDS readiness failed"):
        runner(topology, "start", logs.append)

    assert any("expected endpoint가 아직 발견되지 않음" in message for message in logs)


def test_runtime_readiness_accepts_multi_unit_doctor_envelope() -> None:
    report = {
        "state": "ready",
        "units": {
            "compose": {"ok": True, "results": []},
            "robot": {"ok": True, "results": []},
        },
    }

    assert ConnectionDeploymentRunner._runtime_report_ok(report)
    assert (
        ConnectionDeploymentRunner._runtime_report_detail(report)
        == "expected endpoint가 아직 발견되지 않음"
    )


def test_runtime_readiness_reports_failed_unit_from_multi_unit_envelope() -> None:
    report = {
        "state": "ready",
        "units": {
            "compose": {"ok": True, "results": []},
            "robot": {
                "ok": False,
                "results": [
                    {
                        "name": "DDS peers",
                        "detail": "heartbeat 없음: sim-main",
                    }
                ],
            },
        },
    }

    assert not ConnectionDeploymentRunner._runtime_report_ok(report)
    assert (
        ConnectionDeploymentRunner._runtime_report_detail(report)
        == "robot: heartbeat 없음: sim-main"
    )


def test_host_check_combines_network_preflight_and_runtime_status(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    topology = _topology(tmp_path, security_profile="trusted-network")
    events: list[str] = []
    logs: list[str] = []

    class Capabilities:
        @staticmethod
        def require_for(_host) -> None:
            events.append("require")

    class Operations:
        def __init__(self, host_id: str) -> None:
            self.host_id = host_id

        def runtime_network_check(self, _host) -> None:
            events.append(f"network-check:{self.host_id}")

        def preflight(self, _host):
            events.append(f"preflight:{self.host_id}")
            return Capabilities()

        def status(self, _host):
            events.append(f"status:{self.host_id}")
            return {"state": "stopped", "running_roles": []}

    monkeypatch.setattr(
        ConnectionDeploymentRunner,
        "_operations",
        staticmethod(
            lambda graph: {
                host.host_id: Operations(host.host_id) for host in graph.hosts
            }
        ),
    )
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority",
        local_install_root=tmp_path / "install",
    )

    runner(topology, "check", logs.append)

    assert events == [
        "network-check:operator",
        "preflight:operator",
        "require",
        "status:operator",
        "network-check:jetson",
        "preflight:jetson",
        "require",
        "status:jetson",
    ]
    assert "status: operator = stopped [—]" in logs
    assert "status: jetson = stopped [—]" in logs


def test_start_persists_changed_sidecar_address_and_requires_prepare(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    topology = _topology(tmp_path, security_profile="trusted-network")
    state_path = tmp_path / "topology.json"
    topology.save(state_path)

    class Operations(_NoopNetworkPreparation):
        @staticmethod
        def prepare_runtime_network(host, _output):
            return "100.64.0.99" if host.host_id == "operator" else None

    monkeypatch.setattr(
        ConnectionDeploymentRunner,
        "_operations",
        staticmethod(
            lambda graph: {
                host.host_id: Operations() for host in graph.hosts
            }
        ),
    )
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority",
        topology_state_path=state_path,
        local_install_root=tmp_path / "install",
    )

    with pytest.raises(RuntimeError, match="보안 및 실행 준비"):
        runner(topology, "start", lambda _message: None)

    saved = ConnectionTopology.load(state_path)
    assert saved.host("operator").dds.address == "100.64.0.99"
    assert saved.host("operator").dds.interface == "tailscale0"
    assert saved.dds_graph.discovery_mode == "static"
    assert saved.host("jetson").ssh == topology.host("jetson").ssh


def test_deploy_persists_sidecar_address_before_remote_configuration(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    topology = _topology(tmp_path, security_profile="trusted-network")
    topology = replace(
        topology,
        dds_graph=replace(topology.dds_graph, discovery_mode="static"),
    ).validate()
    state_path = tmp_path / "topology.json"
    topology.save(state_path)
    observed: list[str] = []

    class Operations(_NoopNetworkPreparation):
        @staticmethod
        def prepare_runtime_network(host, _output):
            return "100.64.0.99" if host.host_id == "operator" else None

    class Rollout:
        def __init__(self, updated, _operations) -> None:
            assert updated.host("operator").dds.address == "100.64.0.99"
            assert ConnectionTopology.load(state_path) == updated

        def apply(self, *, progress) -> None:
            observed.append("apply")
            progress("verify", "operator")

    monkeypatch.setattr(
        ConnectionDeploymentRunner,
        "_operations",
        staticmethod(
            lambda graph: {
                host.host_id: Operations() for host in graph.hosts
            }
        ),
    )
    monkeypatch.setattr("elesim_setup.connections.TopologyRollout", Rollout)
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority",
        topology_state_path=state_path,
        local_install_root=tmp_path / "install",
    )

    updated = runner(topology, "deploy", lambda _message: None)

    assert observed == ["apply"]
    assert updated.host("operator").dds.address == "100.64.0.99"


def test_deploy_rejects_changed_sidecar_address_without_a_state_path(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    topology = _topology(tmp_path, security_profile="trusted-network")
    topology = replace(
        topology,
        dds_graph=replace(topology.dds_graph, discovery_mode="static"),
    ).validate()

    class Operations(_NoopNetworkPreparation):
        @staticmethod
        def prepare_runtime_network(host, _output):
            return "100.64.0.99" if host.host_id == "operator" else None

    monkeypatch.setattr(
        ConnectionDeploymentRunner,
        "_operations",
        staticmethod(
            lambda graph: {
                host.host_id: Operations() for host in graph.hosts
            }
        ),
    )
    monkeypatch.setattr(
        "elesim_setup.connections.TopologyRollout",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("remote rollout must not begin before persistence")
        ),
    )
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority",
        local_install_root=tmp_path / "install",
    )

    with pytest.raises(RuntimeError, match="topology state path"):
        runner(topology, "deploy", lambda _message: None)


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
        staticmethod(
            lambda _topology: {
                "operator": _NoopNetworkPreparation(),
                "jetson": _NoopNetworkPreparation(),
            }
        ),
    )
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority",
        local_install_root=tmp_path / "install",
    )

    with pytest.raises(ValueError, match="provision/deploy"):
        runner(topology, "provision", lambda _message: None)


@pytest.mark.parametrize(
    ("has_active_generation", "expected_action"),
    ((False, "provision"), (True, "rotate")),
)
def test_sros2_prepare_selects_create_or_reissue_automatically(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    has_active_generation: bool,
    expected_action: str,
) -> None:
    topology = _topology(tmp_path, security_profile="sros2")
    observed: list[tuple[str, str]] = []

    class FakeAuthority:
        def __init__(self, path: Path) -> None:
            assert path == (tmp_path / "authority/lab").resolve()

        def active(self) -> object | None:
            return object() if has_active_generation else None

    class FakeRollout:
        def __init__(self, _topology, _operations) -> None:
            pass

        def issue_and_apply(self, issuer, generation: str, *, progress) -> None:
            observed.append((expected_action, generation))
            progress("verify", "operator")

    monkeypatch.setattr("elesim_setup.connections.Sros2Authority", FakeAuthority)
    monkeypatch.setattr("elesim_setup.connections.GenerationRollout", FakeRollout)
    monkeypatch.setattr(
        "elesim_setup.connections.new_generation_id",
        lambda: "g-20260807t000000000000z-abcdef123456",
    )
    monkeypatch.setattr(
        ConnectionDeploymentRunner,
        "_operations",
        staticmethod(
            lambda _topology: {
                "operator": _NoopNetworkPreparation(),
                "jetson": _NoopNetworkPreparation(),
            }
        ),
    )

    runner = ConnectionDeploymentRunner(
        tmp_path / "authority",
        local_install_root=tmp_path / "install",
    )
    runner(topology, "prepare", lambda _message: None)

    assert observed == [
        (expected_action, "g-20260807t000000000000z-abcdef123456")
    ]
    journal = json.loads(
        (tmp_path / "authority/lab/transactions/latest.json").read_text(
            encoding="utf-8"
        )
    )
    assert journal["action"] == expected_action
    assert journal["status"] == "completed"


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
