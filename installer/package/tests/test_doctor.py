from __future__ import annotations

import os
import struct
import sys
import types
from types import SimpleNamespace

import pytest

from elesim_setup.doctor import (
    FAIL,
    PASS,
    SKIP,
    WARN,
    DdsGraphSnapshot,
    DdsPeerProbe,
    DoctorReport,
    NetworkDoctor,
    _prepare_dds_environment,
    build_stun_binding_request,
    parse_tcp_endpoint,
    parse_turn_url,
    probe_dds_peer_state,
    validate_stun_response,
)
from elesim_setup.network import detect_tailscale
from elesim_setup.state import DdsSettings, NetworkSettings, TurnSettings


def test_parse_tcp_endpoint_supports_ipv4_hostname_and_ipv6() -> None:
    assert parse_tcp_endpoint("tcp://server.example:5558").host == "server.example"
    assert parse_tcp_endpoint("tcp://[2001:db8::1]:5568").port == 5568
    with pytest.raises(ValueError):
        parse_tcp_endpoint("udp://server:5558")


def test_tailscale_probe_is_read_only_and_returns_current_ipv4() -> None:
    class Result:
        returncode = 0
        stdout = (
            '[{"ifname":"tailscale0","addr_info":['
            '{"family":"inet","local":"100.64.0.10"}]}]'
        )

    calls: list[tuple[object, dict[str, object]]] = []

    def runner(command, **kwargs):
        calls.append((command, kwargs))
        return Result()

    detection = detect_tailscale(runner=runner)

    assert detection.available is True
    assert detection.interface == "tailscale0"
    assert detection.addresses == ("100.64.0.10",)
    assert calls[0][0] == ["ip", "-j", "-4", "addr", "show"]
    assert calls[0][1]["check"] is False


def test_tailscale_probe_accepts_reconnect_suffixes_and_prefers_tailscale0() -> None:
    class Result:
        returncode = 0
        stdout = (
            '[{"ifname":"tailscale1","addr_info":['
            '{"family":"inet","local":"100.100.0.2"}]},'
            '{"ifname":"tailscale0","addr_info":['
            '{"family":"inet","local":"100.100.0.1"}]}]'
        )

    detection = detect_tailscale(runner=lambda *_args, **_kwargs: Result())

    assert detection.available is True
    assert detection.interface == "tailscale0"
    assert detection.addresses == ("100.100.0.1", "100.100.0.2")


@pytest.mark.parametrize(
    "url,expected",
    [
        ("turn:relay.example.com", ("relay.example.com", 3478, "udp")),
        (
            "turn:relay.example.com:443?transport=tcp",
            ("relay.example.com", 443, "tcp"),
        ),
        ("turns://relay.example.com", ("relay.example.com", 5349, "tcp")),
    ],
)
def test_turn_url_parser(url: str, expected: tuple[str, int, str]) -> None:
    target = parse_turn_url(url)
    assert (target.host, target.port, target.transport) == expected


def test_stun_response_validation_checks_transaction() -> None:
    request, transaction_id = build_stun_binding_request(b"x" * 12)
    assert len(request) == 20
    response = struct.pack("!HHI12s", 0x0101, 0, 0x2112A442, transaction_id)
    validate_stun_response(response, transaction_id)
    with pytest.raises(ValueError, match="transaction"):
        validate_stun_response(response, b"y" * 12)


def test_report_warn_and_skip_do_not_make_it_fail() -> None:
    report = DoctorReport()
    report.add("one", PASS, "ok")
    report.add("two", WARN, "warning")
    report.add("three", SKIP, "skipped")
    assert report.ok
    report.add("four", FAIL, "bad")
    assert not report.ok


def test_doctor_reports_dds_graph_rgbd_and_webrtc_control_carrier(
    local_state,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    state = local_state(roles=("sim", "pilot"))
    graph = DdsGraphSnapshot(
        nodes=("/elesim/v6/sim", "/elesim/v6/pilot"),
        topics={
            "/elesim/sim_default/rgbd/frame": (
                "elesim_interfaces/msg/RgbdFrame",
            ),
            "/elesim/v6/peers/sim_default/boot/control": (
                "elesim_interfaces/msg/PeerEnvelope",
            ),
        },
        services={},
    )
    monkeypatch.setattr(
        "elesim_setup.doctor.probe_dds_graph",
        lambda *_args, **_kwargs: graph,
    )

    report = NetworkDoctor(state).run()
    by_name = {result.name: result for result in report.results}

    assert by_name["DDS graph"].status == PASS
    assert by_name["RGBD topic"].status == PASS
    assert by_name["RGBD frame"].status == SKIP
    assert by_name["WebRTC signaling"].status == PASS
    assert report.ok


def test_doctor_reports_expected_peer_descriptors(
    local_state,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    graph = DdsGraphSnapshot(nodes=("/elesim/v6/sim",), topics={}, services={})
    monkeypatch.setattr("elesim_setup.doctor.probe_dds_graph", lambda *_args, **_kwargs: graph)
    monkeypatch.setattr(
        "elesim_setup.doctor.probe_dds_peer_state",
        lambda *_args, **_kwargs: DdsPeerProbe(
            descriptors=("sim-default",),
            heartbeats=("sim-default",),
        ),
    )

    report = NetworkDoctor(
        local_state(),
        expected_peers=("sim-default",),
        strict_peers=True,
    ).run()

    result = next(item for item in report.results if item.name == "DDS peers")
    assert result.status == PASS
    assert report.ok


def test_readiness_only_doctor_runs_only_strict_peer_probe(
    local_state,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    observed: list[tuple[float, tuple[str, ...]]] = []

    def peer_probe(_state, *, timeout_s, expected_peers):
        observed.append((timeout_s, tuple(expected_peers)))
        return DdsPeerProbe(
            descriptors=("sim-default",),
            heartbeats=("sim-default",),
        )

    monkeypatch.setattr("elesim_setup.doctor.probe_dds_peer_state", peer_probe)
    monkeypatch.setattr(
        "elesim_setup.doctor.probe_dds_graph",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("readiness must not inspect the full DDS graph")
        ),
    )

    report = NetworkDoctor(
        local_state(),
        timeout_s=60,
        expected_peers=("sim-default",),
        strict_peers=True,
        readiness_only=True,
    ).run()

    assert report.ok
    assert [result.name for result in report.results] == ["DDS peers"]
    assert observed == [(60.0, ("sim-default",))]


def test_readiness_only_doctor_requires_strict_expected_peers(local_state) -> None:
    with pytest.raises(ValueError, match="strict peer"):
        NetworkDoctor(local_state(), readiness_only=True)
    with pytest.raises(ValueError, match="expected peers"):
        NetworkDoctor(local_state(), strict_peers=True, readiness_only=True)


def test_peer_probe_returns_as_soon_as_all_expected_peers_are_live(
    local_state,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class EndpointDescriptor:
        pass

    class EndpointHeartbeat:
        pass

    message_module = types.ModuleType("elesim_interfaces.msg")
    message_module.EndpointDescriptor = EndpointDescriptor
    message_module.EndpointHeartbeat = EndpointHeartbeat
    package_module = types.ModuleType("elesim_interfaces")
    package_module.msg = message_module
    monkeypatch.setitem(sys.modules, "elesim_interfaces", package_module)
    monkeypatch.setitem(sys.modules, "elesim_interfaces.msg", message_module)

    qos_module = types.ModuleType("rclpy.qos")
    qos_module.DurabilityPolicy = SimpleNamespace(
        TRANSIENT_LOCAL="transient", VOLATILE="volatile"
    )
    qos_module.HistoryPolicy = SimpleNamespace(KEEP_LAST="keep-last")
    qos_module.ReliabilityPolicy = SimpleNamespace(RELIABLE="reliable")
    qos_module.QoSProfile = lambda **values: values
    monkeypatch.setitem(sys.modules, "rclpy.qos", qos_module)

    class Context:
        def shutdown(self) -> None:
            return None

    class Node:
        def __init__(self) -> None:
            self.callbacks = {}

        def create_subscription(self, message_type, _topic, callback, _qos):
            self.callbacks[message_type] = callback
            return message_type

        def destroy_subscription(self, _subscription) -> None:
            return None

        def destroy_node(self) -> None:
            return None

    class FakeRclpy:
        context = SimpleNamespace(Context=Context)

        def __init__(self) -> None:
            self.node = Node()
            self.spin_calls = 0
            self.node_context = None

        @staticmethod
        def init(**_kwargs) -> None:
            return None

        def create_node(self, *_args, **_kwargs):
            self.node_context = _kwargs["context"]
            return self.node

        def spin_once(self, _node, **_kwargs) -> None:
            raise AssertionError("global rclpy executor must not be used")

        def dispatch_once(self, node) -> None:
            self.spin_calls += 1
            message = SimpleNamespace(
                peer=SimpleNamespace(endpoint_id="sim-default")
            )
            node.callbacks[EndpointDescriptor](message)
            node.callbacks[EndpointHeartbeat](message)

    fake = FakeRclpy()
    created_executors = []

    class Executor:
        def __init__(self, *, context) -> None:
            self.context = context
            self.node = None
            self.removed = False
            self.closed = False

        def add_node(self, node) -> None:
            self.node = node

        def spin_once(self, **kwargs) -> None:
            fake.dispatch_once(self.node)

        def remove_node(self, node) -> None:
            assert node is self.node
            self.removed = True

        def shutdown(self) -> None:
            self.closed = True

    def make_executor(*, context):
        executor = Executor(context=context)
        created_executors.append(executor)
        return executor

    fake.executors = SimpleNamespace(SingleThreadedExecutor=make_executor)
    result = probe_dds_peer_state(
        local_state(),
        timeout_s=60,
        expected_peers=("sim-default",),
        import_rclpy=lambda: fake,
    )

    assert result == DdsPeerProbe(
        descriptors=("sim-default",), heartbeats=("sim-default",)
    )
    assert fake.spin_calls == 1
    assert len(created_executors) == 1
    assert created_executors[0].context is fake.node_context
    assert created_executors[0].removed is True
    assert created_executors[0].closed is True


def test_doctor_strict_peer_probe_fails_when_target_is_missing(
    local_state,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    graph = DdsGraphSnapshot(nodes=(), topics={}, services={})
    monkeypatch.setattr("elesim_setup.doctor.probe_dds_graph", lambda *_args, **_kwargs: graph)
    monkeypatch.setattr(
        "elesim_setup.doctor.probe_dds_peer_state",
        lambda *_args, **_kwargs: DdsPeerProbe(
            descriptors=(),
            heartbeats=(),
        ),
    )

    report = NetworkDoctor(
        local_state(),
        expected_peers=("sim-default",),
        strict_peers=True,
    ).run()

    result = next(item for item in report.results if item.name == "DDS peers")
    assert result.status == FAIL
    assert not report.ok


def test_doctor_strict_peer_probe_rejects_stale_descriptor_without_heartbeat(
    local_state,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    graph = DdsGraphSnapshot(nodes=(), topics={}, services={})
    monkeypatch.setattr("elesim_setup.doctor.probe_dds_graph", lambda *_args, **_kwargs: graph)
    monkeypatch.setattr(
        "elesim_setup.doctor.probe_dds_peer_state",
        lambda *_args, **_kwargs: DdsPeerProbe(
            descriptors=("sim-default",),
            heartbeats=(),
        ),
    )

    report = NetworkDoctor(
        local_state(),
        expected_peers=("sim-default",),
        strict_peers=True,
    ).run()

    result = next(item for item in report.results if item.name == "DDS peers")
    assert result.status == FAIL
    assert "heartbeat 없음" in result.detail


def test_doctor_dds_failure_skips_dependent_checks(
    local_state,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def fail(*_args, **_kwargs):
        raise RuntimeError("RMW unavailable")

    monkeypatch.setattr("elesim_setup.doctor.probe_dds_graph", fail)
    report = NetworkDoctor(local_state()).run()
    by_name = {result.name: result.status for result in report.results}

    assert by_name["DDS graph"] == FAIL
    assert by_name["RGBD topic"] == SKIP
    assert by_name["WebRTC signaling"] == SKIP


def test_doctor_reuses_an_installed_role_enclave(
    local_state,
    tmp_path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    keystore = tmp_path / "sros2"
    keystore.mkdir()
    state = local_state(
        roles=("ui",),
        dds=DdsSettings(
            security_profile="sros2",
            security_provisioning="external",
            keystore=str(keystore),
            enclave="/lab",
        ),
    )
    monkeypatch.delenv("ROS_SECURITY_ENCLAVE_OVERRIDE", raising=False)

    _prepare_dds_environment(state)

    assert os.environ["ROS_SECURITY_ENCLAVE_OVERRIDE"] == "/lab/ui_main"


def test_turn_probe_remains_independent_of_dds(
    local_state,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    state = local_state(
        network=NetworkSettings(
            turn_urls=("turn:relay.example.com:3478?transport=udp",),
        ),
        turn=TurnSettings(
            mode="external",
            credential_file="/tmp/turn.credentials.json",
        ),
    )
    monkeypatch.setattr(
        "elesim_setup.doctor.udp_stun_probe",
        lambda *_args, **_kwargs: None,
    )
    monkeypatch.setattr(
        "elesim_setup.doctor.probe_dds_graph",
        lambda *_args, **_kwargs: DdsGraphSnapshot((), {}, {}),
    )

    report = NetworkDoctor(state).run()

    assert next(result for result in report.results if result.name == "TURN 1").status == PASS
