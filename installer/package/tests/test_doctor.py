from __future__ import annotations

import os
import struct

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
