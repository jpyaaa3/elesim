from __future__ import annotations

import struct

import pytest

from elesim_setup.doctor import (
    FAIL,
    PASS,
    SKIP,
    WARN,
    DdsGraphSnapshot,
    DoctorReport,
    NetworkDoctor,
    build_stun_binding_request,
    parse_tcp_endpoint,
    parse_turn_url,
    validate_stun_response,
)
from elesim_setup.state import NetworkSettings, TurnSettings


def test_parse_tcp_endpoint_supports_ipv4_hostname_and_ipv6() -> None:
    assert parse_tcp_endpoint("tcp://server.example:5558").host == "server.example"
    assert parse_tcp_endpoint("tcp://[2001:db8::1]:5568").port == 5568
    with pytest.raises(ValueError):
        parse_tcp_endpoint("udp://server:5558")


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
    state = local_state(roles=("simulator", "controller"))
    graph = DdsGraphSnapshot(
        nodes=("/elesim/v5/simulator", "/elesim/v5/controller"),
        topics={
            "/elesim/sim_default/rgbd/frame": (
                "elesim_interfaces/msg/RgbdFrame",
            ),
            "/elesim/v5/peers/sim_default/boot/control": (
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
