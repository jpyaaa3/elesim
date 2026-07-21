from __future__ import annotations

import socket
import struct
import threading

import pytest

from elesim_setup.doctor import (
    DoctorReport,
    FAIL,
    NetworkDoctor,
    PASS,
    ProtocolSession,
    SKIP,
    TcpTarget,
    build_stun_binding_request,
    parse_tcp_endpoint,
    parse_turn_url,
    tcp_connect,
    validate_stun_response,
)


def test_endpoint_parsers_support_ipv6_and_turn_transport() -> None:
    assert parse_tcp_endpoint("tcp://[2001:db8::5]:5558") == TcpTarget("2001:db8::5", 5558)
    target = parse_turn_url("turn:relay.example.com:3478?transport=udp")
    assert (target.host, target.port, target.transport) == ("relay.example.com", 3478, "udp")
    secure = parse_turn_url("turns:relay.example.com")
    assert (secure.port, secure.transport) == (5349, "tcp")


@pytest.mark.parametrize("value", ["http://example.com", "turn:", "turn:host?transport=sctp"])
def test_invalid_turn_url_is_rejected(value: str) -> None:
    with pytest.raises(ValueError):
        parse_turn_url(value)


def test_stun_response_validation_checks_transaction_and_length() -> None:
    _request, transaction_id = build_stun_binding_request(b"abcdefghijkl")
    response = struct.pack("!HHI12s", 0x0101, 0, 0x2112A442, transaction_id)
    validate_stun_response(response, transaction_id)
    with pytest.raises(ValueError, match="transaction"):
        validate_stun_response(response, b"mnopqrstuvwx")
    with pytest.raises(ValueError, match="short"):
        validate_stun_response(b"bad", transaction_id)


def test_tcp_probe_connects_to_real_local_listener() -> None:
    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    port = listener.getsockname()[1]
    accepted = threading.Event()

    def serve() -> None:
        connection, _address = listener.accept()
        connection.close()
        accepted.set()

    thread = threading.Thread(target=serve)
    thread.start()
    try:
        tcp_connect(TcpTarget("127.0.0.1", port), timeout_s=1.0)
        assert accepted.wait(1.0)
    finally:
        listener.close()
        thread.join(timeout=1.0)


def test_report_exit_semantics_ignore_warn_and_skip() -> None:
    report = DoctorReport()
    report.add("one", PASS, "ok")
    report.add("two", SKIP, "optional")
    assert report.ok
    report.add("three", FAIL, "broken", "fix it")
    assert not report.ok
    assert "조치: fix it" in report.render()


def test_router_tcp_failure_skips_protocol_and_media(local_state) -> None:
    probe = socket.socket()
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    state = local_state(network=local_state().network.__class__(router_port=port))

    report = NetworkDoctor(state, timeout_s=0.2).run()

    by_name = {result.name: result.status for result in report.results}
    assert by_name["Router TCP"] == FAIL
    assert by_name["ZMQ protocol"] == SKIP
    assert by_name["WebRTC"] == SKIP


def test_protocol_session_waits_for_revoke_before_closing() -> None:
    class Message:
        message_type = "simulation_session_revoked"

    class Client:
        def __init__(self) -> None:
            self.sent = []
            self.receive_calls = 0

        def send(self, message_type, **kwargs):
            self.sent.append((message_type, kwargs))

        def receive(self, timeout_ms=0):
            self.receive_calls += 1
            yield Message()

    session = object.__new__(ProtocolSession)
    session.client = Client()
    session.timeout_s = 1.0
    session.session_id = "session-1"

    session._close_simulation_session()

    assert session.client.sent[0][0] == "close_simulation_session"
    assert session.client.receive_calls == 1
    assert session.session_id == ""
