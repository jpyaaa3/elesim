from __future__ import annotations

from collections import deque

import time

from elesim_protocol import DdsTransportError, Envelope, make_envelope
from elesim_ui.peer_runtime import UiPeerHub


class Client:
    def __init__(self) -> None:
        self.registered = True
        self.inbox: deque[Envelope] = deque()
        self.sent: list[Envelope] = []
        self.closed = False
        self.fail_heartbeat = False
        self.peers: set[str] = set()

    def heartbeat(self) -> None:
        if self.fail_heartbeat:
            raise DdsTransportError("DDS socket reset")
        return None

    def has_peer(self, endpoint_id: str) -> bool:
        return str(endpoint_id) in self.peers

    def receive(self, timeout_ms: int = 0):
        while self.inbox:
            yield self.inbox.popleft()

    def send(self, message_type: str, **kwargs: object) -> Envelope:
        message = make_envelope(
            message_type,
            "ui-a",
            target_id=str(kwargs.get("target_id", "sim-a")),
            payload=dict(kwargs.get("payload", {})),
            seq=len(self.sent) + 1,
        )
        self.sent.append(message)
        return message

    def close(self) -> None:
        self.closed = True


def test_one_ui_peer_demultiplexes_operator_and_sim_messages() -> None:
    client = Client()
    hub = UiPeerHub(endpoint_id="ui-a", client=client, autostart=False)
    operator = hub.channel("operator")
    sim = hub.channel("sim")

    hub._dispatch(
        make_envelope(
            "operator_result",
            "pilot-a",
            target_id="ui-a",
            payload={"request_id": "one", "ok": True},
            seq=1,
        )
    )
    hub._dispatch(
        make_envelope(
            "simulation_status",
            "sim-a",
            target_id="ui-a",
            payload={},
            seq=2,
        )
    )

    assert [item.message_type for item in operator.receive()] == ["operator_result"]
    assert [item.message_type for item in sim.receive()] == ["simulation_status"]
    hub.close()
    assert client.closed is True


def test_peer_error_returns_to_the_channel_that_sent_the_request() -> None:
    client = Client()
    hub = UiPeerHub(endpoint_id="ui-a", client=client, autostart=False)
    sim = hub.channel("sim")
    sent = sim.send("open_simulation_session", payload={})
    hub._dispatch(
        make_envelope(
            "error",
            "sim-a",
            target_id="ui-a",
            payload={"reply_to": sent.message_id, "reason": "unavailable"},
            seq=2,
        )
    )

    assert [item.message_type for item in sim.receive()] == ["error"]
    assert tuple(hub.channel("operator").receive()) == ()
    hub.close()


def _wait_until(predicate, *, timeout: float = 1.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    assert predicate()


def test_sim_channel_waits_for_a_live_descriptor() -> None:
    client = Client()
    hub = UiPeerHub(endpoint_id="ui-a", client=client, autostart=False)
    sim = hub.channel("sim")

    assert sim.has_peer("sim-a") is False
    client.peers.add("sim-a")
    assert sim.has_peer("sim-a") is True
    hub.close()


def test_owned_peer_is_recreated_after_initial_or_heartbeat_failure() -> None:
    clients: list[Client] = []
    attempts = 0

    def factory(*_args: object, **_kwargs: object) -> Client:
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise DdsTransportError("DDS participant unavailable")
        value = Client()
        if not clients:
            value.fail_heartbeat = True
        clients.append(value)
        return value

    hub = UiPeerHub(endpoint_id="ui-a", client_factory=factory)
    try:
        _wait_until(lambda: len(clients) >= 1)
        first = clients[0]
        _wait_until(lambda: len(clients) >= 2)
        assert first.closed is True
        assert hub.registered is True
    finally:
        hub.close()
