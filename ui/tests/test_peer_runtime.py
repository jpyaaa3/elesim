from __future__ import annotations

from collections import deque

from elesim_protocol import Envelope, make_envelope
from elesim_ui.peer_runtime import UiPeerHub


class Client:
    def __init__(self) -> None:
        self.registered = True
        self.inbox: deque[Envelope] = deque()
        self.sent: list[Envelope] = []
        self.closed = False

    def heartbeat(self) -> None:
        return None

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
