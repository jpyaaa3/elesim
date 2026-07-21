from __future__ import annotations

from collections import deque

from elesim_protocol import Envelope, make_envelope
from elesim_ui.webrtc_session import UiWebRtcSession


class Endpoint:
    def __init__(self) -> None:
        self.registered = True
        self.inbox: deque[Envelope] = deque()
        self.sent: list[tuple[str, dict[str, object]]] = []
        self.closed = False

    def heartbeat(self) -> None:
        pass

    def receive(self, timeout_ms: int = 0):
        while self.inbox:
            yield self.inbox.popleft()

    def send(self, message_type: str, **kwargs: object) -> None:
        self.sent.append((message_type, kwargs))

    def close(self) -> None:
        self.closed = True


class Receiver:
    created: list["Receiver"] = []

    def __init__(self) -> None:
        self.latest_bgr = None
        self.answers: list[tuple[str, str]] = []
        self.closed = False
        self.created.append(self)

    def create_offer(self) -> dict[str, str]:
        return {"sdp": "offer-sdp", "type": "offer"}

    def accept_answer(self, sdp: str, answer_type: str) -> None:
        self.answers.append((sdp, answer_type))

    def close(self) -> None:
        self.closed = True


def answer(source_id: str) -> Envelope:
    return make_envelope(
        "webrtc_signal",
        source_id,
        target_id="ui-a-video",
        payload={"signal": "answer", "sdp": "answer-sdp", "type": "answer"},
        seq=1,
    )


def test_signaling_creates_receiver_accepts_answer_and_switches_target() -> None:
    Receiver.created.clear()
    endpoint = Endpoint()
    session = UiWebRtcSession(
        "inproc://unused",
        ui_id="ui-a",
        sim_id="sim-a",
        endpoint_factory=lambda *_args, **_kwargs: endpoint,
        receiver_factory=Receiver,
    )

    session.run_cycle(endpoint)
    first = Receiver.created[-1]
    assert endpoint.sent[-1][1]["target_id"] == "sim-a"

    endpoint.inbox.append(answer("sim-a"))
    session.run_cycle(endpoint)
    assert first.answers == [("answer-sdp", "answer")]

    session.switch_target("sim-b")
    session.run_cycle(endpoint)
    assert first.closed is True
    assert endpoint.sent[-1][1]["target_id"] == "sim-b"


def test_close_is_idempotent_even_when_session_was_not_started() -> None:
    session = UiWebRtcSession(
        "inproc://unused",
        ui_id="ui-a",
        sim_id="sim-a",
        endpoint_factory=Endpoint,
        receiver_factory=Receiver,
    )

    session.close()
    session.close()

    assert session.frame() is None
