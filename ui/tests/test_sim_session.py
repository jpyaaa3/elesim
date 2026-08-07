from __future__ import annotations

from collections import deque

import pytest

from elesim_protocol import (
    Envelope,
    SimulationSessionOpenedPayload,
    SimulationSessionRevokedPayload,
    SimulationStatusPayload,
    TurnCredentials,
    WebRtcSignalPayload,
    make_envelope,
)
from elesim_ui.sim_session import UiSimSession


class Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


class Endpoint:
    def __init__(self) -> None:
        self.registered = True
        self.inbox: deque[Envelope] = deque()
        self.sent: list[tuple[str, dict[str, object], Envelope]] = []
        self.closed = False

    def heartbeat(self) -> None:
        pass

    def receive(self, timeout_ms: int = 0):
        while self.inbox:
            yield self.inbox.popleft()

    def send(self, message_type: str, **kwargs: object) -> Envelope:
        envelope = make_envelope(
            message_type,
            "ui-a",
            target_id=str(kwargs.get("target_id", "server")),
            payload=dict(kwargs.get("payload", {})),
            lease_id=str(kwargs.get("lease_id", "")),
            seq=len(self.sent) + 1,
        )
        self.sent.append((message_type, kwargs, envelope))
        return envelope

    def close(self) -> None:
        self.closed = True


class UndiscoveredEndpoint(Endpoint):
    def has_peer(self, _endpoint_id: str) -> bool:
        return False


class Receiver:
    created: list["Receiver"] = []

    def __init__(self) -> None:
        self.latest_bgr = None
        self.turn = None
        self.answers: list[tuple[str, str]] = []
        self.closed = False
        self.created.append(self)

    def create_offer(self, *, turn=None) -> dict[str, str]:
        self.turn = turn
        return {"sdp": "offer-sdp", "type": "offer"}

    def accept_answer(self, sdp: str, answer_type: str) -> None:
        self.answers.append((sdp, answer_type))

    def close(self) -> None:
        self.closed = True


def opened(
    request_id: str,
    *,
    sim_id: str = "sim-a",
    session_id: str = "session-a",
    turn: TurnCredentials | None = None,
) -> Envelope:
    payload = SimulationSessionOpenedPayload(
        request_id=request_id,
        session_id=session_id,
        sim_id=sim_id,
        streams=("observer", "hand_eye_preview"),
        turn=turn,
    )
    return make_envelope(
        "simulation_session_opened",
        sim_id,
        target_id="ui-a",
        payload=payload.to_payload(),
        lease_id=session_id,
        seq=1,
    )


def answer(stream: str, *, session_id: str = "session-a") -> Envelope:
    payload = WebRtcSignalPayload(
        session_id=session_id,
        stream=stream,
        signal="answer",
        sdp=f"{stream}-answer",
        type="answer",
    )
    return make_envelope(
        "webrtc_signal",
        "sim-a",
        target_id="ui-a",
        payload=payload.to_payload(),
        lease_id=session_id,
        seq=2,
    )


def new_session(clock: Clock | None = None) -> UiSimSession:
    return UiSimSession(
        ui_id="ui-a",
        sim_id="sim-a",
        receiver_factory=Receiver,
        clock=clock or Clock(),
        autostart=False,
    )


def test_session_waits_for_sim_grant_before_creating_two_offers() -> None:
    Receiver.created.clear()
    endpoint = Endpoint()
    session = new_session()

    session.run_cycle(endpoint)
    assert [kind for kind, _kwargs, _message in endpoint.sent] == [
        "open_simulation_session"
    ]
    assert Receiver.created == []

    request_id = endpoint.sent[0][1]["payload"]["request_id"]
    turn = TurnCredentials(
        urls=("turn:relay.example:3478",),
        username="user",
        credential="secret",
        expires_at=1234.0,
    )
    endpoint.inbox.append(opened(str(request_id), turn=turn))
    session.run_cycle(endpoint)

    offers = [entry for entry in endpoint.sent if entry[0] == "webrtc_signal"]
    assert len(offers) == 2
    assert {
        entry[1]["payload"]["stream"] for entry in offers
    } == {"observer", "hand_eye_preview"}
    assert all(receiver.turn == turn for receiver in Receiver.created)
    assert session.active_sim_id == "sim-a"
    assert session.connected_streams == ()


def test_session_reports_descriptor_wait_without_sending_until_sim_is_discovered() -> None:
    endpoint = UndiscoveredEndpoint()
    session = new_session()

    session.run_cycle(endpoint)

    assert endpoint.sent == []
    assert "endpoint descriptor" in session.last_error


def test_stream_becomes_connected_only_after_its_answer_is_accepted() -> None:
    Receiver.created.clear()
    endpoint = Endpoint()
    session = new_session()
    session.run_cycle(endpoint)
    request_id = endpoint.sent[0][1]["payload"]["request_id"]
    endpoint.inbox.append(opened(str(request_id)))
    session.run_cycle(endpoint)

    endpoint.inbox.append(answer("observer"))
    session.run_cycle(endpoint)

    assert session.connected_streams == ("observer",)
    observer = session.receiver("observer")
    assert observer.answers == [("observer-answer", "answer")]


def test_turn_refresh_replaces_both_peers_without_reopening_the_session() -> None:
    Receiver.created.clear()
    endpoint = Endpoint()
    session = new_session()
    session.run_cycle(endpoint)
    request_id = str(endpoint.sent[0][1]["payload"]["request_id"])
    first_turn = TurnCredentials(
        urls=("turn:relay.example:3478",),
        username="first",
        credential="first-secret",
        expires_at=1000.0,
    )
    second_turn = TurnCredentials(
        urls=("turn:relay.example:3478",),
        username="second",
        credential="second-secret",
        expires_at=2000.0,
    )
    endpoint.inbox.append(opened(request_id, turn=first_turn))
    session.run_cycle(endpoint)
    endpoint.inbox.extend((answer("observer"), answer("hand_eye_preview")))
    session.run_cycle(endpoint)
    old_receivers = tuple(Receiver.created)
    assert session.connected_streams == ("observer", "hand_eye_preview")

    endpoint.inbox.append(opened(request_id, turn=second_turn))
    session.run_cycle(endpoint)

    offers = [entry for entry in endpoint.sent if entry[0] == "webrtc_signal"]
    assert len(offers) == 4
    assert len(Receiver.created) == 4
    assert all(receiver.closed for receiver in old_receivers)
    assert all(receiver.turn == second_turn for receiver in Receiver.created[-2:])
    assert session.snapshot.session_id == "session-a"
    assert session.connected_streams == ()

    endpoint.inbox.append(opened(request_id, turn=second_turn))
    session.run_cycle(endpoint)
    assert len(Receiver.created) == 4


def test_failed_turn_refresh_keeps_the_working_receivers() -> None:
    Receiver.created.clear()
    endpoint = Endpoint()
    calls = 0

    def receiver_factory() -> Receiver:
        nonlocal calls
        calls += 1
        if calls == 3:
            raise RuntimeError("replacement receiver failed")
        return Receiver()

    session = UiSimSession(
        ui_id="ui-a",
        sim_id="sim-a",
        receiver_factory=receiver_factory,
        clock=Clock(),
        autostart=False,
    )
    session.run_cycle(endpoint)
    request_id = str(endpoint.sent[0][1]["payload"]["request_id"])
    first_turn = TurnCredentials(
        urls=("turn:relay.example:3478",),
        username="first",
        credential="first-secret",
        expires_at=1000.0,
    )
    second_turn = TurnCredentials(
        urls=("turn:relay.example:3478",),
        username="second",
        credential="second-secret",
        expires_at=2000.0,
    )
    endpoint.inbox.append(opened(request_id, turn=first_turn))
    session.run_cycle(endpoint)
    old_receivers = tuple(Receiver.created)

    endpoint.inbox.append(opened(request_id, turn=second_turn))
    session.run_cycle(endpoint)

    assert session.active_sim_id == "sim-a"
    assert all(
        session.receiver(stream) in old_receivers
        for stream in ("observer", "hand_eye_preview")
    )
    assert all(not receiver.closed for receiver in old_receivers)
    assert "replacement receiver failed" in session.last_error


def test_commands_use_the_independent_simulation_session_lease() -> None:
    endpoint = Endpoint()
    session = new_session()
    session.run_cycle(endpoint)
    request_id = endpoint.sent[0][1]["payload"]["request_id"]
    endpoint.inbox.append(opened(str(request_id)))
    session.run_cycle(endpoint)

    command_id = session.send_command("orbit", {"dx": 0.25, "dy": -0.5})
    session.run_cycle(endpoint)

    command = [entry for entry in endpoint.sent if entry[0] == "simulation_command"][-1]
    assert command_id
    assert command[1]["target_id"] == "sim-a"
    assert command[1]["lease_id"] == "session-a"
    assert command[1]["payload"] == {
        "schema_version": 1,
        "request_id": command_id,
        "session_id": "session-a",
        "command": "orbit",
        "arguments": {"dx": 0.25, "dy": -0.5},
    }


def test_adjacent_camera_deltas_are_summed_without_crossing_command_barriers() -> None:
    endpoint = Endpoint()
    session = new_session()
    session.run_cycle(endpoint)
    request_id = endpoint.sent[0][1]["payload"]["request_id"]
    endpoint.inbox.append(opened(str(request_id)))
    session.run_cycle(endpoint)

    session.send_command("orbit", {"dx": 0.2, "dy": -0.1})
    session.send_command("orbit", {"dx": 0.3, "dy": 0.4})
    session.send_command("reset_view")
    session.send_command("orbit", {"dx": -0.1, "dy": 0.2})
    session.run_cycle(endpoint)

    commands = [
        entry[1]["payload"]
        for entry in endpoint.sent
        if entry[0] == "simulation_command"
    ]
    assert [payload["command"] for payload in commands] == [
        "orbit",
        "reset_view",
        "orbit",
    ]
    assert commands[0]["arguments"]["dx"] == pytest.approx(0.5)
    assert commands[0]["arguments"]["dy"] == pytest.approx(0.3)
    assert commands[2]["arguments"] == {"dx": -0.1, "dy": 0.2}


def test_switch_closes_old_peers_and_opens_next_target_after_revocation() -> None:
    Receiver.created.clear()
    endpoint = Endpoint()
    session = new_session()
    session.run_cycle(endpoint)
    request_id = endpoint.sent[0][1]["payload"]["request_id"]
    endpoint.inbox.append(opened(str(request_id)))
    session.run_cycle(endpoint)
    old_receivers = list(Receiver.created)

    session.switch_target("sim-b")
    session.run_cycle(endpoint)
    close = [entry for entry in endpoint.sent if entry[0] == "close_simulation_session"]
    assert len(close) == 1
    assert all(receiver.closed for receiver in old_receivers)

    revoked = SimulationSessionRevokedPayload(
        session_id="session-a",
        sim_id="sim-a",
        reason="closed",
    )
    endpoint.inbox.append(
        make_envelope(
            "simulation_session_revoked",
            "sim-a",
            target_id="ui-a",
            payload=revoked.to_payload(),
            lease_id="session-a",
            seq=3,
        )
    )
    session.run_cycle(endpoint)

    opens = [entry for entry in endpoint.sent if entry[0] == "open_simulation_session"]
    assert len(opens) == 2
    assert opens[-1][1]["payload"]["sim_id"] == "sim-b"


def test_status_is_parsed_as_typed_simulation_state() -> None:
    endpoint = Endpoint()
    session = new_session()
    session.run_cycle(endpoint)
    request_id = endpoint.sent[0][1]["payload"]["request_id"]
    endpoint.inbox.append(opened(str(request_id)))
    session.run_cycle(endpoint)
    status = SimulationStatusPayload(
        epoch=3,
        paused=True,
        speed=0.5,
        debug_visible=False,
        sim_time_s=12.25,
    )
    endpoint.inbox.append(
        make_envelope(
            "simulation_status",
            "sim-a",
            target_id="ui-a",
            payload=status.to_payload(),
            seq=4,
        )
    )

    session.run_cycle(endpoint)

    assert session.status == status


def test_close_is_idempotent_without_starting_the_transport_thread() -> None:
    session = new_session()

    session.close()
    session.close()

    assert session.frame("observer") is None
