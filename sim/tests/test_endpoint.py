from __future__ import annotations

from dataclasses import replace

from elesim_protocol import (
    DdsTransportError,
    Envelope,
    SimMappingConfig,
    SimulationSessionGrantedPayload,
    SimulationStatusPayload,
)
from elesim_sim.control_state import SimulationStateSource
from elesim_sim.endpoint import SimEndpoint


class Client:
    def __init__(self) -> None:
        self.sent: list[tuple[str, dict[str, object]]] = []

    def send(self, message_type: str, **kwargs: object) -> None:
        self.sent.append((message_type, kwargs))


class FlakyClient(Client):
    def __init__(self, failing_type: str) -> None:
        super().__init__()
        self.failing_type = failing_type
        self.fail = True

    def send(self, message_type: str, **kwargs: object) -> None:
        if self.fail and message_type == self.failing_type:
            raise DdsTransportError(f"{message_type} target disappeared")
        super().send(message_type, **kwargs)


def message(payload: dict[str, object], *, seq: int = 1, lease_id: str = "lease-a") -> Envelope:
    return Envelope(
        message_type="motion_command",
        source_id="pilot-a",
        target_id="sim-a",
        payload=payload,
        seq=seq,
        timestamp=1.0,
        message_id=f"message-{seq}",
        lease_id=lease_id,
    )


def endpoint() -> tuple[SimEndpoint, SimulationStateSource, Client]:
    state = SimulationStateSource(SimMappingConfig())
    client = Client()
    value = SimEndpoint(
        endpoint_id="sim-a",
        state=state,
        streams={},
    )
    value.grant_lease("pilot-a", "lease-a")
    return value, state, client


def test_endpoint_applies_only_fresh_commands_from_active_lease() -> None:
    value, state, client = endpoint()
    command = message({"command": "target", "q": [-0.1, 0.2, 0.3, -0.3]})

    value.handle_envelope(client, command)
    value.handle_envelope(client, command)
    value.handle_envelope(client, replace(command, seq=2, lease_id="wrong"))

    assert state.estimate_q().linear_m == -0.1
    assert [entry[1]["payload"]["reason"] for entry in client.sent] == [
        "target",
        "stale_sequence",
        "lease_mismatch",
    ]


def test_estop_is_accepted_without_a_lease() -> None:
    value, state, client = endpoint()
    state.apply_target({"command": "target", "go2_vel": [0.2, 0.0, 0.0]})
    value.revoke_lease()

    value.handle_envelope(client, message({"command": "estop"}, lease_id=""))

    assert state.go2_vel() == (0.0, 0.0, 0.0)
    assert client.sent[-1][1]["payload"]["ok"] is True


def test_runtime_telemetry_is_merged_and_sent_as_canonical_q() -> None:
    value, _state, client = endpoint()
    value.publish_telemetry({"q": [-0.1, 0.2, 0.3, -0.3], "actual_tip": [0.7, 0.0, 0.2]})
    value.publish_telemetry({"go2_base_pos": [0.0, 0.0, 0.3]})
    value.flush_telemetry(client)

    message_type, kwargs = client.sent[-1]
    assert message_type == "telemetry"
    assert kwargs["target_id"] == "pilot-a"
    assert kwargs["lease_id"] == "lease-a"
    assert kwargs["payload"]["q"] == [-0.1, 0.2, 0.3, -0.3]
    assert kwargs["payload"]["go2_base_pos"] == [0.0, 0.0, 0.3]


def test_telemetry_remains_dirty_when_pilot_temporarily_disappears() -> None:
    value, _state, _client = endpoint()
    client = FlakyClient("telemetry")
    value.publish_telemetry({"q": [1.0, 2.0, 3.0, 4.0]})

    value.flush_telemetry(client)
    assert client.sent == []

    client.fail = False
    value.flush_telemetry(client)
    assert client.sent[-1][0] == "telemetry"
    assert client.sent[-1][1]["payload"]["q"] == [1.0, 2.0, 3.0, 4.0]


def grant_simulation_session(value: SimEndpoint, client: Client) -> None:
    granted = SimulationSessionGrantedPayload(
        request_id="open-1",
        session_id="session-a",
        sim_id="sim-a",
        ui_id="ui-a",
        streams=("observer", "hand_eye_preview"),
    )
    value.handle_envelope(
        client,
        Envelope(
            message_type="simulation_session_granted",
            source_id="sim-a",
            target_id="sim-a",
            payload=granted.to_payload(),
            seq=2,
            timestamp=1.0,
            message_id="grant-1",
            lease_id="session-a",
        ),
    )


def test_simulation_command_is_queued_until_the_genesis_thread_completes_it() -> None:
    value, _state, client = endpoint()
    grant_simulation_session(value, client)
    command = Envelope(
        message_type="simulation_command",
        source_id="ui-a",
        target_id="sim-a",
        payload={
            "schema_version": 1,
            "request_id": "pause-1",
            "session_id": "session-a",
            "command": "pause",
            "arguments": {},
        },
        seq=3,
        timestamp=1.0,
        message_id="pause-message",
        lease_id="session-a",
    )

    value.handle_envelope(client, command)
    pending = value.operator_mailbox.drain()
    assert len(pending) == 1
    assert client.sent == []
    value.operator_mailbox.complete(pending[0], ok=True, reason="paused")
    value.flush_simulation_results(client)

    assert client.sent[-1][0] == "simulation_result"
    assert client.sent[-1][1]["target_id"] == "ui-a"
    assert client.sent[-1][1]["payload"]["request_id"] == "pause-1"


def test_simulation_command_rejects_a_mismatched_operator_session_lease() -> None:
    value, _state, client = endpoint()
    grant_simulation_session(value, client)
    command = Envelope(
        message_type="simulation_command",
        source_id="ui-a",
        target_id="sim-a",
        payload={
            "schema_version": 1,
            "request_id": "pause-wrong-lease",
            "session_id": "session-a",
            "command": "pause",
            "arguments": {},
        },
        seq=3,
        timestamp=1.0,
        message_id="pause-wrong-lease-message",
        lease_id="session-other",
    )

    value.handle_envelope(client, command)

    assert value.operator_mailbox.drain() == []
    assert client.sent[-1][0] == "simulation_result"
    assert client.sent[-1][1]["payload"]["ok"] is False
    assert client.sent[-1][1]["payload"]["reason"] == "simulation_session_mismatch"


def test_simulation_result_is_requeued_when_ui_temporarily_disappears() -> None:
    value, _state, _client = endpoint()
    client = FlakyClient("simulation_result")
    grant_simulation_session(value, client)
    command = Envelope(
        message_type="simulation_command",
        source_id="ui-a",
        target_id="sim-a",
        payload={
            "schema_version": 1,
            "request_id": "pause-retry",
            "session_id": "session-a",
            "command": "pause",
            "arguments": {},
        },
        seq=3,
        timestamp=1.0,
        message_id="pause-retry-message",
        lease_id="session-a",
    )
    value.handle_envelope(client, command)
    pending = value.operator_mailbox.drain()
    assert len(pending) == 1
    value.operator_mailbox.complete(pending[0], ok=True, reason="paused")

    value.flush_simulation_results(client)
    retained = value.operator_mailbox.take_results()
    assert [result.request_id for result in retained] == ["pause-retry"]

    value.operator_mailbox.requeue_results(retained)
    client.fail = False
    value.flush_simulation_results(client)
    assert client.sent[-1][0] == "simulation_result"
    assert client.sent[-1][1]["payload"]["request_id"] == "pause-retry"


def test_simulation_status_is_fanned_out_by_the_peer_authority() -> None:
    value, _state, client = endpoint()
    value.publish_simulation_status(
        SimulationStatusPayload(
            epoch=1,
            paused=True,
            speed=1.0,
            debug_visible=False,
            sim_time_s=2.0,
        )
    )
    value.flush_simulation_status(client)

    assert client.sent[-1][0] == "simulation_status"
    assert "target_id" not in client.sent[-1][1]
    assert client.sent[-1][1]["payload"]["epoch"] == 1


def test_webrtc_offer_is_answered_for_the_requested_named_stream() -> None:
    calls: list[tuple[object, ...]] = []
    state = SimulationStateSource(SimMappingConfig())
    client = Client()
    value = SimEndpoint(
        endpoint_id="sim-a",
        state=state,
        streams={},
        webrtc_offer_handler=lambda *args: calls.append(args) or {"sdp": "answer-sdp", "type": "answer"},
    )
    grant_simulation_session(value, client)
    value.handle_envelope(
        client,
        Envelope(
            message_type="webrtc_signal",
            source_id="ui-a",
            target_id="sim-a",
            payload={
                "schema_version": 1,
                "session_id": "session-a",
                "stream": "hand_eye_preview",
                "signal": "offer",
                "sdp": "offer-sdp",
                "type": "offer",
            },
            seq=3,
            timestamp=1.0,
            message_id="offer-1",
            lease_id="session-a",
        ),
    )

    assert calls[0][:3] == ("hand_eye_preview", "offer-sdp", "offer")
    assert client.sent[-1][0] == "webrtc_signal"
    assert client.sent[-1][1]["payload"]["stream"] == "hand_eye_preview"
