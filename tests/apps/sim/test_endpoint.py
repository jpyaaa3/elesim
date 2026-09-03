from __future__ import annotations

import concurrent.futures
import threading
import time
from dataclasses import replace

import pytest

from elesim_protocol import (
    DdsTransportError,
    Envelope,
    PeerIdentity,
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


def test_webrtc_close_callback_failure_is_reported(capsys: pytest.CaptureFixture[str]) -> None:
    value, _state, _client = endpoint()

    def fail(_session_id: str) -> None:
        raise RuntimeError("close boom")

    value._run_webrtc_close(fail, "session-a")

    output = capsys.readouterr().out
    assert "event=webrtc" in output
    assert "state=close_failed" in output
    assert "close boom" in output


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


def test_mock_hug_route_fence_must_name_this_exact_sim_boot_and_lease() -> None:
    value, state, client = endpoint()
    value.peer_identity = PeerIdentity("sim-a", "boot-a")
    before = state.estimate_q()
    payload = {
        "command": "target",
        "q": [-0.1, 0.0, 0.2, 0.2],
        "mock_hug": {
            "solution_id": "hug-1",
            "object_revision": 1,
            "object_sha256": "a" * 64,
            "final_q": [-0.1, 0.0, 0.2, 0.2],
            "target_id": "sim-a",
            "target_boot_id": "old-boot",
            "target_lease_id": "lease-a",
        },
    }

    value.handle_envelope(client, message(payload))

    assert state.estimate_q() == before
    assert client.sent[-1][1]["payload"]["reason"] == "mock_hug_route_fence_mismatch"


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


def test_mock_object_spawn_is_session_bound_and_queued_for_the_genesis_thread() -> None:
    value, _state, client = endpoint()
    grant_simulation_session(value, client)
    value.handle_envelope(
        client,
        Envelope(
            message_type="simulation_command",
            source_id="ui-a",
            target_id="sim-a",
            payload={
                "schema_version": 1,
                "request_id": "spawn-1",
                "session_id": "session-a",
                "command": "spawn_mock_object",
                "arguments": {
                    "asset_id": "demo_box.obj",
                    "position": [0.5, 0.0, 0.4],
                    "euler_deg": [0.0, 0.0, 0.0],
                },
            },
            seq=3,
            timestamp=1.0,
            message_id="spawn-message",
            lease_id="session-a",
        ),
    )

    pending = value.operator_mailbox.drain()
    assert len(pending) == 1
    assert pending[0].command == "spawn_mock_object"
    assert pending[0].arguments["asset_id"] == "demo_box.obj"


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


def test_camera_command_diagnostics_are_sampled_instead_of_logging_every_drag_event(
    capsys: pytest.CaptureFixture[str],
) -> None:
    value, _state, client = endpoint()
    grant_simulation_session(value, client)
    for index in range(2):
        value.handle_envelope(
            client,
            Envelope(
                message_type="simulation_command",
                source_id="ui-a",
                target_id="sim-a",
                payload={
                    "schema_version": 1,
                    "request_id": f"orbit-{index}",
                    "session_id": "session-a",
                    "command": "orbit",
                    "arguments": {"dx": 0.1, "dy": 0.0},
                },
                seq=3 + index,
                timestamp=1.0,
                message_id=f"orbit-message-{index}",
                lease_id="session-a",
            ),
        )

    output = capsys.readouterr().out.splitlines()
    assert sum("event=simulation" in line for line in output) == 1


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
    for _ in range(100):
        value._flush_webrtc_answers(client)
        if calls:
            break
        time.sleep(0.01)

    assert calls[0][:3] == ("hand_eye_preview", "offer-sdp", "offer")
    assert client.sent[-1][0] == "webrtc_signal"
    assert client.sent[-1][1]["payload"]["stream"] == "hand_eye_preview"
    if value._webrtc_executor is not None:
        value._webrtc_executor.shutdown(wait=True, cancel_futures=True)


def test_pending_webrtc_answer_keeps_trace_context() -> None:
    started = threading.Event()
    release = threading.Event()

    def handler(*_args: object) -> dict[str, str]:
        started.set()
        assert release.wait(timeout=2.0)
        return {"sdp": "answer-sdp", "type": "answer"}

    state = SimulationStateSource(SimMappingConfig())
    client = Client()
    value = SimEndpoint(
        endpoint_id="sim-a",
        state=state,
        streams={},
        webrtc_offer_handler=handler,
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
                "stream": "observer",
                "signal": "offer",
                "sdp": "offer-sdp",
                "type": "offer",
            },
            seq=3,
            timestamp=1.0,
            message_id="offer-traced",
            lease_id="session-a",
            trace_context={"traceparent": "trace-context"},
        ),
    )
    try:
        assert started.wait(timeout=2.0)
        value._flush_webrtc_answers(client)
        release.set()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not client.sent:
            value._flush_webrtc_answers(client)
            time.sleep(0.01)
        assert client.sent[-1][1]["trace_context"] == {
            "traceparent": "trace-context"
        }
    finally:
        release.set()
        if value._webrtc_executor is not None:
            value._webrtc_executor.shutdown(wait=True, cancel_futures=True)


def test_completed_webrtc_answer_is_retried_after_the_ui_peer_returns() -> None:
    value, _state, _client = endpoint()
    client = FlakyClient("webrtc_signal")
    grant_simulation_session(value, client)

    retry_key = ("ui-a", "session-a", "hand_eye_preview")
    value._webrtc_generations[retry_key] = 1
    future: concurrent.futures.Future[dict[str, str]] = concurrent.futures.Future()
    future.set_result({"sdp": "answer-sdp", "type": "answer"})
    value._webrtc_futures.append(
        (
            future,
            "ui-a",
            "session-a",
            "hand_eye_preview",
            {"traceparent": "retry-context"},
            1,
        )
    )

    value._flush_webrtc_answers(client)
    assert client.sent == []
    assert retry_key in value._webrtc_answer_retries

    client.fail = False
    value._flush_webrtc_answers(client)

    assert len(client.sent) == 1
    assert client.sent[0][0] == "webrtc_signal"
    assert client.sent[0][1]["target_id"] == "ui-a"
    assert client.sent[0][1]["payload"]["stream"] == "hand_eye_preview"
    assert client.sent[0][1]["payload"]["signal"] == "answer"
    assert client.sent[0][1]["trace_context"] == {
        "traceparent": "retry-context"
    }
    assert retry_key not in value._webrtc_answer_retries
