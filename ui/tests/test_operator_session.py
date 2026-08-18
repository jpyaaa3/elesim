from __future__ import annotations

from collections import deque

import pytest

from elesim_protocol import (
    DdsTransportError,
    OPERATOR_VIEW_SCHEMA_VERSION,
    Envelope,
    encode_value,
    make_envelope,
)
from elesim_ui.operator_session import OperatorSession


class Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


class Endpoint:
    def __init__(self) -> None:
        self.registered = True
        self.sent: list[tuple[str, dict[str, object], Envelope]] = []
        self.inbox: deque[Envelope] = deque()
        self.closed = False

    def heartbeat(self) -> None:
        pass

    def send(self, message_type: str, **kwargs: object) -> Envelope:
        envelope = make_envelope(
            message_type,
            "ui-a",
            target_id=str(kwargs.get("target_id", "server")),
            payload=dict(kwargs.get("payload", {})),
            seq=len(self.sent) + 1,
        )
        self.sent.append((message_type, kwargs, envelope))
        return envelope

    def receive(self, timeout_ms: int = 0):
        while self.inbox:
            yield self.inbox.popleft()

    def close(self) -> None:
        self.closed = True


class FailingHeartbeatEndpoint(Endpoint):
    def heartbeat(self) -> None:
        raise DdsTransportError("DDS socket reset")


def result(request_id: str, value: object = None, *, ok: bool = True) -> Envelope:
    return make_envelope(
        "operator_result",
        "pilot-a",
        target_id="ui-a",
        payload={"request_id": request_id, "ok": ok, "result": encode_value(value)},
        seq=1,
    )


def session(clock: Clock) -> OperatorSession:
    return OperatorSession(
        ui_id="ui-a",
        pilot_id="pilot-a",
        clock=clock,
        request_timeout_s=1.0,
        snapshot_period_s=100.0,
        autostart=False,
    )


def test_construction_and_submission_do_not_wait_for_a_pilot() -> None:
    clock = Clock()
    value = session(clock)

    request_id = value.submit("service_call", "stop_pick_e2e")

    assert request_id
    assert value.pending_count == 1
    assert value.status.pilot_online is False


def test_state_set_is_committed_only_after_the_matching_ack() -> None:
    clock = Clock()
    value = session(clock)
    endpoint = Endpoint()
    value.seed_state({"visual_target_scale": 0.16})

    request_id = value.submit("state_set", "visual_target_scale", value=0.22)
    value.run_cycle(endpoint, now=clock.now)
    assert value.state_value("visual_target_scale") == 0.16

    endpoint.inbox.append(result(request_id))
    value.run_cycle(endpoint, now=clock.now)
    assert value.state_value("visual_target_scale") == 0.22


def test_view_snapshot_atomically_replaces_ui_read_caches() -> None:
    clock = Clock()
    value = session(clock)
    endpoint = Endpoint()

    request_id = value.request_snapshot()
    value.run_cycle(endpoint, now=clock.now)
    endpoint.inbox.append(
        result(
            request_id,
            {
                "schema_version": OPERATOR_VIEW_SCHEMA_VERSION,
                "state": {"pick_running": True, "pick_phase": "aim"},
                "service": {
                    "has_client": True,
                    "active_endpoint": "sim-a",
                    "current_host_state": {"connected": True},
                },
            },
        )
    )
    value.run_cycle(endpoint, now=clock.now)

    assert value.state_value("pick_phase") == "aim"
    assert value.service_value("active_endpoint") == "sim-a"
    assert value.status.pilot_online is True


def test_timed_out_requests_are_retired_without_blocking_the_ui() -> None:
    clock = Clock()
    value = session(clock)
    endpoint = Endpoint()
    value.submit("service_call", "torque_on")
    value.run_cycle(endpoint, now=clock.now)

    clock.now = 1.1
    value.run_cycle(endpoint, now=clock.now)

    assert value.pending_count == 0
    assert "timed out" in value.status.last_error


def test_unsent_high_rate_updates_are_coalesced_to_the_latest_value() -> None:
    clock = Clock()
    value = session(clock)
    endpoint = Endpoint()

    value.submit("state_set", "visual_target_scale", value=0.20)
    latest = value.submit("state_set", "visual_target_scale", value=0.25)
    assert value.pending_count == 1

    value.run_cycle(endpoint, now=clock.now)
    intents = [entry for entry in endpoint.sent if entry[0] == "operator_intent"]
    matching = [
        entry for entry in intents
        if entry[1]["payload"].get("name") == "visual_target_scale"
    ]
    assert len(matching) == 1
    assert matching[0][1]["payload"]["request_id"] == latest
    assert matching[0][1]["payload"]["kwargs"] == {"value": 0.25}


def test_outbox_flush_is_bounded_per_transport_cycle() -> None:
    clock = Clock()
    value = session(clock)
    endpoint = Endpoint()
    # Keep the periodic snapshot request out of this transport-fairness test;
    # the 40 entries below are the complete burst under measurement.
    value._last_snapshot_requested_at = clock.now

    request_ids = [
        value.submit("service_call", "stop_pick_e2e")
        for _ in range(40)
    ]
    assert all(request_ids)

    value.run_cycle(endpoint, now=clock.now)
    assert len(endpoint.sent) == 32
    assert value.pending_count == 40

    value.run_cycle(endpoint, now=clock.now)
    assert len(endpoint.sent) == 40
    assert value.pending_count == 40


def test_completed_callback_queue_is_bounded() -> None:
    clock = Clock()
    value = session(clock)
    for index in range(value.max_pending + 4):
        value._enqueue_callback(lambda _value: None, index)

    assert len(value._callbacks) == value.max_pending
    assert value._callbacks[-1][1] == value.max_pending + 3


def test_peer_error_retires_the_exact_request_immediately() -> None:
    clock = Clock()
    value = session(clock)
    endpoint = Endpoint()
    value.submit("service_call", "torque_on")
    value.run_cycle(endpoint, now=clock.now)
    sent = next(entry for entry in endpoint.sent if entry[0] == "operator_intent")
    endpoint.inbox.append(
        make_envelope(
            "error",
            "pilot-a",
            target_id="ui-a",
            payload={
                "reply_to": sent[2].message_id,
                "reason": "operator pilot is unavailable",
            },
            seq=2,
        )
    )

    value.run_cycle(endpoint, now=clock.now)

    assert "pilot is unavailable" in value.status.last_error
    assert all(
        request[1]["payload"].get("name") != "torque_on"
        for request in endpoint.sent[1:]
        if request[0] == "operator_intent"
    )


def test_transport_reset_marks_operator_dds_offline() -> None:
    clock = Clock()
    value = session(clock)
    value.run_cycle(Endpoint(), now=clock.now)
    assert value.status.dds_online is True

    with pytest.raises(DdsTransportError, match="DDS socket reset"):
        value.run_cycle(FailingHeartbeatEndpoint(), now=clock.now)

    assert value.status.dds_online is False
    assert "operator transport failed" in value.status.last_error
