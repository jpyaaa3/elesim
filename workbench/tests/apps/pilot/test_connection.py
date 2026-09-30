from __future__ import annotations

import threading
from dataclasses import replace

from elesim_pilot.connection import PilotConnection
from elesim_protocol import (
    EndpointDescriptor,
    Envelope,
    SimMappingConfig,
    SimulationStatusPayload,
    make_envelope,
)


IDENTITY_MODEL = {
    "schema_version": 1, "id": "identity", "version": 1,
    "program": {"nodes": [
        {"op": "q", "index": index} for index in range(4)
    ], "outputs": [0, 1, 2, 3]},
}


class Endpoint:
    def __init__(self) -> None:
        self.sent: list[tuple[str, dict[str, object]]] = []

    def send(self, message_type: str, **kwargs: object) -> Envelope:
        sent = make_envelope(
            message_type,
            "pilot-a",
            target_id=str(kwargs.get("target_id", "server")),
            payload=dict(kwargs.get("payload", {})),
            seq=len(self.sent) + 1,
            lease_id=str(kwargs.get("lease_id", "")),
            trace_context=dict(kwargs.get("trace_context") or {}),
        )
        self.sent.append((message_type, kwargs))
        return sent


class StateSink:
    def __init__(self) -> None:
        self.telemetry: list[dict[str, object]] = []
        self.acks: list[dict[str, object]] = []
        self.targets: list[str] = []
        self.connected: list[bool] = []
        self.errors: list[str] = []

    def accept_telemetry(self, payload: dict[str, object]) -> None:
        self.telemetry.append(payload)

    def accept_ack(self, payload: dict[str, object]) -> None:
        self.acks.append(payload)

    def target_changed(self, target_id: str) -> None:
        self.targets.append(target_id)

    def peer_connected(self, connected: bool) -> None:
        self.connected.append(connected)

    def accept_error(self, reason: str) -> None:
        self.errors.append(str(reason))


def envelope(message_type: str, payload: dict[str, object], **values: object) -> Envelope:
    base = Envelope(
        message_type=message_type,
        source_id="server",
        target_id="pilot-a",
        payload=payload,
        seq=1,
        timestamp=1.0,
        message_id="message-1",
    )
    return replace(base, **values)


def connection() -> tuple[PilotConnection, StateSink, Endpoint]:
    sink = StateSink()
    endpoint = Endpoint()
    value = PilotConnection(
        pilot_id="pilot-a",
        initial_target="robot-a",
        mapping=SimMappingConfig(),
        state_sink=sink,
    )
    return value, sink, endpoint


def test_pilot_discovers_and_reselects_after_target_loss() -> None:
    value, sink, endpoint = connection()
    value.handle_envelope(
        endpoint,
        envelope(
            "endpoint_list",
            {
                "endpoints": [
                    EndpointDescriptor(
                        "robot-a", "robot", ("motion.arm",), instance_id="robot-instance"
                    ).to_dict()
                ]
            },
        ),
    )
    assert endpoint.sent[-1] == ("select_target", {"payload": {"target_id": "robot-a"}})

    value.handle_envelope(
        endpoint,
        envelope("target_selected", {"target_id": "robot-a", "lease_id": "lease-a"}),
    )
    value.handle_envelope(endpoint, envelope("target_lost", {"target_id": "robot-a"}))

    assert sink.targets[-2:] == ["robot-a", ""]
    assert endpoint.sent[-1] == ("discover", {"payload": {}})
    value.handle_envelope(
        endpoint,
        envelope("endpoint_list", {"endpoints": value.endpoints}),
    )
    assert endpoint.sent[-1] == ("select_target", {"payload": {"target_id": "robot-a"}})
    value.handle_envelope(
        endpoint,
        envelope("target_selected", {"target_id": "robot-a", "lease_id": "lease-b"}),
    )
    assert value.active_target == "robot-a"
    assert value.lease_id == "lease-b"


def test_queued_old_lease_loss_does_not_clear_new_grant() -> None:
    value, sink, endpoint = connection()
    value.handle_envelope(
        endpoint,
        envelope("target_selected", {"target_id": "robot-a", "lease_id": "new-lease"}),
    )
    for kind in ("target_lost", "target_released"):
        value.handle_envelope(
            endpoint,
            envelope(kind, {"target_id": "robot-a", "reason": "expired"},
                     source_id="robot-a", lease_id="old-lease"),
        )
    assert value.active_target == "robot-a"
    assert value.lease_id == "new-lease"
    assert sink.targets == ["robot-a"]
    assert endpoint.sent == []


def test_target_selection_retries_only_after_discovery_interval() -> None:
    value, _sink, endpoint = connection()
    value.endpoints = [
        EndpointDescriptor(
            "robot-a", "robot", ("motion.arm",), instance_id="robot-instance"
        ).to_dict()
    ]

    value._request_desired_target(endpoint, now=10.0)
    value._request_desired_target(endpoint, now=10.9)
    assert [message for message, _kwargs in endpoint.sent] == ["select_target"]

    value._request_desired_target(endpoint, now=11.0)
    assert [message for message, _kwargs in endpoint.sent] == [
        "select_target",
        "select_target",
    ]


def test_telemetry_and_ack_are_delivered_over_the_peer_connection() -> None:
    value, sink, endpoint = connection()
    value.handle_envelope(
        endpoint,
        envelope("telemetry", {"q": [-0.1, 0.2, 0.3, -0.4], "q_source": "measured"}),
    )
    value.handle_envelope(endpoint, envelope("ack", {"ok": False, "reason": "limit"}))

    assert sink.telemetry == [{"q": [-0.1, 0.2, 0.3, -0.4], "q_source": "measured"}]
    assert sink.acks == [{"ok": False, "reason": "limit"}]


def test_imu_model_selection_retries_until_exact_robot_telemetry_confirms() -> None:
    value, _sink, endpoint = connection()
    value.endpoints = [EndpointDescriptor("robot-a", "robot", ("motion.arm",)).to_dict()]
    value.active_target = "robot-a"
    value.lease_id = "lease-a"
    model = IDENTITY_MODEL
    assert value.select_imu_model(model)["active"] is False

    value.flush_imu_model(endpoint, now=10.0)
    assert len(endpoint.sent) == 1
    sent = endpoint.sent[-1][1]["payload"]
    assert sent["model"] == model
    token = sent["selection_id"]
    value.handle_envelope(endpoint, envelope("telemetry", {
        "imu_model": {"id": "identity", "version": 1},
        "imu_model_selection_id": "old-token",
    }, source_id="robot-a", lease_id="lease-a"))
    assert value.imu_model_status()["active"] is False
    value.flush_imu_model(endpoint, now=11.0)
    assert len(endpoint.sent) == 2
    assert endpoint.sent[-1][1]["payload"]["selection_id"] == token
    value.handle_envelope(endpoint, envelope("telemetry", {
        "imu_model": {"id": "identity", "version": 1},
        "imu_model_selection_id": token,
    }, source_id="robot-a", lease_id="lease-a"))
    assert value.imu_model_status()["active"] is True
    value.flush_imu_model(endpoint, now=12.0)
    assert len(endpoint.sent) == 2


def test_imu_model_selection_stale_ack_retries_but_hardware_rejection_stops() -> None:
    value, _sink, endpoint = connection()
    value.endpoints = [EndpointDescriptor("robot-a", "robot", ("motion.arm",)).to_dict()]
    value.active_target = "robot-a"
    value.lease_id = "lease-a"
    value.select_imu_model(IDENTITY_MODEL)
    value.flush_imu_model(endpoint, now=10.0)
    value.handle_envelope(endpoint, envelope("ack", {
        "reply_to": value._imu_model_message_id, "ok": False, "reason": "stale_sequence",
    }, source_id="robot-a", lease_id="lease-a"))
    value.flush_imu_model(endpoint, now=11.0)
    assert len(endpoint.sent) == 2
    value.handle_envelope(endpoint, envelope("ack", {
        "reply_to": value._imu_model_message_id, "ok": False, "reason": "native_arm_unavailable",
    }, source_id="robot-a", lease_id="lease-a"))
    value.flush_imu_model(endpoint, now=12.0)
    assert len(endpoint.sent) == 2
    assert value.imu_model_status()["error"] == "native_arm_unavailable"


def test_simulation_status_is_typed_and_delivered_only_from_the_active_target() -> None:
    value, _sink, endpoint = connection()
    value.active_target = "sim-a"
    received: list[SimulationStatusPayload] = []
    value.simulation_status_handler = received.append
    payload = SimulationStatusPayload(
        epoch=2,
        paused=True,
        speed=0.5,
        debug_visible=False,
        sim_time_s=3.0,
    ).to_payload()

    value.handle_envelope(
        endpoint,
        envelope("simulation_status", payload, source_id="sim-b"),
    )
    value.handle_envelope(
        endpoint,
        envelope("simulation_status", payload, source_id="sim-a"),
    )

    assert received == [SimulationStatusPayload.from_payload(payload)]


def test_invalid_target_stream_configuration_does_not_escape_connection_thread() -> None:
    value, sink, endpoint = connection()
    value.endpoints = [
        EndpointDescriptor(
            "sim-a", "sim", ("motion.arm",), instance_id="sim-instance"
        ).to_dict()
    ]

    def reject_descriptor(_descriptor: dict[str, object]) -> None:
        raise ValueError("missing media server key")

    value.on_target_selected = reject_descriptor
    value.handle_envelope(
        endpoint,
        envelope("target_selected", {"target_id": "sim-a", "lease_id": "lease-a"}),
    )

    assert value.active_target == "sim-a"
    assert sink.targets == ["sim-a"]
    assert sink.errors == [
        "target stream configuration failed: missing media server key"
    ]


def test_target_submission_is_canonical_and_latest_rate_limited_value_is_retained() -> None:
    value, _sink, endpoint = connection()
    value.active_target = "robot-a"
    value.lease_id = "lease-a"

    value.submit({"t": "target", "source": "slider", "q": [-0.1, 0.0, 0.1, -0.1]})
    value.submit({"t": "target", "source": "slider", "q": [-0.2, 0.1, 0.2, -0.2]})
    value.drain_outbox(endpoint, now=10.0)

    assert endpoint.sent == []
    value.flush_target(endpoint, now=10.1)
    message_type, kwargs = endpoint.sent[-1]
    assert message_type == "motion_command"
    assert kwargs["target_id"] == "robot-a"
    assert kwargs["lease_id"] == "lease-a"
    assert kwargs["payload"] == {
        "command": "target",
        "source": "slider",
        "q": [-0.2, 0.1, 0.2, -0.2],
    }


def test_sim_reset_overtakes_and_discards_queued_motion_targets() -> None:
    value, sink, endpoint = connection()
    value.active_target = "sim-a"
    value.lease_id = "lease-a"

    # One latest-only velocity is already pending and another forced target is
    # still in the queue when respawn arrives from the operator callback.
    value.submit({"t": "target", "go2_vel": [0.4, 0.0, 0.0]})
    value.drain_outbox(endpoint, now=1.0)
    value.submit({"t": "target", "q": [-0.1, 0.0, 0.1, 0.0]}, force=True)
    value._connection_thread_ident = threading.get_ident()
    value._active_endpoint = endpoint

    value.submit({"t": "sim_reset"}, force=True)
    value.drain_outbox(endpoint, now=2.0)
    value.flush_target(endpoint, now=2.0)

    assert [entry[1]["payload"] for entry in endpoint.sent] == [
        {"command": "sim_reset"}
    ]
    assert sink.errors == []


def test_queued_sim_reset_is_sent_before_later_motion() -> None:
    value, _sink, endpoint = connection()
    value.active_target = "sim-a"
    value.lease_id = "lease-a"

    value.submit({"t": "target", "go2_vel": [0.4, 0.0, 0.0]})
    value.submit({"t": "sim_reset"}, force=True)
    value.submit({"t": "target", "go2_vel": [0.1, 0.0, 0.0]}, force=True)
    value.drain_outbox(endpoint, now=1.0)

    assert [entry[1]["payload"] for entry in endpoint.sent] == [
        {"command": "sim_reset"},
        {"command": "target", "go2_vel": [0.1, 0.0, 0.0]},
    ]


def test_estop_bypasses_lease_but_requires_a_known_target() -> None:
    value, _sink, endpoint = connection()
    value.active_target = "robot-a"
    value.submit({"t": "estop"}, force=True)
    value.drain_outbox(endpoint, now=1.0)

    assert endpoint.sent[-1] == (
        "motion_command",
        {"target_id": "robot-a", "payload": {"command": "estop"}, "lease_id": ""},
    )


def test_queued_mock_hug_cannot_be_retargeted_after_submission() -> None:
    value, sink, endpoint = connection()
    value.active_target = "sim-a"
    value.lease_id = "lease-b"
    value.endpoints = [
        {
            "endpoint_id": "sim-a",
            "role": "sim",
            "instance_id": "boot-a",
            "capabilities": ["simulation.mock_hug.v1"],
        }
    ]
    value.submit(
        {
            "t": "target",
            "q": [-0.1, 0.0, 0.2, 0.2],
            "mock_hug": {
                "solution_id": "hug-1",
                "object_revision": 1,
                "object_sha256": "a" * 64,
                "final_q": [-0.1, 0.0, 0.2, 0.2],
                "target_id": "sim-a",
                "target_boot_id": "boot-a",
                "target_lease_id": "lease-a",
            },
        },
        force=True,
    )

    value.drain_outbox(endpoint, now=1.0)

    assert endpoint.sent == []
    assert sink.errors[-1] == "mock_hug_route_fence_changed"


def test_invalid_operator_result_does_not_escape_the_connection_thread() -> None:
    value, sink, endpoint = connection()
    value.operator_handler = lambda _payload: {
        "request_id": "request-bad",
        "ok": True,
        "result": {"rx_age_s": float("inf")},
    }

    value.handle_envelope(
        endpoint,
        envelope(
            "operator_intent",
            {
                "request_id": "request-bad",
                "operation": "view_snapshot",
                "name": "",
                "args": [],
                "kwargs": {},
            },
            source_id="ui-a",
        ),
    )

    message_type, kwargs = endpoint.sent[-1]
    assert message_type == "operator_result"
    assert kwargs["payload"]["request_id"] == "request-bad"
    assert kwargs["payload"]["ok"] is False
    assert "non-finite" in kwargs["payload"]["error"]
    assert sink.errors
