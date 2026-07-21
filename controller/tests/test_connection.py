from __future__ import annotations

from dataclasses import replace

from elesim_controller.connection import ControllerConnection
from elesim_protocol import EndpointDescriptor, Envelope, SimMappingConfig, make_envelope


class Endpoint:
    def __init__(self) -> None:
        self.sent: list[tuple[str, dict[str, object]]] = []

    def send(self, message_type: str, **kwargs: object) -> None:
        make_envelope(
            message_type,
            "controller-a",
            target_id=str(kwargs.get("target_id", "server")),
            payload=dict(kwargs.get("payload", {})),
            seq=len(self.sent) + 1,
            lease_id=str(kwargs.get("lease_id", "")),
            trace_context=dict(kwargs.get("trace_context") or {}),
        )
        self.sent.append((message_type, kwargs))


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

    def router_connected(self, connected: bool) -> None:
        self.connected.append(connected)

    def accept_error(self, reason: str) -> None:
        self.errors.append(str(reason))


def envelope(message_type: str, payload: dict[str, object], **values: object) -> Envelope:
    base = Envelope(
        message_type=message_type,
        source_id="server",
        target_id="controller-a",
        payload=payload,
        seq=1,
        timestamp=1.0,
        message_id="message-1",
    )
    return replace(base, **values)


def connection() -> tuple[ControllerConnection, StateSink, Endpoint]:
    sink = StateSink()
    endpoint = Endpoint()
    value = ControllerConnection(
        server_endpoint="inproc://unused",
        controller_id="controller-a",
        initial_target="robot-a",
        mapping=SimMappingConfig(),
        state_sink=sink,
    )
    return value, sink, endpoint


def test_registered_controller_discovers_and_reselects_after_target_loss() -> None:
    value, sink, endpoint = connection()
    descriptor = EndpointDescriptor(
        "controller-a", "controller", ("operator_control",), instance_id="instance-a"
    )

    value.handle_envelope(
        endpoint,
        envelope("registered", {"endpoint": descriptor.to_dict()}),
    )
    assert endpoint.sent[-1] == ("discover", {"payload": {}})

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


def test_telemetry_and_ack_are_delivered_without_local_zmq_bridge() -> None:
    value, sink, endpoint = connection()
    value.handle_envelope(
        endpoint,
        envelope("telemetry", {"q": [-0.1, 0.2, 0.3, -0.4], "q_source": "measured"}),
    )
    value.handle_envelope(endpoint, envelope("ack", {"ok": False, "reason": "limit"}))

    assert sink.telemetry == [{"q": [-0.1, 0.2, 0.3, -0.4], "q_source": "measured"}]
    assert sink.acks == [{"ok": False, "reason": "limit"}]


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


def test_estop_bypasses_lease_but_requires_a_known_target() -> None:
    value, _sink, endpoint = connection()
    value.active_target = "robot-a"
    value.submit({"t": "estop"}, force=True)
    value.drain_outbox(endpoint, now=1.0)

    assert endpoint.sent[-1] == (
        "motion_command",
        {"target_id": "robot-a", "payload": {"command": "estop"}, "lease_id": ""},
    )


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
