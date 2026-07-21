from __future__ import annotations

from elesim_protocol import (
    CAPABILITY_MOTION_ARM,
    CAPABILITY_OPERATOR_CONTROL,
    EndpointDescriptor,
    make_envelope,
)
from elesim_router.core import RouterCore


def register(
    core: RouterCore,
    identity: bytes,
    endpoint_id: str,
    role: str,
    capabilities: tuple[str, ...],
) -> None:
    descriptor = EndpointDescriptor(
        endpoint_id,
        role,
        capabilities=capabilities,
        instance_id=f"{endpoint_id}-instance",
    )
    reply = core.handle(
        identity,
        make_envelope(
            "register",
            endpoint_id,
            payload={"endpoint": descriptor.to_dict()},
            seq=1,
        ),
        now=1.0,
    )
    assert reply[0].envelope.message_type == "registered"


def test_discovery_filters_role_and_capability() -> None:
    core = RouterCore()
    register(core, b"ui", "ui-a", "ui", ())
    register(core, b"controller", "controller-a", "controller", (CAPABILITY_OPERATOR_CONTROL,))
    register(core, b"robot", "robot-a", "robot", (CAPABILITY_MOTION_ARM,))

    reply = core.handle(
        b"ui",
        make_envelope(
            "discover",
            "ui-a",
            payload={"role": "controller", "capability": CAPABILITY_OPERATOR_CONTROL},
            seq=2,
        ),
        now=1.1,
    )[0].envelope
    assert [item["endpoint_id"] for item in reply.payload["endpoints"]] == ["controller-a"]


def test_motion_requires_controller_and_matching_lease() -> None:
    core = RouterCore()
    register(core, b"controller", "controller-a", "controller", (CAPABILITY_OPERATOR_CONTROL,))
    register(core, b"robot", "robot-a", "robot", (CAPABILITY_MOTION_ARM,))
    selected = core.handle(
        b"controller",
        make_envelope(
            "select_target",
            "controller-a",
            payload={"target_id": "robot-a"},
            seq=2,
        ),
        now=1.1,
    )[-1].envelope
    lease_id = str(selected.payload["lease_id"])

    routed = core.handle(
        b"controller",
        make_envelope(
            "motion_command",
            "controller-a",
            target_id="robot-a",
            lease_id=lease_id,
            payload={"command": "target", "q": [0.0, 0.1, 0.2, 0.3]},
            seq=3,
        ),
        now=1.2,
    )
    assert routed[0].identity == b"robot"

    rejected = core.handle(
        b"robot",
        make_envelope(
            "motion_command",
            "robot-a",
            target_id="robot-a",
            lease_id=lease_id,
            payload={"command": "target", "q": [0.0, 0.1, 0.2, 0.3]},
            seq=2,
        ),
        now=1.3,
    )
    assert rejected[0].envelope.message_type == "error"


def test_operator_traffic_only_crosses_ui_controller_boundary() -> None:
    core = RouterCore()
    register(core, b"ui", "ui-a", "ui", ())
    register(core, b"controller", "controller-a", "controller", (CAPABILITY_OPERATOR_CONTROL,))

    intent = core.handle(
        b"ui",
        make_envelope(
            "operator_intent",
            "ui-a",
            target_id="controller-a",
            payload={"request_id": "r1", "operation": "snapshot"},
            seq=2,
        ),
        now=1.1,
    )
    assert intent[0].identity == b"controller"

    result = core.handle(
        b"controller",
        make_envelope(
            "operator_result",
            "controller-a",
            target_id="ui-a",
            payload={"request_id": "r1", "ok": True},
            seq=2,
        ),
        now=1.2,
    )
    assert result[0].identity == b"ui"


def test_global_sequence_rejects_replay_across_message_types() -> None:
    core = RouterCore()
    register(core, b"ui", "ui-a", "ui", ())
    core.handle(b"ui", make_envelope("heartbeat", "ui-a", seq=2), now=1.1)
    replay = core.handle(b"ui", make_envelope("discover", "ui-a", seq=2), now=1.2)
    assert replay[0].envelope.message_type == "error"
    assert "stale sequence" in str(replay[0].envelope.payload["reason"])


def test_register_sequence_is_part_of_global_replay_protection() -> None:
    core = RouterCore()
    register(core, b"robot", "robot-a", "robot", (CAPABILITY_MOTION_ARM,))
    replay = core.handle(
        b"robot",
        make_envelope("heartbeat", "robot-a", seq=1),
        now=1.1,
    )
    assert replay[0].envelope.message_type == "error"
    assert "stale sequence" in str(replay[0].envelope.payload["reason"])


def test_live_endpoint_id_cannot_be_taken_by_another_instance() -> None:
    core = RouterCore()
    register(core, b"first", "robot-a", "robot", (CAPABILITY_MOTION_ARM,))
    intruder = EndpointDescriptor(
        "robot-a",
        "robot",
        (CAPABILITY_MOTION_ARM,),
        instance_id="different-instance",
    )

    reply = core.handle(
        b"second",
        make_envelope(
            "register",
            "robot-a",
            payload={"endpoint": intruder.to_dict()},
            seq=1,
        ),
        now=1.1,
    )

    assert reply[-1].envelope.message_type == "error"
    assert "already registered" in str(reply[-1].envelope.payload["reason"])
    assert core.endpoints["robot-a"].identity == b"first"


def test_same_instance_reconnect_revokes_old_lease_before_registration() -> None:
    core = RouterCore()
    register(core, b"controller", "controller-a", "controller", (CAPABILITY_OPERATOR_CONTROL,))
    register(core, b"robot-old", "robot-a", "robot", (CAPABILITY_MOTION_ARM,))
    core.handle(
        b"controller",
        make_envelope(
            "select_target",
            "controller-a",
            payload={"target_id": "robot-a"},
            seq=2,
        ),
        now=1.1,
    )
    descriptor = EndpointDescriptor(
        "robot-a",
        "robot",
        (CAPABILITY_MOTION_ARM,),
        instance_id="robot-a-instance",
    )

    routed = core.handle(
        b"robot-new",
        make_envelope(
            "register",
            "robot-a",
            payload={"endpoint": descriptor.to_dict()},
            seq=3,
        ),
        now=1.2,
    )

    assert routed[0].identity == b"controller"
    assert routed[0].envelope.message_type == "target_lost"
    assert routed[-1].identity == b"robot-new"
    assert routed[-1].envelope.message_type == "registered"
    assert core.endpoints["robot-a"].identity == b"robot-new"


def test_router_rejects_invalid_motion_payload_before_forwarding() -> None:
    core = RouterCore()
    register(core, b"controller", "controller-a", "controller", (CAPABILITY_OPERATOR_CONTROL,))
    register(core, b"robot", "robot-a", "robot", (CAPABILITY_MOTION_ARM,))
    selected = core.handle(
        b"controller",
        make_envelope(
            "select_target",
            "controller-a",
            payload={"target_id": "robot-a"},
            seq=2,
        ),
        now=1.1,
    )[-1].envelope
    lease_id = str(selected.payload["lease_id"])

    rejected = core.handle(
        b"controller",
        make_envelope(
            "motion_command",
            "controller-a",
            target_id="robot-a",
            lease_id=lease_id,
            payload={"command": "target", "u": {"linear": 10}},
            seq=3,
        ),
        now=1.2,
    )

    assert rejected[0].identity == b"controller"
    assert rejected[0].envelope.message_type == "error"
    assert "legacy u" in str(rejected[0].envelope.payload["reason"])


def test_server_reply_preserves_trace_context() -> None:
    core = RouterCore()
    descriptor = EndpointDescriptor("ui-a", "ui", instance_id="ui-instance")
    reply = core.handle(
        b"ui",
        make_envelope(
            "register",
            "ui-a",
            payload={"endpoint": descriptor.to_dict()},
            seq=1,
            trace_context={"traceparent": "00-abc-def-01"},
        ),
        now=1.0,
    )[-1].envelope
    assert reply.trace_context == {"traceparent": "00-abc-def-01"}
