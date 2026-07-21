from __future__ import annotations

from elesim_protocol import (
    CAPABILITY_MOTION_ARM,
    CAPABILITY_OPERATOR_CONTROL,
    CAPABILITY_STREAM_HAND_EYE_PREVIEW,
    CAPABILITY_STREAM_OBSERVER,
    EndpointDescriptor,
    MEDIA_KIND_RGB,
    MEDIA_SECURITY_DTLS_SRTP,
    MEDIA_TRANSPORT_WEBRTC,
    MediaStreamDescriptor,
    make_envelope,
)
from elesim_router.core import RouterCore
from elesim_router.simulation_sessions import TurnCredentialIssuer


def register(
    core: RouterCore,
    identity: bytes,
    endpoint_id: str,
    role: str,
    capabilities: tuple[str, ...],
    streams: dict[str, MediaStreamDescriptor] | None = None,
) -> None:
    descriptor = EndpointDescriptor(
        endpoint_id,
        role,
        capabilities=capabilities,
        streams=streams,
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


def simulator_streams() -> dict[str, MediaStreamDescriptor]:
    return {
        name: MediaStreamDescriptor(
            transport=MEDIA_TRANSPORT_WEBRTC,
            media_kind=MEDIA_KIND_RGB,
            endpoint=f"webrtc://sim-a/{name}",
            security=MEDIA_SECURITY_DTLS_SRTP,
        )
        for name in ("observer", "hand_eye_preview")
    }


def open_session(
    core: RouterCore,
    *,
    ui: bytes = b"ui",
    ui_id: str = "ui-a",
    seq: int = 2,
):
    return core.handle(
        ui,
        make_envelope(
            "open_simulation_session",
            ui_id,
            payload={
                "schema_version": 1,
                "request_id": "open-1",
                "simulator_id": "sim-a",
                "streams": ["observer", "hand_eye_preview"],
            },
            seq=seq,
        ),
        now=1.2,
        wall_time=1000.0,
    )


def test_simulation_session_is_independent_from_controller_motion_lease() -> None:
    core = RouterCore()
    register(core, b"controller", "controller-a", "controller", (CAPABILITY_OPERATOR_CONTROL,))
    register(core, b"ui", "ui-a", "ui", ())
    register(
        core,
        b"sim",
        "sim-a",
        "simulator",
        (
            CAPABILITY_MOTION_ARM,
            CAPABILITY_STREAM_OBSERVER,
            CAPABILITY_STREAM_HAND_EYE_PREVIEW,
        ),
        streams=simulator_streams(),
    )
    motion = core.handle(
        b"controller",
        make_envelope(
            "select_target",
            "controller-a",
            payload={"target_id": "sim-a"},
            seq=2,
        ),
        now=1.1,
    )
    session = open_session(core)

    assert motion[-1].envelope.message_type == "target_selected"
    assert [item.envelope.message_type for item in session] == [
        "simulation_session_granted",
        "simulation_session_opened",
    ]
    assert core.active_target_by_controller["controller-a"] == "sim-a"
    assert core.simulation_sessions.by_ui["ui-a"].simulator_id == "sim-a"


def test_only_one_ui_can_control_a_simulator_session() -> None:
    core = RouterCore()
    register(core, b"ui-a", "ui-a", "ui", ())
    register(core, b"ui-b", "ui-b", "ui", ())
    register(
        core,
        b"sim",
        "sim-a",
        "simulator",
        (CAPABILITY_STREAM_OBSERVER, CAPABILITY_STREAM_HAND_EYE_PREVIEW),
        streams=simulator_streams(),
    )
    open_session(core, ui=b"ui-a")
    rejected = open_session(core, ui=b"ui-b", ui_id="ui-b")

    assert rejected[0].envelope.message_type == "error"
    assert "already has an operator" in str(rejected[0].envelope.payload["reason"])


def test_simulation_command_and_webrtc_require_the_active_session() -> None:
    core = RouterCore()
    register(core, b"ui", "ui-a", "ui", ())
    register(
        core,
        b"sim",
        "sim-a",
        "simulator",
        (CAPABILITY_STREAM_OBSERVER, CAPABILITY_STREAM_HAND_EYE_PREVIEW),
        streams=simulator_streams(),
    )
    opened = open_session(core)
    session_id = str(opened[-1].envelope.payload["session_id"])

    command = core.handle(
        b"ui",
        make_envelope(
            "simulation_command",
            "ui-a",
            target_id="sim-a",
            lease_id=session_id,
            payload={
                "schema_version": 1,
                "request_id": "pause-1",
                "session_id": session_id,
                "command": "pause",
                "arguments": {},
            },
            seq=3,
        ),
        now=1.3,
    )
    assert command[0].identity == b"sim"

    wrong = core.handle(
        b"ui",
        make_envelope(
            "webrtc_signal",
            "ui-a",
            target_id="sim-a",
            lease_id="wrong",
            payload={
                "schema_version": 1,
                "session_id": session_id,
                "stream": "observer",
                "signal": "offer",
                "sdp": "v=0\r\n",
                "type": "offer",
            },
            seq=4,
        ),
        now=1.4,
    )
    assert wrong[0].envelope.message_type == "error"
    assert "simulation session" in str(wrong[0].envelope.payload["reason"])


def test_simulation_status_fans_out_to_operator_and_motion_controller() -> None:
    core = RouterCore()
    register(core, b"controller", "controller-a", "controller", (CAPABILITY_OPERATOR_CONTROL,))
    register(core, b"ui", "ui-a", "ui", ())
    register(
        core,
        b"sim",
        "sim-a",
        "simulator",
        (CAPABILITY_MOTION_ARM, CAPABILITY_STREAM_OBSERVER, CAPABILITY_STREAM_HAND_EYE_PREVIEW),
        streams=simulator_streams(),
    )
    core.handle(
        b"controller",
        make_envelope("select_target", "controller-a", payload={"target_id": "sim-a"}, seq=2),
        now=1.1,
    )
    open_session(core)

    routed = core.handle(
        b"sim",
        make_envelope(
            "simulation_status",
            "sim-a",
            payload={
                "schema_version": 1,
                "epoch": 2,
                "paused": True,
                "speed": 1.0,
                "debug_visible": True,
                "sim_time_s": 3.5,
            },
            seq=2,
        ),
        now=1.3,
    )
    assert {item.identity for item in routed} == {b"ui", b"controller"}
    assert all(item.envelope.target_id in {"ui-a", "controller-a"} for item in routed)


def test_closing_session_revokes_simulator_and_ui_without_touching_motion() -> None:
    core = RouterCore()
    register(core, b"controller", "controller-a", "controller", (CAPABILITY_OPERATOR_CONTROL,))
    register(core, b"ui", "ui-a", "ui", ())
    register(
        core,
        b"sim",
        "sim-a",
        "simulator",
        (CAPABILITY_MOTION_ARM, CAPABILITY_STREAM_OBSERVER, CAPABILITY_STREAM_HAND_EYE_PREVIEW),
        streams=simulator_streams(),
    )
    core.handle(
        b"controller",
        make_envelope("select_target", "controller-a", payload={"target_id": "sim-a"}, seq=2),
        now=1.1,
    )
    opened = open_session(core)
    session_id = str(opened[-1].envelope.payload["session_id"])
    closed = core.handle(
        b"ui",
        make_envelope(
            "close_simulation_session",
            "ui-a",
            payload={
                "schema_version": 1,
                "request_id": "close-1",
                "session_id": session_id,
            },
            seq=3,
        ),
        now=1.3,
    )

    assert [item.envelope.message_type for item in closed] == [
        "simulation_session_revoked",
        "simulation_session_revoked",
    ]
    assert core.active_target_by_controller["controller-a"] == "sim-a"
    assert not core.simulation_sessions.by_id


def test_turn_rest_credentials_use_expiring_hmac() -> None:
    issuer = TurnCredentialIssuer(
        urls=("turn:relay.example:3478?transport=udp",),
        static_auth_secret=b"shared-secret",
        ttl_s=3600,
        refresh_before_s=600,
    )
    first = issuer.issue("ui-a", "session-a", now=1000.0)
    second = issuer.issue("ui-a", "session-a", now=1000.0)
    later = issuer.issue("ui-a", "session-a", now=1001.0)

    assert first == second
    assert first != later
    assert first.expires_at == 4600.0
    assert first.username.startswith("4600:")
