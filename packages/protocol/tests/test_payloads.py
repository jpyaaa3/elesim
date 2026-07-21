from __future__ import annotations

import math

import pytest

from elesim_protocol import (
    OPERATOR_VIEW_SCHEMA_VERSION,
    EndpointDescriptor,
    ProtocolError,
    STATE_VALUES,
    SERVICE_CALLS,
    SERVICE_VALUES,
)
from elesim_protocol.payloads import (
    CloseSimulationSessionRequest,
    DiscoverRequest,
    MotionCommandRequest,
    OpenSimulationSessionRequest,
    OperatorIntentRequest,
    OperatorViewSnapshot,
    RegisterRequest,
    SelectTargetRequest,
    SimulationCommandRequest,
    SimulationResultPayload,
    SimulationSessionGrantedPayload,
    SimulationSessionOpenedPayload,
    SimulationSessionRevokedPayload,
    SimulationStatusPayload,
    TelemetryPayload,
    TurnCredentials,
    WebRtcSignalPayload,
    validate_routed_payload,
)


def test_register_requires_wire_instance_identity() -> None:
    descriptor = EndpointDescriptor("robot-a", "robot", instance_id="instance-a")
    parsed = RegisterRequest.from_payload({"endpoint": descriptor.to_dict()})
    assert parsed.endpoint == descriptor

    missing = EndpointDescriptor("robot-a", "robot")
    with pytest.raises(ProtocolError, match="instance_id"):
        RegisterRequest.from_payload({"endpoint": missing.to_dict()})


def test_discovery_and_target_selection_are_typed() -> None:
    assert DiscoverRequest.from_payload({"role": "robot", "capability": "motion.arm"}).role == "robot"
    assert SelectTargetRequest.from_payload({"target_id": "robot-a"}).target_id == "robot-a"
    with pytest.raises(ProtocolError, match="unknown discover payload fields"):
        DiscoverRequest.from_payload({"role": "robot", "typo": True})


def test_motion_contract_rejects_legacy_u_and_invalid_numeric_values() -> None:
    parsed = MotionCommandRequest.from_payload(
        {"command": "target", "q": [-0.1, 0.0, 0.1, -0.1]}
    )
    assert parsed.command == "target"
    assert parsed.q == (-0.1, 0.0, 0.1, -0.1)

    with pytest.raises(ProtocolError, match="legacy u"):
        MotionCommandRequest.from_payload({"command": "target", "u": {"linear": 1}})
    with pytest.raises(ProtocolError, match="finite"):
        MotionCommandRequest.from_payload({"command": "target", "q": [math.nan, 0, 0, 0]})
    with pytest.raises(ProtocolError, match="four"):
        MotionCommandRequest.from_payload({"command": "target", "q": [0, 0, 0]})


def test_telemetry_q_is_a_canonical_four_vector() -> None:
    assert TelemetryPayload.from_payload({"q": [-0.1, 0.0, 0.1, -0.1]}).q == (
        -0.1,
        0.0,
        0.1,
        -0.1,
    )
    with pytest.raises(ProtocolError, match="telemetry q"):
        TelemetryPayload.from_payload({"q": {"linear_m": -0.1}})


def test_operator_intent_requires_known_operation_and_request_id() -> None:
    parsed = OperatorIntentRequest.from_payload(
        {"request_id": "request-a", "operation": "snapshot", "name": ""}
    )
    assert parsed.operation == "snapshot"
    with pytest.raises(ProtocolError, match="unsupported operator operation"):
        OperatorIntentRequest.from_payload(
            {"request_id": "request-a", "operation": "run_arbitrary_python"}
        )


def test_operator_state_surface_does_not_publish_arbitrary_attributes() -> None:
    assert "visual_target_scale" in STATE_VALUES
    assert "unpublished_internal_value" not in STATE_VALUES


def test_operator_view_snapshot_has_an_explicit_versioned_shape() -> None:
    parsed = OperatorViewSnapshot.from_payload(
        {
            "schema_version": OPERATOR_VIEW_SCHEMA_VERSION,
            "state": {"pick_running": True},
            "service": {"has_client": True, "active_endpoint": "robot-a"},
        }
    )

    assert parsed.state == {"pick_running": True}
    assert parsed.service["active_endpoint"] == "robot-a"
    assert parsed.to_payload()["schema_version"] == OPERATOR_VIEW_SCHEMA_VERSION

    with pytest.raises(ProtocolError, match="unsupported operator view schema"):
        OperatorViewSnapshot.from_payload(
            {"schema_version": 999, "state": {}, "service": {}}
        )
    with pytest.raises(ProtocolError, match="unknown operator view snapshot"):
        OperatorViewSnapshot.from_payload(
            {
                "schema_version": OPERATOR_VIEW_SCHEMA_VERSION,
                "state": {},
                "service": {},
                "surprise": True,
            }
        )


def test_operator_view_snapshot_is_a_known_intent_operation() -> None:
    parsed = OperatorIntentRequest.from_payload(
        {"request_id": "request-view", "operation": "view_snapshot", "name": ""}
    )
    assert parsed.operation == "view_snapshot"


def test_operator_allowlist_distinguishes_methods_from_properties() -> None:
    assert "current_host_state" in SERVICE_CALLS
    assert "current_host_state" not in SERVICE_VALUES
    assert "available_endpoints" in SERVICE_VALUES
    assert "_pick_config_effective" not in SERVICE_VALUES


def test_simulation_session_open_and_close_contracts_are_bounded() -> None:
    opened = OpenSimulationSessionRequest.from_payload(
        {
            "schema_version": 1,
            "request_id": "open-1",
            "simulator_id": "sim-a",
            "streams": ["observer", "hand_eye_preview"],
        }
    )
    assert opened.streams == ("observer", "hand_eye_preview")
    assert CloseSimulationSessionRequest.from_payload(
        {"schema_version": 1, "request_id": "close-1", "session_id": "session-a"}
    ).session_id == "session-a"

    with pytest.raises(ProtocolError, match="unsupported simulation stream"):
        OpenSimulationSessionRequest.from_payload(
            {
                "schema_version": 1,
                "request_id": "open-2",
                "simulator_id": "sim-a",
                "streams": ["native_viewer"],
            }
        )


@pytest.mark.parametrize(
    ("command", "arguments"),
    (
        ("orbit", {"dx": 0.1, "dy": -0.2}),
        ("pan", {"dx": -0.1, "dy": 0.2}),
        ("zoom", {"delta": 0.08}),
        ("reset_view", {}),
        ("pause", {}),
        ("resume", {}),
        ("step", {"count": 3}),
        ("reset", {}),
        ("set_speed", {"scale": 0.5}),
        ("set_debug_visible", {"visible": False}),
    ),
)
def test_simulation_command_contract_accepts_the_public_command_surface(
    command: str,
    arguments: dict[str, object],
) -> None:
    parsed = SimulationCommandRequest.from_payload(
        {
            "schema_version": 1,
            "request_id": f"request-{command}",
            "session_id": "session-a",
            "command": command,
            "arguments": arguments,
        }
    )
    assert parsed.command == command
    assert parsed.arguments == arguments


@pytest.mark.parametrize(
    "payload",
    (
        {
            "schema_version": 1,
            "request_id": "bad-step",
            "session_id": "session-a",
            "command": "step",
            "arguments": {"count": 121},
        },
        {
            "schema_version": 1,
            "request_id": "bad-speed",
            "session_id": "session-a",
            "command": "set_speed",
            "arguments": {"scale": 20.0},
        },
        {
            "schema_version": 1,
            "request_id": "bad-orbit",
            "session_id": "session-a",
            "command": "orbit",
            "arguments": {"dx": math.nan, "dy": 0.0},
        },
        {
            "schema_version": 1,
            "request_id": "bad-extra",
            "session_id": "session-a",
            "command": "pause",
            "arguments": {"surprise": True},
        },
    ),
)
def test_simulation_command_contract_rejects_unsafe_or_unknown_arguments(
    payload: dict[str, object],
) -> None:
    with pytest.raises(ProtocolError):
        SimulationCommandRequest.from_payload(payload)


def test_webrtc_signal_is_tied_to_a_session_and_named_stream() -> None:
    parsed = WebRtcSignalPayload.from_payload(
        {
            "schema_version": 1,
            "session_id": "session-a",
            "stream": "observer",
            "signal": "offer",
            "sdp": "v=0\r\n",
            "type": "offer",
        }
    )
    assert parsed.stream == "observer"
    with pytest.raises(ProtocolError, match="signal and type"):
        WebRtcSignalPayload.from_payload(
            {
                "schema_version": 1,
                "session_id": "session-a",
                "stream": "observer",
                "signal": "offer",
                "sdp": "v=0\r\n",
                "type": "answer",
            }
        )


def test_session_and_status_payloads_round_trip_their_typed_shape() -> None:
    turn = TurnCredentials(
        urls=("turn:relay.example:3478?transport=udp",),
        username="1730000000:ui-a",
        credential="secret",
        expires_at=1730000000.0,
    )
    opened_raw = {
        "schema_version": 1,
        "request_id": "open-1",
        "session_id": "session-a",
        "simulator_id": "sim-a",
        "streams": ["observer", "hand_eye_preview"],
        "turn": turn.to_payload(),
    }
    assert SimulationSessionOpenedPayload.from_payload(opened_raw).turn == turn
    granted_raw = dict(opened_raw)
    granted_raw["ui_id"] = "ui-a"
    assert SimulationSessionGrantedPayload.from_payload(granted_raw).ui_id == "ui-a"
    assert SimulationSessionRevokedPayload.from_payload(
        {
            "schema_version": 1,
            "session_id": "session-a",
            "simulator_id": "sim-a",
            "reason": "closed",
        }
    ).reason == "closed"
    assert SimulationResultPayload.from_payload(
        {
            "schema_version": 1,
            "request_id": "command-1",
            "session_id": "session-a",
            "command": "pause",
            "ok": True,
            "reason": "",
        }
    ).ok is True
    status = SimulationStatusPayload.from_payload(
        {
            "schema_version": 1,
            "epoch": 2,
            "paused": True,
            "speed": 0.5,
            "debug_visible": False,
            "sim_time_s": 12.25,
        }
    )
    assert status.epoch == 2
    assert status.paused is True


def test_new_routed_payloads_are_validated_before_transport() -> None:
    with pytest.raises(ProtocolError, match="count"):
        validate_routed_payload(
            "simulation_command",
            {
                "schema_version": 1,
                "request_id": "step-1",
                "session_id": "session-a",
                "command": "step",
                "arguments": {"count": 0},
            },
        )
