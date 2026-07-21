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
    DiscoverRequest,
    MotionCommandRequest,
    OperatorIntentRequest,
    OperatorViewSnapshot,
    RegisterRequest,
    SelectTargetRequest,
    TelemetryPayload,
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
