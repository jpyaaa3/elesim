from __future__ import annotations

import json

import pytest

from elesim_protocol import (
    CAPABILITY_MOTION_ARM,
    EndpointDescriptor,
    PROTOCOL_VERSION,
    ProtocolError,
    dumps_envelope,
    encode_value,
    loads_envelope,
    make_envelope,
)


def test_v3_envelope_round_trip() -> None:
    descriptor = EndpointDescriptor("robot-a", "robot", (CAPABILITY_MOTION_ARM,))
    message = make_envelope(
        "register",
        "robot-a",
        payload={"endpoint": descriptor.to_dict()},
        seq=3,
        trace_context={"traceparent": "00-abc-def-01"},
    )
    decoded = loads_envelope(dumps_envelope(message))
    assert decoded.version == PROTOCOL_VERSION == 3
    assert decoded.trace_context == {"traceparent": "00-abc-def-01"}
    assert EndpointDescriptor.from_dict(decoded.payload["endpoint"]) == descriptor


def test_v2_is_rejected() -> None:
    message = make_envelope("heartbeat", "robot-a", seq=1).to_dict()
    message["version"] = 2
    with pytest.raises(ProtocolError, match="expected 3"):
        loads_envelope(json.dumps(message).encode())


def test_old_sim_role_is_rejected() -> None:
    with pytest.raises(ProtocolError, match="unsupported endpoint role"):
        EndpointDescriptor("sim-a", "sim")


@pytest.mark.parametrize("field", ("timestamp", "seq"))
def test_nonfinite_or_nonintegral_envelope_metadata_is_rejected(field: str) -> None:
    message = make_envelope("heartbeat", "robot-a", seq=1).to_dict()
    message[field] = float("nan") if field == "timestamp" else 1.5
    with pytest.raises(ProtocolError):
        loads_envelope(json.dumps(message).encode())


def test_oversized_envelope_is_rejected_before_json_parsing() -> None:
    message = make_envelope("heartbeat", "robot-a", payload={"padding": "x" * 1_100_000})
    with pytest.raises(ProtocolError, match="too large"):
        loads_envelope(dumps_envelope(message))


@pytest.mark.parametrize("value", (float("nan"), float("inf"), float("-inf")))
def test_operator_value_encoder_rejects_nonfinite_numbers(value: float) -> None:
    with pytest.raises(ProtocolError, match="non-finite"):
        encode_value({"nested": [value]})
