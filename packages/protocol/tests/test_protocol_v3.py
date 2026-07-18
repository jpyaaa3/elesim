from __future__ import annotations

import json

import pytest

from elesim_protocol import (
    CAPABILITY_MOTION_ARM,
    EndpointDescriptor,
    PROTOCOL_VERSION,
    ProtocolError,
    dumps_envelope,
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
