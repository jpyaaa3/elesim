from __future__ import annotations

import json

import pytest

from elesim_protocol import (
    CAPABILITY_MOTION_ARM,
    EndpointDescriptor,
    MediaStreamDescriptor,
    PROTOCOL_VERSION,
    ProtocolError,
    dumps_envelope,
    encode_value,
    loads_envelope,
    make_envelope,
)


def test_v6_envelope_round_trip() -> None:
    descriptor = EndpointDescriptor(
        "robot-a",
        "robot",
        (CAPABILITY_MOTION_ARM,),
        streams={
            "rgbd": MediaStreamDescriptor(
                transport="dds",
                media_kind="rgbd",
                endpoint="/elesim/robot_a/rgbd/frame",
                security="none",
            )
        },
    )
    message = make_envelope(
        "endpoint_list",
        "robot-a",
        payload={"endpoints": [descriptor.to_dict()]},
        seq=3,
        trace_context={"traceparent": "00-abc-def-01"},
    )
    decoded = loads_envelope(dumps_envelope(message))
    assert decoded.version == PROTOCOL_VERSION == 6
    assert decoded.trace_context == {"traceparent": "00-abc-def-01"}
    assert EndpointDescriptor.from_dict(decoded.payload["endpoints"][0]) == descriptor


def test_v5_is_rejected() -> None:
    message = make_envelope("discover", "robot-a", seq=1).to_dict()
    message["version"] = 5
    with pytest.raises(ProtocolError, match="expected 6"):
        loads_envelope(json.dumps(message).encode())


def test_unknown_legacy_simulator_role_is_rejected() -> None:
    with pytest.raises(ProtocolError, match="unsupported endpoint role"):
        EndpointDescriptor("sim-a", "simulator")


def test_media_stream_descriptor_rejects_invalid_security_combinations() -> None:
    with pytest.raises(ProtocolError, match="DDS streams"):
        MediaStreamDescriptor(
            transport="dds",
            media_kind="rgbd",
            endpoint="/elesim/sim_a/rgbd/frame",
            security="dtls-srtp",
        )
    with pytest.raises(ProtocolError, match="DTLS-SRTP"):
        MediaStreamDescriptor(
            transport="webrtc",
            media_kind="rgb",
            endpoint="webrtc://sim-a/observer",
            security="none",
        )


def test_endpoint_descriptor_rejects_legacy_string_streams() -> None:
    with pytest.raises(ProtocolError, match="media stream descriptor"):
        EndpointDescriptor.from_dict(
            {
                "endpoint_id": "sim-a",
                "role": "sim",
                "capabilities": [],
                "streams": {"rgbd": "/elesim/sim_a/rgbd/frame"},
                "instance_id": "sim-instance",
            }
        )


def test_protocol_v6_has_no_legacy_camera_input_message() -> None:
    with pytest.raises(ProtocolError, match="unsupported message type"):
        make_envelope(
            "camera_input",
            "ui-a",
            target_id="sim-a",
            payload={"command": "orbit", "values": [0.1, 0.2]},
        )


@pytest.mark.parametrize("field", ("timestamp", "seq"))
def test_nonfinite_or_nonintegral_envelope_metadata_is_rejected(field: str) -> None:
    message = make_envelope("discover", "robot-a", seq=1).to_dict()
    message[field] = float("nan") if field == "timestamp" else 1.5
    with pytest.raises(ProtocolError):
        loads_envelope(json.dumps(message).encode())


def test_oversized_envelope_is_rejected_before_json_parsing() -> None:
    message = make_envelope("discover", "robot-a", payload={"padding": "x" * 1_100_000})
    with pytest.raises(ProtocolError, match="too large"):
        loads_envelope(dumps_envelope(message))


@pytest.mark.parametrize("value", (float("nan"), float("inf"), float("-inf")))
def test_operator_value_encoder_rejects_nonfinite_numbers(value: float) -> None:
    with pytest.raises(ProtocolError, match="non-finite"):
        encode_value({"nested": [value]})
