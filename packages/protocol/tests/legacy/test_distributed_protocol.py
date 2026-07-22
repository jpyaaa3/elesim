from __future__ import annotations

import pytest

from elesim_protocol import (
    EndpointDescriptor,
    ProtocolError,
    dumps_envelope,
    loads_envelope,
    make_envelope,
)


def test_v4_envelope_round_trip() -> None:
    envelope = make_envelope(
        "motion_command",
        "controller-a",
        target_id="robot-a",
        payload={"command": "target", "q": [0.0, 0.1, 0.2, 0.3]},
        seq=7,
        lease_id="lease-a",
    )
    restored = loads_envelope(dumps_envelope(envelope))
    assert restored.message_type == "motion_command"
    assert restored.source_id == "controller-a"
    assert restored.target_id == "robot-a"
    assert restored.seq == 7
    assert restored.lease_id == "lease-a"


def test_v1_message_is_rejected() -> None:
    with pytest.raises(ProtocolError, match="missing envelope fields"):
        loads_envelope(b'{"t":"target"}')


def test_endpoint_descriptor_validates_role() -> None:
    with pytest.raises(ProtocolError, match="unsupported endpoint role"):
        EndpointDescriptor("bad", "planner")
