from __future__ import annotations

import pytest

from elesim_protocol.contracts import DDS_CONTRACTS, validate_registry
from elesim_protocol.messages import MESSAGE_TYPES, ProtocolError
from elesim_protocol.payloads import validate_routed_payload


def test_every_declared_peer_message_has_one_contract() -> None:
    validate_registry()
    assert set(DDS_CONTRACTS) == set(MESSAGE_TYPES)


def test_robot_presentation_extends_telemetry_without_a_new_wire_shape() -> None:
    contract = DDS_CONTRACTS["telemetry"]
    assert "sim" in contract.receiver_roles
    assert contract.qos == "reliable-control"
    assert validate_routed_payload("telemetry", {
        "q": [-0.1, 0.2, 0.3, -0.3], "q_source": "measured",
    }).raw["q_source"] == "measured"


def test_presentation_route_has_no_motion_authority_and_requires_exact_boot():
    contract = DDS_CONTRACTS["presentation_target"]
    assert contract.sender_roles == ("pilot",)
    assert contract.receiver_roles == ("sim",)
    assert validate_routed_payload("presentation_target", {"robot_id": "robot-a", "robot_boot_id": "boot-a"})
    for invalid in ({"robot_id": "robot-a"}, {"robot_id": "robot-a", "robot_boot_id": ""},
                    {"robot_id": "robot-a", "robot_boot_id": "boot-a", "q": [0] * 4}):
        with pytest.raises(ProtocolError):
            validate_routed_payload("presentation_target", invalid)


@pytest.mark.parametrize("message_type", ("release_target", "renew_target", "renew_simulation_session"))
def test_empty_authority_renewals_reject_accidental_fields(message_type: str) -> None:
    with pytest.raises(ProtocolError):
        validate_routed_payload(message_type, {"unexpected": True})


def test_error_and_ack_fields_are_bounded() -> None:
    assert validate_routed_payload("ack", {"ok": True, "reason": "ok"})["ok"] is True
    with pytest.raises(ProtocolError):
        validate_routed_payload("error", {"reply_to": "", "reason": "bad"})
