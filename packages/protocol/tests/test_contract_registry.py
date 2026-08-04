from __future__ import annotations

import pytest

from elesim_protocol.contracts import DDS_CONTRACTS, validate_registry
from elesim_protocol.messages import MESSAGE_TYPES, ProtocolError
from elesim_protocol.payloads import validate_routed_payload


def test_every_declared_peer_message_has_one_contract() -> None:
    validate_registry()
    assert set(DDS_CONTRACTS) == set(MESSAGE_TYPES)


@pytest.mark.parametrize("message_type", ("release_target", "renew_target", "renew_simulation_session"))
def test_empty_authority_renewals_reject_accidental_fields(message_type: str) -> None:
    with pytest.raises(ProtocolError):
        validate_routed_payload(message_type, {"unexpected": True})


def test_error_and_ack_fields_are_bounded() -> None:
    assert validate_routed_payload("ack", {"ok": True, "reason": "ok"})["ok"] is True
    with pytest.raises(ProtocolError):
        validate_routed_payload("error", {"reply_to": "", "reason": "bad"})
