"""The authoritative registry for the bounded DDS ``PeerEnvelope`` surface.

The transport intentionally carries one bounded envelope rather than exposing
application objects to sibling processes.  This registry keeps the remaining
wire contract reviewable: every message type has an owner, a direction, a QoS
class and a payload policy.  The values are documentation data as well as a
small runtime guard used by :mod:`elesim_protocol.payloads`.

The registry is deliberately kept independent of the payload DTO module to
avoid an import cycle.  A protocol major bump is required before adding a new
message type or changing one of the listed field sets.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from .messages import MESSAGE_TYPES, ProtocolError


@dataclass(frozen=True)
class DdsContract:
    message_type: str
    sender_roles: tuple[str, ...]
    receiver_roles: tuple[str, ...]
    carrier: str
    qos: str
    authority: str
    payload_fields: tuple[str, ...] | None = None
    strict_fields: bool = False
    notes: str = ""

    def validate(self) -> "DdsContract":
        if self.message_type not in MESSAGE_TYPES:
            raise ProtocolError(f"contract is not a declared message type: {self.message_type}")
        if not self.sender_roles or not self.receiver_roles:
            raise ProtocolError(f"contract {self.message_type} must name both endpoints")
        if self.payload_fields is not None and len(set(self.payload_fields)) != len(self.payload_fields):
            raise ProtocolError(f"contract {self.message_type} repeats a payload field")
        return self


def _contract(
    message_type: str,
    sender_roles: tuple[str, ...],
    receiver_roles: tuple[str, ...],
    *,
    qos: str = "reliable-control",
    authority: str = "none",
    payload_fields: tuple[str, ...] | None = None,
    strict_fields: bool = False,
    notes: str = "",
) -> DdsContract:
    return DdsContract(
        message_type=message_type,
        sender_roles=sender_roles,
        receiver_roles=receiver_roles,
        carrier="PeerEnvelope DDS /elesim/<system>/v6/control",
        qos=qos,
        authority=authority,
        payload_fields=payload_fields,
        strict_fields=strict_fields,
        notes=notes,
    ).validate()


_ALL = ("pilot", "robot", "sim", "ui")
_CONTROL = ("reliable-control",)


# Keep this table explicit rather than deriving it from source imports.  A
# review can therefore compare the complete protocol surface without opening
# each application package.  ``None`` means a domain-specific DTO owns the
# fields (or a deliberately open telemetry/operator result object is used).
DDS_CONTRACTS: Mapping[str, DdsContract] = {
    "discover": _contract(
        "discover", ("pilot", "ui"), _ALL,
        payload_fields=("role", "capability"), strict_fields=True,
    ),
    "endpoint_list": _contract(
        "endpoint_list", _ALL, ("pilot", "ui"),
        payload_fields=("endpoints",), strict_fields=True,
    ),
    "operator_intent": _contract(
        "operator_intent", ("ui",), ("pilot",),
        authority="pilot workflow", notes="validated OperatorIntentRequest",
    ),
    "operator_result": _contract(
        "operator_result", ("pilot",), ("ui",),
        payload_fields=("request_id", "ok", "result", "error"), strict_fields=True,
    ),
    "select_target": _contract(
        "select_target", ("pilot",), ("robot", "sim"),
        authority="target owner motion lease", payload_fields=("target_id",), strict_fields=True,
    ),
    "target_selected": _contract(
        "target_selected", ("robot", "sim"), ("pilot",),
        authority="target owner motion lease", payload_fields=("target_id", "lease_id"), strict_fields=True,
    ),
    "renew_target": _contract(
        "renew_target", ("pilot",), ("robot", "sim"),
        authority="target owner motion lease", payload_fields=(), strict_fields=True,
    ),
    "release_target": _contract(
        "release_target", ("pilot",), ("robot", "sim"),
        authority="target owner motion lease", payload_fields=(), strict_fields=True,
    ),
    "target_released": _contract(
        "target_released", ("robot", "sim"), ("pilot",),
        authority="target owner motion lease", payload_fields=("target_id", "reason"), strict_fields=True,
    ),
    "target_lost": _contract(
        "target_lost", ("robot", "sim"), ("pilot",),
        authority="target owner motion lease", payload_fields=("target_id", "reason"), strict_fields=True,
    ),
    "lease_granted": _contract(
        "lease_granted", ("robot", "sim"), ("robot", "sim", "pilot"),
        authority="target owner motion lease", payload_fields=("pilot_id",), strict_fields=True,
    ),
    "lease_revoked": _contract(
        "lease_revoked", ("robot", "sim"), ("robot", "sim", "pilot"),
        authority="target owner motion lease", payload_fields=("reason",), strict_fields=True,
    ),
    "motion_command": _contract(
        "motion_command", ("pilot",), ("robot", "sim"),
        qos="best-effort-motion-depth-1", authority="target owner motion lease",
        notes="validated MotionCommandRequest; estop remains local-safe",
    ),
    "telemetry": _contract(
        "telemetry", ("robot", "sim"), ("pilot", "ui"),
        qos="reliable-control", notes="bounded state map; fields are additive by design",
    ),
    "ack": _contract(
        "ack", _ALL, _ALL,
        payload_fields=("reply_to", "ok", "reason"), strict_fields=True,
    ),
    "open_simulation_session": _contract(
        "open_simulation_session", ("ui",), ("sim",),
        authority="sim UI-session lease", notes="validated OpenSimulationSessionRequest",
    ),
    "simulation_session_opened": _contract(
        "simulation_session_opened", ("sim",), ("ui",),
        authority="sim UI-session lease", notes="validated SimulationSessionOpenedPayload",
    ),
    "simulation_session_granted": _contract(
        "simulation_session_granted", ("sim",), ("sim",),
        authority="sim UI-session lease", notes="local sim notification",
    ),
    "simulation_session_revoked": _contract(
        "simulation_session_revoked", ("sim",), ("ui", "sim"),
        authority="sim UI-session lease", notes="validated SimulationSessionRevokedPayload",
    ),
    "renew_simulation_session": _contract(
        "renew_simulation_session", ("ui",), ("sim",),
        authority="sim UI-session lease", payload_fields=(), strict_fields=True,
    ),
    "close_simulation_session": _contract(
        "close_simulation_session", ("ui",), ("sim",),
        authority="sim UI-session lease", notes="validated CloseSimulationSessionRequest",
    ),
    "simulation_command": _contract(
        "simulation_command", ("ui",), ("sim",),
        authority="sim UI-session lease", notes="validated SimulationCommandRequest",
    ),
    "simulation_result": _contract(
        "simulation_result", ("sim",), ("ui",),
        authority="sim UI-session lease", notes="validated SimulationResultPayload",
    ),
    "simulation_status": _contract(
        "simulation_status", ("sim",), ("ui", "pilot"),
        qos="reliable-control", authority="sim UI-session lease",
        notes="validated SimulationStatusPayload",
    ),
    "webrtc_signal": _contract(
        "webrtc_signal", ("ui", "sim"), ("sim", "ui"),
        authority="sim UI-session lease", notes="validated WebRtcSignalPayload; pixels stay DTLS/SRTP",
    ),
    "error": _contract(
        "error", _ALL, _ALL,
        payload_fields=("reply_to", "reason"), strict_fields=True,
    ),
}


def contract_for(message_type: str) -> DdsContract:
    try:
        return DDS_CONTRACTS[str(message_type)]
    except KeyError as exc:
        raise ProtocolError(f"no DDS contract is registered for {message_type!r}") from exc


def validate_registry() -> None:
    missing = sorted(set(MESSAGE_TYPES) - set(DDS_CONTRACTS))
    extra = sorted(set(DDS_CONTRACTS) - set(MESSAGE_TYPES))
    if missing or extra:
        details = []
        if missing:
            details.append("missing: " + ", ".join(missing))
        if extra:
            details.append("extra: " + ", ".join(extra))
        raise ProtocolError("DDS contract registry mismatch (" + "; ".join(details) + ")")
    for value in DDS_CONTRACTS.values():
        value.validate()


validate_registry()

__all__ = ["DDS_CONTRACTS", "DdsContract", "contract_for", "validate_registry"]
