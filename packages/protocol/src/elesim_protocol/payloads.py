"""Typed protocol-v3 payload contracts.

Envelopes remain JSON objects on the wire. These small DTOs define the fields
that routing and endpoint code may rely on before touching domain state.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real
from typing import Any, Mapping, Optional

from .messages import ENDPOINT_ROLES, EndpointDescriptor, ProtocolError
from .operator import OPERATOR_OPERATIONS, OPERATOR_VIEW_SCHEMA_VERSION


def _object(payload: object, *, context: str) -> dict[str, Any]:
    if not isinstance(payload, Mapping):
        raise ProtocolError(f"{context} payload must be an object")
    return {str(key): value for key, value in payload.items()}


def _unknown(raw: Mapping[str, Any], allowed: set[str], *, context: str) -> None:
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ProtocolError(f"unknown {context} payload fields: {', '.join(unknown)}")


def _identifier(raw: object, *, name: str) -> str:
    value = str(raw or "").strip()
    if not value or len(value) > 128 or any(character.isspace() for character in value):
        raise ProtocolError(f"{name} must contain 1..128 non-whitespace characters")
    return value


def _finite(raw: object, *, name: str) -> float:
    if isinstance(raw, bool) or not isinstance(raw, Real):
        raise ProtocolError(f"{name} must be a finite number")
    value = float(raw)
    if not math.isfinite(value):
        raise ProtocolError(f"{name} must be a finite number")
    return value


def _vector(raw: object, length: int, *, name: str) -> tuple[float, ...]:
    if not isinstance(raw, (list, tuple)) or len(raw) != length:
        count = {3: "three", 4: "four"}.get(length, str(length))
        raise ProtocolError(f"{name} must contain exactly {count} finite numbers")
    return tuple(_finite(value, name=name) for value in raw)


@dataclass(frozen=True)
class RegisterRequest:
    endpoint: EndpointDescriptor

    @classmethod
    def from_payload(cls, payload: object) -> "RegisterRequest":
        raw = _object(payload, context="register")
        _unknown(raw, {"endpoint"}, context="register")
        endpoint_raw = raw.get("endpoint")
        if not isinstance(endpoint_raw, Mapping):
            raise ProtocolError("register payload requires endpoint descriptor")
        endpoint = EndpointDescriptor.from_dict(endpoint_raw)
        if not endpoint.instance_id:
            raise ProtocolError("registered endpoint instance_id must not be empty")
        return cls(endpoint)


@dataclass(frozen=True)
class DiscoverRequest:
    role: str = ""
    capability: str = ""

    @classmethod
    def from_payload(cls, payload: object) -> "DiscoverRequest":
        raw = _object(payload, context="discover")
        _unknown(raw, {"role", "capability"}, context="discover")
        role = str(raw.get("role", "")).strip()
        if role and role not in ENDPOINT_ROLES:
            raise ProtocolError(f"unsupported discovery role: {role!r}")
        capability = str(raw.get("capability", "")).strip()
        if capability:
            _identifier(capability, name="discover capability")
        return cls(role=role, capability=capability)


@dataclass(frozen=True)
class SelectTargetRequest:
    target_id: str

    @classmethod
    def from_payload(cls, payload: object) -> "SelectTargetRequest":
        raw = _object(payload, context="select_target")
        _unknown(raw, {"target_id"}, context="select_target")
        return cls(_identifier(raw.get("target_id"), name="target_id"))


@dataclass(frozen=True)
class OperatorIntentRequest:
    request_id: str
    operation: str
    name: str
    args: tuple[Any, ...]
    kwargs: dict[str, Any]

    @classmethod
    def from_payload(cls, payload: object) -> "OperatorIntentRequest":
        raw = _object(payload, context="operator_intent")
        _unknown(
            raw,
            {"request_id", "operation", "name", "args", "kwargs"},
            context="operator_intent",
        )
        request_id = _identifier(raw.get("request_id"), name="operator request_id")
        operation = str(raw.get("operation", "")).strip()
        if operation not in OPERATOR_OPERATIONS:
            raise ProtocolError(f"unsupported operator operation: {operation!r}")
        name = str(raw.get("name", ""))
        args = raw.get("args", ())
        kwargs = raw.get("kwargs", {})
        if not isinstance(args, (list, tuple)):
            raise ProtocolError("operator args must be a list")
        if not isinstance(kwargs, Mapping):
            raise ProtocolError("operator kwargs must be an object")
        return cls(
            request_id=request_id,
            operation=operation,
            name=name,
            args=tuple(args),
            kwargs={str(key): value for key, value in kwargs.items()},
        )


@dataclass(frozen=True)
class OperatorViewSnapshot:
    """Versioned, read-only UI model returned by a controller."""

    state: dict[str, Any]
    service: dict[str, Any]
    schema_version: int = OPERATOR_VIEW_SCHEMA_VERSION

    @classmethod
    def from_payload(cls, payload: object) -> "OperatorViewSnapshot":
        raw = _object(payload, context="operator view snapshot")
        _unknown(
            raw,
            {"schema_version", "state", "service"},
            context="operator view snapshot",
        )
        version = raw.get("schema_version")
        if type(version) is not int or version != OPERATOR_VIEW_SCHEMA_VERSION:
            raise ProtocolError(
                f"unsupported operator view schema: {version!r}; "
                f"expected {OPERATOR_VIEW_SCHEMA_VERSION}"
            )
        state = raw.get("state")
        service = raw.get("service")
        if not isinstance(state, Mapping):
            raise ProtocolError("operator view snapshot state must be an object")
        if not isinstance(service, Mapping):
            raise ProtocolError("operator view snapshot service must be an object")
        return cls(
            state={str(key): value for key, value in state.items()},
            service={str(key): value for key, value in service.items()},
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "schema_version": int(self.schema_version),
            "state": dict(self.state),
            "service": dict(self.service),
        }


@dataclass(frozen=True)
class MotionCommandRequest:
    command: str
    q: Optional[tuple[float, float, float, float]]
    go2_velocity: Optional[tuple[float, float, float]]
    raw: dict[str, Any]

    @classmethod
    def from_payload(cls, payload: object) -> "MotionCommandRequest":
        raw = _object(payload, context="motion_command")
        command = str(raw.get("command", "")).strip()
        if not command or len(command) > 128 or any(character.isspace() for character in command):
            raise ProtocolError("motion command must contain 1..128 non-whitespace characters")
        if "u" in raw:
            raise ProtocolError("legacy u motor targets are not supported by protocol v3")
        q: Optional[tuple[float, float, float, float]] = None
        if "q" in raw:
            q_values = _vector(raw["q"], 4, name="motion q")
            q = (q_values[0], q_values[1], q_values[2], q_values[3])
        velocity: Optional[tuple[float, float, float]] = None
        if "go2_vel" in raw:
            values = _vector(raw["go2_vel"], 3, name="go2_vel")
            velocity = (values[0], values[1], values[2])
        if command == "go2_velocity":
            values = _vector(
                (raw.get("vx"), raw.get("vy"), raw.get("wz")),
                3,
                name="go2 velocity",
            )
            velocity = (values[0], values[1], values[2])
        return cls(command=command, q=q, go2_velocity=velocity, raw=raw)


@dataclass(frozen=True)
class TelemetryPayload:
    q: Optional[tuple[float, float, float, float]]
    raw: dict[str, Any]

    @classmethod
    def from_payload(cls, payload: object) -> "TelemetryPayload":
        raw = _object(payload, context="telemetry")
        q: Optional[tuple[float, float, float, float]] = None
        if "q" in raw:
            values = _vector(raw["q"], 4, name="telemetry q")
            q = (values[0], values[1], values[2], values[3])
        return cls(q=q, raw=raw)


def validate_routed_payload(message_type: str, payload: object) -> object:
    """Parse payloads whose fields affect router authority or motion safety."""

    if message_type == "register":
        return RegisterRequest.from_payload(payload)
    if message_type == "discover":
        return DiscoverRequest.from_payload(payload)
    if message_type == "select_target":
        return SelectTargetRequest.from_payload(payload)
    if message_type == "operator_intent":
        return OperatorIntentRequest.from_payload(payload)
    if message_type == "motion_command":
        return MotionCommandRequest.from_payload(payload)
    if message_type == "telemetry":
        return TelemetryPayload.from_payload(payload)
    if message_type in {"heartbeat", "release_target"}:
        raw = _object(payload, context=message_type)
        _unknown(raw, set(), context=message_type)
        return raw
    return _object(payload, context=message_type)


__all__ = [
    "DiscoverRequest",
    "MotionCommandRequest",
    "OperatorIntentRequest",
    "OperatorViewSnapshot",
    "RegisterRequest",
    "SelectTargetRequest",
    "TelemetryPayload",
    "validate_routed_payload",
]
