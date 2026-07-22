"""Typed protocol-v4 payload contracts.

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


SIMULATION_SCHEMA_VERSION = 1
SIMULATION_STREAMS = frozenset({"observer", "hand_eye_preview"})
SIMULATION_COMMANDS = frozenset(
    {
        "orbit",
        "pan",
        "zoom",
        "reset_view",
        "pause",
        "resume",
        "step",
        "reset",
        "set_speed",
        "set_debug_visible",
    }
)


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


def _schema(raw: Mapping[str, Any], *, context: str) -> None:
    version = raw.get("schema_version")
    if type(version) is not int or version != SIMULATION_SCHEMA_VERSION:
        raise ProtocolError(
            f"unsupported {context} schema: {version!r}; "
            f"expected {SIMULATION_SCHEMA_VERSION}"
        )


def _boolean(raw: object, *, name: str) -> bool:
    if type(raw) is not bool:
        raise ProtocolError(f"{name} must be a boolean")
    return bool(raw)


def _integer(raw: object, *, name: str, minimum: int, maximum: int) -> int:
    if type(raw) is not int or not minimum <= raw <= maximum:
        raise ProtocolError(f"{name} must be an integer in {minimum}..{maximum}")
    return int(raw)


def _text(raw: object, *, name: str, maximum: int, allow_empty: bool = False) -> str:
    if not isinstance(raw, str):
        raise ProtocolError(f"{name} must be a string")
    value = raw.strip()
    if (not value and not allow_empty) or len(value) > maximum:
        lower = 0 if allow_empty else 1
        raise ProtocolError(f"{name} must contain {lower}..{maximum} characters")
    return value


def _simulation_streams(raw: object) -> tuple[str, ...]:
    if not isinstance(raw, (list, tuple)) or not 1 <= len(raw) <= len(SIMULATION_STREAMS):
        raise ProtocolError("simulation streams must contain one or two named streams")
    streams = tuple(str(value).strip() for value in raw)
    if len(set(streams)) != len(streams):
        raise ProtocolError("simulation streams must be unique")
    unsupported = sorted(set(streams) - SIMULATION_STREAMS)
    if unsupported:
        raise ProtocolError("unsupported simulation stream: " + ", ".join(unsupported))
    return streams


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
            raise ProtocolError("legacy u motor targets are not supported by protocol v4")
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


@dataclass(frozen=True)
class TurnCredentials:
    urls: tuple[str, ...]
    username: str
    credential: str
    expires_at: float

    @classmethod
    def from_payload(cls, payload: object) -> "TurnCredentials":
        raw = _object(payload, context="TURN credentials")
        _unknown(raw, {"urls", "username", "credential", "expires_at"}, context="TURN credentials")
        urls_raw = raw.get("urls")
        if not isinstance(urls_raw, (list, tuple)) or not 1 <= len(urls_raw) <= 8:
            raise ProtocolError("TURN urls must contain 1..8 entries")
        urls = tuple(
            _text(value, name="TURN url", maximum=2048)
            for value in urls_raw
        )
        if any(any(character.isspace() for character in url) for url in urls):
            raise ProtocolError("TURN urls must not contain whitespace")
        expires_at = _finite(raw.get("expires_at"), name="TURN expires_at")
        if expires_at <= 0.0:
            raise ProtocolError("TURN expires_at must be positive")
        return cls(
            urls=urls,
            username=_text(raw.get("username"), name="TURN username", maximum=256),
            credential=_text(raw.get("credential"), name="TURN credential", maximum=512),
            expires_at=expires_at,
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "urls": list(self.urls),
            "username": self.username,
            "credential": self.credential,
            "expires_at": float(self.expires_at),
        }


@dataclass(frozen=True)
class OpenSimulationSessionRequest:
    request_id: str
    simulator_id: str
    streams: tuple[str, ...]
    schema_version: int = SIMULATION_SCHEMA_VERSION

    @classmethod
    def from_payload(cls, payload: object) -> "OpenSimulationSessionRequest":
        raw = _object(payload, context="open_simulation_session")
        _unknown(
            raw,
            {"schema_version", "request_id", "simulator_id", "streams"},
            context="open_simulation_session",
        )
        _schema(raw, context="simulation session")
        return cls(
            request_id=_identifier(raw.get("request_id"), name="simulation request_id"),
            simulator_id=_identifier(raw.get("simulator_id"), name="simulator_id"),
            streams=_simulation_streams(raw.get("streams")),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "request_id": self.request_id,
            "simulator_id": self.simulator_id,
            "streams": list(self.streams),
        }


@dataclass(frozen=True)
class CloseSimulationSessionRequest:
    request_id: str
    session_id: str
    schema_version: int = SIMULATION_SCHEMA_VERSION

    @classmethod
    def from_payload(cls, payload: object) -> "CloseSimulationSessionRequest":
        raw = _object(payload, context="close_simulation_session")
        _unknown(
            raw,
            {"schema_version", "request_id", "session_id"},
            context="close_simulation_session",
        )
        _schema(raw, context="simulation session")
        return cls(
            request_id=_identifier(raw.get("request_id"), name="simulation request_id"),
            session_id=_identifier(raw.get("session_id"), name="simulation session_id"),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "request_id": self.request_id,
            "session_id": self.session_id,
        }


def _optional_turn(raw: object) -> Optional[TurnCredentials]:
    if raw is None:
        return None
    return TurnCredentials.from_payload(raw)


@dataclass(frozen=True)
class SimulationSessionOpenedPayload:
    request_id: str
    session_id: str
    simulator_id: str
    streams: tuple[str, ...]
    turn: Optional[TurnCredentials] = None
    schema_version: int = SIMULATION_SCHEMA_VERSION

    @classmethod
    def from_payload(cls, payload: object) -> "SimulationSessionOpenedPayload":
        raw = _object(payload, context="simulation_session_opened")
        _unknown(
            raw,
            {"schema_version", "request_id", "session_id", "simulator_id", "streams", "turn"},
            context="simulation_session_opened",
        )
        _schema(raw, context="simulation session")
        return cls(
            request_id=_identifier(raw.get("request_id"), name="simulation request_id"),
            session_id=_identifier(raw.get("session_id"), name="simulation session_id"),
            simulator_id=_identifier(raw.get("simulator_id"), name="simulator_id"),
            streams=_simulation_streams(raw.get("streams")),
            turn=_optional_turn(raw.get("turn")),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "request_id": self.request_id,
            "session_id": self.session_id,
            "simulator_id": self.simulator_id,
            "streams": list(self.streams),
            "turn": None if self.turn is None else self.turn.to_payload(),
        }


@dataclass(frozen=True)
class SimulationSessionGrantedPayload:
    request_id: str
    session_id: str
    simulator_id: str
    ui_id: str
    streams: tuple[str, ...]
    turn: Optional[TurnCredentials] = None
    schema_version: int = SIMULATION_SCHEMA_VERSION

    @classmethod
    def from_payload(cls, payload: object) -> "SimulationSessionGrantedPayload":
        raw = _object(payload, context="simulation_session_granted")
        _unknown(
            raw,
            {
                "schema_version",
                "request_id",
                "session_id",
                "simulator_id",
                "ui_id",
                "streams",
                "turn",
            },
            context="simulation_session_granted",
        )
        _schema(raw, context="simulation session")
        return cls(
            request_id=_identifier(raw.get("request_id"), name="simulation request_id"),
            session_id=_identifier(raw.get("session_id"), name="simulation session_id"),
            simulator_id=_identifier(raw.get("simulator_id"), name="simulator_id"),
            ui_id=_identifier(raw.get("ui_id"), name="ui_id"),
            streams=_simulation_streams(raw.get("streams")),
            turn=_optional_turn(raw.get("turn")),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "request_id": self.request_id,
            "session_id": self.session_id,
            "simulator_id": self.simulator_id,
            "ui_id": self.ui_id,
            "streams": list(self.streams),
            "turn": None if self.turn is None else self.turn.to_payload(),
        }


@dataclass(frozen=True)
class SimulationSessionRevokedPayload:
    session_id: str
    simulator_id: str
    reason: str
    schema_version: int = SIMULATION_SCHEMA_VERSION

    @classmethod
    def from_payload(cls, payload: object) -> "SimulationSessionRevokedPayload":
        raw = _object(payload, context="simulation_session_revoked")
        _unknown(
            raw,
            {"schema_version", "session_id", "simulator_id", "reason"},
            context="simulation_session_revoked",
        )
        _schema(raw, context="simulation session")
        return cls(
            session_id=_identifier(raw.get("session_id"), name="simulation session_id"),
            simulator_id=_identifier(raw.get("simulator_id"), name="simulator_id"),
            reason=_text(raw.get("reason"), name="simulation revocation reason", maximum=512),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "session_id": self.session_id,
            "simulator_id": self.simulator_id,
            "reason": self.reason,
        }


@dataclass(frozen=True)
class SimulationCommandRequest:
    request_id: str
    session_id: str
    command: str
    arguments: dict[str, Any]
    schema_version: int = SIMULATION_SCHEMA_VERSION

    @classmethod
    def from_payload(cls, payload: object) -> "SimulationCommandRequest":
        raw = _object(payload, context="simulation_command")
        _unknown(
            raw,
            {"schema_version", "request_id", "session_id", "command", "arguments"},
            context="simulation_command",
        )
        _schema(raw, context="simulation command")
        command = str(raw.get("command", "")).strip()
        if command not in SIMULATION_COMMANDS:
            raise ProtocolError(f"unsupported simulation command: {command!r}")
        arguments = _object(raw.get("arguments", {}), context="simulation command arguments")
        normalized = cls._arguments(command, arguments)
        return cls(
            request_id=_identifier(raw.get("request_id"), name="simulation request_id"),
            session_id=_identifier(raw.get("session_id"), name="simulation session_id"),
            command=command,
            arguments=normalized,
        )

    @staticmethod
    def _arguments(command: str, raw: Mapping[str, Any]) -> dict[str, Any]:
        if command in {"orbit", "pan"}:
            _unknown(raw, {"dx", "dy"}, context=f"{command} arguments")
            dx = _finite(raw.get("dx"), name=f"{command} dx")
            dy = _finite(raw.get("dy"), name=f"{command} dy")
            if abs(dx) > 2.0 or abs(dy) > 2.0:
                raise ProtocolError(f"{command} deltas must be within -2..2")
            return {"dx": dx, "dy": dy}
        if command == "zoom":
            _unknown(raw, {"delta"}, context="zoom arguments")
            delta = _finite(raw.get("delta"), name="zoom delta")
            if abs(delta) > 2.0:
                raise ProtocolError("zoom delta must be within -2..2")
            return {"delta": delta}
        if command == "step":
            _unknown(raw, {"count"}, context="step arguments")
            return {"count": _integer(raw.get("count"), name="step count", minimum=1, maximum=120)}
        if command == "set_speed":
            _unknown(raw, {"scale"}, context="set_speed arguments")
            scale = _finite(raw.get("scale"), name="simulation speed scale")
            if not 0.05 <= scale <= 4.0:
                raise ProtocolError("simulation speed scale must be within 0.05..4.0")
            return {"scale": scale}
        if command == "set_debug_visible":
            _unknown(raw, {"visible"}, context="set_debug_visible arguments")
            return {"visible": _boolean(raw.get("visible"), name="debug visible")}
        _unknown(raw, set(), context=f"{command} arguments")
        return {}

    def to_payload(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "request_id": self.request_id,
            "session_id": self.session_id,
            "command": self.command,
            "arguments": dict(self.arguments),
        }


@dataclass(frozen=True)
class SimulationResultPayload:
    request_id: str
    session_id: str
    command: str
    ok: bool
    reason: str = ""
    schema_version: int = SIMULATION_SCHEMA_VERSION

    @classmethod
    def from_payload(cls, payload: object) -> "SimulationResultPayload":
        raw = _object(payload, context="simulation_result")
        _unknown(
            raw,
            {"schema_version", "request_id", "session_id", "command", "ok", "reason"},
            context="simulation_result",
        )
        _schema(raw, context="simulation result")
        command = str(raw.get("command", "")).strip()
        if command not in SIMULATION_COMMANDS:
            raise ProtocolError(f"unsupported simulation command: {command!r}")
        return cls(
            request_id=_identifier(raw.get("request_id"), name="simulation request_id"),
            session_id=_identifier(raw.get("session_id"), name="simulation session_id"),
            command=command,
            ok=_boolean(raw.get("ok"), name="simulation result ok"),
            reason=_text(
                raw.get("reason", ""),
                name="simulation result reason",
                maximum=512,
                allow_empty=True,
            ),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "request_id": self.request_id,
            "session_id": self.session_id,
            "command": self.command,
            "ok": self.ok,
            "reason": self.reason,
        }


@dataclass(frozen=True)
class SimulationStatusPayload:
    epoch: int
    paused: bool
    speed: float
    debug_visible: bool
    sim_time_s: float
    schema_version: int = SIMULATION_SCHEMA_VERSION

    @classmethod
    def from_payload(cls, payload: object) -> "SimulationStatusPayload":
        raw = _object(payload, context="simulation_status")
        _unknown(
            raw,
            {"schema_version", "epoch", "paused", "speed", "debug_visible", "sim_time_s"},
            context="simulation_status",
        )
        _schema(raw, context="simulation status")
        epoch = _integer(raw.get("epoch"), name="simulation epoch", minimum=0, maximum=2**31 - 1)
        speed = _finite(raw.get("speed"), name="simulation speed")
        sim_time_s = _finite(raw.get("sim_time_s"), name="simulation time")
        if not 0.05 <= speed <= 4.0:
            raise ProtocolError("simulation speed must be within 0.05..4.0")
        if sim_time_s < 0.0:
            raise ProtocolError("simulation time must be non-negative")
        return cls(
            epoch=epoch,
            paused=_boolean(raw.get("paused"), name="simulation paused"),
            speed=speed,
            debug_visible=_boolean(raw.get("debug_visible"), name="debug visible"),
            sim_time_s=sim_time_s,
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "epoch": self.epoch,
            "paused": self.paused,
            "speed": self.speed,
            "debug_visible": self.debug_visible,
            "sim_time_s": self.sim_time_s,
        }


@dataclass(frozen=True)
class WebRtcSignalPayload:
    session_id: str
    stream: str
    signal: str
    sdp: str
    type: str
    schema_version: int = SIMULATION_SCHEMA_VERSION

    @classmethod
    def from_payload(cls, payload: object) -> "WebRtcSignalPayload":
        raw = _object(payload, context="webrtc_signal")
        _unknown(
            raw,
            {"schema_version", "session_id", "stream", "signal", "sdp", "type"},
            context="webrtc_signal",
        )
        _schema(raw, context="WebRTC signal")
        stream = str(raw.get("stream", "")).strip()
        if stream not in SIMULATION_STREAMS:
            raise ProtocolError(f"unsupported simulation stream: {stream!r}")
        signal = str(raw.get("signal", "")).strip()
        signal_type = str(raw.get("type", "")).strip()
        if signal not in {"offer", "answer"} or signal_type != signal:
            raise ProtocolError("WebRTC signal and type must be matching offer or answer values")
        sdp = _text(raw.get("sdp"), name="WebRTC SDP", maximum=524_288)
        return cls(
            session_id=_identifier(raw.get("session_id"), name="simulation session_id"),
            stream=stream,
            signal=signal,
            sdp=sdp,
            type=signal_type,
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "session_id": self.session_id,
            "stream": self.stream,
            "signal": self.signal,
            "sdp": self.sdp,
            "type": self.type,
        }


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
    if message_type == "open_simulation_session":
        return OpenSimulationSessionRequest.from_payload(payload)
    if message_type == "close_simulation_session":
        return CloseSimulationSessionRequest.from_payload(payload)
    if message_type == "simulation_session_opened":
        return SimulationSessionOpenedPayload.from_payload(payload)
    if message_type == "simulation_session_granted":
        return SimulationSessionGrantedPayload.from_payload(payload)
    if message_type == "simulation_session_revoked":
        return SimulationSessionRevokedPayload.from_payload(payload)
    if message_type == "simulation_command":
        return SimulationCommandRequest.from_payload(payload)
    if message_type == "simulation_result":
        return SimulationResultPayload.from_payload(payload)
    if message_type == "simulation_status":
        return SimulationStatusPayload.from_payload(payload)
    if message_type == "webrtc_signal":
        return WebRtcSignalPayload.from_payload(payload)
    if message_type in {"heartbeat", "release_target"}:
        raw = _object(payload, context=message_type)
        _unknown(raw, set(), context=message_type)
        return raw
    return _object(payload, context=message_type)


__all__ = [
    "CloseSimulationSessionRequest",
    "DiscoverRequest",
    "MotionCommandRequest",
    "OpenSimulationSessionRequest",
    "OperatorIntentRequest",
    "OperatorViewSnapshot",
    "RegisterRequest",
    "SelectTargetRequest",
    "SIMULATION_COMMANDS",
    "SIMULATION_SCHEMA_VERSION",
    "SIMULATION_STREAMS",
    "SimulationCommandRequest",
    "SimulationResultPayload",
    "SimulationSessionGrantedPayload",
    "SimulationSessionOpenedPayload",
    "SimulationSessionRevokedPayload",
    "SimulationStatusPayload",
    "TelemetryPayload",
    "TurnCredentials",
    "WebRtcSignalPayload",
    "validate_routed_payload",
]
