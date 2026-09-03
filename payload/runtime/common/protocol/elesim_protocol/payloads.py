"""Typed protocol-v6 application payload contracts.

These small DTOs define the bounded fields that peer code may rely on before
touching domain state.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real
from typing import Any, Mapping, Optional

from .contracts import contract_for
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
        "spawn_mock_object",
        "remove_mock_object",
        "detach_mock_object",
    }
)

MOCK_OBJECT_STATES = frozenset(
    {"empty", "spawned", "executing", "attached", "error"}
)
_MAX_MOCK_OBJECT_ASSETS = 16
_MAX_MOCK_OBJECT_SILHOUETTE_POINTS = 64
_MAX_MOCK_OBJECT_POSITION_M = 10.0
_MAX_MOCK_OBJECT_EULER_DEG = 360.0


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


def _optional_identifier(raw: object, *, name: str) -> str:
    value = str(raw or "").strip()
    if not value:
        return ""
    return _identifier(value, name=name)


def _asset_id(raw: object, *, name: str = "mock object asset id") -> str:
    value = _identifier(raw, name=name)
    if len(value) > 64 or any(
        character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-"
        for character in value
    ):
        raise ProtocolError(f"{name} must be a basename-like identifier")
    if value in {".", ".."}:
        raise ProtocolError(f"{name} must be a basename-like identifier")
    return value


def _sha256(raw: object, *, name: str, allow_empty: bool = False) -> str:
    value = str(raw or "").strip().lower()
    if allow_empty and not value:
        return ""
    if len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
        raise ProtocolError(f"{name} must be a lowercase SHA-256 hex digest")
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


def _bounded_vector(
    raw: object,
    length: int,
    *,
    name: str,
    absolute_maximum: float,
) -> tuple[float, ...]:
    values = _vector(raw, length, name=name)
    if any(abs(value) > absolute_maximum for value in values):
        raise ProtocolError(f"{name} values must be within ±{absolute_maximum:g}")
    return values


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
    """Versioned, read-only UI model returned by a pilot."""

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
class MockHugExecutionRequest:
    solution_id: str
    object_revision: int
    object_sha256: str
    final_q: tuple[float, float, float, float]
    target_id: str = ""
    target_boot_id: str = ""
    target_lease_id: str = ""

    @classmethod
    def from_payload(cls, payload: object) -> "MockHugExecutionRequest":
        raw = _object(payload, context="mock hug execution")
        _unknown(
            raw,
            {
                "solution_id", "object_revision", "object_sha256", "final_q",
                "target_id", "target_boot_id", "target_lease_id",
            },
            context="mock hug execution",
        )
        target_values = tuple(
            _optional_identifier(raw.get(name), name=f"mock hug {name}")
            for name in ("target_id", "target_boot_id", "target_lease_id")
        )
        if any(target_values) and not all(target_values):
            raise ProtocolError("mock hug routing fence must contain target, boot and lease")
        return cls(
            solution_id=_identifier(raw.get("solution_id"), name="mock hug solution id"),
            object_revision=_integer(
                raw.get("object_revision"),
                name="mock hug object revision",
                minimum=1,
                maximum=2**31 - 1,
            ),
            object_sha256=_sha256(raw.get("object_sha256"), name="mock hug object sha256"),
            final_q=tuple(_vector(raw.get("final_q"), 4, name="mock hug final_q")),
            target_id=target_values[0],
            target_boot_id=target_values[1],
            target_lease_id=target_values[2],
        )

    def to_payload(self) -> dict[str, Any]:
        payload = {
            "solution_id": self.solution_id,
            "object_revision": self.object_revision,
            "object_sha256": self.object_sha256,
            "final_q": list(self.final_q),
        }
        if self.target_id:
            payload.update(
                {
                    "target_id": self.target_id,
                    "target_boot_id": self.target_boot_id,
                    "target_lease_id": self.target_lease_id,
                }
            )
        return payload


@dataclass(frozen=True)
class MotionCommandRequest:
    command: str
    q: Optional[tuple[float, float, float, float]]
    go2_velocity: Optional[tuple[float, float, float]]
    raw: dict[str, Any]
    mock_hug: Optional[MockHugExecutionRequest] = None

    @classmethod
    def from_payload(cls, payload: object) -> "MotionCommandRequest":
        raw = _object(payload, context="motion_command")
        command = str(raw.get("command", "")).strip()
        if not command or len(command) > 128 or any(character.isspace() for character in command):
            raise ProtocolError("motion command must contain 1..128 non-whitespace characters")
        if "u" in raw:
            raise ProtocolError("legacy u motor targets are not supported by protocol v6")
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
        mock_hug = None
        if "mock_hug" in raw:
            if command != "target" or q is None:
                raise ProtocolError("mock_hug requires a target motion command with q")
            mock_hug = MockHugExecutionRequest.from_payload(raw["mock_hug"])
        return cls(command=command, q=q, go2_velocity=velocity, mock_hug=mock_hug, raw=raw)


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
    sim_id: str
    streams: tuple[str, ...]
    schema_version: int = SIMULATION_SCHEMA_VERSION

    @classmethod
    def from_payload(cls, payload: object) -> "OpenSimulationSessionRequest":
        raw = _object(payload, context="open_simulation_session")
        _unknown(
            raw,
            {"schema_version", "request_id", "sim_id", "streams"},
            context="open_simulation_session",
        )
        _schema(raw, context="simulation session")
        return cls(
            request_id=_identifier(raw.get("request_id"), name="simulation request_id"),
            sim_id=_identifier(raw.get("sim_id"), name="sim_id"),
            streams=_simulation_streams(raw.get("streams")),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "request_id": self.request_id,
            "sim_id": self.sim_id,
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
    sim_id: str
    streams: tuple[str, ...]
    turn: Optional[TurnCredentials] = None
    schema_version: int = SIMULATION_SCHEMA_VERSION

    @classmethod
    def from_payload(cls, payload: object) -> "SimulationSessionOpenedPayload":
        raw = _object(payload, context="simulation_session_opened")
        _unknown(
            raw,
            {"schema_version", "request_id", "session_id", "sim_id", "streams", "turn"},
            context="simulation_session_opened",
        )
        _schema(raw, context="simulation session")
        return cls(
            request_id=_identifier(raw.get("request_id"), name="simulation request_id"),
            session_id=_identifier(raw.get("session_id"), name="simulation session_id"),
            sim_id=_identifier(raw.get("sim_id"), name="sim_id"),
            streams=_simulation_streams(raw.get("streams")),
            turn=_optional_turn(raw.get("turn")),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "request_id": self.request_id,
            "session_id": self.session_id,
            "sim_id": self.sim_id,
            "streams": list(self.streams),
            "turn": None if self.turn is None else self.turn.to_payload(),
        }


@dataclass(frozen=True)
class SimulationSessionGrantedPayload:
    request_id: str
    session_id: str
    sim_id: str
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
                "sim_id",
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
            sim_id=_identifier(raw.get("sim_id"), name="sim_id"),
            ui_id=_identifier(raw.get("ui_id"), name="ui_id"),
            streams=_simulation_streams(raw.get("streams")),
            turn=_optional_turn(raw.get("turn")),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "request_id": self.request_id,
            "session_id": self.session_id,
            "sim_id": self.sim_id,
            "ui_id": self.ui_id,
            "streams": list(self.streams),
            "turn": None if self.turn is None else self.turn.to_payload(),
        }


@dataclass(frozen=True)
class SimulationSessionRevokedPayload:
    session_id: str
    sim_id: str
    reason: str
    schema_version: int = SIMULATION_SCHEMA_VERSION

    @classmethod
    def from_payload(cls, payload: object) -> "SimulationSessionRevokedPayload":
        raw = _object(payload, context="simulation_session_revoked")
        _unknown(
            raw,
            {"schema_version", "session_id", "sim_id", "reason"},
            context="simulation_session_revoked",
        )
        _schema(raw, context="simulation session")
        return cls(
            session_id=_identifier(raw.get("session_id"), name="simulation session_id"),
            sim_id=_identifier(raw.get("sim_id"), name="sim_id"),
            reason=_text(raw.get("reason"), name="simulation revocation reason", maximum=512),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "session_id": self.session_id,
            "sim_id": self.sim_id,
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
        if command == "spawn_mock_object":
            _unknown(raw, {"asset_id", "position", "euler_deg"}, context="spawn_mock_object arguments")
            return {
                "asset_id": _asset_id(raw.get("asset_id")),
                "position": list(_bounded_vector(
                    raw.get("position"), 3, name="mock object position",
                    absolute_maximum=_MAX_MOCK_OBJECT_POSITION_M,
                )),
                "euler_deg": list(_bounded_vector(
                    raw.get("euler_deg"), 3, name="mock object euler_deg",
                    absolute_maximum=_MAX_MOCK_OBJECT_EULER_DEG,
                )),
            }
        if command in {"remove_mock_object", "detach_mock_object"}:
            _unknown(raw, set(), context=f"{command} arguments")
            return {}
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
class MockObjectStatePayload:
    """Bounded planning projection for one Sim-local immutable OBJ artifact."""

    available_assets: tuple[str, ...] = ()
    state: str = "empty"
    asset_id: str = ""
    revision: int = 0
    sha256: str = ""
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    euler_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    silhouette_xz: tuple[tuple[float, float], ...] = ()
    solution_id: str = ""
    attached: bool = False
    reason: str = ""

    @classmethod
    def from_payload(cls, payload: object) -> "MockObjectStatePayload":
        raw = _object(payload, context="mock object state")
        allowed = {
            "available_assets", "state", "asset_id", "revision", "sha256",
            "position", "euler_deg", "silhouette_xz", "solution_id", "attached", "reason",
        }
        _unknown(raw, allowed, context="mock object state")
        assets_raw = raw.get("available_assets", ())
        if not isinstance(assets_raw, (list, tuple)) or len(assets_raw) > _MAX_MOCK_OBJECT_ASSETS:
            raise ProtocolError(
                f"mock object available_assets must contain at most {_MAX_MOCK_OBJECT_ASSETS} entries"
            )
        assets = tuple(_asset_id(value) for value in assets_raw)
        if len(set(assets)) != len(assets):
            raise ProtocolError("mock object available_assets must not contain duplicates")
        state = str(raw.get("state", "")).strip()
        if state not in MOCK_OBJECT_STATES:
            raise ProtocolError(f"unsupported mock object state: {state!r}")
        revision = _integer(
            raw.get("revision", 0), name="mock object revision", minimum=0, maximum=2**31 - 1
        )
        silhouette_raw = raw.get("silhouette_xz", ())
        if not isinstance(silhouette_raw, (list, tuple)) or len(silhouette_raw) > _MAX_MOCK_OBJECT_SILHOUETTE_POINTS:
            raise ProtocolError(
                "mock object silhouette_xz must contain at most "
                f"{_MAX_MOCK_OBJECT_SILHOUETTE_POINTS} points"
            )
        silhouette = tuple(
            tuple(_vector(point, 2, name="mock object silhouette point"))
            for point in silhouette_raw
        )
        raw_asset_id = str(raw.get("asset_id", "") or "").strip()
        asset_id = _asset_id(raw_asset_id) if raw_asset_id else ""
        digest = _sha256(raw.get("sha256"), name="mock object sha256", allow_empty=True)
        solution_id = _optional_identifier(
            raw.get("solution_id"), name="mock hug solution id"
        )
        attached = _boolean(raw.get("attached", False), name="mock object attached")
        if state != "empty" and (not asset_id or not digest or len(silhouette) < 3):
            raise ProtocolError("active mock object state requires asset_id, sha256 and a polygon")
        if state != "empty" and revision < 1:
            raise ProtocolError("active mock object state requires a positive revision")
        if state == "empty" and (asset_id or digest or silhouette or solution_id or attached):
            raise ProtocolError("empty mock object state cannot retain active object fields")
        if (state in {"executing", "attached"}) != bool(solution_id):
            raise ProtocolError("mock hug solution id must match executing/attached state")
        if attached != (state == "attached"):
            raise ProtocolError("mock object attached flag must match attached state")
        return cls(
            available_assets=assets,
            state=state,
            asset_id=asset_id,
            revision=revision,
            sha256=digest,
            position=tuple(_bounded_vector(
                raw.get("position", (0.0, 0.0, 0.0)), 3,
                name="mock object position", absolute_maximum=_MAX_MOCK_OBJECT_POSITION_M,
            )),
            euler_deg=tuple(_bounded_vector(
                raw.get("euler_deg", (0.0, 0.0, 0.0)), 3,
                name="mock object euler_deg", absolute_maximum=_MAX_MOCK_OBJECT_EULER_DEG,
            )),
            silhouette_xz=silhouette,
            solution_id=solution_id,
            attached=attached,
            reason=_text(raw.get("reason", ""), name="mock object reason", maximum=512, allow_empty=True),
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "available_assets": list(self.available_assets),
            "state": self.state,
            "asset_id": self.asset_id,
            "revision": self.revision,
            "sha256": self.sha256,
            "position": list(self.position),
            "euler_deg": list(self.euler_deg),
            "silhouette_xz": [list(point) for point in self.silhouette_xz],
            "solution_id": self.solution_id,
            "attached": self.attached,
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
    mock_object: Optional[MockObjectStatePayload] = None

    @classmethod
    def from_payload(cls, payload: object) -> "SimulationStatusPayload":
        raw = _object(payload, context="simulation_status")
        _unknown(
            raw,
            {
                "schema_version", "epoch", "paused", "speed", "debug_visible", "sim_time_s",
                "mock_object",
            },
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
            mock_object=(
                None
                if raw.get("mock_object") is None
                else MockObjectStatePayload.from_payload(raw["mock_object"])
            ),
        )

    def to_payload(self) -> dict[str, Any]:
        payload = {
            "schema_version": self.schema_version,
            "epoch": self.epoch,
            "paused": self.paused,
            "speed": self.speed,
            "debug_visible": self.debug_visible,
            "sim_time_s": self.sim_time_s,
        }
        if self.mock_object is not None:
            payload["mock_object"] = self.mock_object.to_payload()
        return payload


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


def _validate_registered_payload(message_type: str, payload: object) -> dict[str, Any]:
    """Apply the registry's structural policy to a non-DTO payload.

    Domain DTOs retain ownership of their own nested validation.  For the
    small acknowledgement/error family the registry is intentionally strict,
    so a typo cannot silently become a second wire field.  Telemetry and
    operator results remain additive maps, but still pass through the common
    object and JSON-boundary checks in :class:`Envelope`.
    """

    raw = _object(payload, context=message_type)
    contract = contract_for(message_type)
    if contract.strict_fields and contract.payload_fields is not None:
        _unknown(raw, set(contract.payload_fields), context=message_type)
    if message_type == "endpoint_list":
        endpoints = raw.get("endpoints")
        if not isinstance(endpoints, (list, tuple)) or len(endpoints) > 64:
            raise ProtocolError("endpoint_list endpoints must contain 0..64 entries")
        for endpoint in endpoints:
            if not isinstance(endpoint, Mapping):
                raise ProtocolError("endpoint_list entries must be objects")
            EndpointDescriptor.from_dict(endpoint)
    if message_type in {"target_selected", "target_released", "target_lost"}:
        _identifier(raw.get("target_id"), name=f"{message_type} target_id")
        if "lease_id" in raw:
            _identifier(raw.get("lease_id"), name=f"{message_type} lease_id")
        if "reason" in raw:
            _text(raw.get("reason"), name=f"{message_type} reason", maximum=512)
    if message_type == "lease_granted":
        _identifier(raw.get("pilot_id"), name="lease_granted pilot_id")
    if message_type in {"lease_revoked", "error"} and "reason" in raw:
        _text(raw.get("reason"), name=f"{message_type} reason", maximum=1024)
    if message_type == "ack":
        if "reply_to" in raw:
            _identifier(raw.get("reply_to"), name="ack reply_to")
        if "ok" in raw:
            _boolean(raw.get("ok"), name="ack ok")
        if "reason" in raw:
            _text(raw.get("reason"), name="ack reason", maximum=512)
    if message_type == "error":
        _identifier(raw.get("reply_to"), name="error reply_to")
        if "reason" not in raw:
            raise ProtocolError("error reason is required")
    if message_type == "operator_result":
        _identifier(raw.get("request_id"), name="operator result request_id")
        _boolean(raw.get("ok"), name="operator result ok")
        if "error" in raw:
            _text(raw.get("error"), name="operator result error", maximum=2048)
    return raw


_ROUTED_PAYLOAD_PARSERS = {
    "discover": DiscoverRequest.from_payload,
    "select_target": SelectTargetRequest.from_payload,
    "operator_intent": OperatorIntentRequest.from_payload,
    "motion_command": MotionCommandRequest.from_payload,
    "telemetry": TelemetryPayload.from_payload,
    "open_simulation_session": OpenSimulationSessionRequest.from_payload,
    "close_simulation_session": CloseSimulationSessionRequest.from_payload,
    "simulation_session_opened": SimulationSessionOpenedPayload.from_payload,
    "simulation_session_granted": SimulationSessionGrantedPayload.from_payload,
    "simulation_session_revoked": SimulationSessionRevokedPayload.from_payload,
    "simulation_command": SimulationCommandRequest.from_payload,
    "simulation_result": SimulationResultPayload.from_payload,
    "simulation_status": SimulationStatusPayload.from_payload,
    "webrtc_signal": WebRtcSignalPayload.from_payload,
}
_EMPTY_PAYLOAD_TYPES = frozenset(
    {"release_target", "renew_target", "renew_simulation_session"}
)


def validate_routed_payload(message_type: str, payload: object) -> object:
    """Parse payloads whose fields affect peer authority or motion safety."""

    parser = _ROUTED_PAYLOAD_PARSERS.get(message_type)
    if parser is not None:
        return parser(payload)
    # Empty lease renewals and releases are explicit contracts, not an
    # untyped catch-all.  This prevents accidental command parameters from
    # leaking into authority methods.
    if message_type in _EMPTY_PAYLOAD_TYPES:
        raw = _object(payload, context=message_type)
        _unknown(raw, set(), context=message_type)
        return raw
    return _validate_registered_payload(message_type, payload)


__all__ = [
    "CloseSimulationSessionRequest",
    "DiscoverRequest",
    "MotionCommandRequest",
    "MockHugExecutionRequest",
    "MockObjectStatePayload",
    "OpenSimulationSessionRequest",
    "OperatorIntentRequest",
    "OperatorViewSnapshot",
    "SelectTargetRequest",
    "SIMULATION_COMMANDS",
    "MOCK_OBJECT_STATES",
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
    "_validate_registered_payload",
    "validate_routed_payload",
]
