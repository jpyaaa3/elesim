#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import json
import math
import time
import uuid
from dataclasses import dataclass
from numbers import Real
from typing import Any, Dict, Mapping, Optional


@dataclass(frozen=True)
class ControlU:
    u_linear: float
    u_roll: float
    u_s1: float
    u_s2: float


@dataclass(frozen=True)
class SimQ:
    linear_m: float
    roll_rad: float
    theta1_rad: float
    theta2_rad: float


# Default arm pose at startup / sim reset (control-panel display [u]).
# Spawn/reset + perception-friendly arm pose (display [u]).
DEFAULT_START_CONTROL_U = ControlU(
    u_linear=250.0,
    u_roll=180.0,
    u_s1=85.0,
    u_s2=45.0,
)

PERCEPTION_READY_CONTROL_U = DEFAULT_START_CONTROL_U


@dataclass(frozen=True)
class SimMappingConfig:
    linear_u_min: float = 0.0
    # Linear actuator usable travel in motor/display [u] units.
    # The motor can rotate farther, but values beyond this collide with hardware.
    linear_u_max: float = 250.0
    linear_u_limit: float = 250.0
    roll_u_min: float = 0.0
    roll_u_max: float = 360.0
    seg_u_min: float = 0.0
    seg_u_max: float = 360.0

    linear_q_min_m: float = -0.230
    linear_q_max_m: float = 0.0
    roll_q_min_rad: float = -math.pi / 2.0
    roll_q_max_rad: float = +math.pi / 2.0
    seg1_q_min_rad: float = -math.radians(36.0)
    seg1_q_max_rad: float = +math.radians(36.0)
    seg2_q_min_rad: float = -math.radians(36.0)
    seg2_q_max_rad: float = +math.radians(36.0)

    command_direction: tuple[int, int, int, int] = (1, 1, 1, 1)


def _clamp(x: float, lo: float, hi: float) -> float:
    return float(min(max(float(x), float(lo)), float(hi)))


def linear_motor_u_limit(cfg: SimMappingConfig) -> float:
    return float(min(float(cfg.linear_u_max), float(cfg.linear_u_limit)))


def _linear_q_forward_m(cfg: SimMappingConfig) -> float:
    """Fully extended (앞) — motor u=0."""
    return float(cfg.linear_q_max_m)


def _linear_q_backward_m(cfg: SimMappingConfig) -> float:
    """Fully retracted (뒤) — motor u=linear_u_max."""
    return float(cfg.linear_q_min_m)


def _map_linear_q_to_u(q_m: float, cfg: SimMappingConfig) -> float:
    """Map prismatic q to motor [u] over the usable linear travel."""
    q_fwd = _linear_q_forward_m(cfg)
    q_bwd = _linear_q_backward_m(cfg)
    u_lo = float(cfg.linear_u_min)
    u_hi = float(cfg.linear_u_max)
    q_lo = min(q_fwd, q_bwd)
    q_hi = max(q_fwd, q_bwd)
    q_m = _clamp(float(q_m), q_lo, q_hi)
    span_q = q_bwd - q_fwd
    if abs(span_q) < 1e-12:
        return u_lo
    t = (q_m - q_fwd) / span_q
    return _clamp(u_lo + t * (u_hi - u_lo), u_lo, u_hi)


def _map_linear_u_to_q(u_linear: float, cfg: SimMappingConfig) -> float:
    """Map motor [u] on 0..linear_u_max back to prismatic q."""
    q_fwd = _linear_q_forward_m(cfg)
    q_bwd = _linear_q_backward_m(cfg)
    u_lo = float(cfg.linear_u_min)
    u_hi = float(cfg.linear_u_max)
    u_linear = _clamp(float(u_linear), u_lo, u_hi)
    if abs(u_hi - u_lo) < 1e-12:
        return q_fwd
    t = (u_linear - u_lo) / (u_hi - u_lo)
    return float(q_fwd + t * (q_bwd - q_fwd))


def _motor_u_from_display_linear(display_u: float, cfg: SimMappingConfig) -> float:
    """Panel/command u -> motor u over the usable linear travel."""
    u_lo = float(cfg.linear_u_min)
    u_hi = float(cfg.linear_u_max)
    direction = int(cfg.command_direction[0])
    panel_u = clamp_linear_motor_u(float(display_u), cfg)
    return _clamp(
        _apply_axis_direction(panel_u, direction, u_lo, u_hi),
        u_lo,
        u_hi,
    )


def _display_u_from_motor_linear(motor_u: float, cfg: SimMappingConfig) -> float:
    """Motor u -> panel display over the usable linear travel."""
    u_lo = float(cfg.linear_u_min)
    u_hi = float(cfg.linear_u_max)
    direction = int(cfg.command_direction[0])
    panel_u = _apply_axis_direction(float(motor_u), direction, u_lo, u_hi)
    return clamp_linear_motor_u(panel_u, cfg)


def clamp_linear_motor_u(u_linear: float, cfg: SimMappingConfig) -> float:
    return _clamp(u_linear, float(cfg.linear_u_min), linear_motor_u_limit(cfg))


def _apply_axis_direction(u_value: float, direction: int, u_min: float, u_max: float) -> float:
    if int(direction) < 0:
        return float(u_min) + float(u_max) - float(u_value)
    return float(u_value)


def _map_axis_to_u(q_value: float, q_min: float, q_max: float, u_min: float, u_max: float) -> float:
    q_lo = min(float(q_min), float(q_max))
    q_hi = max(float(q_min), float(q_max))
    q_value = _clamp(q_value, q_lo, q_hi)
    if abs(float(q_max) - float(q_min)) < 1e-12:
        return float(u_min)
    ratio = (float(q_value) - float(q_min)) / (float(q_max) - float(q_min))
    return _clamp(float(u_min) + ratio * (float(u_max) - float(u_min)), u_min, u_max)


def _map_u_to_axis(u_value: float, u_min: float, u_max: float, q_min: float, q_max: float) -> float:
    u_value = _clamp(u_value, u_min, u_max)
    if abs(float(u_max) - float(u_min)) < 1e-12:
        return float(q_min)
    ratio = (float(u_value) - float(u_min)) / (float(u_max) - float(u_min))
    q_value = float(q_min) + ratio * (float(q_max) - float(q_min))
    return _clamp(q_value, min(float(q_min), float(q_max)), max(float(q_min), float(q_max)))


def sim_q_to_motor_deg(q: SimQ, cfg: SimMappingConfig = SimMappingConfig()) -> ControlU:
    return ControlU(
        u_linear=_map_linear_q_to_u(q.linear_m, cfg),
        u_roll=_map_axis_to_u(q.roll_rad, cfg.roll_q_min_rad, cfg.roll_q_max_rad, cfg.roll_u_min, cfg.roll_u_max),
        u_s1=_map_axis_to_u(q.theta1_rad, cfg.seg1_q_min_rad, cfg.seg1_q_max_rad, cfg.seg_u_min, cfg.seg_u_max),
        u_s2=_map_axis_to_u(q.theta2_rad, cfg.seg2_q_min_rad, cfg.seg2_q_max_rad, cfg.seg_u_min, cfg.seg_u_max),
    )


def motor_deg_to_sim_q(u: ControlU, cfg: SimMappingConfig = SimMappingConfig()) -> SimQ:
    return SimQ(
        linear_m=_map_linear_u_to_q(u.u_linear, cfg),
        roll_rad=_map_u_to_axis(u.u_roll, cfg.roll_u_min, cfg.roll_u_max, cfg.roll_q_min_rad, cfg.roll_q_max_rad),
        theta1_rad=_map_u_to_axis(u.u_s1, cfg.seg_u_min, cfg.seg_u_max, cfg.seg1_q_min_rad, cfg.seg1_q_max_rad),
        theta2_rad=_map_u_to_axis(u.u_s2, cfg.seg_u_min, cfg.seg_u_max, cfg.seg2_q_min_rad, cfg.seg2_q_max_rad),
    )


def default_start_sim_q(cfg: SimMappingConfig = SimMappingConfig()) -> SimQ:
    return control_u_to_sim_q(DEFAULT_START_CONTROL_U, cfg)


def perception_ready_sim_q(cfg: SimMappingConfig = SimMappingConfig()) -> SimQ:
    return control_u_to_sim_q(PERCEPTION_READY_CONTROL_U, cfg)


def control_u_to_sim_q(u: ControlU, cfg: SimMappingConfig = SimMappingConfig()) -> SimQ:
    dirs = tuple(int(v) for v in cfg.command_direction)
    motor_u = ControlU(
        u_linear=_motor_u_from_display_linear(u.u_linear, cfg),
        u_roll=_clamp(_apply_axis_direction(u.u_roll, dirs[1], cfg.roll_u_min, cfg.roll_u_max), cfg.roll_u_min, cfg.roll_u_max),
        u_s1=_clamp(_apply_axis_direction(u.u_s1, dirs[2], cfg.seg_u_min, cfg.seg_u_max), cfg.seg_u_min, cfg.seg_u_max),
        u_s2=_clamp(_apply_axis_direction(u.u_s2, dirs[3], cfg.seg_u_min, cfg.seg_u_max), cfg.seg_u_min, cfg.seg_u_max),
    )
    return motor_deg_to_sim_q(motor_u, cfg)


def sim_q_to_control_u(q: SimQ, cfg: SimMappingConfig = SimMappingConfig()) -> ControlU:
    dirs = tuple(int(v) for v in cfg.command_direction)
    motor_linear = _map_linear_q_to_u(q.linear_m, cfg)
    motor_u = ControlU(
        u_linear=float(motor_linear),
        u_roll=_map_axis_to_u(q.roll_rad, cfg.roll_q_min_rad, cfg.roll_q_max_rad, cfg.roll_u_min, cfg.roll_u_max),
        u_s1=_map_axis_to_u(q.theta1_rad, cfg.seg1_q_min_rad, cfg.seg1_q_max_rad, cfg.seg_u_min, cfg.seg_u_max),
        u_s2=_map_axis_to_u(q.theta2_rad, cfg.seg2_q_min_rad, cfg.seg2_q_max_rad, cfg.seg_u_min, cfg.seg_u_max),
    )
    return ControlU(
        u_linear=clamp_linear_motor_u(
            _apply_axis_direction(motor_u.u_linear, dirs[0], cfg.linear_u_min, cfg.linear_u_max),
            cfg,
        ),
        u_roll=_apply_axis_direction(motor_u.u_roll, dirs[1], cfg.roll_u_min, cfg.roll_u_max),
        u_s1=_apply_axis_direction(motor_u.u_s1, dirs[2], cfg.seg_u_min, cfg.seg_u_max),
        u_s2=_apply_axis_direction(motor_u.u_s2, dirs[3], cfg.seg_u_min, cfg.seg_u_max),
    )


def linear_effective_q_bounds(cfg: SimMappingConfig) -> tuple[float, float]:
    q0 = control_u_to_sim_q(
        ControlU(
            u_linear=float(cfg.linear_u_min),
            u_roll=0.0,
            u_s1=0.0,
            u_s2=0.0,
        ),
        cfg,
    ).linear_m
    q1 = control_u_to_sim_q(
        ControlU(
            u_linear=linear_motor_u_limit(cfg),
            u_roll=0.0,
            u_s1=0.0,
            u_s2=0.0,
        ),
        cfg,
    ).linear_m
    return (float(min(q0, q1)), float(max(q0, q1)))


def now_s() -> float:
    return time.time()


def dumps_msg(msg: Dict[str, Any]) -> bytes:
    return json.dumps(msg, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def loads_msg(buf: bytes) -> Dict[str, Any]:
    return json.loads(buf.decode("utf-8"))


PROTOCOL_VERSION = 5
MAX_ENVELOPE_BYTES = 1_048_576
ENDPOINT_ROLES = frozenset({"controller", "robot", "simulator", "ui"})

CAPABILITY_OPERATOR_CONTROL = "operator_control"
CAPABILITY_MOTION_ARM = "motion.arm"
CAPABILITY_MOTION_GO2 = "motion.go2"
CAPABILITY_STREAM_RGBD = "stream.rgbd"
CAPABILITY_STREAM_OBSERVER = "stream.observer"
CAPABILITY_STREAM_HAND_EYE_PREVIEW = "stream.hand_eye_preview"

MEDIA_TRANSPORT_DDS = "dds"
MEDIA_TRANSPORT_WEBRTC = "webrtc"
MEDIA_TRANSPORTS = frozenset({MEDIA_TRANSPORT_DDS, MEDIA_TRANSPORT_WEBRTC})
MEDIA_KIND_RGB = "rgb"
MEDIA_KIND_RGBD = "rgbd"
MEDIA_KINDS = frozenset({MEDIA_KIND_RGB, MEDIA_KIND_RGBD})
MEDIA_SECURITY_NONE = "none"
MEDIA_SECURITY_DDS = "dds-security"
MEDIA_SECURITY_DTLS_SRTP = "dtls-srtp"
MEDIA_SECURITY_MODES = frozenset(
    {
        MEDIA_SECURITY_NONE,
        MEDIA_SECURITY_DDS,
        MEDIA_SECURITY_DTLS_SRTP,
    }
)

MESSAGE_TYPES = frozenset(
    {
        "discover",
        "endpoint_list",
        "operator_intent",
        "operator_result",
        "select_target",
        "target_selected",
        "renew_target",
        "release_target",
        "target_released",
        "target_lost",
        "lease_granted",
        "lease_revoked",
        "motion_command",
        "telemetry",
        "ack",
        "open_simulation_session",
        "simulation_session_opened",
        "simulation_session_granted",
        "simulation_session_revoked",
        "renew_simulation_session",
        "close_simulation_session",
        "simulation_command",
        "simulation_result",
        "simulation_status",
        "webrtc_signal",
        "error",
    }
)


class ProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class MediaStreamDescriptor:
    """A direct media stream advertised by an endpoint.

    RGB-D uses a typed DDS topic under the deployment security profile.
    Observer and hand-eye video use WebRTC DTLS-SRTP; their offer/answer
    signaling travels directly over DDS.
    """

    transport: str
    media_kind: str
    endpoint: str
    security: str

    def __post_init__(self) -> None:
        if self.transport not in MEDIA_TRANSPORTS:
            raise ProtocolError(f"unsupported media transport: {self.transport!r}")
        if self.media_kind not in MEDIA_KINDS:
            raise ProtocolError(f"unsupported media kind: {self.media_kind!r}")
        endpoint = str(self.endpoint).strip()
        if not endpoint or len(endpoint) > 2048 or any(char.isspace() for char in endpoint):
            raise ProtocolError("media endpoint must contain 1..2048 non-whitespace characters")
        if self.security not in MEDIA_SECURITY_MODES:
            raise ProtocolError(f"unsupported media security mode: {self.security!r}")
        if self.transport == MEDIA_TRANSPORT_WEBRTC:
            if self.media_kind != MEDIA_KIND_RGB:
                raise ProtocolError("WebRTC streams currently support RGB media only")
            if self.security != MEDIA_SECURITY_DTLS_SRTP:
                raise ProtocolError("WebRTC streams must use DTLS-SRTP security")
            return
        if self.transport == MEDIA_TRANSPORT_DDS:
            if self.security not in {MEDIA_SECURITY_NONE, MEDIA_SECURITY_DDS}:
                raise ProtocolError(
                    "DDS streams must use plaintext trusted-network or DDS security"
                )
            return

    def to_dict(self) -> dict[str, str]:
        return {
            "transport": self.transport,
            "media_kind": self.media_kind,
            "endpoint": self.endpoint,
            "security": self.security,
        }

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "MediaStreamDescriptor":
        if not isinstance(raw, Mapping):
            raise ProtocolError("media stream descriptor must be an object")
        unknown = sorted(
            set(raw)
            - {"transport", "media_kind", "endpoint", "security"}
        )
        if unknown:
            raise ProtocolError("unknown media stream descriptor fields: " + ", ".join(unknown))
        return cls(
            transport=str(raw.get("transport", "")),
            media_kind=str(raw.get("media_kind", "")),
            endpoint=str(raw.get("endpoint", "")),
            security=str(raw.get("security", "")),
        )


@dataclass(frozen=True)
class EndpointDescriptor:
    endpoint_id: str
    role: str
    capabilities: tuple[str, ...] = ()
    streams: Optional[dict[str, MediaStreamDescriptor]] = None
    instance_id: str = ""

    def __post_init__(self) -> None:
        _validate_identifier(self.endpoint_id, "endpoint_id")
        if self.role not in ENDPOINT_ROLES:
            raise ProtocolError(f"unsupported endpoint role: {self.role!r}")
        if any(not str(value).strip() for value in self.capabilities):
            raise ProtocolError("endpoint capabilities must not contain empty values")
        if len(set(self.capabilities)) != len(self.capabilities):
            raise ProtocolError("endpoint capabilities must be unique")
        if self.instance_id:
            _validate_identifier(self.instance_id, "instance_id")
        streams = self.streams or {}
        if not isinstance(streams, dict):
            raise ProtocolError("endpoint streams must be an object")
        for key, value in streams.items():
            _validate_identifier(str(key), "media stream name")
            if not isinstance(value, MediaStreamDescriptor):
                raise ProtocolError("endpoint streams must contain media stream descriptors")

    def to_dict(self) -> dict[str, Any]:
        return {
            "endpoint_id": self.endpoint_id,
            "role": self.role,
            "capabilities": list(self.capabilities),
            "streams": {
                str(name): descriptor.to_dict()
                for name, descriptor in (self.streams or {}).items()
            },
            "instance_id": self.instance_id,
        }

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "EndpointDescriptor":
        unknown = sorted(set(raw) - {"endpoint_id", "role", "capabilities", "streams", "instance_id"})
        if unknown:
            raise ProtocolError("unknown endpoint descriptor fields: " + ", ".join(unknown))
        capabilities = raw.get("capabilities", ())
        streams = raw.get("streams", {})
        if not isinstance(capabilities, (list, tuple)):
            raise ProtocolError("endpoint capabilities must be a list")
        if not isinstance(streams, Mapping):
            raise ProtocolError("endpoint streams must be an object")
        return cls(
            endpoint_id=str(raw.get("endpoint_id", "")),
            role=str(raw.get("role", "")),
            capabilities=tuple(str(value) for value in capabilities),
            streams={
                str(key): MediaStreamDescriptor.from_dict(value)
                for key, value in streams.items()
            }
            or None,
            instance_id=str(raw.get("instance_id", "")),
        )


@dataclass(frozen=True)
class Envelope:
    message_type: str
    source_id: str
    target_id: str = "server"
    payload: Optional[dict[str, Any]] = None
    seq: int = 0
    timestamp: float = 0.0
    message_id: str = ""
    lease_id: str = ""
    trace_context: Optional[dict[str, str]] = None
    version: int = PROTOCOL_VERSION

    def __post_init__(self) -> None:
        if isinstance(self.version, bool) or type(self.version) is not int or self.version != PROTOCOL_VERSION:
            raise ProtocolError(
                f"protocol version {self.version!r} is unsupported; expected {PROTOCOL_VERSION}"
            )
        _validate_identifier(self.message_type, "message_type")
        if self.message_type not in MESSAGE_TYPES:
            raise ProtocolError(f"unsupported message type: {self.message_type!r}")
        _validate_identifier(self.source_id, "source_id")
        _validate_identifier(self.target_id, "target_id")
        if isinstance(self.seq, bool) or type(self.seq) is not int or self.seq < 0:
            raise ProtocolError("seq must be non-negative")
        if isinstance(self.timestamp, bool) or not isinstance(self.timestamp, Real) or not math.isfinite(float(self.timestamp)):
            raise ProtocolError("timestamp must be finite")
        if not isinstance(self.payload or {}, dict):
            raise ProtocolError("payload must be an object")
        if not isinstance(self.trace_context or {}, dict):
            raise ProtocolError("trace_context must be an object")
        _validate_json_value(self.payload or {}, context="payload")
        trace = self.trace_context or {}
        if any(
            not isinstance(key, str)
            or not isinstance(value, str)
            or len(key) > 128
            or len(value) > 512
            for key, value in trace.items()
        ):
            raise ProtocolError("trace_context keys or values are invalid")

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": int(self.version),
            "message_id": self.message_id or uuid.uuid4().hex,
            "type": self.message_type,
            "source_id": self.source_id,
            "target_id": self.target_id,
            "seq": int(self.seq),
            "timestamp": float(self.timestamp or time.time()),
            "lease_id": self.lease_id,
            "trace_context": dict(self.trace_context or {}),
            "payload": dict(self.payload or {}),
        }

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "Envelope":
        required = ("version", "message_id", "type", "source_id", "target_id", "seq", "timestamp", "payload")
        missing = [key for key in required if key not in raw]
        if missing:
            raise ProtocolError(f"missing envelope fields: {', '.join(missing)}")
        trace_context = raw.get("trace_context", {})
        if not isinstance(trace_context, Mapping):
            raise ProtocolError("trace_context must be an object")
        payload = raw["payload"]
        if not isinstance(payload, Mapping):
            raise ProtocolError("payload must be an object")
        version = raw["version"]
        seq = raw["seq"]
        timestamp = raw["timestamp"]
        if isinstance(version, bool) or type(version) is not int:
            raise ProtocolError("version must be an integer")
        if isinstance(seq, bool) or type(seq) is not int:
            raise ProtocolError("seq must be an integer")
        if isinstance(timestamp, bool) or not isinstance(timestamp, Real):
            raise ProtocolError("timestamp must be numeric")
        return cls(
            version=version,
            message_id=str(raw["message_id"]),
            message_type=str(raw["type"]),
            source_id=str(raw["source_id"]),
            target_id=str(raw["target_id"]),
            seq=seq,
            timestamp=float(timestamp),
            lease_id=str(raw.get("lease_id", "")),
            trace_context={str(key): str(value) for key, value in trace_context.items()},
            payload=dict(payload),
        )


def _validate_identifier(value: str, field: str) -> None:
    text = str(value).strip()
    if not text or len(text) > 128:
        raise ProtocolError(f"{field} must contain 1..128 characters")
    if any(char.isspace() for char in text):
        raise ProtocolError(f"{field} must not contain whitespace")


def _validate_json_value(value: Any, *, context: str, depth: int = 0) -> None:
    if depth > 16:
        raise ProtocolError(f"{context} nesting is too deep")
    if value is None or isinstance(value, (bool, str)):
        return
    if isinstance(value, int) and not isinstance(value, bool):
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ProtocolError(f"{context} contains a non-finite number")
        return
    if isinstance(value, list):
        for item in value:
            _validate_json_value(item, context=context, depth=depth + 1)
        return
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise ProtocolError(f"{context} keys must be strings")
            _validate_json_value(item, context=context, depth=depth + 1)
        return
    raise ProtocolError(f"{context} contains a non-JSON value: {type(value).__name__}")


def make_envelope(
    message_type: str,
    source_id: str,
    *,
    target_id: str = "server",
    payload: Optional[dict[str, Any]] = None,
    seq: int = 0,
    lease_id: str = "",
    trace_context: Optional[dict[str, str]] = None,
) -> Envelope:
    return Envelope(
        message_type=message_type,
        source_id=source_id,
        target_id=target_id,
        payload=payload or {},
        seq=seq,
        timestamp=time.time(),
        message_id=uuid.uuid4().hex,
        lease_id=lease_id,
        trace_context=trace_context or {},
    )


def dumps_envelope(envelope: Envelope) -> bytes:
    try:
        encoded = json.dumps(
            envelope.to_dict(),
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f"envelope is not JSON-safe: {exc}") from exc
    if len(encoded) > MAX_ENVELOPE_BYTES:
        raise ProtocolError(
            f"envelope is too large: {len(encoded)} bytes > {MAX_ENVELOPE_BYTES}"
        )
    return encoded


def loads_envelope(buf: bytes) -> Envelope:
    if len(buf) > MAX_ENVELOPE_BYTES:
        raise ProtocolError(
            f"envelope is too large: {len(buf)} bytes > {MAX_ENVELOPE_BYTES}"
        )
    try:
        raw = json.loads(buf.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"invalid envelope JSON: {exc}") from exc
    if not isinstance(raw, Mapping):
        raise ProtocolError("envelope root must be an object")
    return Envelope.from_dict(raw)


def pack_state(
    *,
    u: Optional[ControlU] = None,
    q: Optional[SimQ] = None,
    sim_q: Optional[SimQ] = None,
    ts: Optional[float] = None,
    torque_enabled: Optional[bool] = None,
    ik_target_xyz: Optional[tuple[float, float, float]] = None,
    ik_target_dir: Optional[tuple[float, float, float]] = None,
    actual_tip_xyz: Optional[tuple[float, float, float]] = None,
    actual_tip_dir: Optional[tuple[float, float, float]] = None,
    sag_model: Optional[dict[str, Any]] = None,
    claw_closed: Optional[bool] = None,
    claw_current: Optional[int] = None,
    motor_currents_ma: Optional[dict[str, int]] = None,
    motor_positions_raw: Optional[dict[str, int]] = None,
    motor_positions_deg: Optional[dict[str, float]] = None,
    safety_fault: Optional[str] = None,
    perceived_object_label: Optional[str] = None,
    perceived_object_confidence: Optional[float] = None,
    perceived_object_camera: Optional[tuple[float, float, float]] = None,
    perceived_center_uv: Optional[tuple[float, float]] = None,
    perceived_scale: Optional[float] = None,
    perceived_timestamp_s: Optional[float] = None,
    perception_running: Optional[bool] = None,
    perception_failed: Optional[bool] = None,
    perception_status: Optional[str] = None,
    perception_source: Optional[str] = None,
    perception_recording: Optional[bool] = None,
    perception_record_with_overlay: Optional[bool] = None,
    perception_last_record_path: Optional[str] = None,
    perception_last_capture_path: Optional[str] = None,
    perception_hz: Optional[float] = None,
    gaze_running: Optional[bool] = None,
    gaze_mode: Optional[str] = None,
    gaze_status_msg: Optional[str] = None,
    gaze_u_err: Optional[float] = None,
    gaze_v_err: Optional[float] = None,
    gaze_du_roll: Optional[float] = None,
    gaze_du_s1: Optional[float] = None,
    gaze_du_s2: Optional[float] = None,
    gaze_obs_age_s: Optional[float] = None,
    gaze_tick_count: Optional[int] = None,
    gaze_update_count: Optional[int] = None,
    gaze_config: Optional[dict[str, Any]] = None,
    pick_running: Optional[bool] = None,
    pick_failed: Optional[bool] = None,
    pick_phase: Optional[str] = None,
    pick_status_msg: Optional[str] = None,
    debug_markers: Optional[list[dict[str, Any]]] = None,
    go2_vel: Optional[tuple[float, float, float]] = None,
    go2_base_rpy: Optional[tuple[float, float, float]] = None,
    go2_base_pos: Optional[tuple[float, float, float]] = None,
    go2_sim_base_pos: Optional[tuple[float, float, float]] = None,
    go2_base_lin_vel_body: Optional[tuple[float, float, float]] = None,
    go2_base_ang_vel: Optional[tuple[float, float, float]] = None,
    go2_base_timestamp_s: Optional[float] = None,
    go2_leg_q: Optional[tuple[float, ...]] = None,
    go2_leg_dq: Optional[tuple[float, ...]] = None,
    go2_leg_torque_nm: Optional[tuple[float, ...]] = None,
    go2_sport_pose: Optional[str] = None,
    go2_sport_pose_seq: Optional[int] = None,
    go2_obstacles_avoid_enabled: Optional[bool] = None,
    go2_obstacles_avoid_seq: Optional[int] = None,
    sim_target_xyz: Optional[tuple[float, float, float]] = None,
    sim_reset_seq: Optional[int] = None,
    sim_time_s: Optional[float] = None,
    sim_wall_elapsed_s: Optional[float] = None,
    sim_realtime_factor: Optional[float] = None,
    sim_step_count: Optional[int] = None,
) -> Dict[str, Any]:
    ts = now_s() if ts is None else float(ts)
    out: Dict[str, Any] = {"t": "state", "ts": ts}
    if u is not None:
        out["u"] = {"linear": u.u_linear, "roll": u.u_roll, "s1": u.u_s1, "s2": u.u_s2}
    if q is not None:
        out["q"] = {
            "linear_m": q.linear_m,
            "roll_rad": q.roll_rad,
            "theta1_rad": q.theta1_rad,
            "theta2_rad": q.theta2_rad,
        }
    if sim_q is not None:
        out["sim_q"] = {
            "linear_m": sim_q.linear_m,
            "roll_rad": sim_q.roll_rad,
            "theta1_rad": sim_q.theta1_rad,
            "theta2_rad": sim_q.theta2_rad,
        }
    if torque_enabled is not None:
        out["torque_enabled"] = bool(torque_enabled)
    if ik_target_xyz is not None:
        out["ik_target"] = [float(ik_target_xyz[0]), float(ik_target_xyz[1]), float(ik_target_xyz[2])]
    if ik_target_dir is not None:
        out["ik_target_dir"] = [float(ik_target_dir[0]), float(ik_target_dir[1]), float(ik_target_dir[2])]
    if actual_tip_xyz is not None:
        out["actual_tip"] = [float(actual_tip_xyz[0]), float(actual_tip_xyz[1]), float(actual_tip_xyz[2])]
    if actual_tip_dir is not None:
        out["actual_tip_dir"] = [float(actual_tip_dir[0]), float(actual_tip_dir[1]), float(actual_tip_dir[2])]
    if sag_model is not None:
        out["sag_model"] = dict(sag_model)
    if claw_closed is not None:
        out["claw_closed"] = bool(claw_closed)
    if claw_current is not None:
        out["claw_current"] = int(claw_current)
    if motor_currents_ma is not None:
        out["motor_currents_ma"] = {str(k): int(v) for k, v in motor_currents_ma.items()}
    if motor_positions_raw is not None:
        out["motor_positions_raw"] = {str(k): int(v) for k, v in motor_positions_raw.items()}
    if motor_positions_deg is not None:
        out["motor_positions_deg"] = {str(k): float(v) for k, v in motor_positions_deg.items()}
    if safety_fault is not None:
        out["safety_fault"] = str(safety_fault)
    if perceived_object_label is not None:
        out["perceived_object_label"] = str(perceived_object_label)
    if perceived_object_confidence is not None:
        out["perceived_object_confidence"] = float(perceived_object_confidence)
    if perceived_object_camera is not None:
        out["perceived_object_camera"] = [
            float(perceived_object_camera[0]),
            float(perceived_object_camera[1]),
            float(perceived_object_camera[2]),
        ]
    if perceived_center_uv is not None:
        out["perceived_center_uv"] = [float(perceived_center_uv[0]), float(perceived_center_uv[1])]
    if perceived_scale is not None:
        out["perceived_scale"] = float(perceived_scale)
    if perceived_timestamp_s is not None:
        out["perceived_timestamp_s"] = float(perceived_timestamp_s)
    if perception_running is not None:
        out["perception_running"] = bool(perception_running)
    if perception_failed is not None:
        out["perception_failed"] = bool(perception_failed)
    if perception_status is not None:
        out["perception_status"] = str(perception_status)
    if perception_source is not None:
        out["perception_source"] = str(perception_source)
    if perception_recording is not None:
        out["perception_recording"] = bool(perception_recording)
    if perception_record_with_overlay is not None:
        out["perception_record_with_overlay"] = bool(perception_record_with_overlay)
    if perception_last_record_path is not None:
        out["perception_last_record_path"] = str(perception_last_record_path)
    if perception_last_capture_path is not None:
        out["perception_last_capture_path"] = str(perception_last_capture_path)
    if perception_hz is not None:
        out["perception_hz"] = float(perception_hz)
    if gaze_running is not None:
        out["gaze_running"] = bool(gaze_running)
    if gaze_mode is not None:
        out["gaze_mode"] = str(gaze_mode)
    if gaze_status_msg is not None:
        out["gaze_status_msg"] = str(gaze_status_msg)
    if gaze_u_err is not None:
        out["gaze_u_err"] = float(gaze_u_err)
    if gaze_v_err is not None:
        out["gaze_v_err"] = float(gaze_v_err)
    if gaze_du_roll is not None:
        out["gaze_du_roll"] = float(gaze_du_roll)
    if gaze_du_s1 is not None:
        out["gaze_du_s1"] = float(gaze_du_s1)
    if gaze_du_s2 is not None:
        out["gaze_du_s2"] = float(gaze_du_s2)
    if gaze_obs_age_s is not None:
        out["gaze_obs_age_s"] = float(gaze_obs_age_s)
    if gaze_tick_count is not None:
        out["gaze_tick_count"] = int(gaze_tick_count)
    if gaze_update_count is not None:
        out["gaze_update_count"] = int(gaze_update_count)
    if gaze_config is not None:
        out["gaze_config"] = dict(gaze_config)
    if pick_running is not None:
        out["pick_running"] = bool(pick_running)
    if pick_failed is not None:
        out["pick_failed"] = bool(pick_failed)
    if pick_phase is not None:
        out["pick_phase"] = str(pick_phase)
    if pick_status_msg is not None:
        out["pick_status_msg"] = str(pick_status_msg)
    if go2_vel is not None:
        out["go2_vel"] = [float(go2_vel[0]), float(go2_vel[1]), float(go2_vel[2])]
    if go2_base_rpy is not None:
        out["go2_base_rpy"] = [float(go2_base_rpy[0]), float(go2_base_rpy[1]), float(go2_base_rpy[2])]
    if go2_base_pos is not None:
        out["go2_base_pos"] = [float(go2_base_pos[0]), float(go2_base_pos[1]), float(go2_base_pos[2])]
    if go2_sim_base_pos is not None:
        out["go2_sim_base_pos"] = [
            float(go2_sim_base_pos[0]),
            float(go2_sim_base_pos[1]),
            float(go2_sim_base_pos[2]),
        ]
    if go2_base_lin_vel_body is not None:
        out["go2_base_lin_vel_body"] = [
            float(go2_base_lin_vel_body[0]),
            float(go2_base_lin_vel_body[1]),
            float(go2_base_lin_vel_body[2]),
        ]
    if go2_base_ang_vel is not None:
        out["go2_base_ang_vel"] = [
            float(go2_base_ang_vel[0]),
            float(go2_base_ang_vel[1]),
            float(go2_base_ang_vel[2]),
        ]
    if go2_base_timestamp_s is not None:
        out["go2_base_timestamp_s"] = float(go2_base_timestamp_s)
    if go2_leg_q is not None and len(go2_leg_q) == 12:
        out["go2_leg_q"] = [float(v) for v in go2_leg_q]
    if go2_leg_dq is not None and len(go2_leg_dq) == 12:
        out["go2_leg_dq"] = [float(v) for v in go2_leg_dq]
    if go2_leg_torque_nm is not None and len(go2_leg_torque_nm) == 12:
        out["go2_leg_torque_nm"] = [float(v) for v in go2_leg_torque_nm]
    if go2_sport_pose is not None:
        out["go2_sport_pose"] = str(go2_sport_pose).strip().lower()
    if go2_sport_pose_seq is not None:
        out["go2_sport_pose_seq"] = int(go2_sport_pose_seq)
    if go2_obstacles_avoid_enabled is not None:
        out["go2_obstacles_avoid_enabled"] = bool(go2_obstacles_avoid_enabled)
    if go2_obstacles_avoid_seq is not None:
        out["go2_obstacles_avoid_seq"] = int(go2_obstacles_avoid_seq)
    if sim_target_xyz is not None:
        out["sim_target"] = [float(sim_target_xyz[0]), float(sim_target_xyz[1]), float(sim_target_xyz[2])]
    if sim_reset_seq is not None:
        out["sim_reset_seq"] = int(sim_reset_seq)
    if sim_time_s is not None:
        out["sim_time_s"] = float(sim_time_s)
    if sim_wall_elapsed_s is not None:
        out["sim_wall_elapsed_s"] = float(sim_wall_elapsed_s)
    if sim_realtime_factor is not None:
        out["sim_realtime_factor"] = float(sim_realtime_factor)
    if sim_step_count is not None:
        out["sim_step_count"] = int(sim_step_count)
    if debug_markers is not None:
        packed_markers: list[dict[str, Any]] = []
        for raw in list(debug_markers):
            if not isinstance(raw, dict):
                continue
            name = str(raw.get("name", "")).strip()
            frame = str(raw.get("frame", "world")).strip() or "world"
            pos = raw.get("pos", None)
            if not name or not isinstance(pos, (list, tuple)) or len(pos) != 3:
                continue
            marker: dict[str, Any] = {
                "name": name,
                "frame": frame,
                "pos": [float(pos[0]), float(pos[1]), float(pos[2])],
            }
            direction = raw.get("dir", None)
            if isinstance(direction, (list, tuple)) and len(direction) == 3:
                marker["dir"] = [float(direction[0]), float(direction[1]), float(direction[2])]
            color = raw.get("color", None)
            if isinstance(color, (list, tuple)) and len(color) in (3, 4):
                marker["color"] = [float(v) for v in color]
            if "radius" in raw:
                marker["radius"] = float(raw.get("radius", 0.0))
            if "length" in raw:
                marker["length"] = float(raw.get("length", 0.0))
            if "ttl_ms" in raw:
                marker["ttl_ms"] = int(raw.get("ttl_ms", 0))
            packed_markers.append(marker)
        out["debug_markers"] = packed_markers
    return out


def pack_target_q(q: SimQ, *, source: str, seq: int, ts: Optional[float] = None) -> Dict[str, Any]:
    ts = now_s() if ts is None else float(ts)
    return {
        "t": "target",
        "ts": ts,
        "seq": int(seq),
        "source": str(source),
        "q": {
            "linear_m": q.linear_m,
            "roll_rad": q.roll_rad,
            "theta1_rad": q.theta1_rad,
            "theta2_rad": q.theta2_rad,
        },
    }


def unpack_u(d: Dict[str, Any]) -> ControlU:
    return ControlU(
        u_linear=float(d.get("linear", 0.0)),
        u_roll=float(d.get("roll", 0.0)),
        u_s1=float(d.get("s1", 0.0)),
        u_s2=float(d.get("s2", 0.0)),
    )


def unpack_q(d: Dict[str, Any]) -> SimQ:
    return SimQ(
        linear_m=float(d.get("linear_m", 0.0)),
        roll_rad=float(d.get("roll_rad", 0.0)),
        theta1_rad=float(d.get("theta1_rad", 0.0)),
        theta2_rad=float(d.get("theta2_rad", 0.0)),
    )


def unpack_go2_vel(raw: Any) -> tuple[float, float, float]:
    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        raise ValueError("go2_vel must be [vx, vy, wz]")
    return (float(raw[0]), float(raw[1]), float(raw[2]))


def unpack_go2_sport_pose(raw: Any) -> str:
    pose = str(raw).strip().lower()
    if not pose:
        raise ValueError("go2_sport_pose must be a non-empty string")
    return pose


def unpack_go2_obstacles_avoid_enable(raw: Any) -> bool:
    if isinstance(raw, bool):
        return bool(raw)
    if isinstance(raw, (int, float)):
        return bool(int(raw))
    text = str(raw).strip().lower()
    if text in ("1", "true", "yes", "on", "enable", "enabled"):
        return True
    if text in ("0", "false", "no", "off", "disable", "disabled"):
        return False
    raise ValueError("go2_obstacles_avoid_enable must be a boolean")


def unpack_vec3(raw: Any, *, name: str) -> tuple[float, float, float]:
    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        raise ValueError(f"{name} must be [x, y, z]")
    return (float(raw[0]), float(raw[1]), float(raw[2]))
