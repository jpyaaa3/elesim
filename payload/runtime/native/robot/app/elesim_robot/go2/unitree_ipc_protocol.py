"""Bounded local wire contract between Robot and the Unitree DDS bridge."""

from __future__ import annotations

import json
import math
import socket
import struct
import uuid
from dataclasses import dataclass
from numbers import Real
from typing import Any, Mapping, Optional, Sequence

from elesim_robot.go2.odom_parser import OdomSample
from elesim_robot.go2.sport_api import normalize_go2_sport_pose, sport_pose_api_id


PROTOCOL_VERSION = 1
MAX_PACKET_BYTES = 64 * 1024
MAX_SEQUENCE = (1 << 63) - 1
PACKET_KINDS = frozenset({"hello", "heartbeat", "command", "telemetry", "error"})
COMMAND_NAMES = frozenset({"set_velocity", "sport_pose", "obstacles_avoid", "stop"})


class UnitreeIpcProtocolError(ValueError):
    """A local peer sent a malformed, oversized, stale, or unauthorized packet."""


@dataclass(frozen=True)
class UnitreeIpcPacket:
    kind: str
    boot_id: str
    seq: int
    sent_monotonic_s: float
    payload: dict[str, Any]


@dataclass(frozen=True)
class PeerCredentials:
    pid: int
    uid: int
    gid: int


def new_boot_id() -> str:
    return uuid.uuid4().hex


def _finite(raw: object, *, name: str) -> float:
    if isinstance(raw, bool) or not isinstance(raw, Real):
        raise UnitreeIpcProtocolError(f"{name} must be numeric")
    value = float(raw)
    if not math.isfinite(value):
        raise UnitreeIpcProtocolError(f"{name} must be finite")
    return value


def _boot_id(raw: object) -> str:
    value = str(raw).strip()
    try:
        parsed = uuid.UUID(value)
    except (ValueError, AttributeError) as exc:
        raise UnitreeIpcProtocolError("boot_id must be a UUID") from exc
    if parsed.hex != value:
        raise UnitreeIpcProtocolError("boot_id must be lowercase UUID hex")
    return value


def encode_packet(
    kind: str,
    boot_id: str,
    seq: int,
    payload: Mapping[str, object],
    *,
    sent_monotonic_s: float,
) -> bytes:
    packet_kind = str(kind).strip()
    if packet_kind not in PACKET_KINDS:
        raise UnitreeIpcProtocolError(f"unsupported packet kind: {packet_kind!r}")
    boot = _boot_id(boot_id)
    if isinstance(seq, bool) or not isinstance(seq, int) or not 0 <= seq <= MAX_SEQUENCE:
        raise UnitreeIpcProtocolError("seq must be an unsigned 63-bit integer")
    sent = _finite(sent_monotonic_s, name="sent_monotonic_s")
    if sent < 0.0:
        raise UnitreeIpcProtocolError("sent_monotonic_s must be non-negative")
    if not isinstance(payload, Mapping):
        raise UnitreeIpcProtocolError("payload must be an object")
    document = {
        "version": PROTOCOL_VERSION,
        "kind": packet_kind,
        "boot_id": boot,
        "seq": seq,
        "sent_monotonic_s": sent,
        "payload": dict(payload),
    }
    try:
        encoded = json.dumps(
            document,
            ensure_ascii=True,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise UnitreeIpcProtocolError("packet is not finite JSON") from exc
    if len(encoded) > MAX_PACKET_BYTES:
        raise UnitreeIpcProtocolError(
            f"packet exceeds {MAX_PACKET_BYTES} byte limit"
        )
    return encoded


def decode_packet(data: bytes, *, truncated: bool = False) -> UnitreeIpcPacket:
    if truncated or len(data) > MAX_PACKET_BYTES:
        raise UnitreeIpcProtocolError("packet exceeds size limit")
    if not data:
        raise UnitreeIpcProtocolError("empty packet")
    try:
        raw = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise UnitreeIpcProtocolError("packet is not valid UTF-8 JSON") from exc
    if not isinstance(raw, dict):
        raise UnitreeIpcProtocolError("packet must be an object")
    expected = {"version", "kind", "boot_id", "seq", "sent_monotonic_s", "payload"}
    if set(raw) != expected:
        raise UnitreeIpcProtocolError("packet fields do not match protocol v1")
    if raw["version"] != PROTOCOL_VERSION or isinstance(raw["version"], bool):
        raise UnitreeIpcProtocolError("unsupported Unitree IPC protocol version")
    kind = str(raw["kind"]).strip()
    if kind not in PACKET_KINDS:
        raise UnitreeIpcProtocolError(f"unsupported packet kind: {kind!r}")
    seq = raw["seq"]
    if isinstance(seq, bool) or not isinstance(seq, int) or not 0 <= seq <= MAX_SEQUENCE:
        raise UnitreeIpcProtocolError("invalid packet sequence")
    payload = raw["payload"]
    if not isinstance(payload, dict):
        raise UnitreeIpcProtocolError("packet payload must be an object")
    sent = _finite(raw["sent_monotonic_s"], name="sent_monotonic_s")
    if sent < 0.0:
        raise UnitreeIpcProtocolError("sent_monotonic_s must be non-negative")
    if kind in {"hello", "heartbeat"} and payload:
        raise UnitreeIpcProtocolError(f"{kind} payload must be empty")
    return UnitreeIpcPacket(kind, _boot_id(raw["boot_id"]), seq, sent, payload)


def receive_packet(sock: socket.socket) -> Optional[UnitreeIpcPacket]:
    try:
        data, _ancillary, flags, _address = sock.recvmsg(MAX_PACKET_BYTES)
    except BlockingIOError:
        return None
    if not data:
        raise ConnectionError("Unitree IPC peer disconnected")
    return decode_packet(data, truncated=bool(flags & socket.MSG_TRUNC))


def peer_credentials(sock: socket.socket) -> PeerCredentials:
    if not hasattr(socket, "SO_PEERCRED"):
        raise RuntimeError("SO_PEERCRED is required for Unitree IPC")
    raw = sock.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i"))
    pid, uid, gid = struct.unpack("3i", raw)
    return PeerCredentials(pid=int(pid), uid=int(uid), gid=int(gid))


def validate_command(
    payload: Mapping[str, object],
    *,
    max_velocity: Sequence[float],
) -> dict[str, object]:
    name = str(payload.get("name", "")).strip()
    if name not in COMMAND_NAMES:
        raise UnitreeIpcProtocolError(f"unsupported command: {name!r}")
    expected: dict[str, set[str]] = {
        "set_velocity": {"name", "vx", "vy", "wz"},
        "sport_pose": {"name", "pose"},
        "obstacles_avoid": {"name", "enabled"},
        "stop": {"name"},
    }
    if set(payload) != expected[name]:
        raise UnitreeIpcProtocolError(f"unexpected fields for {name}")
    if name == "set_velocity":
        values = tuple(_finite(payload[key], name=key) for key in ("vx", "vy", "wz"))
        limits = tuple(_finite(value, name="velocity limit") for value in max_velocity)
        if len(limits) != 3 or any(limit <= 0.0 for limit in limits):
            raise UnitreeIpcProtocolError("invalid velocity limits")
        if any(abs(value) > limit for value, limit in zip(values, limits)):
            raise UnitreeIpcProtocolError("GO2 velocity exceeds configured limit")
        return {"name": name, "vx": values[0], "vy": values[1], "wz": values[2]}
    if name == "sport_pose":
        pose = normalize_go2_sport_pose(str(payload["pose"]))
        if sport_pose_api_id(pose) is None:
            raise UnitreeIpcProtocolError("unsupported GO2 sport pose")
        return {"name": name, "pose": pose}
    if name == "obstacles_avoid":
        if not isinstance(payload["enabled"], bool):
            raise UnitreeIpcProtocolError("obstacles_avoid.enabled must be boolean")
        return {"name": name, "enabled": payload["enabled"]}
    return {"name": "stop"}


def sample_to_payload(sample: OdomSample) -> dict[str, object]:
    def vector(values: Sequence[object], *, name: str, length: int) -> list[float]:
        if len(values) != length:
            raise UnitreeIpcProtocolError(f"{name} must contain {length} values")
        return [_finite(value, name=name) for value in values]

    def optional(values: Optional[Sequence[object]], *, name: str) -> object:
        return None if values is None else vector(values, name=name, length=12)

    return {
        "pos": vector(sample.pos, name="pos", length=3),
        "rpy": vector(sample.rpy, name="rpy", length=3),
        "lin_vel_body": vector(sample.lin_vel_body, name="lin_vel_body", length=3),
        "ang_vel_body": vector(sample.ang_vel_body, name="ang_vel_body", length=3),
        "timestamp_s": _finite(sample.timestamp_s, name="timestamp_s"),
        "leg_q": optional(sample.leg_q, name="leg_q"),
        "leg_dq": optional(sample.leg_dq, name="leg_dq"),
        "leg_torque_nm": optional(sample.leg_torque_nm, name="leg_torque_nm"),
    }


def sample_from_payload(raw: object) -> OdomSample:
    if not isinstance(raw, Mapping):
        raise UnitreeIpcProtocolError("telemetry sample must be an object")
    expected = {
        "pos", "rpy", "lin_vel_body", "ang_vel_body", "timestamp_s",
        "leg_q", "leg_dq", "leg_torque_nm",
    }
    if set(raw) != expected:
        raise UnitreeIpcProtocolError("telemetry fields do not match protocol v1")

    def vector(name: str, length: int) -> tuple[float, ...]:
        value = raw[name]
        if not isinstance(value, list) or len(value) != length:
            raise UnitreeIpcProtocolError(f"{name} must contain {length} values")
        return tuple(_finite(item, name=name) for item in value)

    def optional(name: str) -> Optional[tuple[float, ...]]:
        return None if raw[name] is None else vector(name, 12)

    return OdomSample(
        pos=vector("pos", 3),
        rpy=vector("rpy", 3),
        lin_vel_body=vector("lin_vel_body", 3),
        ang_vel_body=vector("ang_vel_body", 3),
        timestamp_s=_finite(raw["timestamp_s"], name="timestamp_s"),
        leg_q=optional("leg_q"),
        leg_dq=optional("leg_dq"),
        leg_torque_nm=optional("leg_torque_nm"),
    )


__all__ = [
    "MAX_PACKET_BYTES",
    "PROTOCOL_VERSION",
    "PeerCredentials",
    "UnitreeIpcPacket",
    "UnitreeIpcProtocolError",
    "decode_packet",
    "encode_packet",
    "new_boot_id",
    "peer_credentials",
    "receive_packet",
    "sample_from_payload",
    "sample_to_payload",
    "validate_command",
]
