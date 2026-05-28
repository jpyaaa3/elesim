"""Send perceived object (camera frame) to elesim host; sim converts to world."""

from __future__ import annotations

import os
import time
from typing import Any, Mapping, Optional

try:
    import zmq
except ImportError:
    zmq = None  # type: ignore

_HOST_CLIENT_VERSION = "2025-05-camera-frame-v3-tracker"


class HostPublishError(RuntimeError):
    pass


def _recv_reply(sock: Any) -> dict[str, Any]:
    msg = sock.recv_json(flags=0)
    if not isinstance(msg, dict):
        raise HostPublishError(f"host reply is not a JSON object: {msg!r}")
    return msg


def _wait_acks(sock: Any, *, poller: Any, count: int, timeout_ms: int) -> list[dict[str, Any]]:
    acks: list[dict[str, Any]] = []
    deadline = time.time() + max(timeout_ms, 50) / 1000.0
    while len(acks) < max(1, int(count)) and time.time() < deadline:
        remaining_ms = int(max(0.0, deadline - time.time()) * 1000)
        if remaining_ms <= 0:
            break
        events = dict(poller.poll(timeout=remaining_ms))
        if sock not in events:
            break
        try:
            acks.append(_recv_reply(sock))
        except zmq.Again:
            break
    return acks


def _parse_object_world(ack: dict[str, Any]) -> tuple[float, float, float] | None:
    raw = ack.get("object_world", None)
    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        return None
    return (float(raw[0]), float(raw[1]), float(raw[2]))


def build_perception_target_payload(
    *,
    object_camera_xyz: tuple[float, float, float] | list[float],
    label: str = "",
    track: Optional[Mapping[str, Any]] = None,
) -> dict[str, Any]:
    p = [
        float(object_camera_xyz[0]),
        float(object_camera_xyz[1]),
        float(object_camera_xyz[2]),
    ]
    payload: dict[str, Any] = {
        "t": "target",
        "ts": time.time(),
        "seq": 1,
        "source": "perception",
        "object_camera": p,
        "object_label": str(label),
    }
    if not track:
        return payload
    track_state = str(track.get("track_state", "")).strip()
    if track_state:
        payload["track_state"] = track_state
    if "track_confidence" in track:
        payload["track_confidence"] = float(track["track_confidence"])
    bbox = track.get("bbox_xyxy", None)
    if isinstance(bbox, (list, tuple)) and len(bbox) == 4:
        payload["bbox_xyxy"] = [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])]
    center = track.get("center_uv", None)
    if isinstance(center, (list, tuple)) and len(center) == 2:
        payload["center_uv"] = [float(center[0]), float(center[1])]
    mu = track.get("mu_camera", None)
    if isinstance(mu, (list, tuple)) and len(mu) == 3:
        payload["mu_camera"] = [float(mu[0]), float(mu[1]), float(mu[2])]
    sigma = track.get("sigma_camera", None)
    if isinstance(sigma, (list, tuple)) and len(sigma) == 3:
        payload["sigma_camera"] = [float(sigma[0]), float(sigma[1]), float(sigma[2])]
    if "depth_valid_ratio" in track:
        payload["depth_valid_ratio"] = float(track["depth_valid_ratio"])
    if "lost_count" in track:
        payload["lost_count"] = int(track["lost_count"])
    return payload


def publish_perception_track(
    *,
    endpoint: str,
    object_camera_xyz: tuple[float, float, float] | list[float],
    label: str = "",
    track: Optional[Mapping[str, Any]] = None,
    timeout_ms: int = 500,
) -> tuple[float, float, float] | None:
    """Publish object + optional ROI tracker telemetry to host.py."""
    if zmq is None:
        raise HostPublishError("pyzmq is not installed. Install with: pip install pyzmq")

    payload = build_perception_target_payload(
        object_camera_xyz=object_camera_xyz,
        label=label,
        track=track,
    )
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.DEALER)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVTIMEO, int(max(timeout_ms, 50)))
    identity = f"perception-{os.getpid()}-{int(time.time() * 1000)}".encode("utf-8")
    sock.setsockopt(zmq.IDENTITY, identity)
    sock.connect(str(endpoint))

    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)

    try:
        sock.send_json({"t": "hello", "ts": time.time()}, flags=0)
        sock.send_json(payload, flags=0)
        acks = _wait_acks(sock, poller=poller, count=2, timeout_ms=timeout_ms)
        if not acks:
            raise HostPublishError(
                f"host sent no reply within {timeout_ms} ms "
                f"(is host.py running on {endpoint}? client={_HOST_CLIENT_VERSION})"
            )
        ack = acks[-1]
        if not bool(ack.get("ok", False)):
            raise HostPublishError(f"host rejected message: {ack.get('reason', ack)}")
        return _parse_object_world(ack)
    except zmq.Again as exc:
        raise HostPublishError(
            f"host did not reply within {timeout_ms} ms "
            f"(is host.py running on {endpoint}? client={_HOST_CLIENT_VERSION})"
        ) from exc
    finally:
        try:
            poller.unregister(sock)
        except Exception:
            pass
        try:
            sock.close(0)
        except Exception:
            pass


def publish_perceived_object(
    *,
    endpoint: str,
    object_camera_xyz: tuple[float, float, float] | list[float],
    label: str = "",
    timeout_ms: int = 500,
    track: Optional[Mapping[str, Any]] = None,
) -> tuple[float, float, float] | None:
    """
    Send object position in camera optical frame to host.py (source=perception).

    host.py applies hand-eye + FK and returns world coordinates in the ack when ok.
  Optional ``track`` dict adds ROI tracker fields (mu_camera, track_confidence, ...).
    """
    return publish_perception_track(
        endpoint=endpoint,
        object_camera_xyz=object_camera_xyz,
        label=label,
        track=track,
        timeout_ms=timeout_ms,
    )
