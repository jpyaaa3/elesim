"""Send perceived object (camera frame) to elesim host; sim converts to world."""

from __future__ import annotations

import os
import time
from typing import Any

try:
    import zmq
except ImportError:
    zmq = None  # type: ignore

_HOST_CLIENT_VERSION = "2025-05-camera-frame-v1"


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


def publish_perceived_object(
    *,
    endpoint: str,
    object_camera_xyz: tuple[float, float, float] | list[float],
    label: str = "",
    timeout_ms: int = 500,
) -> None:
    """
    Send object position in camera optical frame to host.py (source=perception).

    elesim sim.py loads hand-eye mount config and converts to world for the green marker.
    """
    if zmq is None:
        raise HostPublishError("pyzmq is not installed. Install with: pip install pyzmq")

    p = [
        float(object_camera_xyz[0]),
        float(object_camera_xyz[1]),
        float(object_camera_xyz[2]),
    ]
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
        sock.send_json(
            {
                "t": "target",
                "ts": time.time(),
                "seq": 1,
                "source": "perception",
                "object_camera": p,
                "object_label": str(label),
            },
            flags=0,
        )
        acks = _wait_acks(sock, poller=poller, count=2, timeout_ms=timeout_ms)
        if not acks:
            raise HostPublishError(
                f"host sent no reply within {timeout_ms} ms "
                f"(is host.py running on {endpoint}? client={_HOST_CLIENT_VERSION})"
            )
        ack = acks[-1]
        if not bool(ack.get("ok", False)):
            raise HostPublishError(f"host rejected message: {ack.get('reason', ack)}")
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
