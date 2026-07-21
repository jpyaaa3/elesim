#!/usr/bin/env python3
"""ZMQ process boundary for the Elesim protocol router."""

from __future__ import annotations

import argparse
import logging
import threading
from collections.abc import Sequence

import zmq

from elesim_protocol import (
    Envelope,
    ProtocolError,
    dumps_envelope,
    loads_envelope,
    make_envelope,
)
from elesim_router.core import RoutedMessage, RouterCore


LOG = logging.getLogger("elesim.router")


def decode_router_frames(frames: Sequence[bytes]) -> tuple[bytes, Envelope]:
    if len(frames) != 2:
        raise ProtocolError(f"ROUTER message must contain exactly two frames, got {len(frames)}")
    identity, payload = frames
    if not isinstance(identity, bytes) or not identity:
        raise ProtocolError("ROUTER identity frame must be non-empty bytes")
    if not isinstance(payload, bytes):
        raise ProtocolError("ROUTER payload frame must be bytes")
    return identity, loads_envelope(payload)


class RoutingServer:
    def __init__(self, bind_endpoint: str, *, heartbeat_timeout_s: float = 3.5) -> None:
        self.bind_endpoint = str(bind_endpoint)
        self.core = RouterCore(heartbeat_timeout_s=heartbeat_timeout_s)
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.ROUTER)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(self.bind_endpoint)
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        self.stop_event = threading.Event()

    def run(self) -> None:
        LOG.info("routing endpoint bound %s", self.bind_endpoint)
        try:
            while not self.stop_event.is_set():
                events = dict(self.poller.poll(100))
                if self.socket in events:
                    self._receive_and_route()
                self._send(self.core.expire())
        except KeyboardInterrupt:
            pass
        finally:
            self.socket.close(0)

    def close(self) -> None:
        self.stop_event.set()

    def _receive_and_route(self) -> None:
        frames = self.socket.recv_multipart()
        identity = frames[0] if frames and isinstance(frames[0], bytes) else b""
        try:
            identity, request = decode_router_frames(frames)
            routed = self.core.handle(identity, request)
        except ProtocolError as exc:
            routed = self._wire_error(identity, str(exc)) if identity else []
        except Exception as exc:  # keep one malformed request from killing the router
            LOG.exception("unexpected router request failure")
            routed = self._wire_error(identity, f"internal router error: {type(exc).__name__}") if identity else []
        self._send(routed)

    @staticmethod
    def _wire_error(identity: bytes, reason: str) -> list[RoutedMessage]:
        return [
            RoutedMessage(
                identity,
                make_envelope(
                    "error",
                    "server",
                    target_id="unknown",
                    payload={"ok": False, "reason": str(reason)},
                    seq=1,
                ),
            )
        ]

    def _send(self, routed: list[RoutedMessage]) -> None:
        for item in routed:
            self.socket.send_multipart([item.identity, dumps_envelope(item.envelope)])


def _run() -> None:
    parser = argparse.ArgumentParser(description="Elesim distributed routing server")
    parser.add_argument("--bind", default="tcp://0.0.0.0:5558")
    parser.add_argument("--heartbeat-timeout", type=float, default=3.5)
    args = parser.parse_args()
    RoutingServer(args.bind, heartbeat_timeout_s=args.heartbeat_timeout).run()


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    _run()


if __name__ == "__main__":
    main()
