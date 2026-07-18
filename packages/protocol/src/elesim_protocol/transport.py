"""Reusable ZMQ transport for distributed Elesim processes."""

from __future__ import annotations

import os
import time
from typing import Iterator, Optional

import zmq

from .messages import EndpointDescriptor, Envelope, dumps_envelope, loads_envelope, make_envelope


class EndpointClient:
    def __init__(
        self,
        server_endpoint: str,
        descriptor: EndpointDescriptor,
        *,
        heartbeat_s: float = 1.0,
    ) -> None:
        self.server_endpoint = str(server_endpoint)
        self.descriptor = descriptor
        self.heartbeat_s = max(0.1, float(heartbeat_s))
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.DEALER)
        self.socket.setsockopt(zmq.LINGER, 0)
        identity = f"{descriptor.role}:{descriptor.endpoint_id}:{os.getpid()}".encode("utf-8")
        self.socket.setsockopt(zmq.IDENTITY, identity)
        self.socket.connect(self.server_endpoint)
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        self.seq = 0
        self._last_heartbeat = 0.0
        self.send("register", payload={"endpoint": descriptor.to_dict()})

    def send(
        self,
        message_type: str,
        *,
        target_id: str = "server",
        payload: Optional[dict[str, object]] = None,
        lease_id: str = "",
    ) -> Envelope:
        self.seq += 1
        envelope = make_envelope(
            message_type,
            self.descriptor.endpoint_id,
            target_id=target_id,
            payload=dict(payload or {}),
            seq=self.seq,
            lease_id=lease_id,
        )
        self.socket.send(dumps_envelope(envelope))
        return envelope

    def heartbeat(self, *, force: bool = False) -> None:
        now = time.monotonic()
        if force or now - self._last_heartbeat >= self.heartbeat_s:
            self._last_heartbeat = now
            self.send("heartbeat")

    def receive(self, timeout_ms: int = 0) -> Iterator[Envelope]:
        events = dict(self.poller.poll(max(0, int(timeout_ms))))
        if self.socket not in events:
            return
        while True:
            try:
                data = self.socket.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            yield loads_envelope(data)

    def close(self) -> None:
        try:
            self.poller.unregister(self.socket)
        except (KeyError, AttributeError):
            pass
        self.socket.close(0)
