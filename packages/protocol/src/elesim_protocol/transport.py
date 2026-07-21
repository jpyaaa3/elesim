"""Reusable ZMQ transport and endpoint registration lifecycle."""

from __future__ import annotations

import os
import time
import uuid
from collections import deque
from dataclasses import dataclass, replace
from typing import Callable, Iterator, Mapping, Optional

import zmq

from .messages import (
    EndpointDescriptor,
    Envelope,
    dumps_envelope,
    loads_envelope,
)
from .payloads import validate_routed_payload


class TransportError(RuntimeError):
    pass


class EndpointSession:
    """Pure registration and heartbeat state machine used by EndpointClient."""

    def __init__(
        self,
        descriptor: EndpointDescriptor,
        *,
        heartbeat_s: float,
        registration_retry_s: float,
        server_timeout_s: float,
    ) -> None:
        if not descriptor.instance_id:
            raise ValueError("EndpointSession descriptor requires instance_id")
        self.descriptor = descriptor
        self.heartbeat_s = max(0.1, float(heartbeat_s))
        self.registration_retry_s = max(0.1, float(registration_retry_s))
        self.server_timeout_s = max(self.heartbeat_s * 2.0, float(server_timeout_s))
        self.registered = False
        self.last_server_seen: Optional[float] = None
        self.last_registration_sent: Optional[float] = None
        self.last_heartbeat_sent: Optional[float] = None

    def next_action(self, *, now: float) -> Optional[str]:
        current = float(now)
        if (
            self.registered
            and self.last_server_seen is not None
            and current - self.last_server_seen > self.server_timeout_s
        ):
            self.registered = False
            self.last_registration_sent = None
        if not self.registered:
            if (
                self.last_registration_sent is None
                or current - self.last_registration_sent >= self.registration_retry_s
            ):
                return "register"
            return None
        if (
            self.last_heartbeat_sent is None
            or current - self.last_heartbeat_sent >= self.heartbeat_s
        ):
            return "heartbeat"
        return None

    def note_sent(self, action: str, *, now: float) -> None:
        if action == "register":
            self.last_registration_sent = float(now)
        elif action == "heartbeat":
            self.last_heartbeat_sent = float(now)

    def observe(self, envelope: Envelope, *, now: float) -> bool:
        current = float(now)
        self.last_server_seen = current
        if envelope.message_type == "registered":
            endpoint_raw = (envelope.payload or {}).get("endpoint")
            try:
                endpoint = EndpointDescriptor.from_dict(endpoint_raw or {})
            except Exception:
                return False
            if (
                endpoint.endpoint_id != self.descriptor.endpoint_id
                or endpoint.instance_id != self.descriptor.instance_id
            ):
                return False
            self.registered = True
            self.last_heartbeat_sent = current
            return True
        if envelope.message_type == "error":
            reason = str((envelope.payload or {}).get("reason", "")).lower()
            if "not registered" in reason:
                self.registered = False
                self.last_registration_sent = None
        return True

    def server_alive(self, *, now: float) -> bool:
        return (
            self.last_server_seen is not None
            and float(now) - self.last_server_seen <= self.server_timeout_s
        )


@dataclass(frozen=True)
class _PendingMessage:
    message_type: str
    target_id: str
    payload: dict[str, object]
    lease_id: str
    trace_context: dict[str, str]
    message_id: str
    timestamp: float


class EndpointClient:
    def __init__(
        self,
        server_endpoint: str,
        descriptor: EndpointDescriptor,
        *,
        heartbeat_s: float = 1.0,
        registration_retry_s: float = 1.0,
        server_timeout_s: float = 3.5,
        max_pending: int = 512,
        trace_context_provider: Optional[Callable[[], Mapping[str, str]]] = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.server_endpoint = str(server_endpoint)
        self.descriptor = (
            descriptor
            if descriptor.instance_id
            else replace(descriptor, instance_id=uuid.uuid4().hex)
        )
        self.clock = clock
        self.max_pending = max(1, int(max_pending))
        self.trace_context_provider = trace_context_provider
        self.session = EndpointSession(
            self.descriptor,
            heartbeat_s=heartbeat_s,
            registration_retry_s=registration_retry_s,
            server_timeout_s=server_timeout_s,
        )
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.DEALER)
        self.socket.setsockopt(zmq.LINGER, 0)
        identity = (
            f"{self.descriptor.role}:{self.descriptor.endpoint_id}:"
            f"{self.descriptor.instance_id}:{os.getpid()}"
        ).encode("utf-8")
        self.socket.setsockopt(zmq.IDENTITY, identity)
        self.socket.connect(self.server_endpoint)
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        self.seq = 0
        self._pending: deque[_PendingMessage] = deque()
        self.heartbeat(force=True)

    @property
    def registered(self) -> bool:
        return self.session.registered

    @property
    def last_server_seen(self) -> Optional[float]:
        return self.session.last_server_seen

    def server_alive(self) -> bool:
        return self.session.server_alive(now=self.clock())

    def send(
        self,
        message_type: str,
        *,
        target_id: str = "server",
        payload: Optional[dict[str, object]] = None,
        lease_id: str = "",
        trace_context: Optional[Mapping[str, str]] = None,
    ) -> Envelope:
        body = dict(payload or {})
        validate_routed_payload(message_type, body)
        trace = self._trace_context(trace_context)
        pending = _PendingMessage(
            message_type=str(message_type),
            target_id=str(target_id),
            payload=body,
            lease_id=str(lease_id),
            trace_context=trace,
            message_id=uuid.uuid4().hex,
            timestamp=time.time(),
        )
        if self.session.registered:
            return self._emit(pending)
        if len(self._pending) >= self.max_pending:
            raise TransportError(
                f"endpoint pending queue is full ({self.max_pending}); server is not registered"
            )
        self._pending.append(pending)
        return self._as_envelope(pending, seq=0)

    def heartbeat(self, *, force: bool = False) -> None:
        now = self.clock()
        action = self.session.next_action(now=now)
        if force and action is None:
            action = "heartbeat" if self.session.registered else "register"
        if action == "register":
            self._emit_control(
                "register",
                payload={"endpoint": self.descriptor.to_dict()},
            )
            self.session.note_sent("register", now=now)
        elif action == "heartbeat":
            self._emit_control("heartbeat", payload={})
            self.session.note_sent("heartbeat", now=now)

    def receive(self, timeout_ms: int = 0) -> Iterator[Envelope]:
        events = dict(self.poller.poll(max(0, int(timeout_ms))))
        if self.socket not in events:
            return
        while True:
            try:
                data = self.socket.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            envelope = loads_envelope(data)
            was_registered = self.session.registered
            accepted = self.session.observe(envelope, now=self.clock())
            if not accepted:
                continue
            if not was_registered and self.session.registered:
                self._flush_pending()
            yield envelope

    def close(self) -> None:
        try:
            self.poller.unregister(self.socket)
        except (KeyError, AttributeError):
            pass
        self.socket.close(0)

    def _trace_context(
        self, explicit: Optional[Mapping[str, str]]
    ) -> dict[str, str]:
        if explicit is not None:
            return {str(key): str(value) for key, value in explicit.items()}
        if self.trace_context_provider is None:
            return {}
        return {
            str(key): str(value)
            for key, value in self.trace_context_provider().items()
        }

    def _emit_control(self, message_type: str, *, payload: dict[str, object]) -> Envelope:
        pending = _PendingMessage(
            message_type=message_type,
            target_id="server",
            payload=payload,
            lease_id="",
            trace_context=self._trace_context(None),
            message_id=uuid.uuid4().hex,
            timestamp=time.time(),
        )
        return self._emit(pending)

    def _emit(self, pending: _PendingMessage) -> Envelope:
        self.seq += 1
        envelope = self._as_envelope(pending, seq=self.seq)
        self.socket.send(dumps_envelope(envelope))
        return envelope

    def _as_envelope(self, pending: _PendingMessage, *, seq: int) -> Envelope:
        return Envelope(
            message_type=pending.message_type,
            source_id=self.descriptor.endpoint_id,
            target_id=pending.target_id,
            payload=dict(pending.payload),
            seq=int(seq),
            timestamp=pending.timestamp,
            message_id=pending.message_id,
            lease_id=pending.lease_id,
            trace_context=dict(pending.trace_context),
        )

    def _flush_pending(self) -> None:
        while self._pending and self.session.registered:
            self._emit(self._pending.popleft())


__all__ = ["EndpointClient", "EndpointSession", "TransportError"]
