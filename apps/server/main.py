#!/usr/bin/env python3
"""Endpoint registry, lease authority and message router."""

from __future__ import annotations

import argparse
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import zmq

from engine.config import load_runtime_role_config
from engine.core.protocol import (
    EndpointDescriptor,
    Envelope,
    ProtocolError,
    dumps_envelope,
    loads_envelope,
    make_envelope,
)
from engine.observability.tracing import configure_tracing, shutdown_tracing, span


_ROOT = Path(__file__).resolve().parents[2]


@dataclass
class RegisteredEndpoint:
    identity: bytes
    descriptor: EndpointDescriptor
    last_seen: float


@dataclass(frozen=True)
class RoutedMessage:
    identity: bytes
    envelope: Envelope


class RouterCore:
    def __init__(self, *, heartbeat_timeout_s: float = 3.5) -> None:
        self.heartbeat_timeout_s = max(0.5, float(heartbeat_timeout_s))
        self.endpoints: dict[str, RegisteredEndpoint] = {}
        self.endpoint_by_identity: dict[bytes, str] = {}
        self.active_target_by_controller: dict[str, str] = {}
        self.controller_by_target: dict[str, str] = {}
        self.lease_by_controller: dict[str, str] = {}
        self._last_seq: dict[tuple[str, str], int] = {}

    def _reply(
        self,
        identity: bytes,
        request: Envelope,
        message_type: str,
        payload: dict[str, object],
    ) -> RoutedMessage:
        return RoutedMessage(
            identity,
            make_envelope(
                message_type,
                "server",
                target_id=request.source_id,
                payload={"reply_to": request.message_id, **payload},
            ),
        )

    def _error(self, identity: bytes, request: Envelope, reason: str) -> list[RoutedMessage]:
        return [self._reply(identity, request, "error", {"ok": False, "reason": str(reason)})]

    def _registered_source(self, identity: bytes, request: Envelope) -> Optional[RegisteredEndpoint]:
        endpoint_id = self.endpoint_by_identity.get(identity)
        if endpoint_id != request.source_id:
            return None
        return self.endpoints.get(request.source_id)

    def handle(self, identity: bytes, request: Envelope, *, now: Optional[float] = None) -> list[RoutedMessage]:
        current = time.monotonic() if now is None else float(now)
        if request.message_type == "register":
            raw = (request.payload or {}).get("endpoint")
            if not isinstance(raw, dict):
                return self._error(identity, request, "register requires endpoint descriptor")
            try:
                descriptor = EndpointDescriptor.from_dict(raw)
            except ProtocolError as exc:
                return self._error(identity, request, str(exc))
            if descriptor.endpoint_id != request.source_id:
                return self._error(identity, request, "source_id does not match endpoint descriptor")
            previous_id = self.endpoint_by_identity.get(identity)
            if previous_id and previous_id != descriptor.endpoint_id:
                self._drop(previous_id)
            previous = self.endpoints.get(descriptor.endpoint_id)
            if previous is not None and previous.identity != identity:
                self.endpoint_by_identity.pop(previous.identity, None)
            self.endpoints[descriptor.endpoint_id] = RegisteredEndpoint(identity, descriptor, current)
            self.endpoint_by_identity[identity] = descriptor.endpoint_id
            return [self._reply(identity, request, "registered", {"ok": True, "endpoint": descriptor.to_dict()})]

        source = self._registered_source(identity, request)
        if source is None:
            return self._error(identity, request, "endpoint is not registered")
        source.last_seen = current
        seq_key = (request.source_id, request.message_type)
        previous_seq = self._last_seq.get(seq_key, -1)
        if request.seq <= previous_seq:
            return self._error(identity, request, "stale sequence")
        self._last_seq[seq_key] = request.seq

        if request.message_type == "heartbeat":
            return [self._reply(identity, request, "heartbeat_ack", {"ok": True})]
        if request.message_type == "list_endpoints":
            available = [
                endpoint.descriptor.to_dict()
                for endpoint in self.endpoints.values()
                if endpoint.descriptor.role in {"robot", "sim"}
            ]
            return [self._reply(identity, request, "endpoint_list", {"endpoints": available})]
        if request.message_type == "select_target":
            if source.descriptor.role != "controller":
                return self._error(identity, request, "only controllers can select targets")
            target_id = str((request.payload or {}).get("target_id", ""))
            target = self.endpoints.get(target_id)
            if target is None or target.descriptor.role not in {"robot", "sim"}:
                return self._error(identity, request, "target is unavailable")
            owner = self.controller_by_target.get(target_id)
            if owner and owner != request.source_id:
                return self._error(identity, request, "target is already leased")
            routed = self._release_controller(request.source_id)
            lease_id = uuid.uuid4().hex
            self.active_target_by_controller[request.source_id] = target_id
            self.controller_by_target[target_id] = request.source_id
            self.lease_by_controller[request.source_id] = lease_id
            routed.append(
                RoutedMessage(
                    target.identity,
                    make_envelope(
                        "lease_granted",
                        "server",
                        target_id=target_id,
                        payload={"controller_id": request.source_id},
                        lease_id=lease_id,
                    ),
                )
            )
            routed.append(
                self._reply(
                    identity,
                    request,
                    "target_selected",
                    {"ok": True, "target_id": target_id, "lease_id": lease_id},
                )
            )
            return routed
        if request.message_type == "release_target":
            routed = self._release_controller(request.source_id)
            routed.append(self._reply(identity, request, "target_released", {"ok": True}))
            return routed
        if request.message_type == "command" and (request.payload or {}).get("command") == "estop":
            if source.descriptor.role != "controller":
                return self._error(identity, request, "only controllers can issue estop")
            target = self.endpoints.get(request.target_id)
            if target is None or target.descriptor.role not in {"robot", "sim"}:
                return self._error(identity, request, "estop target is unavailable")
            return [RoutedMessage(target.identity, request)]
        if request.message_type in {"command", "camera_input"}:
            target_id = self.active_target_by_controller.get(request.source_id, "")
            lease_id = self.lease_by_controller.get(request.source_id, "")
            if not target_id or request.target_id != target_id or request.lease_id != lease_id:
                return self._error(identity, request, "command does not match active lease")
            target = self.endpoints.get(target_id)
            if target is None:
                return self._error(identity, request, "target is unavailable")
            return [RoutedMessage(target.identity, request)]
        if request.message_type in {"state", "ack", "webrtc_signal"}:
            target = self.endpoints.get(request.target_id)
            if target is None:
                return self._error(identity, request, "message target is unavailable")
            if request.message_type in {"state", "ack"} and source.descriptor.role in {"robot", "sim"}:
                owner = self.controller_by_target.get(source.descriptor.endpoint_id, "")
                if owner and request.target_id != owner:
                    return self._error(identity, request, "endpoint may only send to its lease owner")
            return [RoutedMessage(target.identity, request)]
        return self._error(identity, request, f"unsupported message type: {request.message_type}")

    def expire(self, *, now: Optional[float] = None) -> list[RoutedMessage]:
        current = time.monotonic() if now is None else float(now)
        expired = [
            endpoint_id
            for endpoint_id, endpoint in self.endpoints.items()
            if current - endpoint.last_seen > self.heartbeat_timeout_s
        ]
        routed: list[RoutedMessage] = []
        for endpoint_id in expired:
            routed.extend(self._drop(endpoint_id))
        return routed

    def _release_controller(self, controller_id: str) -> list[RoutedMessage]:
        target_id = self.active_target_by_controller.pop(controller_id, "")
        lease_id = self.lease_by_controller.pop(controller_id, "")
        if not target_id:
            return []
        self.controller_by_target.pop(target_id, None)
        target = self.endpoints.get(target_id)
        if target is None:
            return []
        return [
            RoutedMessage(
                target.identity,
                make_envelope(
                    "lease_revoked",
                    "server",
                    target_id=target_id,
                    payload={"controller_id": controller_id},
                    lease_id=lease_id,
                ),
            )
        ]

    def _drop(self, endpoint_id: str) -> list[RoutedMessage]:
        endpoint = self.endpoints.pop(endpoint_id, None)
        if endpoint is not None:
            self.endpoint_by_identity.pop(endpoint.identity, None)
        if endpoint_id in self.active_target_by_controller:
            return self._release_controller(endpoint_id)
        owner = self.controller_by_target.pop(endpoint_id, "")
        if owner:
            self.active_target_by_controller.pop(owner, None)
            self.lease_by_controller.pop(owner, None)
            controller = self.endpoints.get(owner)
            if controller is not None:
                return [
                    RoutedMessage(
                        controller.identity,
                        make_envelope(
                            "target_lost",
                            "server",
                            target_id=owner,
                            payload={"target_id": endpoint_id},
                        ),
                    )
                ]
        return []


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

    def run(self) -> None:
        print(f"[server] routing endpoint bound {self.bind_endpoint}")
        try:
            while True:
                events = dict(self.poller.poll(100))
                if self.socket in events:
                    identity, data = self.socket.recv_multipart()
                    try:
                        request = loads_envelope(data)
                        routed = self.core.handle(identity, request)
                    except ProtocolError as exc:
                        error = make_envelope(
                            "error",
                            "server",
                            target_id="unknown",
                            payload={"ok": False, "reason": str(exc)},
                        )
                        routed = [RoutedMessage(identity, error)]
                    self._send(routed)
                self._send(self.core.expire())
        except KeyboardInterrupt:
            pass
        finally:
            self.socket.close(0)

    def _send(self, routed: list[RoutedMessage]) -> None:
        for item in routed:
            self.socket.send_multipart([item.identity, dumps_envelope(item.envelope)])


def _run() -> None:
    parser = argparse.ArgumentParser(description="Elesim distributed routing server")
    parser.add_argument("--runtime-config", default=str(_ROOT / "configs/runtime/server.yaml"))
    parser.add_argument("--bind", default="")
    args = parser.parse_args()
    runtime = load_runtime_role_config(args.runtime_config)
    if runtime.role != "server":
        raise ValueError(f"runtime role must be server, got {runtime.role!r}")
    endpoint = str(args.bind).strip() or runtime.bind_endpoint or runtime.server_endpoint
    RoutingServer(endpoint).run()


def main() -> None:
    configure_tracing("elesim-server")
    try:
        with span("server.process.run"):
            _run()
    finally:
        shutdown_tracing()


if __name__ == "__main__":
    main()
