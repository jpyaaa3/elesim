#!/usr/bin/env python3
"""Endpoint registry, lease authority and message router."""

from __future__ import annotations

import argparse
import logging
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Optional

import zmq

from elesim_protocol import (
    CAPABILITY_MOTION_ARM,
    CAPABILITY_MOTION_GO2,
    EndpointDescriptor,
    Envelope,
    ProtocolError,
    dumps_envelope,
    loads_envelope,
    make_envelope,
)


LOG = logging.getLogger("elesim.router")


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
        self._last_seq: dict[str, int] = {}

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
        previous_seq = self._last_seq.get(request.source_id, -1)
        if request.seq <= previous_seq:
            return self._error(identity, request, "stale sequence")
        self._last_seq[request.source_id] = request.seq

        if request.message_type == "heartbeat":
            return [self._reply(identity, request, "heartbeat_ack", {"ok": True})]
        if request.message_type == "discover":
            payload = request.payload or {}
            role_filter = str(payload.get("role", "")).strip()
            capability_filter = str(payload.get("capability", "")).strip()
            available = [
                endpoint.descriptor.to_dict()
                for endpoint in self.endpoints.values()
                if endpoint.descriptor.endpoint_id != request.source_id
                and (not role_filter or endpoint.descriptor.role == role_filter)
                and (
                    not capability_filter
                    or capability_filter in endpoint.descriptor.capabilities
                )
            ]
            return [self._reply(identity, request, "endpoint_list", {"endpoints": available})]
        if request.message_type == "select_target":
            if source.descriptor.role != "controller":
                return self._error(identity, request, "only controllers can select targets")
            target_id = str((request.payload or {}).get("target_id", ""))
            target = self.endpoints.get(target_id)
            motion_capabilities = {CAPABILITY_MOTION_ARM, CAPABILITY_MOTION_GO2}
            if (
                target is None
                or target.descriptor.role not in {"robot", "simulator"}
                or not motion_capabilities.intersection(target.descriptor.capabilities)
            ):
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
        if request.message_type == "motion_command" and (request.payload or {}).get("command") == "estop":
            if source.descriptor.role != "controller":
                return self._error(identity, request, "only controllers can issue estop")
            target = self.endpoints.get(request.target_id)
            if target is None or target.descriptor.role not in {"robot", "simulator"}:
                return self._error(identity, request, "estop target is unavailable")
            return [RoutedMessage(target.identity, request)]
        if request.message_type in {"motion_command", "camera_input"}:
            if source.descriptor.role != "controller":
                return self._error(identity, request, "only controllers can send target commands")
            target_id = self.active_target_by_controller.get(request.source_id, "")
            lease_id = self.lease_by_controller.get(request.source_id, "")
            if not target_id or request.target_id != target_id or request.lease_id != lease_id:
                return self._error(identity, request, "command does not match active lease")
            target = self.endpoints.get(target_id)
            if target is None:
                return self._error(identity, request, "target is unavailable")
            return [RoutedMessage(target.identity, request)]
        if request.message_type == "operator_intent":
            if source.descriptor.role != "ui":
                return self._error(identity, request, "only UI endpoints send operator intent")
            target = self.endpoints.get(request.target_id)
            if target is None or target.descriptor.role != "controller":
                return self._error(identity, request, "operator controller is unavailable")
            return [RoutedMessage(target.identity, request)]
        if request.message_type in {"operator_result", "ui_state"}:
            if source.descriptor.role != "controller":
                return self._error(identity, request, "only controllers send operator state")
            target = self.endpoints.get(request.target_id)
            if target is None or target.descriptor.role != "ui":
                return self._error(identity, request, "operator UI is unavailable")
            return [RoutedMessage(target.identity, request)]
        if request.message_type in {"telemetry", "ack", "webrtc_signal"}:
            target = self.endpoints.get(request.target_id)
            if target is None:
                return self._error(identity, request, "message target is unavailable")
            if request.message_type in {"telemetry", "ack"} and source.descriptor.role in {"robot", "simulator"}:
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
        self._last_seq.pop(endpoint_id, None)
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
        self.stop_event = threading.Event()

    def run(self) -> None:
        LOG.info("routing endpoint bound %s", self.bind_endpoint)
        try:
            while not self.stop_event.is_set():
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

    def close(self) -> None:
        self.stop_event.set()

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
