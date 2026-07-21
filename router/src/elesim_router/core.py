"""Pure endpoint registry, lease authority and routing decisions."""

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass
from typing import Mapping, Optional

from elesim_protocol import (
    CAPABILITY_MOTION_ARM,
    CAPABILITY_MOTION_GO2,
    DiscoverRequest,
    EndpointDescriptor,
    Envelope,
    MotionCommandRequest,
    OperatorIntentRequest,
    ProtocolError,
    RegisterRequest,
    SelectTargetRequest,
    TelemetryPayload,
    make_envelope,
    validate_routed_payload,
)


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
        self._server_seq: dict[str, int] = {}

    def _server_envelope(
        self,
        message_type: str,
        *,
        target_id: str,
        payload: Optional[dict[str, object]] = None,
        lease_id: str = "",
        trace_context: Optional[Mapping[str, str]] = None,
    ) -> Envelope:
        next_seq = self._server_seq.get(target_id, 0) + 1
        self._server_seq[target_id] = next_seq
        return make_envelope(
            message_type,
            "server",
            target_id=target_id,
            payload=payload or {},
            lease_id=lease_id,
            trace_context={str(key): str(value) for key, value in (trace_context or {}).items()},
            seq=next_seq,
        )

    def _reply(
        self,
        identity: bytes,
        request: Envelope,
        message_type: str,
        payload: dict[str, object],
    ) -> RoutedMessage:
        return RoutedMessage(
            identity,
            self._server_envelope(
                message_type,
                target_id=request.source_id,
                payload={"reply_to": request.message_id, **payload},
                trace_context=request.trace_context,
            ),
        )

    def _error(self, identity: bytes, request: Envelope, reason: str) -> list[RoutedMessage]:
        return [self._reply(identity, request, "error", {"ok": False, "reason": str(reason)})]

    def _registered_source(
        self, identity: bytes, request: Envelope
    ) -> Optional[RegisteredEndpoint]:
        endpoint_id = self.endpoint_by_identity.get(identity)
        if endpoint_id != request.source_id:
            return None
        return self.endpoints.get(request.source_id)

    def handle(
        self,
        identity: bytes,
        request: Envelope,
        *,
        now: Optional[float] = None,
    ) -> list[RoutedMessage]:
        current = time.monotonic() if now is None else float(now)
        if request.message_type == "register":
            return self._register(identity, request, now=current)

        source = self._registered_source(identity, request)
        if source is None:
            return self._error(identity, request, "endpoint is not registered")
        previous_seq = self._last_seq.get(request.source_id, -1)
        if request.seq <= previous_seq:
            return self._error(identity, request, "stale sequence")
        self._last_seq[request.source_id] = request.seq
        source.last_seen = current
        try:
            parsed = validate_routed_payload(request.message_type, request.payload or {})
        except ProtocolError as exc:
            return self._error(identity, request, str(exc))

        if request.message_type == "heartbeat":
            return [self._reply(identity, request, "heartbeat_ack", {"ok": True})]
        if request.message_type == "discover":
            assert isinstance(parsed, DiscoverRequest)
            return self._discover(identity, request, parsed)
        if request.message_type == "select_target":
            assert isinstance(parsed, SelectTargetRequest)
            return self._select_target(identity, request, source, parsed)
        if request.message_type == "release_target":
            routed = self._release_controller(
                request.source_id,
                trace_context=request.trace_context,
            )
            routed.append(self._reply(identity, request, "target_released", {"ok": True}))
            return routed
        if request.message_type == "motion_command":
            assert isinstance(parsed, MotionCommandRequest)
            return self._route_motion(identity, request, source, parsed)
        if request.message_type == "camera_input":
            return self._route_leased_controller_message(identity, request, source)
        if request.message_type == "operator_intent":
            assert isinstance(parsed, OperatorIntentRequest)
            return self._route_operator_intent(identity, request, source)
        if request.message_type in {"operator_result", "ui_state"}:
            return self._route_operator_response(identity, request, source)
        if request.message_type == "telemetry":
            assert isinstance(parsed, TelemetryPayload)
            return self._route_endpoint_response(identity, request, source)
        if request.message_type == "ack":
            return self._route_endpoint_response(identity, request, source)
        if request.message_type == "webrtc_signal":
            return self._route_webrtc(identity, request, source)
        return self._error(identity, request, f"unsupported message type: {request.message_type}")

    def _register(
        self,
        identity: bytes,
        request: Envelope,
        *,
        now: float,
    ) -> list[RoutedMessage]:
        try:
            registration = RegisterRequest.from_payload(request.payload or {})
        except ProtocolError as exc:
            return self._error(identity, request, str(exc))
        descriptor = registration.endpoint
        if descriptor.endpoint_id != request.source_id:
            return self._error(identity, request, "source_id does not match endpoint descriptor")

        identity_owner = self.endpoint_by_identity.get(identity)
        if identity_owner and identity_owner != descriptor.endpoint_id:
            return self._error(identity, request, "transport identity is already registered")

        previous = self.endpoints.get(descriptor.endpoint_id)
        previous_seq = self._last_seq.get(descriptor.endpoint_id, -1)
        if previous is not None:
            if previous.descriptor.instance_id != descriptor.instance_id:
                return self._error(
                    identity,
                    request,
                    "endpoint_id is already registered by another instance",
                )
            if request.seq <= previous_seq:
                return self._error(identity, request, "stale sequence")

        routed: list[RoutedMessage] = []
        if previous is not None and previous.identity != identity:
            routed.extend(
                self._drop(
                    descriptor.endpoint_id,
                    trace_context=request.trace_context,
                )
            )
        self.endpoints[descriptor.endpoint_id] = RegisteredEndpoint(identity, descriptor, now)
        self.endpoint_by_identity[identity] = descriptor.endpoint_id
        self._last_seq[descriptor.endpoint_id] = request.seq
        routed.append(
            self._reply(
                identity,
                request,
                "registered",
                {"ok": True, "endpoint": descriptor.to_dict()},
            )
        )
        return routed

    def _discover(
        self,
        identity: bytes,
        request: Envelope,
        query: DiscoverRequest,
    ) -> list[RoutedMessage]:
        available = [
            endpoint.descriptor.to_dict()
            for endpoint in sorted(
                self.endpoints.values(),
                key=lambda endpoint: endpoint.descriptor.endpoint_id,
            )
            if endpoint.descriptor.endpoint_id != request.source_id
            and (not query.role or endpoint.descriptor.role == query.role)
            and (
                not query.capability
                or query.capability in endpoint.descriptor.capabilities
            )
        ]
        return [self._reply(identity, request, "endpoint_list", {"endpoints": available})]

    def _select_target(
        self,
        identity: bytes,
        request: Envelope,
        source: RegisteredEndpoint,
        selection: SelectTargetRequest,
    ) -> list[RoutedMessage]:
        if source.descriptor.role != "controller":
            return self._error(identity, request, "only controllers can select targets")
        target = self.endpoints.get(selection.target_id)
        motion_capabilities = {CAPABILITY_MOTION_ARM, CAPABILITY_MOTION_GO2}
        if (
            target is None
            or target.descriptor.role not in {"robot", "simulator"}
            or not motion_capabilities.intersection(target.descriptor.capabilities)
        ):
            return self._error(identity, request, "target is unavailable")
        owner = self.controller_by_target.get(selection.target_id)
        if owner and owner != request.source_id:
            return self._error(identity, request, "target is already leased")

        routed = self._release_controller(
            request.source_id,
            trace_context=request.trace_context,
        )
        lease_id = uuid.uuid4().hex
        self.active_target_by_controller[request.source_id] = selection.target_id
        self.controller_by_target[selection.target_id] = request.source_id
        self.lease_by_controller[request.source_id] = lease_id
        routed.append(
            RoutedMessage(
                target.identity,
                self._server_envelope(
                    "lease_granted",
                    target_id=selection.target_id,
                    payload={"controller_id": request.source_id},
                    lease_id=lease_id,
                    trace_context=request.trace_context,
                ),
            )
        )
        routed.append(
            self._reply(
                identity,
                request,
                "target_selected",
                {
                    "ok": True,
                    "target_id": selection.target_id,
                    "lease_id": lease_id,
                },
            )
        )
        return routed

    def _route_motion(
        self,
        identity: bytes,
        request: Envelope,
        source: RegisteredEndpoint,
        command: MotionCommandRequest,
    ) -> list[RoutedMessage]:
        if command.command == "estop":
            if source.descriptor.role != "controller":
                return self._error(identity, request, "only controllers can issue estop")
            target = self.endpoints.get(request.target_id)
            if target is None or target.descriptor.role not in {"robot", "simulator"}:
                return self._error(identity, request, "estop target is unavailable")
            return [RoutedMessage(target.identity, request)]
        return self._route_leased_controller_message(identity, request, source)

    def _route_leased_controller_message(
        self,
        identity: bytes,
        request: Envelope,
        source: RegisteredEndpoint,
    ) -> list[RoutedMessage]:
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

    def _route_operator_intent(
        self,
        identity: bytes,
        request: Envelope,
        source: RegisteredEndpoint,
    ) -> list[RoutedMessage]:
        if source.descriptor.role != "ui":
            return self._error(identity, request, "only UI endpoints send operator intent")
        target = self.endpoints.get(request.target_id)
        if target is None or target.descriptor.role != "controller":
            return self._error(identity, request, "operator controller is unavailable")
        return [RoutedMessage(target.identity, request)]

    def _route_operator_response(
        self,
        identity: bytes,
        request: Envelope,
        source: RegisteredEndpoint,
    ) -> list[RoutedMessage]:
        if source.descriptor.role != "controller":
            return self._error(identity, request, "only controllers send operator state")
        target = self.endpoints.get(request.target_id)
        if target is None or target.descriptor.role != "ui":
            return self._error(identity, request, "operator UI is unavailable")
        return [RoutedMessage(target.identity, request)]

    def _route_endpoint_response(
        self,
        identity: bytes,
        request: Envelope,
        source: RegisteredEndpoint,
    ) -> list[RoutedMessage]:
        target = self.endpoints.get(request.target_id)
        if target is None:
            return self._error(identity, request, "message target is unavailable")
        if source.descriptor.role in {"robot", "simulator"}:
            owner = self.controller_by_target.get(source.descriptor.endpoint_id, "")
            if owner and request.target_id != owner:
                return self._error(
                    identity,
                    request,
                    "endpoint may only send to its lease owner",
                )
        return [RoutedMessage(target.identity, request)]

    def _route_webrtc(
        self,
        identity: bytes,
        request: Envelope,
        source: RegisteredEndpoint,
    ) -> list[RoutedMessage]:
        target = self.endpoints.get(request.target_id)
        if target is None:
            return self._error(identity, request, "WebRTC target is unavailable")
        role_pair = {source.descriptor.role, target.descriptor.role}
        if "ui" not in role_pair or not role_pair.intersection({"controller", "simulator"}):
            return self._error(identity, request, "WebRTC roles are not permitted")
        return [RoutedMessage(target.identity, request)]

    def expire(self, *, now: Optional[float] = None) -> list[RoutedMessage]:
        current = time.monotonic() if now is None else float(now)
        expired = sorted(
            endpoint_id
            for endpoint_id, endpoint in self.endpoints.items()
            if current - endpoint.last_seen > self.heartbeat_timeout_s
        )
        routed: list[RoutedMessage] = []
        for endpoint_id in expired:
            routed.extend(self._drop(endpoint_id))
        return routed

    def _release_controller(
        self,
        controller_id: str,
        *,
        trace_context: Optional[Mapping[str, str]] = None,
    ) -> list[RoutedMessage]:
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
                self._server_envelope(
                    "lease_revoked",
                    target_id=target_id,
                    payload={"controller_id": controller_id},
                    lease_id=lease_id,
                    trace_context=trace_context,
                ),
            )
        ]

    def _drop(
        self,
        endpoint_id: str,
        *,
        trace_context: Optional[Mapping[str, str]] = None,
    ) -> list[RoutedMessage]:
        endpoint = self.endpoints.pop(endpoint_id, None)
        if endpoint is not None:
            self.endpoint_by_identity.pop(endpoint.identity, None)
        self._last_seq.pop(endpoint_id, None)
        self._server_seq.pop(endpoint_id, None)
        if endpoint_id in self.active_target_by_controller:
            return self._release_controller(
                endpoint_id,
                trace_context=trace_context,
            )
        owner = self.controller_by_target.pop(endpoint_id, "")
        if owner:
            self.active_target_by_controller.pop(owner, None)
            self.lease_by_controller.pop(owner, None)
            controller = self.endpoints.get(owner)
            if controller is not None:
                return [
                    RoutedMessage(
                        controller.identity,
                        self._server_envelope(
                            "target_lost",
                            target_id=owner,
                            payload={"target_id": endpoint_id},
                            trace_context=trace_context,
                        ),
                    )
                ]
        return []


__all__ = ["RegisteredEndpoint", "RoutedMessage", "RouterCore"]
