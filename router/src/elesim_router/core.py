"""Pure endpoint registry, lease authority and routing decisions."""

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, replace
from typing import Callable, Mapping, Optional

from elesim_protocol import (
    CAPABILITY_MOTION_ARM,
    CAPABILITY_MOTION_GO2,
    CloseSimulationSessionRequest,
    DiscoverRequest,
    EndpointDescriptor,
    Envelope,
    MotionCommandRequest,
    OpenSimulationSessionRequest,
    OperatorIntentRequest,
    ProtocolError,
    RegisterRequest,
    SelectTargetRequest,
    SimulationCommandRequest,
    SimulationResultPayload,
    SimulationSessionGrantedPayload,
    SimulationSessionOpenedPayload,
    SimulationSessionRevokedPayload,
    SimulationStatusPayload,
    TelemetryPayload,
    WebRtcSignalPayload,
    make_envelope,
    validate_routed_payload,
)

from .simulation_sessions import (
    SimulationSession,
    SimulationSessionError,
    SimulationSessionRegistry,
    TurnCredentialIssuer,
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
    def __init__(
        self,
        *,
        heartbeat_timeout_s: float = 3.5,
        turn_issuer: Optional[TurnCredentialIssuer] = None,
        endpoint_authorizer: Optional[Callable[[str, str, str], bool]] = None,
    ) -> None:
        self.heartbeat_timeout_s = max(0.5, float(heartbeat_timeout_s))
        self.endpoints: dict[str, RegisteredEndpoint] = {}
        self.endpoint_by_identity: dict[bytes, str] = {}
        self.active_target_by_controller: dict[str, str] = {}
        self.controller_by_target: dict[str, str] = {}
        self.lease_by_controller: dict[str, str] = {}
        self.simulation_sessions = SimulationSessionRegistry()
        self.turn_issuer = turn_issuer
        self.endpoint_authorizer = endpoint_authorizer
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
        wall_time: Optional[float] = None,
        authenticated_user_id: str = "",
    ) -> list[RoutedMessage]:
        current = time.monotonic() if now is None else float(now)
        current_wall = time.time() if wall_time is None else float(wall_time)
        if request.message_type == "register":
            return self._register(
                identity,
                request,
                now=current,
                authenticated_user_id=authenticated_user_id,
            )

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
        if request.message_type == "open_simulation_session":
            assert isinstance(parsed, OpenSimulationSessionRequest)
            return self._open_simulation_session(
                identity,
                request,
                source,
                parsed,
                wall_time=current_wall,
            )
        if request.message_type == "close_simulation_session":
            assert isinstance(parsed, CloseSimulationSessionRequest)
            return self._close_simulation_session(identity, request, source, parsed)
        if request.message_type == "simulation_command":
            assert isinstance(parsed, SimulationCommandRequest)
            return self._route_simulation_command(identity, request, source, parsed)
        if request.message_type == "simulation_result":
            assert isinstance(parsed, SimulationResultPayload)
            return self._route_simulation_result(identity, request, source, parsed)
        if request.message_type == "simulation_status":
            assert isinstance(parsed, SimulationStatusPayload)
            return self._route_simulation_status(identity, request, source)
        if request.message_type == "webrtc_signal":
            assert isinstance(parsed, WebRtcSignalPayload)
            return self._route_webrtc(identity, request, source, parsed)
        return self._error(identity, request, f"unsupported message type: {request.message_type}")

    def _register(
        self,
        identity: bytes,
        request: Envelope,
        *,
        now: float,
        authenticated_user_id: str,
    ) -> list[RoutedMessage]:
        try:
            registration = RegisterRequest.from_payload(request.payload or {})
        except ProtocolError as exc:
            return self._error(identity, request, str(exc))
        descriptor = registration.endpoint
        if descriptor.endpoint_id != request.source_id:
            return self._error(identity, request, "source_id does not match endpoint descriptor")
        if self.endpoint_authorizer is not None and not self.endpoint_authorizer(
            str(authenticated_user_id),
            descriptor.endpoint_id,
            descriptor.role,
        ):
            return self._error(identity, request, "authenticated identity is not authorized for endpoint")

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

    def _open_simulation_session(
        self,
        identity: bytes,
        request: Envelope,
        source: RegisteredEndpoint,
        opened: OpenSimulationSessionRequest,
        *,
        wall_time: float,
    ) -> list[RoutedMessage]:
        if source.descriptor.role != "ui":
            return self._error(identity, request, "only UI endpoints open simulation sessions")
        simulator = self.endpoints.get(opened.simulator_id)
        if simulator is None or simulator.descriptor.role != "simulator":
            return self._error(identity, request, "simulator is unavailable")
        advertised = simulator.descriptor.streams or {}
        missing = sorted(set(opened.streams) - set(advertised))
        if missing:
            return self._error(
                identity,
                request,
                "simulator does not advertise streams: " + ", ".join(missing),
            )
        try:
            session = self.simulation_sessions.open(
                request_id=opened.request_id,
                ui_id=request.source_id,
                simulator_id=opened.simulator_id,
                streams=opened.streams,
            )
        except SimulationSessionError as exc:
            return self._error(identity, request, str(exc))
        session.request_id = opened.request_id
        ui_turn = simulator_turn = None
        if self.turn_issuer is not None:
            ui_turn = self.turn_issuer.issue(session.ui_id, session.session_id, now=wall_time)
            simulator_turn = self.turn_issuer.issue(
                session.simulator_id,
                session.session_id,
                now=wall_time,
            )
            session.turn_expires_at = min(ui_turn.expires_at, simulator_turn.expires_at)
        granted = SimulationSessionGrantedPayload(
            request_id=opened.request_id,
            session_id=session.session_id,
            simulator_id=session.simulator_id,
            ui_id=session.ui_id,
            streams=session.streams,
            turn=simulator_turn,
        )
        response = SimulationSessionOpenedPayload(
            request_id=opened.request_id,
            session_id=session.session_id,
            simulator_id=session.simulator_id,
            streams=session.streams,
            turn=ui_turn,
        )
        return [
            RoutedMessage(
                simulator.identity,
                self._server_envelope(
                    "simulation_session_granted",
                    target_id=session.simulator_id,
                    payload=granted.to_payload(),
                    lease_id=session.session_id,
                    trace_context=request.trace_context,
                ),
            ),
            RoutedMessage(
                identity,
                self._server_envelope(
                    "simulation_session_opened",
                    target_id=session.ui_id,
                    payload=response.to_payload(),
                    lease_id=session.session_id,
                    trace_context=request.trace_context,
                ),
            ),
        ]

    def _close_simulation_session(
        self,
        identity: bytes,
        request: Envelope,
        source: RegisteredEndpoint,
        closed: CloseSimulationSessionRequest,
    ) -> list[RoutedMessage]:
        if source.descriptor.role != "ui":
            return self._error(identity, request, "only UI endpoints close simulation sessions")
        session = self.simulation_sessions.by_id.get(closed.session_id)
        if session is None or session.ui_id != request.source_id:
            return self._error(identity, request, "simulation session is not owned by this UI")
        return self._revoke_simulation_session(
            session,
            reason="closed",
            trace_context=request.trace_context,
        )

    @staticmethod
    def _session_matches(
        session: Optional[SimulationSession],
        request: Envelope,
        session_id: str,
    ) -> bool:
        return (
            session is not None
            and session.session_id == session_id
            and request.lease_id == session.session_id
        )

    def _route_simulation_command(
        self,
        identity: bytes,
        request: Envelope,
        source: RegisteredEndpoint,
        command: SimulationCommandRequest,
    ) -> list[RoutedMessage]:
        session = self.simulation_sessions.by_ui.get(request.source_id)
        if (
            source.descriptor.role != "ui"
            or not self._session_matches(session, request, command.session_id)
            or session is None
            or request.target_id != session.simulator_id
        ):
            return self._error(identity, request, "command does not match active simulation session")
        simulator = self.endpoints.get(session.simulator_id)
        if simulator is None:
            return self._error(identity, request, "simulator is unavailable")
        return [RoutedMessage(simulator.identity, request)]

    def _route_simulation_result(
        self,
        identity: bytes,
        request: Envelope,
        source: RegisteredEndpoint,
        result: SimulationResultPayload,
    ) -> list[RoutedMessage]:
        session = self.simulation_sessions.by_simulator.get(request.source_id)
        if (
            source.descriptor.role != "simulator"
            or not self._session_matches(session, request, result.session_id)
            or session is None
            or request.target_id != session.ui_id
        ):
            return self._error(identity, request, "result does not match active simulation session")
        ui = self.endpoints.get(session.ui_id)
        if ui is None:
            return self._error(identity, request, "simulation UI is unavailable")
        return [RoutedMessage(ui.identity, request)]

    def _route_simulation_status(
        self,
        identity: bytes,
        request: Envelope,
        source: RegisteredEndpoint,
    ) -> list[RoutedMessage]:
        if source.descriptor.role != "simulator":
            return self._error(identity, request, "only simulators publish simulation status")
        recipients: set[str] = set()
        controller_id = self.controller_by_target.get(request.source_id, "")
        if controller_id:
            recipients.add(controller_id)
        session = self.simulation_sessions.by_simulator.get(request.source_id)
        if session is not None:
            recipients.add(session.ui_id)
        routed: list[RoutedMessage] = []
        for endpoint_id in sorted(recipients):
            endpoint = self.endpoints.get(endpoint_id)
            if endpoint is not None:
                routed.append(
                    RoutedMessage(
                        endpoint.identity,
                        replace(request, target_id=endpoint_id),
                    )
                )
        return routed

    def _route_webrtc(
        self,
        identity: bytes,
        request: Envelope,
        source: RegisteredEndpoint,
        signal: WebRtcSignalPayload,
    ) -> list[RoutedMessage]:
        if source.descriptor.role == "ui":
            session = self.simulation_sessions.by_ui.get(request.source_id)
            expected_target = "" if session is None else session.simulator_id
        elif source.descriptor.role == "simulator":
            session = self.simulation_sessions.by_simulator.get(request.source_id)
            expected_target = "" if session is None else session.ui_id
        else:
            session = None
            expected_target = ""
        if (
            not self._session_matches(session, request, signal.session_id)
            or session is None
            or request.target_id != expected_target
            or signal.stream not in session.streams
        ):
            return self._error(identity, request, "WebRTC signal does not match active simulation session")
        target = self.endpoints.get(expected_target)
        if target is None:
            return self._error(identity, request, "WebRTC target is unavailable")
        return [RoutedMessage(target.identity, request)]

    def expire(
        self,
        *,
        now: Optional[float] = None,
        wall_time: Optional[float] = None,
    ) -> list[RoutedMessage]:
        current = time.monotonic() if now is None else float(now)
        current_wall = time.time() if wall_time is None else float(wall_time)
        expired = sorted(
            endpoint_id
            for endpoint_id, endpoint in self.endpoints.items()
            if current - endpoint.last_seen > self.heartbeat_timeout_s
        )
        routed: list[RoutedMessage] = []
        for endpoint_id in expired:
            routed.extend(self._drop(endpoint_id))
        routed.extend(self._refresh_turn_credentials(wall_time=current_wall))
        return routed

    def _refresh_turn_credentials(self, *, wall_time: float) -> list[RoutedMessage]:
        if self.turn_issuer is None:
            return []
        routed: list[RoutedMessage] = []
        for session in tuple(self.simulation_sessions.by_id.values()):
            if not self.turn_issuer.refresh_due(session.turn_expires_at, now=wall_time):
                continue
            ui = self.endpoints.get(session.ui_id)
            simulator = self.endpoints.get(session.simulator_id)
            if ui is None or simulator is None:
                continue
            ui_turn = self.turn_issuer.issue(session.ui_id, session.session_id, now=wall_time)
            simulator_turn = self.turn_issuer.issue(
                session.simulator_id,
                session.session_id,
                now=wall_time,
            )
            session.turn_expires_at = min(ui_turn.expires_at, simulator_turn.expires_at)
            opened = SimulationSessionOpenedPayload(
                request_id=session.request_id,
                session_id=session.session_id,
                simulator_id=session.simulator_id,
                streams=session.streams,
                turn=ui_turn,
            )
            granted = SimulationSessionGrantedPayload(
                request_id=session.request_id,
                session_id=session.session_id,
                simulator_id=session.simulator_id,
                ui_id=session.ui_id,
                streams=session.streams,
                turn=simulator_turn,
            )
            routed.extend(
                (
                    RoutedMessage(
                        simulator.identity,
                        self._server_envelope(
                            "simulation_session_granted",
                            target_id=session.simulator_id,
                            payload=granted.to_payload(),
                            lease_id=session.session_id,
                        ),
                    ),
                    RoutedMessage(
                        ui.identity,
                        self._server_envelope(
                            "simulation_session_opened",
                            target_id=session.ui_id,
                            payload=opened.to_payload(),
                            lease_id=session.session_id,
                        ),
                    ),
                )
            )
        return routed

    def _revoke_simulation_session(
        self,
        session: SimulationSession,
        *,
        reason: str,
        trace_context: Optional[Mapping[str, str]] = None,
        excluded_endpoint: str = "",
    ) -> list[RoutedMessage]:
        self.simulation_sessions.close(session.session_id)
        payload = SimulationSessionRevokedPayload(
            session_id=session.session_id,
            simulator_id=session.simulator_id,
            reason=reason,
        ).to_payload()
        routed: list[RoutedMessage] = []
        for endpoint_id in (session.simulator_id, session.ui_id):
            if endpoint_id == excluded_endpoint:
                continue
            endpoint = self.endpoints.get(endpoint_id)
            if endpoint is None:
                continue
            routed.append(
                RoutedMessage(
                    endpoint.identity,
                    self._server_envelope(
                        "simulation_session_revoked",
                        target_id=endpoint_id,
                        payload=payload,
                        lease_id=session.session_id,
                        trace_context=trace_context,
                    ),
                )
            )
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
        routed: list[RoutedMessage] = []
        session = (
            self.simulation_sessions.by_ui.get(endpoint_id)
            or self.simulation_sessions.by_simulator.get(endpoint_id)
        )
        if session is not None:
            routed.extend(
                self._revoke_simulation_session(
                    session,
                    reason="endpoint disconnected",
                    trace_context=trace_context,
                    excluded_endpoint=endpoint_id,
                )
            )
        endpoint = self.endpoints.pop(endpoint_id, None)
        if endpoint is not None:
            self.endpoint_by_identity.pop(endpoint.identity, None)
        self._last_seq.pop(endpoint_id, None)
        self._server_seq.pop(endpoint_id, None)
        if endpoint_id in self.active_target_by_controller:
            routed.extend(
                self._release_controller(
                    endpoint_id,
                    trace_context=trace_context,
                )
            )
        owner = self.controller_by_target.pop(endpoint_id, "")
        if owner:
            self.active_target_by_controller.pop(owner, None)
            self.lease_by_controller.pop(owner, None)
            controller = self.endpoints.get(owner)
            if controller is not None:
                routed.append(
                    RoutedMessage(
                        controller.identity,
                        self._server_envelope(
                            "target_lost",
                            target_id=owner,
                            payload={"target_id": endpoint_id},
                            trace_context=trace_context,
                        ),
                    )
                )
        return routed


__all__ = ["RegisteredEndpoint", "RoutedMessage", "RouterCore"]
