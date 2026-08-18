"""Protocol-v6 direct DDS endpoint owned by the sim deployment."""

from __future__ import annotations

import concurrent.futures
import threading
import time
from typing import Any, Callable, Mapping, Optional

from elesim_protocol import (
    CAPABILITY_MOTION_ARM,
    CAPABILITY_MOTION_GO2,
    CAPABILITY_STREAM_HAND_EYE_PREVIEW,
    CAPABILITY_STREAM_OBSERVER,
    CAPABILITY_STREAM_RGBD,
    DdsRuntimeSettings,
    DdsTransportError,
    EndpointDescriptor,
    Envelope,
    MediaStreamDescriptor,
    PeerClient,
    PeerIdentity,
    ProtocolError,
    SimulationCommandRequest,
    SimulationResultPayload,
    SimulationSessionGrantedPayload,
    SimulationSessionRevokedPayload,
    SimulationStatusPayload,
    TurnCredentials,
    WebRtcSignalPayload,
)

from .control_state import SimulationStateSource
from .simulation.operator_control import (
    SimulationOperatorCommand,
    SimulationOperatorMailbox,
)


_MAX_WEBRTC_INFLIGHT = 8
_MAX_SIMULATION_RESULTS_PER_CYCLE = 32


class SimEndpoint:
    """Own the Sim DDS peer while Genesis communicates through memory."""

    def __init__(
        self,
        *,
        endpoint_id: str,
        state: SimulationStateSource,
        streams: Mapping[str, MediaStreamDescriptor],
        settings: Optional[DdsRuntimeSettings] = None,
        operator_mailbox: Optional[SimulationOperatorMailbox] = None,
        webrtc_offer_handler: Optional[
            Callable[[str, str, str, Optional[TurnCredentials], str], Mapping[str, Any]]
        ] = None,
        webrtc_session_close_handler: Optional[Callable[[str], None]] = None,
        simulation_session_ready_provider: Optional[Callable[[], tuple[bool, str]]] = None,
        turn_credential_provider: Optional[Callable[[str, str, float], Any]] = None,
        endpoint_factory: Callable[..., Any] = PeerClient,
    ) -> None:
        self.endpoint_id = str(endpoint_id)
        self.state = state
        self.streams = dict(streams)
        self.settings = settings
        self.operator_mailbox = operator_mailbox or SimulationOperatorMailbox()
        self.webrtc_offer_handler = webrtc_offer_handler
        self.webrtc_session_close_handler = webrtc_session_close_handler
        self.simulation_session_ready_provider = simulation_session_ready_provider
        self.turn_credential_provider = turn_credential_provider
        self.endpoint_factory = endpoint_factory

        self.pilot_id = ""
        self.active_lease = ""
        self.last_control_seq = -1
        self.simulation_ui_id = ""
        self.simulation_session_id = ""
        self.simulation_streams: tuple[str, ...] = ()
        self.turn_credentials: Optional[TurnCredentials] = None
        self.last_simulation_seq = -1

        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.peer_identity: Optional[PeerIdentity] = None
        self._telemetry_lock = threading.Lock()
        self._telemetry: dict[str, Any] = {}
        self._telemetry_dirty = False
        self._telemetry_revision = 0
        self._status_lock = threading.Lock()
        self._status: Optional[SimulationStatusPayload] = None
        self._status_dirty = False
        self._status_revision = 0
        self._diagnostic_seen: dict[str, float] = {}
        self._webrtc_executor: Optional[concurrent.futures.ThreadPoolExecutor] = (
            concurrent.futures.ThreadPoolExecutor(
                max_workers=2,
                thread_name_prefix="sim-webrtc-signaling",
            )
            if self.webrtc_offer_handler is not None
            else None
        )
        self._webrtc_futures: list[
            tuple[
                concurrent.futures.Future[Any],
                str,
                str,
                str,
                Optional[Mapping[str, str]],
            ]
        ] = []

    def start(self) -> None:
        self.stop_event.clear()
        self.ready.clear()
        self.thread = threading.Thread(target=self._run, name="sim-dds", daemon=True)
        self.thread.start()
        if not self.ready.wait(timeout=3.0):
            raise RuntimeError("sim protocol endpoint failed to start")

    def close(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=3.0)
        if self._webrtc_executor is not None:
            self._webrtc_executor.shutdown(wait=False, cancel_futures=True)

    def grant_lease(self, pilot_id: str, lease_id: str) -> None:
        self.state.revoke_control()
        self.pilot_id = str(pilot_id)
        self.active_lease = str(lease_id)
        self.last_control_seq = -1

    def revoke_lease(self) -> None:
        self.state.revoke_control()
        self.pilot_id = ""
        self.active_lease = ""
        self.last_control_seq = -1

    def grant_simulation_session(self, granted: SimulationSessionGrantedPayload) -> None:
        changed = granted.session_id != self.simulation_session_id
        self.simulation_ui_id = granted.ui_id
        self.simulation_session_id = granted.session_id
        self.simulation_streams = tuple(granted.streams)
        self.turn_credentials = granted.turn
        if changed:
            self.last_simulation_seq = -1

    def revoke_simulation_session(self) -> None:
        session_id = self.simulation_session_id
        self.simulation_ui_id = ""
        self.simulation_session_id = ""
        self.simulation_streams = ()
        self.turn_credentials = None
        self.last_simulation_seq = -1
        if session_id and self.webrtc_session_close_handler is not None:
            self._close_webrtc_session_async(session_id)

    def publish_telemetry(self, payload: Mapping[str, Any]) -> None:
        with self._telemetry_lock:
            self._telemetry.update(dict(payload))
            self._telemetry_dirty = True
            self._telemetry_revision += 1

    def publish_simulation_status(self, status: SimulationStatusPayload) -> None:
        if not isinstance(status, SimulationStatusPayload):
            raise TypeError("simulation status publisher requires SimulationStatusPayload")
        with self._status_lock:
            self._status = status
            self._status_dirty = True
            self._status_revision += 1

    def flush_telemetry(self, client: Any) -> None:
        if not self.pilot_id or not self.active_lease:
            return
        with self._telemetry_lock:
            if not self._telemetry_dirty:
                return
            payload = dict(self._telemetry)
            revision = self._telemetry_revision
            pilot_id = self.pilot_id
            lease_id = self.active_lease
        if not self._send_best_effort(
            client,
            "telemetry",
            target_id=pilot_id,
            payload=payload,
            lease_id=lease_id,
        ):
            return
        with self._telemetry_lock:
            if (
                self._telemetry_revision == revision
                and self.pilot_id == pilot_id
                and self.active_lease == lease_id
            ):
                self._telemetry_dirty = False

    def flush_simulation_status(self, client: Any) -> None:
        with self._status_lock:
            if not self._status_dirty or self._status is None:
                return
            status = self._status
            revision = self._status_revision
        if not self._send_best_effort(
            client,
            "simulation_status",
            payload=status.to_payload(),
        ):
            return
        with self._status_lock:
            if self._status_revision == revision:
                self._status_dirty = False

    def flush_simulation_results(self, client: Any) -> None:
        results = [
            result
            for result in self.operator_mailbox.take_results(
                max_items=_MAX_SIMULATION_RESULTS_PER_CYCLE
            )
            if (
                result.target_id == self.simulation_ui_id
                and result.payload.session_id == self.simulation_session_id
            )
        ]
        for index, result in enumerate(results):
            if not self._send_best_effort(
                client,
                "simulation_result",
                target_id=result.target_id,
                payload=result.payload.to_payload(),
                lease_id=self.simulation_session_id,
            ):
                self.operator_mailbox.requeue_results(results[index:])
                return

    def handle_envelope(self, client: Any, message: Envelope) -> None:
        message_type = message.message_type
        if message_type in {
            "lease_granted",
            "lease_revoked",
            "simulation_session_granted",
            "simulation_session_revoked",
            "motion_command",
            "simulation_command",
            "webrtc_signal",
        }:
            self._diagnostic(
                "receive",
                dedupe=f"{message_type}:{message.source_id}",
                source=message.source_id,
                target=self.endpoint_id,
                type=message_type,
            )
        if message_type == "lease_granted":
            self.grant_lease(
                str((message.payload or {}).get("pilot_id", "")),
                message.lease_id,
            )
            return
        if message_type == "lease_revoked":
            self.revoke_lease()
            return
        if message_type == "simulation_session_granted":
            try:
                granted = SimulationSessionGrantedPayload.from_payload(message.payload or {})
            except ProtocolError:
                return
            if granted.sim_id == self.endpoint_id and message.lease_id == granted.session_id:
                self.grant_simulation_session(granted)
            return
        if message_type == "simulation_session_revoked":
            try:
                revoked = SimulationSessionRevokedPayload.from_payload(message.payload or {})
            except ProtocolError:
                return
            if revoked.session_id == self.simulation_session_id:
                self.revoke_simulation_session()
            return
        if message_type == "motion_command":
            ok, reason = self._apply_motion(message)
            self._diagnostic(
                "motion",
                dedupe=f"motion:{message.source_id}:{reason}",
                source=message.source_id,
                target=self.endpoint_id,
                ok=ok,
                reason=reason,
            )
            self._send_best_effort(
                client,
                "ack",
                target_id=message.source_id,
                payload={"reply_to": message.message_id, "ok": ok, "reason": reason},
                lease_id=message.lease_id,
                trace_context=message.trace_context,
            )
            return
        if message_type == "simulation_command":
            self._queue_simulation_command(client, message)
            return
        if message_type == "webrtc_signal":
            self._handle_webrtc_offer(client, message)

    def _close_webrtc_session_async(self, session_id: str) -> None:
        handler = self.webrtc_session_close_handler
        if handler is None:
            return
        executor = self._webrtc_executor
        if executor is not None:
            try:
                executor.submit(handler, str(session_id))
                return
            except RuntimeError:
                pass
        threading.Thread(
            target=self._run_webrtc_close,
            args=(handler, str(session_id)),
            name="sim-webrtc-close",
            daemon=True,
        ).start()

    @staticmethod
    def _run_webrtc_close(handler: Callable[[str], None], session_id: str) -> None:
        try:
            handler(session_id)
        except Exception:
            pass

    def _validate_motion(self, message: Envelope, *, allow_estop: bool = False) -> tuple[bool, str]:
        if allow_estop:
            return True, "accepted"
        if message.source_id != self.pilot_id or message.lease_id != self.active_lease:
            return False, "lease_mismatch"
        if message.seq <= self.last_control_seq:
            return False, "stale_sequence"
        self.last_control_seq = message.seq
        return True, "accepted"

    def _validate_simulation_session(self, message: Envelope, session_id: str) -> tuple[bool, str]:
        if (
            message.source_id != self.simulation_ui_id
            or message.lease_id != self.simulation_session_id
            or session_id != self.simulation_session_id
        ):
            return False, "simulation_session_mismatch"
        if message.seq <= self.last_simulation_seq:
            return False, "stale_sequence"
        self.last_simulation_seq = message.seq
        return True, "accepted"

    def _apply_motion(self, message: Envelope) -> tuple[bool, str]:
        payload = dict(message.payload or {})
        command = str(payload.get("command", ""))
        ok, reason = self._validate_motion(message, allow_estop=command == "estop")
        if not ok:
            return ok, reason
        try:
            return True, self.state.apply_command(payload)
        except ValueError as exc:
            reason = str(exc)
            return False, reason if reason else "invalid_command"

    def _queue_simulation_command(self, client: Any, message: Envelope) -> None:
        try:
            request = SimulationCommandRequest.from_payload(message.payload or {})
        except ProtocolError:
            return
        ok, reason = self._validate_simulation_session(message, request.session_id)
        if ok:
            queued = self.operator_mailbox.enqueue(
                SimulationOperatorCommand.from_request(request, ui_id=message.source_id)
            )
            if queued:
                self._diagnostic(
                    "simulation",
                    dedupe=f"command:{request.command}:{request.request_id}",
                    source=message.source_id,
                    target=self.endpoint_id,
                    command=request.command,
                    state="queued",
                )
                return
            reason = "simulation command queue is full"
        result = SimulationResultPayload(
            request_id=request.request_id,
            session_id=request.session_id,
            command=request.command,
            ok=False,
            reason=reason,
        )
        self._send_best_effort(
            client,
            "simulation_result",
            target_id=message.source_id,
            payload=result.to_payload(),
            lease_id=message.lease_id,
            trace_context=message.trace_context,
        )
        self._diagnostic(
            "simulation",
            dedupe=f"command-rejected:{request.command}:{request.request_id}",
            source=message.source_id,
            target=self.endpoint_id,
            command=request.command,
            state="rejected",
            reason=reason,
        )

    def _handle_webrtc_offer(self, client: Any, message: Envelope) -> None:
        if self.webrtc_offer_handler is None:
            return
        try:
            signal = WebRtcSignalPayload.from_payload(message.payload or {})
        except ProtocolError:
            return
        ok, _reason = self._validate_simulation_session(message, signal.session_id)
        if not ok or signal.signal != "offer" or signal.stream not in self.simulation_streams:
            return
        executor = self._webrtc_executor
        if executor is None:
            return
        if len(self._webrtc_futures) >= _MAX_WEBRTC_INFLIGHT:
            self._diagnostic(
                "webrtc",
                dedupe=f"inflight:{signal.session_id}",
                source=self.endpoint_id,
                target=message.source_id,
                stream=signal.stream,
                state="busy",
                reason="bounded signaling work limit reached",
            )
            return
        try:
            future = executor.submit(
                self.webrtc_offer_handler,
                signal.stream,
                signal.sdp,
                signal.type,
                self.turn_credentials,
                signal.session_id,
            )
        except RuntimeError:
            return
        self._webrtc_futures.append(
            (
                future,
                message.source_id,
                signal.session_id,
                signal.stream,
                message.trace_context,
            )
        )

    def _flush_webrtc_answers(self, client: Any) -> None:
        if not self._webrtc_futures:
            return
        pending: list[
            tuple[
                concurrent.futures.Future[Any],
                str,
                str,
                str,
                Optional[Mapping[str, str]],
            ]
        ] = []
        for future, source_id, session_id, stream, trace_context in self._webrtc_futures:
            if not future.done():
                pending.append((future, source_id, session_id, stream, trace_context))
                continue
            try:
                answer = future.result()
                if not isinstance(answer, Mapping):
                    raise ValueError("media worker returned a non-mapping answer")
            except Exception as exc:
                self._diagnostic(
                    "webrtc",
                    dedupe=f"answer-error:{session_id}:{stream}",
                    source=self.endpoint_id,
                    target=source_id,
                    stream=stream,
                    state="answer-failed",
                    reason=str(exc),
                )
                continue
            if session_id != self.simulation_session_id:
                continue
            payload = WebRtcSignalPayload(
                session_id=session_id,
                stream=stream,
                signal="answer",
                sdp=str(answer.get("sdp", "")),
                type=str(answer.get("type", "")),
            )
            sent = self._send_best_effort(
                client,
                "webrtc_signal",
                target_id=source_id,
                payload=payload.to_payload(),
                lease_id=self.simulation_session_id,
                trace_context=trace_context,
            )
            self._diagnostic(
                "webrtc",
                dedupe=f"answer:{session_id}:{stream}",
                source=self.endpoint_id,
                target=source_id,
                stream=stream,
                state="answer-sent" if sent else "answer-retry",
            )
        self._webrtc_futures = pending

    def _send_best_effort(
        self,
        client: Any,
        message_type: str,
        *,
        target_id: Optional[str] = None,
        payload: Optional[Mapping[str, object]] = None,
        lease_id: str = "",
        trace_context: Optional[Mapping[str, str]] = None,
    ) -> bool:
        kwargs: dict[str, object] = {
            "payload": dict(payload or {}),
            "lease_id": lease_id,
        }
        if target_id is not None:
            kwargs["target_id"] = target_id
        if trace_context is not None:
            kwargs["trace_context"] = trace_context
        try:
            client.send(message_type, **kwargs)
        except DdsTransportError as exc:
            self._diagnostic(
                "send",
                dedupe=f"{message_type}:{target_id or 'authority'}",
                source=self.endpoint_id,
                target=target_id or "authority",
                type=message_type,
                state="retrying",
                reason=str(exc),
            )
            return False
        return True

    def _diagnostic(
        self,
        event: str,
        *,
        dedupe: str,
        interval: float = 5.0,
        **fields: object,
    ) -> None:
        now = time.monotonic()
        previous = self._diagnostic_seen.get(dedupe)
        if previous is not None and now - previous < interval:
            return
        self._diagnostic_seen[dedupe] = now
        rendered = " ".join(
            f"{key}={str(value).replace(chr(10), ' ')[:160]}"
            for key, value in fields.items()
        )
        print(f"[sim-dds] event={event} {rendered}", flush=True)

    def _run(self) -> None:
        capabilities = [CAPABILITY_MOTION_ARM, CAPABILITY_MOTION_GO2]
        if "rgbd" in self.streams:
            capabilities.append(CAPABILITY_STREAM_RGBD)
        if "observer" in self.streams:
            capabilities.append(CAPABILITY_STREAM_OBSERVER)
        if "hand_eye_preview" in self.streams:
            capabilities.append(CAPABILITY_STREAM_HAND_EYE_PREVIEW)
        endpoint_kwargs: dict[str, Any] = {
            "settings": self.settings,
            "turn_credential_provider": self.turn_credential_provider,
        }
        if self.simulation_session_ready_provider is not None:
            endpoint_kwargs["simulation_session_ready_provider"] = (
                self.simulation_session_ready_provider
            )
        client = self.endpoint_factory(
            EndpointDescriptor(
                self.endpoint_id,
                "sim",
                tuple(capabilities),
                streams=self.streams,
            ),
            **endpoint_kwargs,
        )
        identity = getattr(getattr(client, "node", None), "identity", None)
        if not isinstance(identity, PeerIdentity):
            raise RuntimeError("sim DDS peer did not expose its boot identity")
        self.peer_identity = identity
        self.ready.set()
        was_registered = client.registered
        try:
            while not self.stop_event.is_set():
                try:
                    client.heartbeat()
                    for message in client.receive(timeout_ms=20):
                        self.handle_envelope(client, message)
                    self._flush_webrtc_answers(client)
                    if was_registered and not client.registered:
                        self.revoke_lease()
                        self.revoke_simulation_session()
                    was_registered = client.registered
                    self.flush_telemetry(client)
                    self.flush_simulation_status(client)
                    self.flush_simulation_results(client)
                except DdsTransportError as exc:
                    self._diagnostic(
                        "transport",
                        dedupe="loop",
                        source=self.endpoint_id,
                        state="retrying",
                        reason=str(exc),
                    )
                    self.stop_event.wait(0.1)
        finally:
            self.revoke_lease()
            self.revoke_simulation_session()
            client.close()
            if self._webrtc_executor is not None:
                self._webrtc_executor.shutdown(wait=False, cancel_futures=True)


__all__ = ["SimEndpoint"]
