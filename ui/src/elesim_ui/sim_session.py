"""UI-owned simulation lease, command, status and WebRTC lifecycle."""

from __future__ import annotations

import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass
from typing import Any, Callable, Mapping, Optional

from elesim_protocol import (
    CloseSimulationSessionRequest,
    DdsRuntimeSettings,
    EndpointDescriptor,
    OpenSimulationSessionRequest,
    PeerClient,
    ProtocolError,
    DdsTransportError,
    SimulationCommandRequest,
    SimulationResultPayload,
    SimulationSessionOpenedPayload,
    SimulationSessionRevokedPayload,
    SimulationStatusPayload,
    TurnCredentials,
    WebRtcSignalPayload,
)

from .webrtc import WebRtcVideoReceiver


SIMULATION_STREAMS = ("observer", "hand_eye_preview")
_COALESCED_COMMANDS = frozenset(
    {"orbit", "pan", "zoom", "set_speed", "set_debug_visible"}
)


@dataclass(frozen=True)
class UiSimulationSnapshot:
    requested_sim_id: str
    active_sim_id: str
    session_id: str
    connected_streams: tuple[str, ...]
    status: Optional[SimulationStatusPayload]
    pending_commands: int
    last_result: Optional[SimulationResultPayload]
    last_error: str


@dataclass(frozen=True)
class _QueuedCommand:
    request_id: str
    command: str
    arguments: dict[str, Any]


def _coalesce_command(
    previous: _QueuedCommand,
    current: _QueuedCommand,
) -> _QueuedCommand:
    if current.command in {"orbit", "pan"}:
        arguments = {
            axis: max(
                -2.0,
                min(2.0, float(previous.arguments[axis]) + float(current.arguments[axis])),
            )
            for axis in ("dx", "dy")
        }
        return _QueuedCommand(current.request_id, current.command, arguments)
    if current.command == "zoom":
        delta = float(previous.arguments["delta"]) + float(current.arguments["delta"])
        return _QueuedCommand(
            current.request_id,
            current.command,
            {"delta": max(-2.0, min(2.0, delta))},
        )
    return current


class UiSimSession:
    """Run one UI-to-sim operator session on its own protocol endpoint."""

    def __init__(
        self,
        *,
        ui_id: str,
        sim_id: str,
        settings: Optional[DdsRuntimeSettings] = None,
        peer: Any = None,
        peer_factory: Callable[..., Any] = PeerClient,
        receiver_factory: Callable[[], Any] = WebRtcVideoReceiver,
        retry_s: float = 0.5,
        open_timeout_s: Optional[float] = None,
        poll_ms: int = 50,
        max_pending_commands: int = 128,
        clock: Callable[[], float] = time.monotonic,
        autostart: bool = True,
    ) -> None:
        self.endpoint_id = str(ui_id)
        self.settings = settings
        self.peer = peer
        self.peer_factory = peer_factory
        self.receiver_factory = receiver_factory
        self.retry_s = max(0.05, float(retry_s))
        if open_timeout_s is None:
            heartbeat_timeout = float(
                getattr(settings, "heartbeat_timeout_s", 3.5)
            )
            open_timeout_s = max(3.0, heartbeat_timeout * 2.0)
        self.open_timeout_s = max(self.retry_s, float(open_timeout_s))
        self.poll_ms = max(0, int(poll_ms))
        self.max_pending_commands = max(8, int(max_pending_commands))
        self.clock = clock

        self._lock = threading.RLock()
        self._requested_sim_id = str(sim_id)
        self._active_sim_id = ""
        self._session_id = ""
        self._opening_request_id = ""
        self._opening_deadline = 0.0
        self._closing_session_id = ""
        self._retry_after = 0.0
        self._receivers: dict[str, Any] = {}
        self._turn: Optional[TurnCredentials] = None
        self._connected_streams: set[str] = set()
        self._status: Optional[SimulationStatusPayload] = None
        self._commands: deque[_QueuedCommand] = deque()
        self._pending_command_ids: set[str] = set()
        self._sent_messages: dict[str, tuple[str, str]] = {}
        self._last_result: Optional[SimulationResultPayload] = None
        self._last_error = ""
        self._last_error_log = ""
        self._last_error_log_at = 0.0
        self._was_registered = False
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        if autostart:
            self.start()

    @property
    def active_sim_id(self) -> str:
        with self._lock:
            return self._active_sim_id

    @property
    def connected_streams(self) -> tuple[str, ...]:
        with self._lock:
            return self._connected_stream_names()

    @property
    def status(self) -> Optional[SimulationStatusPayload]:
        with self._lock:
            return self._status

    @property
    def last_error(self) -> str:
        with self._lock:
            return self._last_error

    @property
    def snapshot(self) -> UiSimulationSnapshot:
        with self._lock:
            return UiSimulationSnapshot(
                requested_sim_id=self._requested_sim_id,
                active_sim_id=self._active_sim_id,
                session_id=self._session_id,
                connected_streams=self._connected_stream_names(),
                status=self._status,
                pending_commands=len(self._commands) + len(self._pending_command_ids),
                last_result=self._last_result,
                last_error=self._last_error,
            )

    def _connected_stream_names(self) -> tuple[str, ...]:
        return tuple(
            stream for stream in SIMULATION_STREAMS if stream in self._connected_streams
        )

    def receiver(self, stream: str) -> Any:
        with self._lock:
            return self._receivers.get(str(stream))

    def frame(self, stream: str):
        receiver = self.receiver(stream)
        if receiver is None:
            return None
        view_getter = getattr(receiver, "latest_frame_view", None)
        if callable(view_getter):
            return view_getter()
        getter = getattr(receiver, "latest_frame", None)
        if getter is not None:
            return getter()
        return receiver.latest_bgr

    def frame_version(self, stream: str) -> Optional[int]:
        receiver = self.receiver(stream)
        if receiver is None:
            return None
        getter = getattr(receiver, "frame_version", None)
        if not callable(getter):
            return None
        try:
            return int(getter())
        except Exception:
            return None

    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._run,
                name="ui-sim-session",
                daemon=True,
            )
            self._thread.start()

    def switch_target(self, sim_id: str) -> None:
        with self._lock:
            self._requested_sim_id = str(sim_id).strip()
            self._retry_after = 0.0
            opening = self._opening_request_id
            if opening:
                self._forget_sent_request_locked("open", opening)
            self._opening_request_id = ""
            self._opening_deadline = 0.0
            self._commands.clear()

    def send_command(self, command: str, arguments: Mapping[str, Any] | None = None) -> str:
        command_name = str(command).strip()
        with self._lock:
            session_id = self._session_id
        if not session_id:
            self._set_error("simulation session is not connected")
            return ""
        request_id = uuid.uuid4().hex
        parsed = SimulationCommandRequest.from_payload(
            {
                "schema_version": 1,
                "request_id": request_id,
                "session_id": session_id,
                "command": command_name,
                "arguments": dict(arguments or {}),
            }
        )
        queued = _QueuedCommand(parsed.request_id, parsed.command, parsed.arguments)
        with self._lock:
            if (
                command_name in _COALESCED_COMMANDS
                and self._commands
                and self._commands[-1].command == command_name
            ):
                self._commands[-1] = _coalesce_command(self._commands[-1], queued)
                return request_id
            if (
                len(self._commands) + len(self._pending_command_ids)
                >= self.max_pending_commands
            ):
                self._set_error(
                    "simulation command backlog is full "
                    f"({self.max_pending_commands}); waiting for Sim acknowledgements"
                )
                return ""
            self._commands.append(queued)
        return request_id

    def run_cycle(self, client: Any) -> None:
        try:
            client.heartbeat()
        except DdsTransportError as exc:
            # A transport reset invalidates the remote session lease even if
            # the client has not observed ``registered=False`` yet.  Drop
            # stale receivers/commands so the next live descriptor opens a
            # fresh session instead of sending into a dead DDS graph.
            self._lose_session(f"simulation transport failed: {exc}")
            raise
        for message in client.receive(timeout_ms=self.poll_ms):
            self._handle_message(client, message)

        registered = bool(client.registered)
        if self._was_registered and not registered:
            self._lose_session("DDS peer became unavailable")
        self._was_registered = registered
        if not registered:
            return
        self._drive_session(client)
        self._flush_commands(client)

    def _drive_session(self, client: Any) -> None:
        now = self.clock()
        with self._lock:
            requested = self._requested_sim_id
            active = self._active_sim_id
            opening = self._opening_request_id
            opening_deadline = self._opening_deadline
            closing = self._closing_session_id
            retry_after = self._retry_after
        if (
            not active
            and opening
            and opening_deadline > 0.0
            and now >= opening_deadline
        ):
            with self._lock:
                # The reply may have crossed the timeout boundary and already
                # be queued. Keep the request id check in _handle_opened so a
                # late reply cannot attach a stale lease to a new attempt.
                if self._opening_request_id == opening:
                    self._opening_request_id = ""
                    self._opening_deadline = 0.0
                    self._forget_sent_request_locked("open", opening)
                    self._retry_after = now + self.retry_s
            self._set_error(
                f"simulation session open timed out for {requested!r}; retrying"
            )
            return
        if active and requested != active:
            if not closing:
                self._send_close(client)
            return
        if not active and requested and not opening and not closing and now >= retry_after:
            has_peer = getattr(client, "has_peer", None)
            if callable(has_peer) and not has_peer(requested):
                self._set_error(
                    f"simulation peer {requested!r} is not live; "
                    "waiting for its exact DDS endpoint descriptor "
                    "and fresh heartbeat"
                )
                with self._lock:
                    self._retry_after = now + self.retry_s
                return
            self._send_open(client, requested)

    def _send_open(self, client: Any, sim_id: str) -> None:
        request = OpenSimulationSessionRequest(
            request_id=uuid.uuid4().hex,
            sim_id=sim_id,
            streams=SIMULATION_STREAMS,
        )
        envelope = client.send(
            "open_simulation_session",
            payload=request.to_payload(),
        )
        with self._lock:
            self._opening_request_id = request.request_id
            self._opening_deadline = self.clock() + self.open_timeout_s
            self._sent_messages[str(envelope.message_id)] = ("open", request.request_id)
        self._set_error(
            f"simulation session open requested for {sim_id!r}; waiting for Sim"
        )

    def _send_close(self, client: Any) -> None:
        with self._lock:
            session_id = self._session_id
        if not session_id:
            return
        request = CloseSimulationSessionRequest(
            request_id=uuid.uuid4().hex,
            session_id=session_id,
        )
        envelope = client.send(
            "close_simulation_session",
            payload=request.to_payload(),
            lease_id=session_id,
        )
        with self._lock:
            self._closing_session_id = session_id
            self._sent_messages[str(envelope.message_id)] = ("close", session_id)
            self._commands.clear()
        self._close_receivers()

    def _flush_commands(self, client: Any) -> None:
        while True:
            with self._lock:
                if self._closing_session_id or not self._commands:
                    return
                queued = self._commands.popleft()
                session_id = self._session_id
                target_id = self._active_sim_id
            if not session_id or not target_id:
                return
            request = SimulationCommandRequest(
                request_id=queued.request_id,
                session_id=session_id,
                command=queued.command,
                arguments=queued.arguments,
            )
            try:
                envelope = client.send(
                    "simulation_command",
                    target_id=target_id,
                    payload=request.to_payload(),
                    lease_id=session_id,
                )
            except Exception:
                # Do not lose an operator command merely because the Sim
                # boot vanished between discovery and the direct DDS write.
                with self._lock:
                    if (
                        self._session_id == session_id
                        and self._active_sim_id == target_id
                    ):
                        self._commands.appendleft(queued)
                raise
            with self._lock:
                self._pending_command_ids.add(request.request_id)
                self._sent_messages[str(envelope.message_id)] = (
                    "command",
                    request.request_id,
                )

    def _handle_message(self, client: Any, message: Any) -> None:
        try:
            if message.message_type == "simulation_session_opened":
                self._handle_opened(client, message)
            elif message.message_type == "simulation_session_revoked":
                self._handle_revoked(message)
            elif message.message_type == "webrtc_signal":
                self._handle_webrtc_answer(message)
            elif message.message_type == "simulation_status":
                self._handle_status(message)
            elif message.message_type == "simulation_result":
                self._handle_result(message)
            elif message.message_type == "error":
                self._handle_peer_error(message)
        except (ProtocolError, RuntimeError, ValueError, KeyError) as exc:
            self._set_error(f"simulation message rejected: {exc}")

    def _handle_opened(self, client: Any, message: Any) -> None:
        opened = SimulationSessionOpenedPayload.from_payload(message.payload or {})
        if message.lease_id != opened.session_id:
            raise ProtocolError("simulation session lease does not match opened payload")
        if message.source_id != opened.sim_id:
            raise ProtocolError("simulation session source does not match opened payload")
        with self._lock:
            refreshing = (
                opened.session_id == self._session_id
                and opened.sim_id == self._active_sim_id
            )
            if refreshing and opened.turn == self._turn:
                return
            expected_request = self._opening_request_id
            requested = self._requested_sim_id
        if not refreshing and (
            opened.request_id != expected_request or opened.sim_id != requested
        ):
            return

        if not refreshing:
            with self._lock:
                self._active_sim_id = opened.sim_id
                self._session_id = opened.session_id
                self._opening_request_id = ""
                self._opening_deadline = 0.0
                self._closing_session_id = ""
                self._status = None
                self._last_error = ""
                self._forget_sent_request_locked("open", opened.request_id)
        try:
            receivers = self._negotiate_receivers(client, opened)
        except Exception:
            if not refreshing:
                self._send_close(client)
            raise

        with self._lock:
            stale = (
                self._session_id != opened.session_id
                or self._active_sim_id != opened.sim_id
            )
            previous = tuple(self._receivers.values())
            if not stale:
                self._receivers = receivers
                self._turn = opened.turn
                self._connected_streams.clear()
                self._last_error = ""
        if stale:
            self._close_receiver_set(receivers.values())
            return
        self._close_receiver_set(previous)

    def _negotiate_receivers(
        self,
        client: Any,
        opened: SimulationSessionOpenedPayload,
    ) -> dict[str, Any]:
        receivers: dict[str, Any] = {}
        signals: list[WebRtcSignalPayload] = []
        try:
            for stream in opened.streams:
                receiver = self.receiver_factory()
                receivers[stream] = receiver
                offer = receiver.create_offer(turn=opened.turn)
                signals.append(
                    WebRtcSignalPayload(
                        session_id=opened.session_id,
                        stream=stream,
                        signal="offer",
                        sdp=str(offer["sdp"]),
                        type=str(offer["type"]),
                    )
                )
            for signal in signals:
                client.send(
                    "webrtc_signal",
                    target_id=opened.sim_id,
                    payload=signal.to_payload(),
                    lease_id=opened.session_id,
                )
        except Exception:
            self._close_receiver_set(receivers.values())
            raise
        return receivers

    def _handle_revoked(self, message: Any) -> None:
        revoked = SimulationSessionRevokedPayload.from_payload(message.payload or {})
        with self._lock:
            if revoked.session_id not in {self._session_id, self._closing_session_id}:
                return
        self._clear_session()

    def _handle_webrtc_answer(self, message: Any) -> None:
        signal = WebRtcSignalPayload.from_payload(message.payload or {})
        with self._lock:
            session_id = self._session_id
            sim_id = self._active_sim_id
            receiver = self._receivers.get(signal.stream)
        if (
            signal.signal != "answer"
            or signal.session_id != session_id
            or message.lease_id != session_id
            or message.source_id != sim_id
            or receiver is None
        ):
            return
        receiver.accept_answer(signal.sdp, signal.type)
        with self._lock:
            self._connected_streams.add(signal.stream)

    def _handle_status(self, message: Any) -> None:
        status = SimulationStatusPayload.from_payload(message.payload or {})
        with self._lock:
            if message.source_id == self._active_sim_id:
                self._status = status

    def _handle_result(self, message: Any) -> None:
        result = SimulationResultPayload.from_payload(message.payload or {})
        with self._lock:
            if (
                result.session_id != self._session_id
                or message.source_id != self._active_sim_id
            ):
                return
            self._pending_command_ids.discard(result.request_id)
            self._forget_sent_request_locked("command", result.request_id)
            self._last_result = result
            failure = (
                result.reason or f"{result.command} failed"
                if not result.ok
                else ""
            )
        if failure:
            self._set_error(failure)

    def _handle_peer_error(self, message: Any) -> None:
        payload = dict(message.payload or {})
        reply_to = str(payload.get("reply_to", ""))
        reason = str(payload.get("reason", "DDS peer rejected simulation request"))
        with self._lock:
            kind, request_id = self._sent_messages.pop(reply_to, ("", ""))
            if kind == "open" and request_id == self._opening_request_id:
                self._opening_request_id = ""
                self._opening_deadline = 0.0
                self._retry_after = self.clock() + self.retry_s
            elif kind == "close" and request_id == self._closing_session_id:
                self._clear_session_locked()
            elif kind == "command":
                self._pending_command_ids.discard(request_id)
        self._set_error(reason)

    def _lose_session(self, reason: str) -> None:
        self._close_receivers()
        with self._lock:
            self._clear_session_locked()
            self._retry_after = self.clock() + self.retry_s
        self._set_error(reason)

    def _clear_session(self) -> None:
        self._close_receivers()
        with self._lock:
            self._clear_session_locked()

    def _clear_session_locked(self) -> None:
        self._active_sim_id = ""
        self._session_id = ""
        self._opening_request_id = ""
        self._opening_deadline = 0.0
        self._closing_session_id = ""
        self._turn = None
        self._connected_streams.clear()
        self._status = None
        self._commands.clear()
        self._pending_command_ids.clear()
        self._sent_messages.clear()

    def _forget_sent_request_locked(self, kind: str, request_id: str) -> None:
        """Drop transport bookkeeping after a request receives its reply."""

        wanted = (str(kind), str(request_id))
        for message_id, entry in tuple(self._sent_messages.items()):
            if entry == wanted:
                self._sent_messages.pop(message_id, None)

    def _close_receivers(self) -> None:
        with self._lock:
            receivers = tuple(self._receivers.values())
            self._receivers.clear()
            self._connected_streams.clear()
        self._close_receiver_set(receivers)

    @staticmethod
    def _close_receiver_set(receivers: Any) -> None:
        for receiver in receivers:
            try:
                receiver.close()
            except Exception:
                pass

    def _set_error(self, message: str) -> None:
        value = str(message)
        now = self.clock()
        with self._lock:
            self._last_error = value
            should_log = (
                value != self._last_error_log
                or now - self._last_error_log_at >= 5.0
            )
            if should_log:
                self._last_error_log = value
                self._last_error_log_at = now
        if should_log:
            print(f"[ui-dds] {value}", flush=True)

    def _run(self) -> None:
        client = self.peer
        owns_client = client is None
        try:
            if client is None:
                client = self.peer_factory(
                    EndpointDescriptor(self.endpoint_id, "ui", ()),
                    settings=self.settings,
                )
            while not self._stop.is_set():
                try:
                    self.run_cycle(client)
                except Exception as exc:
                    self._set_error(f"simulation transport failed: {exc}")
                    self._stop.wait(self.retry_s)
        except Exception as exc:
            self._set_error(f"simulation transport unavailable: {exc}")
        finally:
            if client is not None:
                try:
                    if self._session_id:
                        self._send_close(client)
                except Exception:
                    pass
                if owns_client:
                    client.close()
            self._clear_session()

    def close(self) -> None:
        self._stop.set()
        with self._lock:
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=3.0)
        self._clear_session()


__all__ = ["SIMULATION_STREAMS", "UiSimulationSnapshot", "UiSimSession"]
