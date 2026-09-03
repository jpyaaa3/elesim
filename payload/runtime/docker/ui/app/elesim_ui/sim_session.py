"""UI-owned simulation lease, command, status and WebRTC lifecycle."""

from __future__ import annotations

import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass
from typing import Any, Callable, Mapping, Optional

from elesim_protocol import (
    CAPABILITY_SIM_MOCK_HUG,
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
    current_trace_context,
)
from elesim_protocol.tracing import sampled_span, span

from .webrtc import WebRtcVideoReceiver


SIMULATION_STREAMS = ("observer", "hand_eye_preview")
_COALESCED_COMMANDS = frozenset(
    {"orbit", "pan", "zoom", "set_speed", "set_debug_visible"}
)
_MAX_STREAM_RETRY_DELAY_S = 5.0
_STREAM_RETRY_COUNT_CAP = 16
_STREAM_STARTUP_TIMEOUT_S = 8.0
_STREAM_STALL_TIMEOUT_S = 5.0
_MAX_OPEN_RETRY_DELAY_S = 5.0
_MAX_OPEN_RETRY_EXPONENT = 16
_MAX_COMMAND_FLUSH_PER_CYCLE = 32


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
        # Camera gestures are latest-only and coalesced.  A 50 ms receive
        # timeout made the UI-to-Sim path visibly quantized at 20 Hz before
        # the Sim endpoint even handled the command.
        poll_ms: int = 10,
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
        self._open_retry_count = 0
        self._receivers: dict[str, Any] = {}
        self._stream_retry_at: dict[str, float] = {}
        self._stream_retry_count: dict[str, int] = {}
        self._stream_connected_at: dict[str, float] = {}
        self._stream_offer_sent_at: dict[str, float] = {}
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
            stream
            for stream in SIMULATION_STREAMS
            if (
                stream in self._connected_streams
                and self._stream_has_live_frame(stream)
            )
        )

    def _stream_has_live_frame(self, stream: str) -> bool:
        """Treat a negotiated track as LIVE only after pixels are decoded."""

        receiver = self._receivers.get(str(stream))
        age_getter = getattr(receiver, "frame_age_s", None)
        if not callable(age_getter):
            # Keep compatibility with embedded/test receivers which only
            # expose offer/answer callbacks and have no decoder clock.
            return True
        try:
            age = age_getter()
            return age is not None and float(age) < _STREAM_STALL_TIMEOUT_S
        except Exception:
            return True

    def receiver(self, stream: str) -> Any:
        with self._lock:
            return self._receivers.get(str(stream))

    def frame(self, stream: str):
        name = str(stream)
        with self._lock:
            if (
                name not in self._connected_streams
                or not self._stream_has_live_frame(name)
            ):
                return None
            receiver = self._receivers.get(name)
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

    def stream_error(self, stream: str) -> str:
        """Return the latest decoder error for one WebRTC stream, if any."""

        receiver = self.receiver(stream)
        if receiver is None:
            return ""
        value = getattr(receiver, "last_error", "")
        return str(value).strip()

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
            self._open_retry_count = 0
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
        self._check_stream_liveness()
        self._retry_failed_streams(client)
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
                    self._schedule_open_retry_locked(now=now)
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
        with span(
            "elesim_ui.sim_session.UiSimSession._send_open",
            attributes={
                "code.function.name": "elesim_ui.sim_session.UiSimSession._send_open",
                "elesim.flow.id": "simulation.session.open",
            },
            kind="producer",
        ):
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
        with span(
            "elesim_ui.sim_session.UiSimSession._send_close",
            attributes={
                "code.function.name": "elesim_ui.sim_session.UiSimSession._send_close",
                "elesim.flow.id": "simulation.session.close",
            },
            kind="producer",
        ):
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
        for _ in range(_MAX_COMMAND_FLUSH_PER_CYCLE):
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
                with sampled_span(
                    "elesim_ui.sim_session.UiSimSession._flush_commands",
                    sample_key=f"ui.simulation:{queued.command}",
                    every=10 if queued.command in _COALESCED_COMMANDS else 1,
                    attributes={
                        "code.function.name": "elesim_ui.sim_session.UiSimSession._flush_commands",
                        "elesim.flow.id": f"simulation.command.{queued.command}",
                        "elesim.simulation.command": queued.command,
                    },
                    kind="producer",
                ):
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
                self._retry_after = 0.0
                self._open_retry_count = 0
                self._status = None
                self._last_error = ""
                self._forget_sent_request_locked("open", opened.request_id)
        try:
            receivers = self._negotiate_receivers(
                client,
                opened,
                allow_partial=not refreshing,
            )
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
                missing_streams = tuple(
                    stream
                    for stream in opened.streams
                    if stream not in receivers
                )
                self._receivers = receivers
                self._turn = opened.turn
                self._connected_streams.clear()
                self._stream_retry_at.clear()
                self._stream_retry_count.clear()
                self._stream_connected_at.clear()
                now = self.clock()
                self._stream_offer_sent_at = {
                    stream: now for stream in receivers
                }
                if not missing_streams:
                    self._last_error = ""
            else:
                missing_streams = ()
        if stale:
            self._close_receiver_set(receivers.values())
            return
        self._close_receiver_set(previous)
        for stream in missing_streams:
            self._schedule_stream_retry(stream)

    def _negotiate_receivers(
        self,
        client: Any,
        opened: SimulationSessionOpenedPayload,
        *,
        allow_partial: bool = False,
    ) -> dict[str, Any]:
        receivers: dict[str, Any] = {}
        signals: list[WebRtcSignalPayload] = []
        for stream in opened.streams:
            receiver = None
            try:
                receiver = self._prepare_receiver(
                    self.receiver_factory(),
                    stream,
                )
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
            except Exception as exc:
                if receiver is not None:
                    self._close_receiver_set((receiver,))
                receivers.pop(stream, None)
                if not allow_partial:
                    self._close_receiver_set(receivers.values())
                    raise
                self._set_error(
                    f"{stream} WebRTC offer failed: "
                    f"{str(exc).strip() or type(exc).__name__}"
                )

        for signal in signals:
            try:
                client.send(
                    "webrtc_signal",
                    target_id=opened.sim_id,
                    payload=signal.to_payload(),
                    lease_id=opened.session_id,
                )
            except Exception as exc:
                receiver = receivers.pop(signal.stream, None)
                if receiver is not None:
                    self._close_receiver_set((receiver,))
                if not allow_partial:
                    self._close_receiver_set(receivers.values())
                    raise
                self._set_error(
                    f"{signal.stream} WebRTC offer send failed: "
                    f"{str(exc).strip() or type(exc).__name__}"
                )
        return receivers

    def _prepare_receiver(self, receiver: Any, stream: str) -> Any:
        """Attach stream identity and recovery callback without constraining embedders."""

        name = str(stream)
        if hasattr(receiver, "stream_name"):
            try:
                receiver.stream_name = name
            except Exception:
                pass
        setter = getattr(receiver, "set_error_callback", None)
        if callable(setter):
            try:
                setter(
                    lambda detail, stream=name, receiver=receiver: self._handle_stream_error(
                        stream,
                        receiver,
                        detail,
                    )
                )
            except Exception:
                pass
        return receiver

    def _handle_stream_error(self, stream: str, receiver: Any, detail: str) -> None:
        """Retry one failed WebRTC track while preserving the DDS session."""

        name = str(stream).strip()
        with self._lock:
            if (
                not name
                or not self._session_id
                or self._receivers.get(name) is not receiver
            ):
                return
            self._connected_streams.discard(name)
            self._stream_connected_at.pop(name, None)
            self._stream_offer_sent_at.pop(name, None)
        self._schedule_stream_retry(name)
        self._set_error(f"{name} WebRTC {detail}; retrying")

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
        try:
            receiver.accept_answer(signal.sdp, signal.type)
        except Exception as exc:
            reporter = getattr(receiver, "report_error", None)
            if callable(reporter):
                try:
                    reporter("answer", exc)
                except Exception:
                    pass
            self._set_error(
                f"{signal.stream} WebRTC answer rejected: "
                f"{str(exc).strip() or type(exc).__name__}"
            )
            self._schedule_stream_retry(signal.stream)
            # Keep the other stream and the DDS simulation lease alive.  A
            # codec/ICE failure in one m-line must not make the whole session
            # look disconnected or discard a healthy hand-eye stream.
            return
        with self._lock:
            self._connected_streams.add(signal.stream)
            self._stream_connected_at[signal.stream] = self.clock()
            self._stream_offer_sent_at.pop(signal.stream, None)
            self._stream_retry_at.pop(signal.stream, None)
            self._stream_retry_count.pop(signal.stream, None)

    def _schedule_stream_retry(self, stream: str) -> None:
        name = str(stream).strip()
        if not name:
            return
        with self._lock:
            attempts = min(
                int(self._stream_retry_count.get(name, 0)),
                _STREAM_RETRY_COUNT_CAP,
            )
            delay = min(
                self.retry_s * (2.0 ** attempts),
                _MAX_STREAM_RETRY_DELAY_S,
            )
            self._stream_retry_at[name] = self.clock() + delay

    def _check_stream_liveness(self) -> None:
        """Recover a named track which is connected but produces no frames.

        ICE/DTLS can remain connected while an H.264 decoder is waiting for a
        usable IDR after a burst loss.  Without this watchdog the UI reports a
        stream as LIVE forever (or as WAIT after a one-shot receive error),
        even though the DDS session and the other video track are healthy.
        The retry is per stream and uses the same capped, latest-only recovery
        path as explicit WebRTC errors.
        """

        now = self.clock()
        stalled: list[tuple[str, float]] = []
        with self._lock:
            active = tuple(self._connected_streams)
            connected_at = dict(self._stream_connected_at)
            receivers = dict(self._receivers)
            connected = set(self._connected_streams)
            offer_sent_at = dict(self._stream_offer_sent_at)
            retry_at = dict(self._stream_retry_at)
        pending_answers: list[str] = []
        for stream, sent_at in offer_sent_at.items():
            if (
                stream in receivers
                and stream not in connected
                and stream not in retry_at
                and now - float(sent_at) >= _STREAM_STARTUP_TIMEOUT_S
            ):
                pending_answers.append(stream)
        for stream in active:
            receiver = receivers.get(stream)
            age_getter = getattr(receiver, "frame_age_s", None)
            if receiver is None or not callable(age_getter):
                # Test/embedded receivers predating the watchdog have no
                # decoder clock; their explicit connection callbacks remain
                # authoritative.
                continue
            try:
                age = age_getter()
            except Exception:
                continue
            if age is None:
                age = now - float(connected_at.get(stream, now))
                threshold = _STREAM_STARTUP_TIMEOUT_S
            else:
                threshold = _STREAM_STALL_TIMEOUT_S
            try:
                age_value = float(age)
            except (TypeError, ValueError):
                continue
            if age_value >= threshold:
                stalled.append((stream, age_value))

        for stream in pending_answers:
            with self._lock:
                if (
                    stream not in self._receivers
                    or stream in self._connected_streams
                    or stream in self._stream_retry_at
                ):
                    continue
            self._schedule_stream_retry(stream)
            self._set_error(
                f"{stream} WebRTC answer timed out; retrying"
            )

        for stream, age in stalled:
            with self._lock:
                if stream not in self._connected_streams:
                    continue
                self._connected_streams.discard(stream)
                self._stream_connected_at.pop(stream, None)
                self._stream_offer_sent_at.pop(stream, None)
            self._schedule_stream_retry(stream)
            receiver = receivers.get(stream)
            stats_getter = getattr(receiver, "stats_snapshot", None)
            stats = {}
            if callable(stats_getter):
                try:
                    stats = dict(stats_getter())
                except Exception:
                    stats = {}
            stats_suffix = ""
            if stats:
                stats_suffix = " (" + ", ".join(
                    f"{name}={value}"
                    for name, value in stats.items()
                ) + ")"
            self._set_error(
                f"{stream} WebRTC stalled ({age:.1f}s without a decoded frame)"
                f"{stats_suffix}; "
                "retrying"
            )

    def _retry_failed_streams(self, client: Any) -> None:
        """Retry one failed WebRTC m-line without tearing down the session."""

        now = self.clock()
        with self._lock:
            session_id = self._session_id
            sim_id = self._active_sim_id
            turn = self._turn
            due = tuple(
                stream
                for stream, retry_at in self._stream_retry_at.items()
                if float(retry_at) <= now and stream not in self._connected_streams
            )
        if not session_id or not sim_id or not due:
            return
        for stream in due:
            with self._lock:
                if (
                    self._session_id != session_id
                    or self._active_sim_id != sim_id
                    or stream in self._connected_streams
                ):
                    continue
            receiver = None
            try:
                receiver = self._prepare_receiver(self.receiver_factory(), stream)
                offer = receiver.create_offer(turn=turn)
                client.send(
                    "webrtc_signal",
                    target_id=sim_id,
                    payload=WebRtcSignalPayload(
                        session_id=session_id,
                        stream=stream,
                        signal="offer",
                        sdp=str(offer["sdp"]),
                        type=str(offer["type"]),
                    ).to_payload(),
                    lease_id=session_id,
                )
                with self._lock:
                    if self._session_id != session_id:
                        raise RuntimeError("simulation session changed during WebRTC retry")
                    previous = self._receivers.get(stream)
                    self._receivers[stream] = receiver
                    self._connected_streams.discard(stream)
                    retry_count = min(
                        int(self._stream_retry_count.get(stream, 0)) + 1,
                        _STREAM_RETRY_COUNT_CAP,
                    )
                    self._stream_retry_count[stream] = retry_count
                    self._stream_offer_sent_at[stream] = now
                    # Keep trying while the DDS session is alive.  The delay
                    # is capped and the counter itself is bounded, so a dead
                    # media path cannot create an unbounded queue or timer.
                    self._stream_retry_at[stream] = now + min(
                        self.retry_s * (2.0 ** retry_count),
                        _MAX_STREAM_RETRY_DELAY_S,
                    )
                if previous is not None and previous is not receiver:
                    try:
                        previous.close()
                    except Exception:
                        pass
                receiver = None
                self._set_error(f"{stream} WebRTC negotiation retry sent")
            except Exception as exc:
                if receiver is not None:
                    try:
                        receiver.close()
                    except Exception:
                        pass
                with self._lock:
                    retry_count = min(
                        int(self._stream_retry_count.get(stream, 0)) + 1,
                        _STREAM_RETRY_COUNT_CAP,
                    )
                    self._stream_retry_count[stream] = retry_count
                    self._stream_retry_at[stream] = now + min(
                        self.retry_s * (2.0 ** retry_count),
                        _MAX_STREAM_RETRY_DELAY_S,
                    )
                self._set_error(
                    f"{stream} WebRTC retry failed: "
                    f"{str(exc).strip() or type(exc).__name__}"
                )

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
                self._schedule_open_retry_locked(
                    building="scene is still building" in reason.lower()
                )
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
        self._open_retry_count = 0
        self._turn = None
        self._connected_streams.clear()
        self._stream_retry_at.clear()
        self._stream_retry_count.clear()
        self._stream_connected_at.clear()
        self._stream_offer_sent_at.clear()
        self._status = None
        self._commands.clear()
        self._pending_command_ids.clear()
        self._sent_messages.clear()

    def _schedule_open_retry_locked(
        self,
        *,
        building: bool = False,
        now: Optional[float] = None,
    ) -> None:
        """Bound retries while Sim is doing its potentially long scene build.

        Sim advertises its DDS endpoint before Genesis is ready, so the UI can
        receive a deterministic ``scene is still building`` error for a cold
        start.  Keep ordinary transport retries quick, but back off this
        expected readiness response so a two-minute build does not generate a
        request every heartbeat.
        """

        current = self.clock() if now is None else float(now)
        if not building:
            self._open_retry_count = 0
            delay = self.retry_s
        else:
            exponent = min(self._open_retry_count, _MAX_OPEN_RETRY_EXPONENT)
            delay = min(
                self.retry_s * (2.0**exponent),
                _MAX_OPEN_RETRY_DELAY_S,
            )
            self._open_retry_count = min(
                self._open_retry_count + 1,
                _MAX_OPEN_RETRY_EXPONENT,
            )
        self._retry_after = current + delay

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
                    EndpointDescriptor(self.endpoint_id, "ui", (CAPABILITY_SIM_MOCK_HUG,)),
                    settings=self.settings,
                    trace_context_provider=current_trace_context,
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
