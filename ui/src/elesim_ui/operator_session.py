"""Non-blocking operator request pump owned by the UI deployment."""

from __future__ import annotations

import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass
from typing import Any, Callable, Optional

from elesim_protocol import (
    DdsTransportError,
    DdsRuntimeSettings,
    EndpointDescriptor,
    OPERATOR_OPERATIONS,
    OperatorViewSnapshot,
    PeerClient,
    decode_value,
    encode_value,
)


ResultCallback = Callable[[Any], None]
ErrorCallback = Callable[[str], None]

_COALESCED_SERVICE_CALLS = frozenset(
    {
        "apply_partial_control_u",
        "send_go2_velocity",
        "update_gaze_stabilizer_config",
    }
)
# Keep one burst of UI requests from monopolising the DDS pump.  The next
# cycle sends the remaining bounded requests after heartbeat/receive have had
# a chance to run again.
_MAX_OUTBOX_FLUSH_PER_CYCLE = 32


@dataclass(frozen=True)
class OperatorStatus:
    dds_online: bool
    pilot_online: bool
    pending_count: int
    last_error: str
    last_snapshot_age_s: float


@dataclass
class _Request:
    request_id: str
    operation: str
    name: str
    args: tuple[Any, ...]
    kwargs: dict[str, Any]
    created_at: float
    timeout_s: float
    on_result: Optional[ResultCallback] = None
    on_error: Optional[ErrorCallback] = None
    sent_at: Optional[float] = None
    message_id: str = ""


class OperatorSession:
    """Own the UI operator request pump and expose thread-safe read caches."""

    def __init__(
        self,
        *,
        ui_id: str,
        pilot_id: str,
        request_timeout_s: float = 1.5,
        snapshot_period_s: float = 0.10,
        pilot_stale_s: float = 1.0,
        clock: Callable[[], float] = time.monotonic,
        settings: Optional[DdsRuntimeSettings] = None,
        peer: Any = None,
        peer_factory: Callable[..., Any] = PeerClient,
        max_pending: int = 256,
        autostart: bool = True,
    ) -> None:
        self.ui_id = str(ui_id)
        self.pilot_id = str(pilot_id)
        self.request_timeout_s = max(0.05, float(request_timeout_s))
        self.snapshot_period_s = max(0.02, float(snapshot_period_s))
        self.pilot_stale_s = max(self.snapshot_period_s * 2.0, float(pilot_stale_s))
        self.clock = clock
        self.settings = settings
        self.peer = peer
        self.peer_factory = peer_factory
        self.max_pending = max(16, int(max_pending))

        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._requests: dict[str, _Request] = {}
        self._outbox: deque[str] = deque()
        self._message_to_request: dict[str, str] = {}
        self._callbacks: deque[tuple[Callable[[Any], None], Any]] = deque()
        self._state: dict[str, Any] = {}
        self._service: dict[str, Any] = {}
        self._snapshot_request_id = ""
        self._last_snapshot_requested_at: Optional[float] = None
        self._last_snapshot_at: Optional[float] = None
        self._dds_online = False
        self._last_error = ""
        self._last_error_log = ""
        self._last_error_log_at = 0.0
        if autostart:
            self.start()

    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._run,
                name="ui-operator",
                daemon=True,
            )
            self._thread.start()

    def close(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=3.0)

    def seed_state(self, values: dict[str, Any]) -> None:
        with self._lock:
            self._state.update(values)

    def state_value(self, name: str, default: Any = None) -> Any:
        with self._lock:
            return self._state.get(str(name), default)

    def service_value(self, name: str, default: Any = None) -> Any:
        with self._lock:
            return self._service.get(str(name), default)

    def dispatch_callbacks(self, *, limit: int = 32) -> None:
        """Run completed request callbacks on the caller's UI thread."""
        for _ in range(max(0, int(limit))):
            with self._lock:
                if not self._callbacks:
                    return
                callback, value = self._callbacks.popleft()
            callback(value)

    @property
    def pending_count(self) -> int:
        with self._lock:
            return len(self._requests)

    @property
    def status(self) -> OperatorStatus:
        now = self.clock()
        with self._lock:
            age = (
                float("inf")
                if self._last_snapshot_at is None
                else max(0.0, now - self._last_snapshot_at)
            )
            return OperatorStatus(
                dds_online=self._dds_online,
                pilot_online=age <= self.pilot_stale_s,
                pending_count=len(self._requests),
                last_error=self._last_error,
                last_snapshot_age_s=age,
            )

    def submit(
        self,
        operation: str,
        name: str = "",
        *args: Any,
        on_result: Optional[ResultCallback] = None,
        on_error: Optional[ErrorCallback] = None,
        request_timeout_s: Optional[float] = None,
        **kwargs: Any,
    ) -> str:
        operation = str(operation)
        if operation not in OPERATOR_OPERATIONS:
            raise ValueError(f"unsupported operator operation: {operation}")
        request = _Request(
            request_id=uuid.uuid4().hex,
            operation=operation,
            name=str(name),
            args=tuple(args),
            kwargs=dict(kwargs),
            created_at=self.clock(),
            timeout_s=(
                self.request_timeout_s
                if request_timeout_s is None
                else max(0.05, float(request_timeout_s))
            ),
            on_result=on_result,
            on_error=on_error,
        )
        with self._lock:
            key = self._coalesce_key(request)
            if key is not None:
                for prior_id, prior in tuple(self._requests.items()):
                    if prior.sent_at is None and self._coalesce_key(prior) == key:
                        self._requests.pop(prior_id, None)
            if len(self._requests) >= self.max_pending:
                message = f"operator request queue is full ({self.max_pending})"
                self._record_error(message)
                if on_error is not None:
                    self._enqueue_callback(on_error, message)
                return request.request_id
            self._requests[request.request_id] = request
            self._outbox.append(request.request_id)
        return request.request_id

    @staticmethod
    def _coalesce_key(request: _Request) -> tuple[str, str] | None:
        if request.operation == "state_set":
            return request.operation, request.name
        if (
            request.operation == "service_call"
            and request.name in _COALESCED_SERVICE_CALLS
        ):
            return request.operation, request.name
        return None

    def request_snapshot(self) -> str:
        with self._lock:
            current = self._snapshot_request_id
            if current and current in self._requests:
                return current
        request_id = self.submit("view_snapshot")
        with self._lock:
            self._snapshot_request_id = request_id
            self._last_snapshot_requested_at = self.clock()
        return request_id

    def run_cycle(self, endpoint: Any, *, now: Optional[float] = None) -> None:
        current = self.clock() if now is None else float(now)
        try:
            endpoint.heartbeat()
        except DdsTransportError as exc:
            # A failed heartbeat must not leave the UI presenting the last
            # successful DDS state as online while requests wait indefinitely.
            self._dds_online = False
            self._record_error(f"operator transport failed: {exc}")
            raise
        self._dds_online = bool(endpoint.registered)
        # Keep operator intents responsive independently of the 10 Hz view
        # snapshot cadence.  Slider values are latest-only, so the shorter
        # wait reduces input latency without creating a growing queue.
        for message in endpoint.receive(timeout_ms=10):
            self._handle_message(message, now=current)
        self._expire_requests(now=current)
        self._schedule_snapshot(now=current)
        if endpoint.registered:
            self._flush_outbox(endpoint, now=current)

    def _run(self) -> None:
        endpoint = self.peer
        owns_endpoint = endpoint is None
        try:
            if endpoint is None:
                endpoint = self.peer_factory(
                    EndpointDescriptor(self.ui_id, "ui", ()),
                    settings=self.settings,
                )
            while not self._stop.is_set():
                try:
                    self.run_cycle(endpoint)
                except Exception as exc:
                    self._record_error(f"operator transport failed: {exc}")
                    self._stop.wait(0.10)
        except Exception as exc:
            self._record_error(f"operator transport unavailable: {exc}")
        finally:
            if endpoint is not None and owns_endpoint:
                endpoint.close()

    def _schedule_snapshot(self, *, now: float) -> None:
        with self._lock:
            if self._snapshot_request_id in self._requests:
                return
            last = self._last_snapshot_requested_at
            due = last is None or now - last >= self.snapshot_period_s
        if due:
            self.request_snapshot()

    def _flush_outbox(self, endpoint: Any, *, now: float) -> None:
        for _ in range(_MAX_OUTBOX_FLUSH_PER_CYCLE):
            with self._lock:
                if not self._outbox:
                    return
                request_id = self._outbox.popleft()
                request = self._requests.get(request_id)
            if request is None or request.sent_at is not None:
                continue
            payload = {
                "request_id": request.request_id,
                "operation": request.operation,
                "name": request.name,
                "args": [encode_value(value) for value in request.args],
                "kwargs": {
                    str(key): encode_value(value)
                    for key, value in request.kwargs.items()
                },
            }
            try:
                envelope = endpoint.send(
                    "operator_intent",
                    target_id=self.pilot_id,
                    payload=payload,
                )
            except Exception:
                # Discovery can report a graph as ready immediately before the
                # selected Pilot boot disappears. Preserve the unsent
                # request so the next cycle can retry it or expire it normally.
                with self._lock:
                    live = self._requests.get(request_id)
                    if live is not None and live.sent_at is None:
                        self._outbox.appendleft(request_id)
                raise
            with self._lock:
                live = self._requests.get(request_id)
                if live is not None:
                    live.sent_at = now
                    live.message_id = str(envelope.message_id)
                    self._message_to_request[live.message_id] = request_id

    def _handle_message(self, message: Any, *, now: float) -> None:
        payload = dict(message.payload or {})
        if message.message_type == "operator_result":
            self._complete(
                str(payload.get("request_id", "")),
                ok=bool(payload.get("ok", False)),
                result=decode_value(payload.get("result")),
                error=str(payload.get("error", "operator request failed")),
                now=now,
            )
        elif message.message_type == "error":
            reply_to = str(payload.get("reply_to", ""))
            with self._lock:
                request_id = self._message_to_request.get(reply_to, "")
            if request_id:
                self._complete(
                    request_id,
                    ok=False,
                    error=str(payload.get("reason", "DDS peer rejected operator request")),
                    now=now,
                )

    def _complete(
        self,
        request_id: str,
        *,
        ok: bool,
        result: Any = None,
        error: str = "",
        now: float,
    ) -> None:
        with self._lock:
            request = self._requests.pop(request_id, None)
            if request is None:
                return
            if request.message_id:
                self._message_to_request.pop(request.message_id, None)
            if request_id == self._snapshot_request_id:
                self._snapshot_request_id = ""
        if not ok:
            message = error or "operator request failed"
            self._record_error(message)
            if request.on_error is not None:
                self._enqueue_callback(request.on_error, message)
            return
        try:
            if request.operation == "view_snapshot":
                view = OperatorViewSnapshot.from_payload(result)
                with self._lock:
                    self._state = dict(view.state)
                    self._service = dict(view.service)
                    self._last_snapshot_at = now
                    self._last_error = ""
            elif request.operation == "state_set":
                with self._lock:
                    self._state[request.name] = decode_value(request.kwargs.get("value"))
            else:
                with self._lock:
                    self._last_snapshot_requested_at = None
            if request.on_result is not None:
                self._enqueue_callback(request.on_result, result)
        except Exception as exc:
            message = f"operator response invalid: {exc}"
            self._record_error(message)
            if request.on_error is not None:
                self._enqueue_callback(request.on_error, message)

    def _expire_requests(self, *, now: float) -> None:
        with self._lock:
            expired = [
                request_id
                for request_id, request in self._requests.items()
                if now - request.created_at >= request.timeout_s
            ]
        for request_id in expired:
            self._complete(
                request_id,
                ok=False,
                error=f"operator request timed out: {request_id}",
                now=now,
            )

    def _record_error(self, message: str) -> None:
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

    def _enqueue_callback(
        self,
        callback: Callable[[Any], None],
        value: Any,
    ) -> None:
        """Bound callbacks so a slow UI callback cannot create a backlog."""

        with self._lock:
            if len(self._callbacks) >= self.max_pending:
                self._callbacks.popleft()
            self._callbacks.append((callback, value))
