"""Direct protocol-v3 connection for the controller deployment."""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Mapping, Optional

from elesim_protocol import (
    CAPABILITY_OPERATOR_CONTROL,
    EndpointClient,
    EndpointDescriptor,
    Envelope,
    ProtocolError,
    SimMappingConfig,
    encode_value,
)


@dataclass(frozen=True)
class _Submission:
    kind: str
    payload: dict[str, Any]
    force: bool = False


def canonical_motion_payload(message: Mapping[str, Any]) -> dict[str, Any]:
    """Convert the controller command API to one canonical v3 motion payload."""

    command = str(message.get("t", "")).strip()
    if not command:
        raise ValueError("controller command requires t")
    body = {
        str(key): value
        for key, value in message.items()
        if key not in {"t", "ts", "seq"}
    }
    if "u" in body:
        raise ValueError("controller must resolve partial motor input to canonical q")
    q = body.get("q")
    if isinstance(q, Mapping):
        body["q"] = [
            float(q["linear_m"]),
            float(q["roll_rad"]),
            float(q["theta1_rad"]),
            float(q["theta2_rad"]),
        ]
    return {"command": command, **body}


class ControllerConnection:
    """Owns the only router socket used by the controller process.

    Workflow threads submit plain commands to a queue. The connection thread
    alone touches the ZeroMQ socket, which avoids cross-thread socket use.
    """

    def __init__(
        self,
        *,
        server_endpoint: str,
        controller_id: str,
        initial_target: str,
        mapping: SimMappingConfig,
        state_sink: Any,
        send_hz: float = 30.0,
        discover_period_s: float = 1.0,
        endpoint_factory: Callable[..., Any] = EndpointClient,
    ) -> None:
        self.server_endpoint = str(server_endpoint)
        self.controller_id = str(controller_id)
        self.desired_target = str(initial_target)
        self.mapping = mapping
        self.state_sink = state_sink
        self.send_period_s = 0.0 if send_hz <= 0.0 else 1.0 / float(send_hz)
        self.discover_period_s = max(0.1, float(discover_period_s))
        self.endpoint_factory = endpoint_factory

        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.active_target = ""
        self.lease_id = ""
        self.endpoints: list[dict[str, Any]] = []
        self.operator_handler: Optional[Callable[[dict[str, Any]], dict[str, Any]]] = None
        self.on_target_selected: Optional[Callable[[dict[str, Any]], None]] = None

        self._outbox: queue.Queue[_Submission] = queue.Queue()
        self._pending_target: Optional[dict[str, Any]] = None
        self._last_target_sent_at: Optional[float] = None
        self._last_discover_at: Optional[float] = None
        self._selection_requested = ""

    def start(self) -> None:
        if self.thread is not None and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.ready.clear()
        self.thread = threading.Thread(
            target=self._run,
            name="controller-router",
            daemon=True,
        )
        self.thread.start()
        if not self.ready.wait(timeout=3.0):
            raise RuntimeError("controller protocol connection failed to start")

    def close(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=3.0)
        self.state_sink.router_connected(False)

    def submit(self, message: Mapping[str, Any], *, force: bool = False) -> None:
        self._outbox.put(_Submission("motion", dict(message), bool(force)))

    def send_camera_input(self, command: str, values: tuple[float, ...] = ()) -> None:
        self._outbox.put(
            _Submission(
                "camera",
                {"command": str(command), "values": [float(value) for value in values]},
                True,
            )
        )

    def select_target(self, target_id: str) -> None:
        self.desired_target = str(target_id)
        self._selection_requested = ""
        self._outbox.put(_Submission("select", {"target_id": self.desired_target}, True))

    def _run(self) -> None:
        endpoint = self.endpoint_factory(
            self.server_endpoint,
            EndpointDescriptor(
                self.controller_id,
                "controller",
                (CAPABILITY_OPERATOR_CONTROL,),
            ),
        )
        self.ready.set()
        try:
            while not self.stop_event.is_set():
                endpoint.heartbeat()
                try:
                    messages = tuple(endpoint.receive(timeout_ms=20))
                except (ProtocolError, ValueError) as exc:
                    self.state_sink.accept_error(f"protocol receive failed: {exc}")
                    messages = ()
                self.state_sink.router_connected(bool(endpoint.server_alive()))
                for message in messages:
                    self.handle_envelope(endpoint, message)
                now = time.monotonic()
                self.drain_outbox(endpoint, now=now)
                self.flush_target(endpoint, now=now)
                if endpoint.registered and (
                    self._last_discover_at is None
                    or now - self._last_discover_at >= self.discover_period_s
                ):
                    endpoint.send("discover", payload={})
                    self._last_discover_at = now
        finally:
            endpoint.close()

    def handle_envelope(self, endpoint: Any, message: Envelope) -> None:
        payload = dict(message.payload or {})
        message_type = message.message_type
        if message_type == "registered":
            self.state_sink.router_connected(True)
            endpoint.send("discover", payload={})
            self._last_discover_at = time.monotonic()
            return
        if message_type == "endpoint_list":
            raw_endpoints = payload.get("endpoints", [])
            self.endpoints = [dict(value) for value in raw_endpoints if isinstance(value, Mapping)]
            self._request_desired_target(endpoint)
            return
        if message_type == "target_selected":
            self.active_target = str(payload.get("target_id", ""))
            self.lease_id = str(payload.get("lease_id", ""))
            self._selection_requested = self.active_target
            self.state_sink.target_changed(self.active_target)
            descriptor = next(
                (value for value in self.endpoints if value.get("endpoint_id") == self.active_target),
                None,
            )
            if descriptor is not None and self.on_target_selected is not None:
                self.on_target_selected(dict(descriptor))
            return
        if message_type in {"target_lost", "target_released"}:
            self.active_target = ""
            self.lease_id = ""
            self._selection_requested = ""
            self.state_sink.target_changed("")
            endpoint.send("discover", payload={})
            self._last_discover_at = time.monotonic()
            return
        if message_type == "telemetry":
            if not self.active_target or message.source_id == self.active_target:
                self.state_sink.accept_telemetry(payload)
            return
        if message_type == "ack":
            self.state_sink.accept_ack(payload)
            return
        if message_type == "error":
            self.state_sink.accept_error(str(payload.get("reason", "router error")))
            return
        if message_type == "operator_intent":
            request_id = str(payload.get("request_id", ""))
            if self.operator_handler is None:
                result = {
                    "request_id": request_id,
                    "ok": False,
                    "error": "controller_not_ready",
                }
            else:
                try:
                    result = self.operator_handler(payload)
                except Exception as exc:
                    reason = f"operator handler failed: {exc}"
                    self.state_sink.accept_error(reason)
                    result = {"request_id": request_id, "ok": False, "error": reason}
            self._send_operator_result(endpoint, message, request_id=request_id, result=result)

    def _send_operator_result(
        self,
        endpoint: Any,
        message: Envelope,
        *,
        request_id: str,
        result: Mapping[str, Any],
    ) -> None:
        try:
            wire_result = encode_value(dict(result))
            if not isinstance(wire_result, dict):
                raise ProtocolError("operator result must be an object")
        except (ProtocolError, TypeError, ValueError) as exc:
            reason = f"invalid operator result: {exc}"
            self.state_sink.accept_error(reason)
            wire_result = {"request_id": request_id, "ok": False, "error": reason}
        try:
            endpoint.send(
                "operator_result",
                target_id=message.source_id,
                payload=wire_result,
                trace_context=message.trace_context,
            )
        except (ProtocolError, TypeError, ValueError) as exc:
            self.state_sink.accept_error(f"operator result send failed: {exc}")

    def _request_desired_target(self, endpoint: Any) -> None:
        if not self.desired_target or self.active_target == self.desired_target:
            return
        if self._selection_requested == self.desired_target:
            return
        if not any(value.get("endpoint_id") == self.desired_target for value in self.endpoints):
            return
        endpoint.send("select_target", payload={"target_id": self.desired_target})
        self._selection_requested = self.desired_target

    def drain_outbox(self, endpoint: Any, *, now: float) -> None:
        while True:
            try:
                submission = self._outbox.get_nowait()
            except queue.Empty:
                return
            if submission.kind == "select":
                self._selection_requested = ""
                self._request_desired_target(endpoint)
                continue
            if submission.kind == "camera":
                if self.active_target and self.lease_id:
                    endpoint.send(
                        "camera_input",
                        target_id=self.active_target,
                        payload=submission.payload,
                        lease_id=self.lease_id,
                    )
                continue
            payload = canonical_motion_payload(submission.payload)
            if payload["command"] == "target" and not submission.force:
                self._pending_target = payload
                continue
            self._send_motion(endpoint, payload, allow_without_lease=payload["command"] == "estop")
            if payload["command"] == "target":
                self._last_target_sent_at = float(now)

    def flush_target(self, endpoint: Any, *, now: float) -> None:
        if self._pending_target is None:
            return
        if self._last_target_sent_at is not None and now - self._last_target_sent_at < self.send_period_s:
            return
        if not self.active_target or not self.lease_id:
            return
        payload = self._pending_target
        self._pending_target = None
        self._send_motion(endpoint, payload)
        self._last_target_sent_at = float(now)

    def _send_motion(
        self,
        endpoint: Any,
        payload: dict[str, Any],
        *,
        allow_without_lease: bool = False,
    ) -> None:
        if not self.active_target:
            self.state_sink.accept_error("no_active_target")
            return
        if not self.lease_id and not allow_without_lease:
            self.state_sink.accept_error("no_active_lease")
            return
        endpoint.send(
            "motion_command",
            target_id=self.active_target,
            payload=payload,
            lease_id=self.lease_id,
        )


__all__ = ["ControllerConnection", "canonical_motion_payload"]
