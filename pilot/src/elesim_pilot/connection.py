"""Direct protocol-v6 DDS connection for the pilot deployment."""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Mapping, Optional

from elesim_protocol import (
    CAPABILITY_OPERATOR_CONTROL,
    DdsRuntimeSettings,
    DdsTransportError,
    EndpointDescriptor,
    Envelope,
    PeerClient,
    ProtocolError,
    SimMappingConfig,
    SimulationStatusPayload,
    encode_value,
)


@dataclass(frozen=True)
class _Submission:
    kind: str
    payload: dict[str, Any]
    force: bool = False


def canonical_motion_payload(message: Mapping[str, Any]) -> dict[str, Any]:
    """Convert the pilot command API to one canonical motion payload."""

    command = str(message.get("t", "")).strip()
    if not command:
        raise ValueError("pilot command requires t")
    body = {
        str(key): value
        for key, value in message.items()
        if key not in {"t", "ts", "seq"}
    }
    if "u" in body:
        raise ValueError("pilot must resolve partial motor input to canonical q")
    q = body.get("q")
    if isinstance(q, Mapping):
        body["q"] = [
            float(q["linear_m"]),
            float(q["roll_rad"]),
            float(q["theta1_rad"]),
            float(q["theta2_rad"]),
        ]
    return {"command": command, **body}


class PilotConnection:
    """Owns the Pilot's DDS participant and direct peer traffic.

    Workflow threads submit plain commands to a queue. The connection thread
    alone spins the ROS executor, which keeps rclpy lifecycle and callbacks on
    one thread.
    """

    def __init__(
        self,
        *,
        pilot_id: str,
        initial_target: str,
        mapping: SimMappingConfig,
        state_sink: Any,
        dds_settings: Optional[DdsRuntimeSettings] = None,
        send_hz: float = 30.0,
        discover_period_s: float = 1.0,
        endpoint_factory: Callable[..., Any] = PeerClient,
    ) -> None:
        self.pilot_id = str(pilot_id)
        self.desired_target = str(initial_target)
        self.mapping = mapping
        self.state_sink = state_sink
        self.dds_settings = dds_settings
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
        self.simulation_status_handler: Optional[
            Callable[[SimulationStatusPayload], None]
        ] = None

        self._outbox: queue.Queue[_Submission] = queue.Queue()
        self._pending_target: Optional[dict[str, Any]] = None
        self._last_target_sent_at: Optional[float] = None
        self._last_discover_at: Optional[float] = None
        self._selection_requested = ""
        self._selection_requested_at: Optional[float] = None

    def start(self) -> None:
        if self.thread is not None and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.ready.clear()
        self.thread = threading.Thread(
            target=self._run,
            name="pilot-dds",
            daemon=True,
        )
        self.thread.start()
        if not self.ready.wait(timeout=3.0):
            raise RuntimeError("pilot protocol connection failed to start")

    def close(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=3.0)
        self.state_sink.peer_connected(False)

    def submit(self, message: Mapping[str, Any], *, force: bool = False) -> None:
        self._outbox.put(_Submission("motion", dict(message), bool(force)))

    def select_target(self, target_id: str) -> None:
        self.desired_target = str(target_id)
        self._selection_requested = ""
        self._selection_requested_at = None
        self._outbox.put(_Submission("select", {"target_id": self.desired_target}, True))

    def _run(self) -> None:
        endpoint = self.endpoint_factory(
            EndpointDescriptor(
                self.pilot_id,
                "pilot",
                (CAPABILITY_OPERATOR_CONTROL,),
            ),
            settings=self.dds_settings,
        )
        self.ready.set()
        try:
            while not self.stop_event.is_set():
                try:
                    endpoint.heartbeat()
                    messages = tuple(endpoint.receive(timeout_ms=20))
                    self.state_sink.peer_connected(bool(endpoint.registered))
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
                except DdsTransportError as exc:
                    self.state_sink.peer_connected(False)
                    self.state_sink.accept_error(f"DDS peer transport failed: {exc}")
                    self.stop_event.wait(0.1)
                except (ProtocolError, ValueError) as exc:
                    self.state_sink.accept_error(f"protocol receive failed: {exc}")
        finally:
            endpoint.close()

    def handle_envelope(self, endpoint: Any, message: Envelope) -> None:
        payload = dict(message.payload or {})
        message_type = message.message_type
        if message_type == "endpoint_list":
            raw_endpoints = payload.get("endpoints", [])
            self.endpoints = [dict(value) for value in raw_endpoints if isinstance(value, Mapping)]
            self._request_desired_target(endpoint)
            return
        if message_type == "target_selected":
            self.active_target = str(payload.get("target_id", ""))
            self.lease_id = str(payload.get("lease_id", ""))
            self._selection_requested = self.active_target
            self._selection_requested_at = None
            self.state_sink.target_changed(self.active_target)
            descriptor = next(
                (value for value in self.endpoints if value.get("endpoint_id") == self.active_target),
                None,
            )
            if descriptor is not None and self.on_target_selected is not None:
                try:
                    self.on_target_selected(dict(descriptor))
                except (KeyError, ProtocolError, TypeError, ValueError) as exc:
                    self.state_sink.accept_error(
                        f"target stream configuration failed: {exc}"
                    )
            return
        if message_type in {"target_lost", "target_released"}:
            self.active_target = ""
            self.lease_id = ""
            self._selection_requested = ""
            self._selection_requested_at = None
            self.state_sink.target_changed("")
            endpoint.send("discover", payload={})
            self._last_discover_at = time.monotonic()
            return
        if message_type == "telemetry":
            if not self.active_target or message.source_id == self.active_target:
                self.state_sink.accept_telemetry(payload)
            return
        if message_type == "simulation_status":
            if self.active_target and message.source_id != self.active_target:
                return
            try:
                status = SimulationStatusPayload.from_payload(payload)
            except ProtocolError as exc:
                self.state_sink.accept_error(f"invalid simulation status: {exc}")
                return
            if self.simulation_status_handler is not None:
                self.simulation_status_handler(status)
            return
        if message_type == "ack":
            self.state_sink.accept_ack(payload)
            return
        if message_type == "error":
            self.state_sink.accept_error(str(payload.get("reason", "peer error")))
            return
        if message_type == "operator_intent":
            request_id = str(payload.get("request_id", ""))
            if self.operator_handler is None:
                result = {
                    "request_id": request_id,
                    "ok": False,
                    "error": "pilot_not_ready",
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
        except (DdsTransportError, ProtocolError, TypeError, ValueError) as exc:
            self.state_sink.accept_error(f"operator result send failed: {exc}")

    def _request_desired_target(
        self,
        endpoint: Any,
        *,
        now: Optional[float] = None,
    ) -> None:
        if not self.desired_target or self.active_target == self.desired_target:
            return
        if not any(value.get("endpoint_id") == self.desired_target for value in self.endpoints):
            return
        current = time.monotonic() if now is None else float(now)
        if (
            self._selection_requested == self.desired_target
            and self._selection_requested_at is not None
            and current - self._selection_requested_at < self.discover_period_s
        ):
            return
        endpoint.send("select_target", payload={"target_id": self.desired_target})
        self._selection_requested = self.desired_target
        self._selection_requested_at = current

    def drain_outbox(self, endpoint: Any, *, now: float) -> None:
        while True:
            try:
                submission = self._outbox.get_nowait()
            except queue.Empty:
                return
            if submission.kind == "select":
                self._selection_requested = ""
                self._selection_requested_at = None
                self._request_desired_target(endpoint, now=now)
                continue
            payload = canonical_motion_payload(submission.payload)
            if payload["command"] == "target" and not submission.force:
                self._pending_target = payload
                continue
            try:
                self._send_motion(
                    endpoint,
                    payload,
                    allow_without_lease=payload["command"] == "estop",
                )
            except DdsTransportError:
                self._outbox.put(submission)
                raise
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
        self._send_motion(endpoint, payload)
        self._pending_target = None
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


__all__ = ["PilotConnection", "canonical_motion_payload"]
