"""Protocol-v3 endpoint owned by the simulator deployment."""

from __future__ import annotations

import threading
from typing import Any, Callable, Mapping, Optional

from elesim_protocol import (
    CAPABILITY_MOTION_ARM,
    CAPABILITY_MOTION_GO2,
    CAPABILITY_STREAM_RENDERED,
    CAPABILITY_STREAM_RGBD,
    EndpointClient,
    EndpointDescriptor,
    Envelope,
)

from .control_state import SimulationStateSource


class SimulatorEndpoint:
    """Owns router lifecycle while Genesis communicates through memory only."""

    def __init__(
        self,
        *,
        server_endpoint: str,
        endpoint_id: str,
        state: SimulationStateSource,
        streams: Mapping[str, str],
        camera_input_handler: Optional[Callable[[str, object], None]] = None,
        webrtc_offer_handler: Optional[Callable[[str, str], Mapping[str, Any]]] = None,
        endpoint_factory: Callable[..., Any] = EndpointClient,
    ) -> None:
        self.server_endpoint = str(server_endpoint)
        self.endpoint_id = str(endpoint_id)
        self.state = state
        self.streams = dict(streams)
        self.camera_input_handler = camera_input_handler
        self.webrtc_offer_handler = webrtc_offer_handler
        self.endpoint_factory = endpoint_factory
        self.controller_id = ""
        self.active_lease = ""
        self.last_control_seq = -1
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self._telemetry_lock = threading.Lock()
        self._telemetry: dict[str, Any] = {}
        self._telemetry_dirty = False

    def start(self) -> None:
        self.stop_event.clear()
        self.ready.clear()
        self.thread = threading.Thread(target=self._run, name="simulator-router", daemon=True)
        self.thread.start()
        if not self.ready.wait(timeout=3.0):
            raise RuntimeError("simulator protocol endpoint failed to start")

    def close(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=3.0)

    def grant_lease(self, controller_id: str, lease_id: str) -> None:
        self.state.revoke_control()
        self.controller_id = str(controller_id)
        self.active_lease = str(lease_id)
        self.last_control_seq = -1

    def revoke_lease(self) -> None:
        self.state.revoke_control()
        self.controller_id = ""
        self.active_lease = ""
        self.last_control_seq = -1

    def publish_telemetry(self, payload: Mapping[str, Any]) -> None:
        with self._telemetry_lock:
            self._telemetry.update(dict(payload))
            self._telemetry_dirty = True

    def flush_telemetry(self, client: Any) -> None:
        if not self.controller_id or not self.active_lease:
            return
        with self._telemetry_lock:
            if not self._telemetry_dirty:
                return
            payload = dict(self._telemetry)
            self._telemetry_dirty = False
        client.send(
            "telemetry",
            target_id=self.controller_id,
            payload=payload,
            lease_id=self.active_lease,
        )

    def handle_envelope(self, client: Any, message: Envelope) -> None:
        if message.message_type == "lease_granted":
            self.grant_lease(
                str((message.payload or {}).get("controller_id", "")),
                message.lease_id,
            )
            return
        if message.message_type == "lease_revoked":
            self.revoke_lease()
            return
        if message.message_type == "motion_command":
            ok, reason = self._apply_motion(message)
            client.send(
                "ack",
                target_id=message.source_id,
                payload={"reply_to": message.message_id, "ok": ok, "reason": reason},
                lease_id=message.lease_id,
                trace_context=message.trace_context,
            )
            return
        if message.message_type == "camera_input":
            ok, _reason = self._validate_control(message)
            if ok and self.camera_input_handler is not None:
                payload = message.payload or {}
                self.camera_input_handler(str(payload.get("command", "")), payload.get("values", ()))
            return
        if message.message_type == "webrtc_signal" and self.webrtc_offer_handler is not None:
            payload = message.payload or {}
            if str(payload.get("signal", "")) == "offer":
                answer = self.webrtc_offer_handler(
                    str(payload.get("sdp", "")),
                    str(payload.get("type", "offer")),
                )
                client.send(
                    "webrtc_signal",
                    target_id=message.source_id,
                    payload={"signal": "answer", **dict(answer)},
                    trace_context=message.trace_context,
                )

    def _validate_control(self, message: Envelope, *, allow_estop: bool = False) -> tuple[bool, str]:
        if allow_estop:
            return True, "accepted"
        if message.source_id != self.controller_id or message.lease_id != self.active_lease:
            return False, "lease_mismatch"
        if message.seq <= self.last_control_seq:
            return False, "stale_sequence"
        self.last_control_seq = message.seq
        return True, "accepted"

    def _apply_motion(self, message: Envelope) -> tuple[bool, str]:
        payload = dict(message.payload or {})
        command = str(payload.get("command", ""))
        ok, reason = self._validate_control(message, allow_estop=command == "estop")
        if not ok:
            return ok, reason
        try:
            return True, self.state.apply_command(payload)
        except ValueError as exc:
            reason = str(exc)
            return False, reason if reason else "invalid_command"

    def _run(self) -> None:
        client = self.endpoint_factory(
            self.server_endpoint,
            EndpointDescriptor(
                self.endpoint_id,
                "simulator",
                (
                    CAPABILITY_MOTION_ARM,
                    CAPABILITY_MOTION_GO2,
                    CAPABILITY_STREAM_RGBD,
                    CAPABILITY_STREAM_RENDERED,
                ),
                streams=self.streams,
            ),
        )
        self.ready.set()
        was_registered = client.registered
        try:
            while not self.stop_event.is_set():
                client.heartbeat()
                for message in client.receive(timeout_ms=20):
                    self.handle_envelope(client, message)
                if was_registered and not client.registered:
                    self.revoke_lease()
                was_registered = client.registered
                self.flush_telemetry(client)
        finally:
            self.revoke_lease()
            client.close()


__all__ = ["SimulatorEndpoint"]
