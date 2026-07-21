"""UI-owned WebRTC receiver and router signaling lifecycle."""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Optional

from elesim_protocol import CAPABILITY_STREAM_RENDERED, EndpointClient, EndpointDescriptor
from elesim_ui.webrtc import WebRtcVideoReceiver


class UiWebRtcSession:
    def __init__(
        self,
        server_endpoint: str,
        *,
        ui_id: str,
        sim_id: str,
        endpoint_factory: Callable[..., Any] = EndpointClient,
        receiver_factory: Callable[[], Any] = WebRtcVideoReceiver,
        retry_s: float = 0.5,
        poll_ms: int = 100,
    ) -> None:
        self.server_endpoint = str(server_endpoint)
        self.ui_id = str(ui_id)
        self.sim_id = str(sim_id)
        self.endpoint_factory = endpoint_factory
        self.receiver_factory = receiver_factory
        self.retry_s = max(0.05, float(retry_s))
        self.poll_ms = max(0, int(poll_ms))

        self.receiver: Optional[Any] = None
        self._lock = threading.RLock()
        self._requested_sim_id = self.sim_id
        self._active_sim_id = ""
        self._retry_after = 0.0
        self._last_error = ""
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None

    @property
    def active_sim_id(self) -> str:
        with self._lock:
            return self._active_sim_id

    @property
    def last_error(self) -> str:
        with self._lock:
            return self._last_error

    def start(self) -> None:
        with self._lock:
            if self.thread is not None and self.thread.is_alive():
                return
            self.stop_event.clear()
            self.thread = threading.Thread(
                target=self._run,
                name="ui-webrtc-signaling",
                daemon=True,
            )
            self.thread.start()

    def _run(self) -> None:
        client = None
        try:
            client = self.endpoint_factory(
                self.server_endpoint,
                EndpointDescriptor(
                    f"{self.ui_id}-video",
                    "ui",
                    (CAPABILITY_STREAM_RENDERED,),
                ),
            )
            while not self.stop_event.is_set():
                try:
                    self.run_cycle(client)
                except Exception as exc:
                    self._set_error(f"WebRTC signaling failed: {exc}")
                    self.stop_event.wait(self.retry_s)
        except Exception as exc:
            self._set_error(f"WebRTC signaling unavailable: {exc}")
        finally:
            if client is not None:
                client.close()

    def run_cycle(self, client: Any) -> None:
        client.heartbeat()
        for message in client.receive(timeout_ms=self.poll_ms):
            if message.message_type != "webrtc_signal":
                continue
            payload = message.payload or {}
            if str(payload.get("signal", "")) != "answer":
                continue
            with self._lock:
                receiver = self.receiver
                active = self._active_sim_id
            if receiver is not None and message.source_id == active:
                receiver.accept_answer(
                    str(payload.get("sdp", "")),
                    str(payload.get("type", "answer")),
                )

        with self._lock:
            requested = self._requested_sim_id
            active = self._active_sim_id
            retry_after = self._retry_after
        if not bool(client.registered) or requested == active:
            return

        now = time.monotonic()
        if now < retry_after:
            return
        if not requested:
            previous = self._replace_receiver(None, active_sim_id="")
            if previous is not None:
                previous.close()
            return

        replacement = None
        try:
            replacement = self.receiver_factory()
            offer = replacement.create_offer()
            client.send(
                "webrtc_signal",
                target_id=requested,
                payload={"signal": "offer", **offer},
            )
        except Exception as exc:
            if replacement is not None:
                replacement.close()
            with self._lock:
                self._retry_after = now + self.retry_s
            self._set_error(f"WebRTC offer failed for {requested}: {exc}")
            return

        previous = self._replace_receiver(replacement, active_sim_id=requested)
        if previous is not None:
            previous.close()
        with self._lock:
            self._retry_after = 0.0
            self._last_error = ""

    def _replace_receiver(self, replacement: Any, *, active_sim_id: str) -> Any:
        with self._lock:
            previous = self.receiver
            self.receiver = replacement
            self._active_sim_id = str(active_sim_id)
        return previous

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._last_error = str(message)

    def frame(self):
        with self._lock:
            receiver = self.receiver
        return None if receiver is None else receiver.latest_bgr

    def switch_target(self, sim_id: str) -> None:
        with self._lock:
            self._requested_sim_id = str(sim_id)
            self._retry_after = 0.0

    def close(self) -> None:
        self.stop_event.set()
        with self._lock:
            thread = self.thread
        if thread is not None and thread is not threading.current_thread() and thread.is_alive():
            thread.join(timeout=3.0)
        receiver = self._replace_receiver(None, active_sim_id="")
        if receiver is not None:
            receiver.close()
