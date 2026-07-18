"""UI-owned direct WebRTC receiver with ZMQ signaling through the server."""

from __future__ import annotations

import threading
from typing import Optional

from engine.core.distributed import EndpointClient
from engine.core.protocol import EndpointDescriptor
from engine.vision.webrtc import WebRtcVideoReceiver


class UiWebRtcSession:
    def __init__(self, server_endpoint: str, *, ui_id: str, sim_id: str) -> None:
        self.server_endpoint = str(server_endpoint)
        self.ui_id = str(ui_id)
        self.sim_id = str(sim_id)
        self.receiver: Optional[WebRtcVideoReceiver] = None
        self._lock = threading.Lock()
        self._requested_sim_id = self.sim_id
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name="ui-webrtc-signaling", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def _run(self) -> None:
        client = EndpointClient(
            self.server_endpoint,
            EndpointDescriptor(self.ui_id, "ui", ("rendered_view",)),
        )
        registered = False
        active_sim_id = ""
        try:
            while not self.stop_event.is_set():
                client.heartbeat()
                for message in client.receive(timeout_ms=100):
                    if message.message_type == "registered":
                        registered = True
                    elif message.message_type == "webrtc_signal":
                        payload = message.payload or {}
                        if str(payload.get("signal", "")) == "answer":
                            with self._lock:
                                receiver = self.receiver
                            if receiver is not None and message.source_id == active_sim_id:
                                receiver.accept_answer(
                                    str(payload.get("sdp", "")),
                                    str(payload.get("type", "answer")),
                                )
                with self._lock:
                    requested = self._requested_sim_id
                if registered and requested and requested != active_sim_id:
                    replacement = WebRtcVideoReceiver()
                    offer = replacement.create_offer()
                    with self._lock:
                        previous = self.receiver
                        self.receiver = replacement
                    if previous is not None:
                        previous.close()
                    active_sim_id = requested
                    client.send(
                        "webrtc_signal",
                        target_id=active_sim_id,
                        payload={"signal": "offer", **offer},
                    )
        finally:
            client.close()

    def frame(self):
        with self._lock:
            receiver = self.receiver
        return None if receiver is None else receiver.latest_bgr

    def switch_target(self, sim_id: str) -> None:
        with self._lock:
            self._requested_sim_id = str(sim_id)

    def close(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=3.0)
        with self._lock:
            receiver = self.receiver
            self.receiver = None
        if receiver is not None:
            receiver.close()
