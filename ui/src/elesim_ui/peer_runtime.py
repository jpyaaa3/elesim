"""One DDS peer and boot identity shared by all UI subsystems."""

from __future__ import annotations

import threading
from collections import deque
from typing import Any, Callable, Iterator, Mapping, Optional

from elesim_protocol import (
    DdsRuntimeSettings,
    DdsTransportError,
    EndpointDescriptor,
    Envelope,
    PeerClient,
)


_SIMULATION_MESSAGES = frozenset(
    {
        "simulation_session_opened",
        "simulation_session_revoked",
        "simulation_status",
        "simulation_result",
        "webrtc_signal",
    }
)
_CHANNELS = frozenset({"operator", "sim"})


class UiPeerChannel:
    """Restricted view of the shared UI peer used by one UI subsystem."""

    def __init__(self, hub: "UiPeerHub", name: str) -> None:
        if name not in _CHANNELS:
            raise ValueError(f"unknown UI peer channel: {name}")
        self._hub = hub
        self.name = name

    @property
    def registered(self) -> bool:
        return self._hub.registered

    def heartbeat(self) -> None:
        # The hub owns the only DDS executor and heartbeat cadence.
        return None

    def has_peer(self, endpoint_id: str) -> bool:
        """Expose live DDS descriptor readiness without owning the peer."""

        return self._hub.has_peer(endpoint_id)

    def send(self, message_type: str, **kwargs: Any) -> Envelope:
        return self._hub.send(self.name, message_type, **kwargs)

    def receive(self, timeout_ms: int = 0) -> Iterator[Envelope]:
        yield from self._hub.receive(self.name, timeout_ms=timeout_ms)

    def close(self) -> None:
        # A channel does not own the process-level peer.
        return None


class UiPeerHub:
    """Serialize DDS access and demultiplex one UI endpoint into two sessions."""

    def __init__(
        self,
        *,
        endpoint_id: str,
        settings: Optional[DdsRuntimeSettings] = None,
        client_factory: Callable[..., Any] = PeerClient,
        client: Any = None,
        poll_ms: int = 20,
        max_pending: int = 1024,
        autostart: bool = True,
    ) -> None:
        self.endpoint_id = str(endpoint_id).strip()
        if not self.endpoint_id:
            raise ValueError("UI endpoint_id is required")
        self.poll_ms = max(0, int(poll_ms))
        self.max_pending = max(32, int(max_pending))
        self._client = client
        self._client_factory = client_factory
        self._settings = settings
        self._owns_client = client is None
        self._io_lock = threading.Lock()
        self._condition = threading.Condition()
        self._queues: dict[str, deque[Envelope]] = {
            name: deque() for name in _CHANNELS
        }
        self._sent_channels: dict[str, str] = {}
        self._sent_order: deque[str] = deque()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_error = ""
        if autostart:
            self.start()

    @property
    def registered(self) -> bool:
        if self._stop.is_set():
            return False
        with self._io_lock:
            client = self._client
            try:
                return client is not None and bool(client.registered)
            except Exception:
                return False

    @property
    def last_error(self) -> str:
        with self._condition:
            return self._last_error

    def channel(self, name: str) -> UiPeerChannel:
        return UiPeerChannel(self, str(name))

    def has_peer(self, endpoint_id: str) -> bool:
        """Return descriptor readiness, never TCP/SSH reachability."""

        with self._io_lock:
            client = self._client
            if client is None:
                return False
            checker = getattr(client, "has_peer", None)
            if callable(checker):
                try:
                    return bool(checker(str(endpoint_id).strip()))
                except Exception:
                    return False
            try:
                return bool(client.registered)
            except Exception:
                return False

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="ui-dds-peer",
            daemon=True,
        )
        self._thread.start()

    def send(
        self,
        channel: str,
        message_type: str,
        **kwargs: Any,
    ) -> Envelope:
        if channel not in _CHANNELS:
            raise ValueError(f"unknown UI peer channel: {channel}")
        with self._io_lock:
            client = self._client
            if client is None:
                raise DdsTransportError(
                    "UI DDS peer is reconnecting; request will be retried"
                )
            envelope = client.send(message_type, **kwargs)
        message_id = str(envelope.message_id)
        if message_id:
            with self._condition:
                self._sent_channels[message_id] = channel
                self._sent_order.append(message_id)
                while len(self._sent_order) > self.max_pending:
                    expired = self._sent_order.popleft()
                    self._sent_channels.pop(expired, None)
        return envelope

    def receive(self, channel: str, *, timeout_ms: int = 0) -> Iterator[Envelope]:
        if channel not in _CHANNELS:
            raise ValueError(f"unknown UI peer channel: {channel}")
        timeout_s = max(0, int(timeout_ms)) / 1000.0
        with self._condition:
            if not self._queues[channel] and timeout_s > 0.0:
                self._condition.wait(timeout=timeout_s)
            pending = tuple(self._queues[channel])
            self._queues[channel].clear()
        yield from pending

    def close(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=3.0)
        with self._io_lock:
            client = self._client
            self._client = None
            if client is not None:
                client.close()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                with self._io_lock:
                    if self._client is None:
                        self._client = self._client_factory(
                            EndpointDescriptor(self.endpoint_id, "ui", ()),
                            settings=self._settings,
                        )
                    client = self._client
                    client.heartbeat()
                    messages = tuple(
                        client.receive(timeout_ms=self.poll_ms)
                    )
                for message in messages:
                    self._dispatch(message)
                with self._condition:
                    self._last_error = ""
            except Exception as exc:
                with self._condition:
                    self._last_error = f"DDS peer failed: {exc}"
                if self._owns_client:
                    self._retire_client()
                self._stop.wait(0.1)

    def _retire_client(self) -> None:
        with self._io_lock:
            client = self._client
            self._client = None
        if client is not None:
            try:
                client.close()
            except Exception:
                pass
        with self._condition:
            # Replies from the retired boot must never be delivered to a new
            # PeerClient. Application requests remain bounded in their owners.
            self._sent_channels.clear()
            self._sent_order.clear()
            for queue in self._queues.values():
                queue.clear()
            self._condition.notify_all()

    def _dispatch(self, message: Envelope) -> None:
        destination = (
            "sim"
            if message.message_type in _SIMULATION_MESSAGES
            else "operator"
        )
        if message.message_type == "error":
            reply_to = str((message.payload or {}).get("reply_to", ""))
            with self._condition:
                destination = self._sent_channels.pop(reply_to, destination)
        with self._condition:
            queue = self._queues[destination]
            if len(queue) >= self.max_pending:
                queue.popleft()
                self._last_error = (
                    f"{destination} DDS inbox exceeded {self.max_pending} messages"
                )
            queue.append(message)
            self._condition.notify_all()


__all__ = ["UiPeerChannel", "UiPeerHub"]
