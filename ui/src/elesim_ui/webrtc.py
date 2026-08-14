"""Optional aiortc receiver for direct sim video streams."""

from __future__ import annotations

import asyncio
import queue
import threading
from typing import Any, Optional

import numpy as np

from elesim_protocol import TurnCredentials

try:
    from aiortc import (
        RTCConfiguration,
        RTCIceServer,
        RTCPeerConnection,
        RTCSessionDescription,
    )
except ImportError:
    RTCConfiguration = None  # type: ignore[assignment]
    RTCIceServer = None  # type: ignore[assignment]
    RTCPeerConnection = None  # type: ignore[assignment]
    RTCSessionDescription = None  # type: ignore[assignment]


def available() -> bool:
    return RTCPeerConnection is not None


def _ice_configuration(turn: Optional[TurnCredentials]) -> Any:
    if not available() or RTCConfiguration is None:
        return None
    if turn is None:
        return RTCConfiguration(iceServers=[])
    return RTCConfiguration(
        iceServers=[
            RTCIceServer(
                urls=list(turn.urls),
                username=turn.username,
                credential=turn.credential,
            )
        ]
    )


class WebRtcVideoReceiver:
    def __init__(self) -> None:
        if not available():
            raise RuntimeError("WebRTC requires aiortc")
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(
            target=self.loop.run_forever,
            name="webrtc-receiver",
            daemon=True,
        )
        self.thread.start()
        self.peer: Any = None
        self.latest_bgr: Optional[np.ndarray] = None
        self._frame_version = 0
        self._frame_lock = threading.Lock()
        self._consume_tasks: set[asyncio.Task[Any]] = set()
        self._closed = False

    def create_offer(self, *, turn: Optional[TurnCredentials] = None) -> dict[str, str]:
        if self._closed:
            raise RuntimeError("WebRTC receiver is closed")
        future = asyncio.run_coroutine_threadsafe(
            self._create_offer(turn=turn),
            self.loop,
        )
        return future.result(timeout=15.0)

    async def _create_offer(self, *, turn: Optional[TurnCredentials]) -> dict[str, str]:
        if self.peer is not None:
            await self.peer.close()
        await self._cancel_consumer_tasks()
        self.peer = RTCPeerConnection(configuration=_ice_configuration(turn))
        self.peer.addTransceiver("video", direction="recvonly")

        @self.peer.on("track")
        def _track(track: Any) -> None:
            if track.kind == "video":
                task = asyncio.create_task(self._consume(track))
                self._consume_tasks.add(task)
                task.add_done_callback(self._consume_tasks.discard)

        offer = await self.peer.createOffer()
        await self.peer.setLocalDescription(offer)
        return {
            "sdp": self.peer.localDescription.sdp,
            "type": self.peer.localDescription.type,
        }

    async def _consume(self, track: Any) -> None:
        while True:
            try:
                frame = await track.recv()
            except Exception:
                return
            # aiortc's remote track queue is intentionally lossless, which is
            # the wrong policy for a live camera.  If decoding/rendering falls
            # behind, discard queued frames before converting the newest one;
            # otherwise the UI can remain seconds behind the simulation.
            frame = self._drain_to_latest(track, frame)
            value = frame.to_ndarray(format="bgr24")
            with self._frame_lock:
                self.latest_bgr = np.ascontiguousarray(value)
                self._frame_version += 1

    async def _cancel_consumer_tasks(self) -> None:
        tasks = tuple(self._consume_tasks)
        for task in tasks:
            task.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        self._consume_tasks.clear()

    @staticmethod
    def _drain_to_latest(track: Any, frame: Any) -> Any:
        pending = getattr(track, "_queue", None)
        get_nowait = getattr(pending, "get_nowait", None)
        if not callable(get_nowait):
            return frame
        while True:
            try:
                candidate = get_nowait()
            except (asyncio.QueueEmpty, queue.Empty):
                return frame
            except (RuntimeError, AttributeError):
                return frame
            if candidate is None:
                return frame
            frame = candidate

    def latest_frame(self) -> Optional[np.ndarray]:
        """Return a stable latest-only copy for the rendering thread."""

        with self._frame_lock:
            return None if self.latest_bgr is None else self.latest_bgr.copy()

    def latest_frame_view(self) -> Optional[np.ndarray]:
        """Return the immutable-by-replacement latest frame without a copy.

        The receiver replaces the array object for every decoded frame and
        never mutates an array after publication.  The UI uploads this view
        synchronously, so avoiding a full-frame copy keeps the render loop
        from competing with WebRTC decode.
        """

        with self._frame_lock:
            return self.latest_bgr

    def frame_version(self) -> int:
        with self._frame_lock:
            return int(self._frame_version)

    def accept_answer(self, sdp: str, answer_type: str = "answer") -> None:
        if self.peer is None:
            raise RuntimeError("create_offer() must be called first")
        future = asyncio.run_coroutine_threadsafe(
            self.peer.setRemoteDescription(
                RTCSessionDescription(sdp=str(sdp), type=str(answer_type))
            ),
            self.loop,
        )
        future.result(timeout=15.0)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True

        async def shutdown() -> None:
            peer = self.peer
            self.peer = None
            if peer is not None:
                try:
                    await peer.close()
                except Exception:
                    pass
            await self._cancel_consumer_tasks()

        if self.loop.is_running():
            try:
                asyncio.run_coroutine_threadsafe(shutdown(), self.loop).result(timeout=5.0)
            except Exception:
                pass
        if self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.thread is not threading.current_thread():
            self.thread.join(timeout=3.0)


__all__ = ["WebRtcVideoReceiver", "available"]
