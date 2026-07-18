"""Optional aiortc helpers for direct simulator video streaming."""

from __future__ import annotations

import asyncio
import threading
import time
from fractions import Fraction
from typing import Any, Callable, Optional

import numpy as np

try:
    import av
    from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
except ImportError:
    av = None  # type: ignore[assignment]
    RTCPeerConnection = None  # type: ignore[assignment]
    RTCSessionDescription = None  # type: ignore[assignment]
    VideoStreamTrack = object  # type: ignore[assignment,misc]


def available() -> bool:
    return av is not None and RTCPeerConnection is not None


class LatestFrameTrack(VideoStreamTrack):  # type: ignore[misc]
    def __init__(self, provider: Callable[[], Optional[np.ndarray]], *, fps: float = 30.0) -> None:
        super().__init__()
        self.provider = provider
        self.fps = max(1.0, float(fps))
        self.started = time.monotonic()
        self.index = 0

    async def recv(self) -> Any:
        self.index += 1
        target = self.started + self.index / self.fps
        await asyncio.sleep(max(0.0, target - time.monotonic()))
        frame = self.provider()
        if frame is None:
            frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        video = av.VideoFrame.from_ndarray(np.ascontiguousarray(frame), format="bgr24")
        video.pts = self.index
        video.time_base = Fraction(1, round(self.fps))
        return video


class WebRtcVideoSender:
    def __init__(self, provider: Callable[[], Optional[np.ndarray]], *, fps: float = 30.0) -> None:
        if not available():
            raise RuntimeError("WebRTC requires aiortc and av")
        self.provider = provider
        self.fps = float(fps)
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever, name="webrtc-sender", daemon=True)
        self.thread.start()
        self.peers: set[Any] = set()

    def accept_offer(self, sdp: str, offer_type: str = "offer") -> dict[str, str]:
        future = asyncio.run_coroutine_threadsafe(self._accept_offer(sdp, offer_type), self.loop)
        return future.result(timeout=10.0)

    async def _accept_offer(self, sdp: str, offer_type: str) -> dict[str, str]:
        peer = RTCPeerConnection()
        self.peers.add(peer)
        peer.addTrack(LatestFrameTrack(self.provider, fps=self.fps))

        @peer.on("connectionstatechange")
        async def _state_changed() -> None:
            if peer.connectionState in {"failed", "closed", "disconnected"}:
                await peer.close()
                self.peers.discard(peer)

        await peer.setRemoteDescription(RTCSessionDescription(sdp=sdp, type=offer_type))
        answer = await peer.createAnswer()
        await peer.setLocalDescription(answer)
        return {"sdp": peer.localDescription.sdp, "type": peer.localDescription.type}

    def close(self) -> None:
        async def shutdown() -> None:
            await asyncio.gather(*(peer.close() for peer in tuple(self.peers)), return_exceptions=True)
            self.peers.clear()

        asyncio.run_coroutine_threadsafe(shutdown(), self.loop).result(timeout=5.0)
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=3.0)


class WebRtcVideoReceiver:
    def __init__(self) -> None:
        if not available():
            raise RuntimeError("WebRTC requires aiortc and av")
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever, name="webrtc-receiver", daemon=True)
        self.thread.start()
        self.peer: Any = None
        self.latest_bgr: Optional[np.ndarray] = None

    def create_offer(self) -> dict[str, str]:
        future = asyncio.run_coroutine_threadsafe(self._create_offer(), self.loop)
        return future.result(timeout=10.0)

    async def _create_offer(self) -> dict[str, str]:
        self.peer = RTCPeerConnection()
        self.peer.addTransceiver("video", direction="recvonly")

        @self.peer.on("track")
        def _track(track: Any) -> None:
            if track.kind == "video":
                asyncio.create_task(self._consume(track))

        offer = await self.peer.createOffer()
        await self.peer.setLocalDescription(offer)
        return {"sdp": self.peer.localDescription.sdp, "type": self.peer.localDescription.type}

    async def _consume(self, track: Any) -> None:
        while True:
            try:
                frame = await track.recv()
            except Exception:
                return
            self.latest_bgr = frame.to_ndarray(format="bgr24")

    def accept_answer(self, sdp: str, answer_type: str = "answer") -> None:
        if self.peer is None:
            raise RuntimeError("create_offer() must be called first")
        future = asyncio.run_coroutine_threadsafe(
            self.peer.setRemoteDescription(RTCSessionDescription(sdp=sdp, type=answer_type)),
            self.loop,
        )
        future.result(timeout=10.0)

    def close(self) -> None:
        if self.peer is not None:
            asyncio.run_coroutine_threadsafe(self.peer.close(), self.loop).result(timeout=5.0)
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=3.0)

