"""Optional aiortc receiver for direct simulator video streams."""

from __future__ import annotations

import asyncio
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
        self.peer = RTCPeerConnection(configuration=_ice_configuration(turn))
        self.peer.addTransceiver("video", direction="recvonly")

        @self.peer.on("track")
        def _track(track: Any) -> None:
            if track.kind == "video":
                asyncio.create_task(self._consume(track))

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
            self.latest_bgr = frame.to_ndarray(format="bgr24")

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
        if self.peer is not None and self.loop.is_running():
            try:
                asyncio.run_coroutine_threadsafe(
                    self.peer.close(),
                    self.loop,
                ).result(timeout=5.0)
            except Exception:
                pass
        if self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.thread is not threading.current_thread():
            self.thread.join(timeout=3.0)


__all__ = ["WebRtcVideoReceiver", "available"]
