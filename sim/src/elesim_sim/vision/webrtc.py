"""Optional aiortc helpers for named sim video streams."""

from __future__ import annotations

import asyncio
import threading
import time
from fractions import Fraction
from typing import Any, Callable, Mapping, Optional

import numpy as np

from elesim_protocol import TurnCredentials

try:
    import av
    from aiortc import (
        RTCConfiguration,
        RTCIceServer,
        RTCPeerConnection,
        RTCSessionDescription,
        VideoStreamTrack,
    )
except ImportError:
    av = None  # type: ignore[assignment]
    RTCConfiguration = None  # type: ignore[assignment]
    RTCIceServer = None  # type: ignore[assignment]
    RTCPeerConnection = None  # type: ignore[assignment]
    RTCSessionDescription = None  # type: ignore[assignment]
    VideoStreamTrack = object  # type: ignore[assignment,misc]


def available() -> bool:
    return av is not None and RTCPeerConnection is not None


def ice_configuration(turn: Optional[TurnCredentials]) -> Any:
    if not available() or RTCConfiguration is None:
        return None
    if turn is None:
        return RTCConfiguration(iceServers=[])
    server = RTCIceServer(
        urls=list(turn.urls),
        username=turn.username,
        credential=turn.credential,
    )
    return RTCConfiguration(iceServers=[server])


class LatestFrameTrack(VideoStreamTrack):  # type: ignore[misc]
    def __init__(
        self,
        provider: Callable[[], Optional[np.ndarray]],
        *,
        fps: float = 30.0,
        frame_size: Optional[tuple[int, int]] = None,
        on_error: Optional[Callable[[str], None]] = None,
    ) -> None:
        super().__init__()
        self.provider = provider
        self.fps = max(1.0, float(fps))
        configured_size = frame_size is not None
        if frame_size is None:
            frame_size = (640, 480)
        width, height = (int(frame_size[0]), int(frame_size[1]))
        if width <= 0 or height <= 0:
            raise ValueError("WebRTC frame_size must contain positive dimensions")
        self.frame_size = (width, height)
        self._enforce_frame_size = configured_size
        self._fallback = np.zeros((height, width, 3), dtype=np.uint8)
        self.started = time.monotonic()
        self.index = 0
        self.on_error = on_error
        self._last_error = ""
        self._last_error_at = 0.0

    async def recv(self) -> Any:
        self.index += 1
        target = self.started + self.index / self.fps
        await asyncio.sleep(max(0.0, target - time.monotonic()))
        try:
            frame = self.provider()
        except Exception as exc:
            self._report_error("provider", exc)
            frame = None
        if frame is None:
            # Keep the RTP frame dimensions stable while the camera is still
            # warming up.  A stream that starts at 640x480 and changes to an
            # observer's configured size on its first real frame can make the
            # decoder show a torn/corrupt image.
            frame = self._fallback
        try:
            video = av.VideoFrame.from_ndarray(
                np.ascontiguousarray(frame),
                format="bgr24",
            )
            if self._enforce_frame_size and (
                video.width != self.frame_size[0]
                or video.height != self.frame_size[1]
            ):
                video = video.reformat(
                    width=self.frame_size[0],
                    height=self.frame_size[1],
                    format="bgr24",
                )
        except Exception as exc:
            self._report_error("encode", exc)
            video = av.VideoFrame.from_ndarray(self._fallback, format="bgr24")
        video.pts = self.index
        video.time_base = Fraction(1, round(self.fps))
        return video

    def _report_error(self, stage: str, exc: Exception) -> None:
        detail = f"{stage}: {str(exc).strip() or exc.__class__.__name__}"[:512]
        now = time.monotonic()
        if detail == self._last_error and now - self._last_error_at < 5.0:
            return
        self._last_error = detail
        self._last_error_at = now
        if self.on_error is not None:
            try:
                self.on_error(detail)
            except Exception:
                pass
            return
        print(f"[webrtc] frame fallback: {detail}", flush=True)


class WebRtcVideoSender:
    def __init__(
        self,
        provider: Callable[[], Optional[np.ndarray]],
        *,
        fps: float = 30.0,
        frame_size: Optional[tuple[int, int]] = None,
    ) -> None:
        if not available():
            raise RuntimeError("WebRTC requires aiortc and av")
        self.provider = provider
        self.fps = float(fps)
        self.frame_size = frame_size
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(
            target=self.loop.run_forever,
            name="webrtc-sender",
            daemon=True,
        )
        self.thread.start()
        self.peers: set[Any] = set()
        self.peers_by_session: dict[str, set[Any]] = {}

    def accept_offer(
        self,
        sdp: str,
        offer_type: str = "offer",
        *,
        turn: Optional[TurnCredentials] = None,
        session_id: str = "",
    ) -> dict[str, str]:
        future = asyncio.run_coroutine_threadsafe(
            self._accept_offer(sdp, offer_type, turn=turn, session_id=session_id),
            self.loop,
        )
        return future.result(timeout=15.0)

    async def _accept_offer(
        self,
        sdp: str,
        offer_type: str,
        *,
        turn: Optional[TurnCredentials],
        session_id: str,
    ) -> dict[str, str]:
        peer = RTCPeerConnection(configuration=ice_configuration(turn))
        peer.addTrack(
            LatestFrameTrack(
                self.provider,
                fps=self.fps,
                frame_size=self.frame_size,
            )
        )

        @peer.on("connectionstatechange")
        async def _state_changed() -> None:
            if peer.connectionState in {"failed", "closed", "disconnected"}:
                await peer.close()
                self._forget_peer(peer)

        try:
            await peer.setRemoteDescription(RTCSessionDescription(sdp=sdp, type=offer_type))
            answer = await peer.createAnswer()
            await peer.setLocalDescription(answer)
        except Exception:
            await peer.close()
            raise

        previous = tuple(self.peers_by_session.get(session_id, ()))
        self.peers.add(peer)
        self.peers_by_session[session_id] = {peer}
        if previous:
            await asyncio.gather(*(old.close() for old in previous), return_exceptions=True)
            for old in previous:
                self._forget_peer(old)
        return {"sdp": peer.localDescription.sdp, "type": peer.localDescription.type}

    def _forget_peer(self, peer: Any) -> None:
        self.peers.discard(peer)
        for session_id, peers in tuple(self.peers_by_session.items()):
            peers.discard(peer)
            if not peers:
                self.peers_by_session.pop(session_id, None)

    def close_session(self, session_id: str) -> None:
        async def shutdown() -> None:
            peers = tuple(self.peers_by_session.get(str(session_id), ()))
            await asyncio.gather(*(peer.close() for peer in peers), return_exceptions=True)
            for peer in peers:
                self._forget_peer(peer)

        asyncio.run_coroutine_threadsafe(shutdown(), self.loop).result(timeout=5.0)

    def close(self) -> None:
        async def shutdown() -> None:
            await asyncio.gather(
                *(peer.close() for peer in tuple(self.peers)),
                return_exceptions=True,
            )
            self.peers.clear()
            self.peers_by_session.clear()

        asyncio.run_coroutine_threadsafe(shutdown(), self.loop).result(timeout=5.0)
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=3.0)


class NamedWebRtcVideoSender:
    """One independent peer connection namespace per advertised stream."""

    def __init__(
        self,
        providers: Mapping[str, Callable[[], Optional[np.ndarray]]],
        *,
        fps: Mapping[str, float],
        frame_sizes: Optional[Mapping[str, tuple[int, int]]] = None,
    ) -> None:
        sizes = frame_sizes or {}
        self.senders = {
            str(name): WebRtcVideoSender(
                provider,
                fps=float(fps[name]),
                frame_size=sizes.get(str(name)),
            )
            for name, provider in providers.items()
        }

    def accept_offer(
        self,
        stream: str,
        sdp: str,
        offer_type: str,
        turn: Optional[TurnCredentials],
        session_id: str,
    ) -> dict[str, str]:
        sender = self.senders.get(str(stream))
        if sender is None:
            raise ValueError(f"unsupported WebRTC stream: {stream}")
        return sender.accept_offer(
            sdp,
            offer_type,
            turn=turn,
            session_id=session_id,
        )

    def close_session(self, session_id: str) -> None:
        for sender in self.senders.values():
            sender.close_session(session_id)

    def close(self) -> None:
        for sender in self.senders.values():
            sender.close()


__all__ = [
    "LatestFrameTrack",
    "NamedWebRtcVideoSender",
    "WebRtcVideoSender",
    "available",
    "ice_configuration",
]
