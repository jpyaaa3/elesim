"""Optional aiortc receiver for direct sim video streams."""

from __future__ import annotations

import asyncio
import queue
import threading
import time
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
    def __init__(
        self,
        *,
        stream_name: str = "",
        on_error: Optional[Any] = None,
    ) -> None:
        if not available():
            raise RuntimeError("WebRTC requires aiortc")
        self.stream_name = str(stream_name).strip()
        self._on_error = on_error
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
        self._first_frame_at = 0.0
        self._last_frame_at = 0.0
        self._first_frame_reported = False
        self._frame_lock = threading.Lock()
        self._consume_tasks: set[asyncio.Task[Any]] = set()
        self._last_error = ""
        self._last_error_at = 0.0
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
        peer = self.peer
        peer.addTransceiver("video", direction="recvonly")

        @peer.on("connectionstatechange")
        async def _connection_state_changed() -> None:
            state = str(getattr(peer, "connectionState", "unknown"))
            stream = f" stream={self.stream_name}" if self.stream_name else ""
            print(f"[ui-webrtc]{stream} connection={state}", flush=True)
            # ``disconnected`` is a transient ICE state in aiortc; tearing
            # down a healthy receiver at that point turns a short network
            # gap into an avoidable renegotiation.  The receive task reports
            # MediaStreamError if the track actually ends, and the session
            # liveness watchdog covers a peer that stays disconnected while
            # no decoded frames arrive.  Only terminal states retry here.
            if state in {"failed", "closed"}:
                self._report_error("connection", RuntimeError(state))

        @peer.on("track")
        def _track(track: Any) -> None:
            if track.kind == "video":
                stream = f" stream={self.stream_name}" if self.stream_name else ""
                print(f"[ui-webrtc]{stream} track=received", flush=True)
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
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self._report_error("receive", exc)
                return
            # aiortc's remote track queue is intentionally lossless, which is
            # the wrong policy for a live camera.  If decoding/rendering falls
            # behind, discard queued frames before converting the newest one;
            # otherwise the UI can remain seconds behind the simulation.
            try:
                frame = self._drain_to_latest(track, frame)
                value = frame.to_ndarray(format="bgr24")
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                # A single malformed/corrupt encoded frame must not terminate
                # the named stream forever.  Keep consuming the next frame;
                # this is especially important while NVENC sends its first
                # SPS/keyframe after a renegotiation.
                self._report_error("decode", exc)
                continue
            with self._frame_lock:
                first_frame = self._first_frame_at <= 0.0
                self.latest_bgr = np.ascontiguousarray(value)
                self._frame_version += 1
                now = time.monotonic()
                if self._first_frame_at <= 0.0:
                    self._first_frame_at = now
                self._last_frame_at = now
                first_frame = first_frame and not self._first_frame_reported
                if first_frame:
                    self._first_frame_reported = True
            if first_frame:
                stream = f" stream={self.stream_name}" if self.stream_name else ""
                print(
                    f"[ui-webrtc]{stream} frame=decoded "
                    f"size={int(value.shape[1])}x{int(value.shape[0])}",
                    flush=True,
                )

    def _report_error(self, stage: str, exc: BaseException) -> None:
        detail = f"{stage}: {str(exc).strip() or exc.__class__.__name__}"[:512]
        now = time.monotonic()
        if detail == self._last_error and now - self._last_error_at < 5.0:
            return
        self._last_error = detail
        self._last_error_at = now
        stream = f" stream={self.stream_name}" if self.stream_name else ""
        print(f"[ui-webrtc]{stream} {detail}", flush=True)
        callback = self._on_error
        if callback is not None:
            try:
                callback(detail)
            except Exception:
                pass

    @property
    def last_error(self) -> str:
        return self._last_error

    def report_error(self, stage: str, exc: BaseException) -> None:
        """Record an answer/connection error outside the receive task."""

        self._report_error(str(stage), exc)

    def set_error_callback(self, callback: Optional[Any]) -> None:
        """Set the owner callback used for bounded stream recovery."""

        self._on_error = callback

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

    def frame_age_s(self) -> Optional[float]:
        """Return seconds since the last decoded frame, or ``None`` initially.

        A WebRTC peer can remain in ``connected`` state while the decoder is
        receiving only incomplete H.264 pictures.  Session state must not use
        that ICE state as proof that pixels are arriving; this small
        latest-frame clock lets the owner trigger a stream-only renegotiation
        without disturbing the healthy named stream.
        """

        with self._frame_lock:
            last = float(self._last_frame_at)
        if last <= 0.0:
            return None
        return max(0.0, time.monotonic() - last)

    def stats_snapshot(self, *, timeout_s: float = 0.2) -> dict[str, int | float]:
        """Return a small inbound-RTP snapshot for a stalled-stream log.

        The call stays on the peer's asyncio loop and is best-effort: a
        broken peer must never block DDS/session recovery.  Keep the default
        wait short because this is called from the DDS/session worker, where a
        slow stats coroutine could otherwise delay lease/heartbeat servicing.
        aiortc exposes
        packet loss and jitter through its ``inbound-rtp`` report; older
        embedders which do not provide ``getStats`` simply return an empty
        mapping.
        """

        peer = self.peer
        getter = getattr(peer, "getStats", None)
        if peer is None or not callable(getter) or self._closed:
            return {}

        async def collect() -> Any:
            return await getter()

        try:
            report = asyncio.run_coroutine_threadsafe(collect(), self.loop).result(
                timeout=max(0.05, float(timeout_s))
            )
        except Exception:
            return {}
        values = getattr(report, "values", None)
        if not callable(values):
            return {}
        for stat in values():
            if (
                str(getattr(stat, "type", "")) == "inbound-rtp"
                and str(
                    getattr(stat, "kind", getattr(stat, "mediaType", ""))
                )
                == "video"
            ):
                result: dict[str, int | float] = {}
                for name in ("packetsReceived", "packetsLost", "jitter"):
                    value = getattr(stat, name, None)
                    if value is None:
                        continue
                    try:
                        result[name] = float(value) if name == "jitter" else int(value)
                    except (TypeError, ValueError):
                        continue
                return result
        return {}

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
