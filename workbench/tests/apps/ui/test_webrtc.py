from __future__ import annotations

import asyncio
import queue

from elesim_ui.webrtc import WebRtcVideoReceiver


class _Track:
    def __init__(self, *frames: object) -> None:
        self._queue: queue.Queue[object] = queue.Queue()
        for frame in frames:
            self._queue.put(frame)


def test_receiver_drops_queued_frames_and_keeps_latest() -> None:
    first = object()
    latest = object()
    track = _Track(latest)

    assert WebRtcVideoReceiver._drain_to_latest(track, first) is latest
    assert track._queue.empty()


def test_receiver_drain_handles_asyncio_queue() -> None:
    first = object()
    latest = object()
    track = type("Track", (), {})()
    track._queue = asyncio.Queue()
    track._queue.put_nowait(latest)

    assert WebRtcVideoReceiver._drain_to_latest(track, first) is latest
    assert track._queue.empty()
