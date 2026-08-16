from __future__ import annotations

import multiprocessing as mp

import numpy as np
import pytest

from elesim_sim.media.worker import (
    MediaWorkerClient,
    SharedFrameMailbox,
    VideoStreamSpec,
)
from elesim_sim.vision.webrtc import available as webrtc_available


def test_video_stream_spec_rejects_unbounded_or_invalid_dimensions() -> None:
    with pytest.raises(ValueError):
        VideoStreamSpec("observer", 0.0, 640, 480)
    with pytest.raises(ValueError):
        VideoStreamSpec("observer", 20.0, 0, 480)


def test_shared_frame_mailbox_is_latest_only_and_shape_stable() -> None:
    context = mp.get_context("fork")
    mailbox = SharedFrameMailbox.create(context, width=4, height=3)
    image, sequence, captured_at = mailbox.latest()
    assert image is None
    assert sequence == 0
    assert captured_at == 0.0

    first = np.full((3, 4, 3), 7, dtype=np.uint8)
    second = np.full((3, 4, 3), 9, dtype=np.uint8)
    assert mailbox.publish(first, captured_at=10.0) == 1
    assert mailbox.publish(second, captured_at=11.0) == 2

    image, sequence, captured_at = mailbox.latest()
    assert image is not None
    assert sequence == 2
    assert captured_at == 11.0
    assert image.shape == (3, 4, 3)
    assert image.dtype == np.uint8
    assert int(image[0, 0, 0]) == 9
    stats = mailbox.stats()
    assert stats["published"] == 2
    assert stats["overwritten"] == 1

    with pytest.raises(ValueError):
        mailbox.publish(np.zeros((3, 4), dtype=np.uint8))
    with pytest.raises(ValueError):
        mailbox.publish(np.zeros((3, 4, 3), dtype=np.float32))


def test_media_worker_handshake_and_shutdown_are_bounded() -> None:
    if not webrtc_available():
        pytest.skip("aiortc/av is not installed in this test environment")
    worker = MediaWorkerClient(
        {"observer": VideoStreamSpec("observer", 10.0, 4, 3)},
        command_timeout_s=3.0,
    )
    try:
        worker.start(timeout_s=8.0)
        assert worker.ready is True
        worker.mailboxes["observer"].publish(
            np.full((3, 4, 3), 11, dtype=np.uint8)
        )
        assert worker.diagnostics()["streams"]["observer"]["sequence"] == 1
    finally:
        worker.close()
    assert worker.process.is_alive() is False
