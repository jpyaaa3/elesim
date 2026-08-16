from __future__ import annotations

import threading
import time

import numpy as np

from elesim_sim.media import SharedFrameMailbox
from elesim_sim.media import FrameDispatchWorker
from elesim_sim.vision.frame_hub import FrameHub
from elesim_sim.runtime import SimScene


def test_frame_dispatch_worker_overwrites_while_consumer_is_busy() -> None:
    first_started = threading.Event()
    release_first = threading.Event()
    seen: list[tuple[str, int]] = []

    def consume(stream: str, frame: int) -> None:
        seen.append((stream, frame))
        if frame == 1:
            first_started.set()
            assert release_first.wait(timeout=2.0)

    worker = FrameDispatchWorker(("observer",), consume)
    worker.start()
    try:
        assert worker.submit("observer", 1)
        assert first_started.wait(timeout=2.0)
        assert worker.submit("observer", 2)
        assert worker.submit("observer", 3)
        release_first.set()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and seen[-1:] != [("observer", 3)]:
            time.sleep(0.01)
        assert seen == [("observer", 1), ("observer", 3)]
        stats = worker.stats()["observer"]
        assert stats["submitted"] == 3
        assert stats["processed"] == 2
        assert stats["overwritten"] == 1
    finally:
        worker.close()


def test_frame_dispatch_worker_contains_callback_failures() -> None:
    errors: list[str] = []
    processed = threading.Event()

    def consume(_stream: str, _frame: object) -> None:
        processed.set()
        raise RuntimeError("publisher unavailable")

    worker = FrameDispatchWorker(
        ("observer",),
        consume,
        on_error=lambda stream, exc: errors.append(f"{stream}:{exc}"),
    )
    worker.start()
    try:
        assert worker.submit("observer", object())
        assert processed.wait(timeout=2.0)
        assert errors == ["observer:publisher unavailable"]
        stats = worker.stats()["observer"]
        assert stats["failed"] == 1
        assert stats["processed"] == 1
        assert stats["last_error"] == "publisher unavailable"
    finally:
        worker.close()


def test_sim_scene_transport_sinks_do_not_block_camera_capture() -> None:
    class Frame:
        color_bgr = np.full((3, 4, 3), 17, dtype=np.uint8)

    class Camera:
        def capture(self, **_kwargs: object) -> Frame:
            return Frame()

    entered_publisher = threading.Event()
    release_publisher = threading.Event()

    class SlowPublisher:
        def publish(self, _frame: object) -> None:
            entered_publisher.set()
            assert release_publisher.wait(timeout=2.0)

    import multiprocessing as mp

    mailbox = SharedFrameMailbox.create(mp.get_context("fork"), width=4, height=3)
    scene = SimScene(
        frame_hub=FrameHub(("rgbd", "observer", "hand_eye_preview")),
        video_mailboxes={"hand_eye_preview": mailbox},
    )
    scene.eye_camera = Camera()
    scene.camera_publisher = SlowPublisher()
    scene.configure_frame_dispatchers()
    scene.start_frame_dispatchers()
    try:
        started = time.monotonic()
        scene.maybe_publish_camera(
            arm_q=None,
            max_hz=30.0,
            force=True,
            rgb_enabled=True,
            depth_enabled=False,
        )
        elapsed = time.monotonic() - started
        assert elapsed < 0.5
        assert entered_publisher.wait(timeout=2.0)

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and mailbox.stats()["sequence"] != 1:
            time.sleep(0.01)
        assert mailbox.stats()["sequence"] == 1
        assert scene.frame_hub.latest("hand_eye_preview") is not None
    finally:
        release_publisher.set()
        scene.close_frame_dispatchers()
