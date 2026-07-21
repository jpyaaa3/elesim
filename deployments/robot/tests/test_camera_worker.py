from __future__ import annotations

import time

from elesim_robot.camera.worker import CameraPublisherThread


class BrokenCamera:
    def start(self) -> None:
        raise RuntimeError("camera unplugged")

    def stop(self) -> None:
        pass


class UnusedPublisher:
    def close(self) -> None:
        pass


def test_camera_thread_surfaces_background_start_failure() -> None:
    worker = CameraPublisherThread(
        "inproc://unused",
        width=640,
        height=480,
        fps=30,
        camera_factory=lambda **_kwargs: BrokenCamera(),
        publisher_factory=lambda *_args, **_kwargs: UnusedPublisher(),
    )
    worker.start()
    deadline = time.monotonic() + 1.0
    while worker.status()["running"] and time.monotonic() < deadline:
        time.sleep(0.001)

    status = worker.status()
    worker.stop()

    assert status["healthy"] is False
    assert "camera unplugged" in status["error"]
