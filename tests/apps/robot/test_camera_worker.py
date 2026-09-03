from __future__ import annotations

import time

from elesim_protocol import DdsRuntimeSettings
from elesim_robot.camera.worker import CameraPublisherThread


class BrokenCamera:
    def start(self) -> None:
        raise RuntimeError("camera unplugged")

    def stop(self) -> None:
        pass


class UnusedPublisher:
    def close(self) -> None:
        pass


class RecordingPublisher(UnusedPublisher):
    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs


class OneFrameCamera:
    def start(self) -> None:
        raise RuntimeError("done after publisher construction")

    def stop(self) -> None:
        pass


def test_camera_thread_surfaces_background_start_failure() -> None:
    worker = CameraPublisherThread(
        "/elesim/test/rgbd/frame",
        endpoint_id="robot-a",
        boot_id="boot-a",
        dds_settings=DdsRuntimeSettings(),
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


def test_camera_worker_forwards_dds_identity_to_the_publisher() -> None:
    captured: dict[str, object] = {}

    def publisher_factory(_endpoint: str, **kwargs):
        captured.update(kwargs)
        return RecordingPublisher(**kwargs)

    worker = CameraPublisherThread(
        "/elesim/test/rgbd/frame",
        endpoint_id="robot-a",
        boot_id="boot-a",
        dds_settings=DdsRuntimeSettings(domain_id=17),
        width=640,
        height=480,
        fps=30,
        camera_factory=lambda **_kwargs: OneFrameCamera(),
        publisher_factory=publisher_factory,
    )
    worker.start()
    deadline = time.monotonic() + 1.0
    while worker.status()["running"] and time.monotonic() < deadline:
        time.sleep(0.001)
    worker.stop()

    assert captured["endpoint_id"] == "robot-a"
    assert captured["boot_id"] == "boot-a"
    assert captured["settings"] == DdsRuntimeSettings(domain_id=17)
    assert captured["send_depth"] is True
