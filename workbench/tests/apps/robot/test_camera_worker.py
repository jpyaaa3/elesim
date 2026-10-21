from __future__ import annotations

import time
from types import SimpleNamespace
import zlib

import numpy as np

from elesim_protocol import DdsRuntimeSettings, decode_image_payload
from elesim_robot.camera.worker import CameraPublisherThread
from elesim_robot.camera import realsense


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


class PublishedFrames:
    def __init__(self, topic: str, store: dict[str, list[object]]) -> None:
        self.topic = topic
        self.store = store

    def publish(self, frame: object) -> bool:
        self.store.setdefault(self.topic, []).append(frame)
        return True


class OneFrameCamera:
    def start(self) -> None:
        raise RuntimeError("done after publisher construction")

    def stop(self) -> None:
        pass


def test_absent_realsense_does_not_enter_sdk_pipeline_or_create_dds_publisher(monkeypatch):
    def unexpected_pipeline(*_args, **_kwargs):
        raise AssertionError("absent camera must not enter blocking SDK startup")

    monkeypatch.setattr(realsense, "rs", SimpleNamespace(
        context=lambda: SimpleNamespace(query_devices=lambda: []),
        pipeline=unexpected_pipeline,
    ))
    worker = CameraPublisherThread(
        "/elesim/test/rgbd/frame", endpoint_id="robot-a", boot_id="boot-a",
        dds_settings=DdsRuntimeSettings(), width=640, height=480, fps=30,
        camera_factory=realsense.RealSenseCamera,
        publisher_factory=unexpected_pipeline,
    )
    worker.start()
    deadline = time.monotonic() + 1.0
    while worker.status()["running"] and time.monotonic() < deadline:
        time.sleep(0.001)
    worker.stop()
    status = worker.status()
    assert status["running"] is False
    assert status["healthy"] is False
    assert "No RealSense device connected" in status["error"]


def test_present_realsense_uses_the_enumerated_device_context(monkeypatch):
    context = SimpleNamespace(query_devices=lambda: [object()])
    pipeline = object()
    def make_pipeline(selected_context):
        assert selected_context is context
        return pipeline

    monkeypatch.setattr(realsense, "rs", SimpleNamespace(
        context=lambda: context, pipeline=make_pipeline,
        config=lambda: SimpleNamespace(enable_stream=lambda *_args: None),
        stream=SimpleNamespace(color=1, depth=2),
        format=SimpleNamespace(bgr8=1, z16=2),
        align=lambda _stream: object(),
    ))
    camera = realsense.RealSenseCamera()
    assert camera._pipeline is pipeline


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


def test_encoded_stereo_worker_publishes_rotated_left_rgbd_and_right_color() -> None:
    published: dict[str, list[object]] = {}

    class StereoCamera:
        def __init__(self) -> None:
            self.captures = 0

        def start(self) -> None:
            pass

        def capture(self, *, timeout_ms: int):
            del timeout_ms
            self.captures += 1
            if self.captures > 1:
                raise RuntimeError("test capture complete")
            return SimpleNamespace(
                color_bgr=np.array(
                    [[[1, 0, 0], [2, 0, 0]], [[3, 0, 0], [4, 0, 0]]],
                    dtype=np.uint8,
                ),
                depth_raw=np.array([[10, 20], [30, 40]], dtype=np.uint16),
                depth_scale=0.001,
                intrinsics=SimpleNamespace(
                    fx=100.0, fy=110.0, cx=0.25, cy=0.5, width=2, height=2
                ),
                right_color_bgr=np.array(
                    [[[5, 0, 0], [6, 0, 0]], [[7, 0, 0], [8, 0, 0]]],
                    dtype=np.uint8,
                ),
                right_intrinsics=SimpleNamespace(
                    fx=101.0, fy=111.0, cx=0.75, cy=0.6, width=2, height=2
                ),
            )

        def stop(self) -> None:
            pass

    worker = CameraPublisherThread(
        "/elesim/robot/rgbd/frame",
        right_topic="/elesim/robot/rgbd/right",
        rotate_180=True,
        endpoint_id="robot-a",
        boot_id="boot-a",
        dds_settings=DdsRuntimeSettings(),
        width=2,
        height=2,
        fps=30,
        wire_format="encoded-rgbd-v1",
        color_codec="raw",
        camera_factory=lambda **_kwargs: StereoCamera(),
        publisher_factory=lambda topic, **_kwargs: PublishedFrames(topic, published),
    )
    worker.start()
    deadline = time.monotonic() + 1.0
    while worker.status()["running"] and time.monotonic() < deadline:
        time.sleep(0.001)
    worker.stop()

    left = published["/elesim/robot/rgbd/frame"][0]
    right = published["/elesim/robot/rgbd/right"][0]
    np.testing.assert_array_equal(
        decode_image_payload(
            left.color_payload, "raw", width=2, height=2, encoding="bgr8"
        ).reshape(2, 2, 3)[:, :, 0],
        [[4, 3], [2, 1]],
    )
    assert left.has_depth is True
    depth = np.frombuffer(zlib.decompress(left.depth_payload), dtype=np.uint16)
    assert depth.reshape(2, 2).tolist() == [[40, 30], [20, 10]]
    assert (left.metadata.cx, left.metadata.cy) == (0.75, 0.5)
    np.testing.assert_array_equal(
        decode_image_payload(
            right.color_payload, "raw", width=2, height=2, encoding="bgr8"
        ).reshape(2, 2, 3)[:, :, 0],
        [[8, 7], [6, 5]],
    )
    assert right.has_depth is False
    assert (right.metadata.cx, right.metadata.cy) == (0.25, 0.4)
    assert left.seq == right.seq == 1
    assert left.source_boot_id == right.source_boot_id == "boot-a"
