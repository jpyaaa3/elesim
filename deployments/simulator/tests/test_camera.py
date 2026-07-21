from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from elesim_simulator.vision.sim_camera.mount import (
    _OPTICAL_FROM_GENESIS_CAMERA,
    hand_eye_to_genesis_attach_T,
    load_hand_eye_offset_T,
)
from elesim_simulator.vision.sim_camera.pose import (
    _link_world_transform,
    camera_axes_from_genesis_camera_object,
)
from elesim_simulator.vision.sim_camera.publisher import SimCameraPublisher
from elesim_simulator.vision.sim_camera.subscriber import SimCameraSubscriber
from elesim_simulator.vision.sim_camera.types import SimCameraFrame, SimCameraIntrinsics


CONFIG_DIR = Path(__file__).parents[1] / "config"


class _FakeTensor:
    def __init__(self, data) -> None:
        self._data = np.asarray(data, dtype=float)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self._data


class _FakeLink:
    def __init__(self, pos, quat_wxyz) -> None:
        self._pos = _FakeTensor(pos)
        self._quat = _FakeTensor(quat_wxyz)

    def get_pos(self):
        return self._pos

    def get_quat(self):
        return self._quat


def test_hand_eye_optical_axes_match_camera_calibration() -> None:
    calibration = CONFIG_DIR / "calibration/hand_eye.camera.json"
    transform = load_hand_eye_offset_T(calibration)
    rotation = transform[:3, :3]
    np.testing.assert_allclose(rotation[:, 2], [1.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(rotation[:, 0], [0.0, -1.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(rotation[:, 1], [0.0, 0.0, -1.0], atol=1e-6)


def test_genesis_camera_object_matches_link_attachment() -> None:
    calibration = CONFIG_DIR / "calibration/hand_eye.camera.json"
    link = _FakeLink([0.2, 0.0, 0.5], [1.0, 0.0, 0.0, 0.0])
    world_genesis = _link_world_transform(link) @ hand_eye_to_genesis_attach_T(calibration)

    class _FakeCamera:
        def get_transform(self):
            return world_genesis

    origin, look, right = camera_axes_from_genesis_camera_object(_FakeCamera(), axis_len_m=0.1)
    world_optical = world_genesis @ _OPTICAL_FROM_GENESIS_CAMERA
    np.testing.assert_allclose(origin, world_optical[:3, 3], atol=1e-6)
    np.testing.assert_allclose(look, world_optical[:3, :3] @ [0.0, 0.0, 0.1], atol=1e-6)
    np.testing.assert_allclose(right, world_optical[:3, :3] @ [0.1, 0.0, 0.0], atol=1e-6)


def test_sim_camera_pub_sub_roundtrip() -> None:
    endpoint = "inproc://simulator_camera_roundtrip"
    publisher = SimCameraPublisher(endpoint, use_jpeg=False)
    subscriber = SimCameraSubscriber(endpoint, use_jpeg=False)
    subscriber.connect()
    try:
        time.sleep(0.05)
        height, width = 48, 64
        frame = SimCameraFrame(
            color_bgr=np.zeros((height, width, 3), dtype=np.uint8),
            depth_raw=np.full((height, width), 500, dtype=np.uint16),
            depth_scale=0.001,
            intrinsics=SimCameraIntrinsics(
                fx=50.0,
                fy=50.0,
                cx=32.0,
                cy=24.0,
                width=width,
                height=height,
            ),
            seq=1,
            ts=time.time(),
            arm_q=(0.0, 0.0, 0.0, 0.0),
        )
        received = None
        for _ in range(20):
            publisher.publish(frame)
            received = subscriber.recv_latest(timeout_ms=100)
            if received is not None:
                break
            time.sleep(0.02)
        assert received is not None
        assert received.color_bgr.shape == (height, width, 3)
        assert received.depth_raw.shape == (height, width)
        assert received.seq == 1
    finally:
        publisher.close()
        subscriber.close()
