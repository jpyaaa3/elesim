from __future__ import annotations

import time

import numpy as np

from elesim_controller.vision.sim_camera.publisher import SimCameraPublisher
from elesim_controller.vision.sim_camera.subscriber import SimCameraSubscriber
from elesim_controller.vision.sim_camera.types import SimCameraFrame, SimCameraIntrinsics


def test_sim_camera_pub_sub_roundtrip() -> None:
    endpoint = "inproc://sim_camera_test"
    pub = SimCameraPublisher(endpoint, use_jpeg=False)
    time.sleep(0.05)
    sub = SimCameraSubscriber(endpoint, use_jpeg=False)
    sub.connect()
    time.sleep(0.05)
    h, w = 48, 64
    intr = SimCameraIntrinsics(fx=50.0, fy=50.0, cx=32.0, cy=24.0, width=w, height=h)
    color = np.zeros((h, w, 3), dtype=np.uint8)
    color[:, :, 2] = 200
    depth = np.full((h, w), 500, dtype=np.uint16)
    frame = SimCameraFrame(
        color_bgr=color,
        depth_raw=depth,
        depth_scale=0.001,
        intrinsics=intr,
        seq=1,
        ts=time.time(),
        arm_q=(0.0, 0.0, 0.0, 0.0),
    )
    assert pub.publish(frame)
    got = None
    for _ in range(20):
        got = sub.recv_latest(timeout_ms=100)
        if got is not None:
            break
        pub.publish(frame)
        time.sleep(0.02)
    assert got is not None
    assert got.color_bgr.shape == (h, w, 3)
    assert got.depth_raw.shape == (h, w)
    assert int(got.seq) == 1
    meta = frame.to_meta_dict()
    assert meta["t"] == "sim_camera_frame"
    pub.close()
    sub.close()
