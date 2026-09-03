from __future__ import annotations

import numpy as np

from elesim_sim.vision.frame_hub import FrameHub
from elesim_sim.vision.sim_camera.types import SimCameraFrame, SimCameraIntrinsics


def frame(seq: int, value: int) -> SimCameraFrame:
    return SimCameraFrame(
        color_bgr=np.full((3, 4, 3), value, dtype=np.uint8),
        depth_raw=np.full((3, 4), value, dtype=np.uint16),
        depth_scale=0.001,
        intrinsics=SimCameraIntrinsics(fx=1, fy=1, cx=2, cy=1.5, width=4, height=3),
        seq=seq,
        ts=float(seq),
    )


def test_frame_hub_keeps_only_the_latest_frame_per_named_stream() -> None:
    hub = FrameHub(("rgbd", "observer", "hand_eye_preview"))
    first = frame(1, 10)
    latest = frame(2, 20)
    hub.publish("rgbd", first)
    hub.publish("rgbd", latest)

    assert hub.latest("rgbd") is latest
    assert int(hub.latest_bgr("rgbd")[0, 0, 0]) == 20
    assert hub.version("rgbd") == 2


def test_frame_hub_streams_are_independent() -> None:
    hub = FrameHub(("rgbd", "observer", "hand_eye_preview"))
    observer = frame(1, 30)
    hand_eye = frame(1, 40)
    hub.publish("observer", observer)
    hub.publish("hand_eye_preview", hand_eye)

    assert hub.latest("observer") is observer
    assert hub.latest("hand_eye_preview") is hand_eye
    assert hub.latest("rgbd") is None
