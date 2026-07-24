from __future__ import annotations

from threading import Lock
from types import SimpleNamespace

import numpy as np

from elesim_protocol import DdsRgbdSubscriber


def _image(
    data: bytes,
    *,
    width: int,
    height: int,
    encoding: str,
    step: int,
) -> SimpleNamespace:
    return SimpleNamespace(
        data=data,
        width=width,
        height=height,
        encoding=encoding,
        step=step,
        is_bigendian=False,
    )


def _message(*, source_id: str, boot_id: str) -> SimpleNamespace:
    width, height = 4, 3
    color = np.arange(width * height * 3, dtype=np.uint8).reshape(
        height,
        width,
        3,
    )
    depth = np.arange(width * height, dtype=np.uint16).reshape(height, width)
    return SimpleNamespace(
        source=SimpleNamespace(endpoint_id=source_id, boot_id=boot_id),
        frame_sequence=7,
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=12, nanosec=500_000_000)
        ),
        color=_image(
            color.tobytes(),
            width=width,
            height=height,
            encoding="bgr8",
            step=width * 3,
        ),
        depth=_image(
            depth.tobytes(),
            width=width,
            height=height,
            encoding="16UC1",
            step=width * 2,
        ),
        camera_info=SimpleNamespace(
            k=[100.0, 0.0, 2.0, 0.0, 101.0, 1.5, 0.0, 0.0, 1.0]
        ),
        depth_scale=0.001,
        has_arm_q=True,
        arm_q=[-0.1, 0.2, 0.3, -0.4],
        has_camera_pose=True,
        camera_world_origin=[1.0, 2.0, 3.0],
        camera_world_look=[0.0, 0.0, 1.0],
        camera_world_right=[1.0, 0.0, 0.0],
    )


def test_typed_rgbd_decode_preserves_coherent_frame_metadata() -> None:
    sample = DdsRgbdSubscriber._decode(
        _message(source_id="sim-a", boot_id="boot-a")
    )
    assert sample.source_id == "sim-a"
    assert sample.source_boot_id == "boot-a"
    assert sample.seq == 7
    assert sample.ts == 12.5
    assert sample.color_bgr.shape == (3, 4, 3)
    assert sample.depth_raw.shape == (3, 4)
    assert sample.depth_raw.dtype == np.uint16
    assert sample.arm_q == (-0.1, 0.2, 0.3, -0.4)
    assert sample.camera_world_origin == (1.0, 2.0, 3.0)


def test_selected_peer_boot_filter_rejects_stale_rgbd_writer() -> None:
    subscriber = object.__new__(DdsRgbdSubscriber)
    subscriber._lock = Lock()
    subscriber._latest = None
    subscriber._expected_source_id = "sim-a"
    subscriber._expected_boot_id = "boot-new"

    subscriber._receive(_message(source_id="sim-a", boot_id="boot-old"))
    assert subscriber._latest is None
    accepted = _message(source_id="sim-a", boot_id="boot-new")
    subscriber._receive(accepted)
    assert subscriber._latest is accepted
