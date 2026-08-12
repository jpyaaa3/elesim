from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from elesim_protocol.rgbd import DdsRgbdPublisher, RgbdFrame, RgbdIntrinsics


def test_application_rgbd_frame_metadata_is_protocol_owned() -> None:
    frame = RgbdFrame(
        color_bgr=np.zeros((2, 3, 3), dtype=np.uint8),
        depth_raw=np.zeros((2, 3), dtype=np.uint16),
        depth_scale=0.001,
        intrinsics=RgbdIntrinsics(
            fx=1.0,
            fy=2.0,
            cx=3.0,
            cy=4.0,
            width=3,
            height=2,
        ),
        seq=7,
        ts=8.5,
        arm_q=(1.0, 2.0, 3.0, 4.0),
        camera_world_origin=(5.0, 6.0, 7.0),
    )

    assert frame.to_meta_dict() == {
        "t": "rgbd_frame",
        "seq": 7,
        "ts": 8.5,
        "width": 3,
        "height": 2,
        "fx": 1.0,
        "fy": 2.0,
        "cx": 3.0,
        "cy": 4.0,
        "depth_scale": 0.001,
        "arm_q": [1.0, 2.0, 3.0, 4.0],
        "camera_world_origin": [5.0, 6.0, 7.0],
    }


def _publisher_with(fake_publish, *, message_factory=None) -> DdsRgbdPublisher:
    publisher = object.__new__(DdsRgbdPublisher)
    publisher._closed = False
    publisher.endpoint_id = "sim-default"
    publisher.boot_id = "boot"
    publisher.send_depth = True
    publisher.wall_clock = lambda: 10.0
    publisher.published = 0
    publisher.dropped = 0
    publisher.last_drop_reason = ""

    def message():
        header = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id="")
        return SimpleNamespace(
            source=SimpleNamespace(endpoint_id="", boot_id=""),
            header=header,
            color=SimpleNamespace(),
            depth=SimpleNamespace(),
            camera_info=SimpleNamespace(),
        )

    publisher._RosRgbdFrame = message if message_factory is None else message_factory
    publisher._publisher = SimpleNamespace(publish=fake_publish)
    return publisher


def _frame() -> RgbdFrame:
    return RgbdFrame(
        color_bgr=np.zeros((2, 3, 3), dtype=np.uint8),
        depth_raw=np.zeros((2, 3), dtype=np.uint16),
        depth_scale=0.001,
        intrinsics=RgbdIntrinsics(1.0, 2.0, 3.0, 4.0, 3, 2),
    )


def test_malformed_rgbd_frame_is_counted_as_a_drop() -> None:
    publisher = _publisher_with(lambda _message: None)
    malformed = _frame()
    malformed = RgbdFrame(
        color_bgr=malformed.color_bgr[:, :, 0],
        depth_raw=malformed.depth_raw,
        depth_scale=malformed.depth_scale,
        intrinsics=malformed.intrinsics,
    )

    assert publisher.publish(malformed) is False
    assert publisher.dropped == 1
    assert "shape HxWx3" in publisher.last_drop_reason


@pytest.mark.parametrize(
    ("changes", "reason"),
    (
        ({"seq": -1}, "sequence must fit uint64"),
        ({"arm_q": (1.0, 2.0, 3.0)}, "arm_q must contain 4"),
        ({"camera_world_origin": (1.0, 2.0)}, "camera_world_origin must contain 3"),
    ),
)
def test_rosidl_fixed_field_violations_are_malformed_frame_drops(
    changes: dict[str, object], reason: str
) -> None:
    publisher = _publisher_with(lambda _message: None)
    values = dict(vars(_frame()))
    values.update(changes)

    assert publisher.publish(RgbdFrame(**values)) is False
    assert publisher.dropped == 1
    assert reason in publisher.last_drop_reason


def test_dds_publish_failure_is_not_hidden_as_a_frame_drop() -> None:
    def fail(_message) -> None:
        raise RuntimeError("RMW publisher failed")

    publisher = _publisher_with(fail)

    with pytest.raises(RuntimeError, match="RMW publisher failed"):
        publisher.publish(_frame())
    assert publisher.dropped == 0
    assert publisher.published == 0


def test_stale_rosidl_message_is_not_hidden_as_a_frame_drop() -> None:
    class StaleRgbdMessage:
        __slots__ = (
            "source",
            "header",
            "color",
            "depth",
            "camera_info",
            "frame_sequence",
            "depth_scale",
        )

        def __init__(self) -> None:
            self.source = SimpleNamespace(endpoint_id="", boot_id="")
            self.header = SimpleNamespace(
                stamp=SimpleNamespace(sec=0, nanosec=0), frame_id=""
            )
            self.color = SimpleNamespace()
            self.depth = SimpleNamespace()
            self.camera_info = SimpleNamespace()
            self.frame_sequence = 0
            self.depth_scale = 0.0

    publisher = _publisher_with(
        lambda _message: None,
        message_factory=StaleRgbdMessage,
    )

    with pytest.raises(AttributeError, match="has_arm_q"):
        publisher.publish(_frame())
    assert publisher.dropped == 0
    assert publisher.published == 0
