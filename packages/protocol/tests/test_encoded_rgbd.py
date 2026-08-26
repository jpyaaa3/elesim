from __future__ import annotations

import math

import numpy as np
import pytest

from elesim_protocol.encoded_rgbd import (
    DdsEncodedRgbdPublisher,
    DdsEncodedRgbdSubscriber,
    MAX_ENCODED_PAYLOAD_BYTES,
    EncodedRgbdFrame,
    RgbdEncodedMetadata,
    decode_payload,
    encode_payload,
    validate_encoded_frame,
    decode_image_payload,
    encode_image_payload,
)


def _frame(**changes: object) -> EncodedRgbdFrame:
    values: dict[str, object] = {
        "source_id": "pilot-main",
        "source_boot_id": "boot-1",
        "seq": 7,
        "ts": 8.5,
        "width": 3,
        "height": 2,
        "color_codec": "zlib",
        "color_encoding": "bgr8",
        "color_payload": encode_payload(b"c" * 18, "zlib"),
        "depth_codec": "zlib",
        "depth_encoding": "16UC1",
        "depth_payload": encode_payload(b"d" * 12, "zlib"),
        "depth_scale": 0.001,
        "metadata": RgbdEncodedMetadata(
            fx=1.0,
            fy=2.0,
            cx=3.0,
            cy=4.0,
            calibration_id="zed-mini-v1",
        ),
    }
    values.update(changes)
    return EncodedRgbdFrame(**values)  # type: ignore[arg-type]


def test_payload_round_trip_is_deterministic() -> None:
    payload = bytes(range(32))
    encoded = encode_payload(payload, "zlib")

    assert encoded != payload
    assert decode_payload(encoded, "zlib") == payload
    assert decode_payload(payload, "raw") == payload


def test_image_codecs_round_trip_without_a_second_publisher_decode() -> None:
    pytest.importorskip("cv2")
    color = np.zeros((3, 4, 3), dtype=np.uint8)
    color[1, 2] = (10, 20, 30)
    encoded = encode_image_payload(color, "jpeg", encoding="bgr8", quality=90)
    decoded = decode_image_payload(
        encoded, "jpeg", width=4, height=3, encoding="bgr8"
    )
    assert decoded.shape == color.shape
    assert decoded.dtype == np.uint8

    depth = np.arange(12, dtype=np.uint16).reshape(3, 4)
    encoded_depth = encode_image_payload(depth, "png", encoding="16uc1")
    np.testing.assert_array_equal(
        decode_image_payload(
            encoded_depth, "png", width=4, height=3, encoding="16uc1"
        ),
        depth,
    )


def test_encoded_frame_validates_depth_presence_and_metadata() -> None:
    frame = _frame()

    assert validate_encoded_frame(frame) is frame
    assert frame.has_depth is True
    assert frame.metadata.calibration_id == "zed-mini-v1"

    no_depth = _frame(
        depth_codec="",
        depth_encoding="",
        depth_payload=b"",
        depth_scale=0.0,
        has_depth=False,
    )
    assert validate_encoded_frame(no_depth) is no_depth


@pytest.mark.parametrize(
    ("changes", "message"),
    (
        ({"seq": -1}, "sequence must fit uint64"),
        ({"width": 0}, "dimensions must be positive"),
        ({"color_codec": "av1"}, "unsupported color codec"),
        (
            {
                "has_depth": True,
                "depth_codec": "",
                "depth_encoding": "16UC1",
                "depth_payload": b"",
            },
            "depth codec",
        ),
        ({"has_depth": False, "depth_payload": b"not-empty"}, "depth payload"),
        ({"depth_scale": -1.0}, "depth scale"),
        ({"ts": math.nan}, "timestamp must be finite"),
        ({"metadata": RgbdEncodedMetadata(1.0, 0.0, 3.0, 4.0)}, "intrinsics"),
    ),
)
def test_encoded_frame_rejects_malformed_wire_values(
    changes: dict[str, object], message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        validate_encoded_frame(_frame(**changes))


def test_encoded_frame_rejects_payload_over_bound() -> None:
    with pytest.raises(ValueError, match="payload exceeds"):
        validate_encoded_frame(
            _frame(color_payload=b"x" * (MAX_ENCODED_PAYLOAD_BYTES + 1))
        )


def test_has_depth_is_derived_from_payload_state_when_omitted() -> None:
    frame = _frame(depth_codec="", depth_encoding="", depth_payload=b"")

    assert frame.has_depth is False
    validate_encoded_frame(frame)


def test_dds_publisher_maps_encoded_frame_without_raw_image_fields() -> None:
    sent: list[object] = []
    publisher = object.__new__(DdsEncodedRgbdPublisher)
    publisher._closed = False
    publisher.endpoint_id = "pilot-main"
    publisher.boot_id = "boot-1"
    publisher.wall_clock = lambda: 10.0
    publisher.published = 0
    publisher.dropped = 0
    publisher.last_drop_reason = ""

    class Message:
        def __init__(self) -> None:
            self.source = type("Source", (), {})()
            self.capture_time = type("Time", (), {"sec": 0, "nanosec": 0})()

    publisher._RosEncodedRgbdFrame = Message
    publisher._publisher = type("Publisher", (), {"publish": lambda _self, value: sent.append(value)})()

    frame = _frame()
    assert publisher.publish(frame) is True
    assert publisher.published == 1
    message = sent[0]
    assert message.source.endpoint_id == "pilot-main"
    assert message.frame_sequence == 7
    assert message.color_data == list(frame.color_payload)
    assert message.depth_data == list(frame.depth_payload)
    assert message.has_depth is True


def test_dds_publisher_maps_numpy_pose_metadata_without_truthiness_check() -> None:
    sent: list[object] = []
    publisher = object.__new__(DdsEncodedRgbdPublisher)
    publisher._closed = False
    publisher.endpoint_id = "pilot-main"
    publisher.boot_id = "boot-1"
    publisher.wall_clock = lambda: 10.0
    publisher.published = 0
    publisher.dropped = 0
    publisher.last_drop_reason = ""

    class Message:
        def __init__(self) -> None:
            self.source = type("Source", (), {})()
            self.capture_time = type("Time", (), {"sec": 0, "nanosec": 0})()

    publisher._RosEncodedRgbdFrame = Message
    publisher._publisher = type(
        "Publisher", (), {"publish": lambda _self, value: sent.append(value)}
    )()

    frame = _frame(
        metadata=RgbdEncodedMetadata(
            fx=1.0,
            fy=2.0,
            cx=3.0,
            cy=4.0,
            arm_q=np.array([0.1, 0.2, 0.3, 0.4]),
            camera_world_origin=np.array([1.0, 2.0, 3.0]),
            camera_world_look=np.array([4.0, 5.0, 6.0]),
            camera_world_right=np.array([7.0, 8.0, 9.0]),
        )
    )

    assert publisher.publish(frame) is True
    message = sent[0]
    assert message.arm_q == [0.1, 0.2, 0.3, 0.4]
    assert message.camera_world_origin == [1.0, 2.0, 3.0]
    assert message.camera_world_look == [4.0, 5.0, 6.0]
    assert message.camera_world_right == [7.0, 8.0, 9.0]


def test_dds_subscriber_decodes_and_validates_latest_sample() -> None:
    message = type("Message", (), {})()
    message.source = type("Source", (), {"endpoint_id": "pilot-main", "boot_id": "boot-1"})()
    message.capture_time = type("Time", (), {"sec": 8, "nanosec": 500_000_000})()
    message.frame_sequence = 7
    message.width = 3
    message.height = 2
    message.color_codec = "zlib"
    message.color_encoding = "bgr8"
    message.color_data = list(encode_payload(b"c" * 18, "zlib"))
    message.has_depth = True
    message.depth_codec = "zlib"
    message.depth_encoding = "16UC1"
    message.depth_data = list(encode_payload(b"d" * 12, "zlib"))
    message.depth_scale = 0.001
    message.fx, message.fy, message.cx, message.cy = 1.0, 2.0, 3.0, 4.0
    message.calibration_id = "zed-mini-v1"
    message.has_arm_q = False
    message.arm_q = [0.0] * 4
    message.has_camera_pose = False
    message.camera_world_origin = [0.0] * 3
    message.camera_world_look = [0.0] * 3
    message.camera_world_right = [0.0] * 3

    decoded = DdsEncodedRgbdSubscriber._decode(message)
    assert decoded.source_id == "pilot-main"
    assert decoded.seq == 7
    assert decoded.metadata.calibration_id == "zed-mini-v1"
    assert decode_payload(decoded.color_payload, decoded.color_codec) == b"c" * 18
