from __future__ import annotations

import threading
import time
from types import SimpleNamespace

import av
import numpy as np

from elesim_ui.sim_view import SimView
from elesim_ui.zed_preview import ZedCameraPreview, _decode_color


class FakeSubscriber:
    def __init__(self, *_args, **_kwargs) -> None:
        self.closed = False
        self.sent = False

    def connect(self) -> None:
        pass

    def recv_latest(self, *, timeout_ms: int):
        if not self.sent:
            self.sent = True
            return SimpleNamespace(source_id="robot-go2")
        time.sleep(min(timeout_ms / 1000.0, 0.01))
        return None

    def close(self) -> None:
        self.closed = True


def test_zed_preview_receives_latest_frame_off_ui_thread() -> None:
    subscriber = FakeSubscriber()
    ready = threading.Event()
    color = SimpleNamespace(shape=(480, 640, 3))
    preview = ZedCameraPreview(
        "/elesim/pilot_main/rgbd/frame",
        endpoint_id="ui-test",
        settings=SimpleNamespace(),
        subscriber_factory=lambda *_args, **_kwargs: subscriber,
        decoder=lambda _frame: (ready.set(), color)[1],
    )
    preview.start()
    try:
        assert ready.wait(timeout=1.0)
        assert preview.frame is color
        assert preview.frame_version == 1
        assert preview.source_id == "robot-go2"
        assert preview.status == "LIVE"
    finally:
        preview.close()
    assert subscriber.closed


def test_zed_camera_view_is_selected_when_available() -> None:
    preview = SimpleNamespace()
    right_preview = SimpleNamespace()

    view = SimView(zed_preview=preview, zed_right_preview=right_preview)

    assert view.state.camera_source == "zed"
    assert view.zed_right_preview is right_preview


def test_jpeg_color_is_decoded_to_bgr_for_the_ui_texture() -> None:
    image = np.zeros((48, 64, 3), dtype=np.uint8)
    image[:, :, 2] = 220
    encoder = av.CodecContext.create("mjpeg", "w")
    encoder.width = 64
    encoder.height = 48
    encoder.pix_fmt = "yuvj420p"
    payload = bytes(
        encoder.encode(av.VideoFrame.from_ndarray(image, format="bgr24"))[0]
    )
    encoded = SimpleNamespace(
        color_codec="jpeg",
        color_encoding="bgr8",
        color_payload=payload,
        width=64,
        height=48,
    )

    decoded = _decode_color(encoded)

    assert decoded.shape == image.shape
    assert decoded.dtype == image.dtype
