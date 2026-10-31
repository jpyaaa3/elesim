from __future__ import annotations

import threading

import numpy as np
import pytest

from elesim_pilot.vision.rgbd import (
    EncodedRgbdFrame,
    RgbdEdgeBroker,
    RgbdEncodingPolicy,
    encode_rgbd_frame,
)
from elesim_pilot.vision.rgbd.edge import _without_depth_for_preview
from elesim_pilot.vision.rgbd.edge import DdsRgbdEdgePublisher, DdsRgbdRelay
from types import SimpleNamespace
from elesim_pilot.vision.rgbd.types import RgbdFrame, RgbdIntrinsics
from elesim_protocol.encoded_rgbd import RgbdEncodedMetadata, encode_payload


def _frame(seq: int) -> RgbdFrame:
    return RgbdFrame(
        color_bgr=np.full((2, 3, 3), seq, dtype=np.uint8),
        depth_raw=np.full((2, 3), seq, dtype=np.uint16),
        depth_scale=0.001,
        intrinsics=RgbdIntrinsics(1.0, 1.0, 1.0, 1.0, 3, 2),
        seq=seq,
        ts=float(seq),
    )


def _encoded(seq: int) -> EncodedRgbdFrame:
    return EncodedRgbdFrame(
        source_id="pilot-main",
        source_boot_id="boot-1",
        seq=seq,
        ts=float(seq),
        width=3,
        height=2,
        color_codec="zlib",
        color_encoding="bgr8",
        color_payload=encode_payload(b"color", "zlib"),
        depth_codec="zlib",
        depth_encoding="16UC1",
        depth_payload=encode_payload(b"depth", "zlib"),
        depth_scale=0.001,
        metadata=RgbdEncodedMetadata(1.0, 1.0, 1.0, 1.0),
    )


def test_encoded_input_is_passed_through_without_reencoding() -> None:
    calls: list[int] = []

    def encoder(frame: RgbdFrame) -> EncodedRgbdFrame:
        calls.append(frame.seq)
        return _encoded(frame.seq)

    broker = RgbdEdgeBroker(encoder=encoder, worker=False)
    incoming = _encoded(7)
    assert broker.submit(incoming)
    assert broker.recv_latest() is incoming
    assert calls == []
    stats = broker.stats()
    assert stats.passed_through == 1
    assert stats.encoded == 0
    broker.close()


def test_worker_keeps_one_latest_pending_source() -> None:
    started = threading.Event()
    release = threading.Event()
    calls: list[int] = []

    def encoder(frame: RgbdFrame) -> EncodedRgbdFrame:
        calls.append(frame.seq)
        if frame.seq == 1:
            started.set()
            assert release.wait(1.0)
        return _encoded(frame.seq)

    broker = RgbdEdgeBroker(encoder=encoder)
    assert broker.submit(_frame(1))
    assert started.wait(1.0)
    assert broker.submit(_frame(2))
    assert broker.submit(_frame(3))
    release.set()
    result = broker.recv_latest(timeout_s=1.0)
    assert result is not None
    assert result.seq == 3
    assert calls == [1, 3]
    assert broker.stats().replaced == 1
    broker.close()


def test_submit_does_not_mutate_source_or_copy_before_encoder() -> None:
    source = _frame(4)
    seen: list[RgbdFrame] = []

    def encoder(frame: RgbdFrame) -> EncodedRgbdFrame:
        seen.append(frame)
        return _encoded(frame.seq)

    broker = RgbdEdgeBroker(encoder=encoder, worker=False)
    original_color = source.color_bgr.copy()
    original_depth = source.depth_raw.copy()
    assert broker.submit(source)
    assert np.array_equal(source.color_bgr, original_color)
    assert np.array_equal(source.depth_raw, original_depth)
    assert seen == [source]
    broker.close()


def test_default_policy_encodes_color_and_depth_once() -> None:
    encoded = encode_rgbd_frame(
        _frame(5),
        policy=RgbdEncodingPolicy(color="raw", depth="zlib"),
    )
    assert encoded.color_codec == "raw"
    assert encoded.depth_codec == "zlib"
    assert encoded.color_payload == _frame(5).color_bgr.tobytes()
    assert encoded.depth_payload != _frame(5).depth_raw.tobytes()
    assert encoded.seq == 5


def test_left_preview_strips_depth_without_changing_color_or_identity() -> None:
    source = _encoded(9)

    preview = _without_depth_for_preview(source)

    assert preview.color_payload == source.color_payload
    assert preview.color_codec == source.color_codec
    assert preview.seq == source.seq
    assert preview.source_id == source.source_id
    assert preview.source_boot_id == source.source_boot_id
    assert preview.metadata == source.metadata
    assert preview.has_depth is False
    assert preview.depth_codec == ""
    assert preview.depth_encoding == ""
    assert preview.depth_payload == b""
    assert preview.depth_scale == 0.0


def test_close_keeps_native_publisher_alive_until_callback_has_stopped(monkeypatch):
    from elesim_protocol import encoded_rgbd
    entered,release = threading.Event(),threading.Event()
    class Publisher:
        closed = False
        def __init__(self,*args,**kwargs): pass
        def publish(self,frame):
            entered.set()
            release.wait(3.)
        def close(self): self.closed = True
    monkeypatch.setattr(encoded_rgbd,'DdsEncodedRgbdPublisher',Publisher)
    publisher = DdsRgbdEdgePublisher('/rgbd',endpoint_id='test')
    publisher.submit(_encoded(7))
    assert entered.wait(1.)
    try:
        with pytest.raises(TimeoutError): publisher.close(timeout_s=.001)
        assert publisher.broker._thread.is_alive() and not publisher._publisher.closed
    finally:
        release.set()
        publisher.close()
    assert publisher._publisher.closed and publisher.broker._thread is None


def test_relay_close_cannot_destroy_a_subscriber_still_spinning():
    entered,release = threading.Event(),threading.Event()
    closed = []
    def receive(**kwargs):
        entered.set()
        release.wait(3.)
        return None
    relay = DdsRgbdRelay.__new__(DdsRgbdRelay)
    relay._stop = threading.Event()
    relay._subscriber = SimpleNamespace(connect=lambda:None,recv_latest=receive,
        close=lambda:closed.append('subscriber'))
    relay._publisher = SimpleNamespace(close=lambda:closed.append('publisher'))
    relay._thread = threading.Thread(target=relay._run)
    relay.start()
    assert entered.wait(1.)
    try:
        with pytest.raises(TimeoutError): relay.close(timeout_s=.001)
        assert relay._thread.is_alive() and closed == []
    finally:
        release.set()
        relay.close()
    assert closed == ['subscriber','publisher']
