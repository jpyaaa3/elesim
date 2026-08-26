from __future__ import annotations

import threading

import numpy as np

from elesim_pilot.vision.rgbd import (
    EncodedRgbdFrame,
    RgbdEdgeBroker,
    RgbdEncodingPolicy,
    encode_rgbd_frame,
)
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
