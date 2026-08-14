from __future__ import annotations

import asyncio

import numpy as np

from elesim_sim.vision.webrtc import LatestFrameTrack


def _recv(track: LatestFrameTrack):
    return asyncio.run(track.recv())


def test_latest_frame_track_uses_black_frame_after_provider_failure() -> None:
    errors: list[str] = []

    def provider() -> np.ndarray:
        raise RuntimeError("camera temporarily unavailable")

    track = LatestFrameTrack(provider, fps=1_000.0, on_error=errors.append)
    frame = _recv(track).to_ndarray(format="bgr24")

    assert frame.shape == (480, 640, 3)
    assert not frame.any()
    assert errors == ["provider: camera temporarily unavailable"]


def test_latest_frame_track_recovers_after_invalid_frame() -> None:
    frames = iter(
        (
            np.zeros((0, 0, 3), dtype=np.uint8),
            np.full((12, 16, 3), 7, dtype=np.uint8),
        )
    )
    errors: list[str] = []
    track = LatestFrameTrack(lambda: next(frames), fps=1_000.0, on_error=errors.append)

    fallback = _recv(track).to_ndarray(format="bgr24")
    recovered = _recv(track).to_ndarray(format="bgr24")

    assert fallback.shape == (480, 640, 3)
    assert not fallback.any()
    assert recovered.shape == (12, 16, 3)
    assert int(recovered.mean()) == 7
    assert errors and errors[0].startswith("encode:")


def test_latest_frame_track_uses_configured_dimensions_during_warmup() -> None:
    errors: list[str] = []
    track = LatestFrameTrack(
        lambda: None,
        fps=1_000.0,
        frame_size=(960, 540),
        on_error=errors.append,
    )

    frame = _recv(track).to_ndarray(format="bgr24")

    assert frame.shape == (540, 960, 3)
    assert not frame.any()
    assert errors == []


def test_latest_frame_track_keeps_configured_dimensions_for_real_frames() -> None:
    source = np.full((12, 16, 3), 7, dtype=np.uint8)
    track = LatestFrameTrack(
        lambda: source,
        fps=1_000.0,
        frame_size=(32, 24),
    )

    frame = _recv(track).to_ndarray(format="bgr24")

    assert frame.shape == (24, 32, 3)
    assert int(frame.mean()) == 7
