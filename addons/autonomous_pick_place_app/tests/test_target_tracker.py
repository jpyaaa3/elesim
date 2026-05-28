"""Unit tests for ROI depth target tracker (synthetic depth, no camera/YOLO)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from perception.depth_pose import CameraIntrinsics
from perception.detector import DetectionResult
from perception.target_tracker import TargetTracker, TrackState, _window_median_mad


def _synthetic_frame(
    *,
    z_m: float = 0.65,
    cx: int = 320,
    cy: int = 240,
    w: int = 640,
    h: int = 480,
) -> tuple[np.ndarray, CameraIntrinsics, float]:
    depth_scale = 0.001
    depth_raw = np.zeros((h, w), dtype=np.uint16)
    depth_raw[:, :] = int(round(z_m / depth_scale))
    # hole in center to test valid ratio when needed
    intrinsics = CameraIntrinsics(fx=615.0, fy=615.0, cx=320.0, cy=240.0, width=w, height=h)
    color = np.zeros((h, w, 3), dtype=np.uint8)
    _ = color
    return depth_raw, intrinsics, depth_scale


class TestWindowStats(unittest.TestCase):
    def test_median_mad_constant_samples(self) -> None:
        samples = [np.array([0.01, -0.02, 0.65], dtype=float) for _ in range(10)]
        mu, sigma = _window_median_mad(samples)
        np.testing.assert_allclose(mu, [0.01, -0.02, 0.65], atol=1e-9)
        np.testing.assert_allclose(sigma, [1e-6, 1e-6, 1e-6], atol=1e-5)


class TestTargetTracker(unittest.TestCase):
    def test_lock_and_track_produces_mu(self) -> None:
        depth, intrinsics, scale = _synthetic_frame(z_m=0.70)
        tracker = TargetTracker(window_size=10, min_samples=3, lost_frames_threshold=20)
        det = DetectionResult(
            mask=np.zeros((480, 640), dtype=np.uint8),
            bbox_xyxy=(300, 220, 340, 260),
            label="cup",
            confidence=0.9,
        )
        self.assertTrue(tracker.try_lock(det, width=640, height=480))
        self.assertEqual(tracker.state, TrackState.TRACKING_3D)

        packet = None
        for _ in range(5):
            packet = tracker.update(depth_raw=depth, intrinsics=intrinsics, depth_scale=scale)
        assert packet is not None
        self.assertEqual(packet.track_state, TrackState.TRACKING_3D.value)
        self.assertTrue(packet.publishable)
        self.assertIsNotNone(packet.mu_camera)
        self.assertIsNotNone(packet.sigma_camera)
        assert packet.mu_camera is not None
        self.assertAlmostEqual(float(packet.mu_camera[2]), 0.70, delta=0.05)
        self.assertGreater(packet.track_confidence, 0.3)
        self.assertGreater(packet.depth_valid_ratio, 0.5)

    def test_depth_hole_causes_lost(self) -> None:
        depth, intrinsics, scale = _synthetic_frame(z_m=0.65)
        tracker = TargetTracker(
            window_size=5,
            min_samples=2,
            min_depth_valid_ratio=0.15,
            lost_frames_threshold=3,
        )
        det = DetectionResult(
            mask=np.zeros((480, 640), dtype=np.uint8),
            bbox_xyxy=(300, 220, 340, 260),
            label="obj",
            confidence=0.8,
        )
        tracker.try_lock(det, width=640, height=480)
        for _ in range(4):
            tracker.update(depth_raw=depth, intrinsics=intrinsics, depth_scale=scale)

        # zero depth in ROI
        bad = depth.copy()
        bad[220:261, 300:341] = 0
        packet = None
        for _ in range(5):
            packet = tracker.update(depth_raw=bad, intrinsics=intrinsics, depth_scale=scale)
        assert packet is not None
        self.assertEqual(packet.track_state, TrackState.LOST.value)
        self.assertGreater(packet.lost_count, 0)

    def test_search_does_not_need_lock_for_update(self) -> None:
        depth, intrinsics, scale = _synthetic_frame()
        tracker = TargetTracker()
        packet = tracker.update(depth_raw=depth, intrinsics=intrinsics, depth_scale=scale)
        self.assertEqual(packet.track_state, TrackState.SEARCH.value)
        self.assertFalse(packet.publishable)

    def test_needs_yolo_only_in_search_and_lost(self) -> None:
        tracker = TargetTracker(redetect_interval_frames=2)
        self.assertTrue(tracker.needs_yolo())
        det = DetectionResult(
            mask=np.zeros((480, 640), dtype=np.uint8),
            bbox_xyxy=(10, 10, 50, 50),
            label="x",
            confidence=1.0,
        )
        tracker.try_lock(det, width=640, height=480)
        self.assertFalse(tracker.needs_yolo())
        tracker.state = TrackState.LOST
        tracker._frame_idx = 0
        self.assertTrue(tracker.needs_yolo())
        tracker._frame_idx = 1
        self.assertFalse(tracker.needs_yolo())


if __name__ == "__main__":
    unittest.main()
