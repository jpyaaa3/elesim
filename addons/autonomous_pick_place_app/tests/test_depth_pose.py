"""Tests for depth-based camera-frame position estimation."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from perception.depth_pose import CameraIntrinsics, estimate_object_position_camera


class TestDepthPose(unittest.TestCase):
    def test_synthetic_depth_median_position(self) -> None:
        fx, fy, cx, cy = 600.0, 600.0, 320.0, 240.0
        w, h = 640, 480
        z_m = 0.8
        depth_scale = 0.001

        u0, v0, u1, v1 = 300, 220, 340, 260
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[v0 : v1 + 1, u0 : u1 + 1] = 255

        depth_raw = np.zeros((h, w), dtype=np.uint16)
        depth_raw[mask > 0] = int(round(z_m / depth_scale))

        intrinsics = CameraIntrinsics(fx=fx, fy=fy, cx=cx, cy=cy, width=w, height=h)
        p = estimate_object_position_camera(mask, depth_raw, intrinsics, depth_scale)

        u_mid = 0.5 * (u0 + u1)
        v_mid = 0.5 * (v0 + v1)
        expected_x = (u_mid - cx) * z_m / fx
        expected_y = (v_mid - cy) * z_m / fy

        np.testing.assert_allclose(p[2], z_m, atol=0.02)
        np.testing.assert_allclose(p[0], expected_x, atol=0.03)
        np.testing.assert_allclose(p[1], expected_y, atol=0.03)

    def test_rejects_empty_mask(self) -> None:
        depth = np.ones((10, 10), dtype=np.uint16) * 500
        mask = np.zeros((10, 10), dtype=np.uint8)
        intrinsics = CameraIntrinsics(fx=100, fy=100, cx=5, cy=5, width=10, height=10)
        with self.assertRaises(RuntimeError):
            estimate_object_position_camera(mask, depth, intrinsics, 0.001)


if __name__ == "__main__":
    unittest.main()
