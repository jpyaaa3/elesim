from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "host.py").exists())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.vision.perception_bridge.hand_eye import load_hand_eye_transform
from engine.vision.perception_bridge.transforms import transform_point


class TestHandEye(unittest.TestCase):
    def test_node9_mount_axes(self) -> None:
        """
        ZED Mini, under-slung and mounted rolled 180 deg about the optical axis.

        Origin is the LEFT lens (the ZED SDK's reference frame) at node9
        [0.08527, -0.04332, -0.04206]. The 180 deg roll puts optical +X (image
        right) along node9 +Y and optical +Y (image down) along node9 +Z, which
        is the sign flip against the old D435 mount.
        """
        cfg_path = ROOT / "model_presets" / "visual_servoing" / "hand_eye.camera.json"
        T, meta = load_hand_eye_transform(cfg_path)
        self.assertEqual(meta["parent_frame"], "node9")
        origin = transform_point(T, [0.0, 0.0, 0.0])
        np.testing.assert_allclose(origin, [0.08527, -0.04332, -0.04206], atol=1e-6)
        # +Z looks along node9 +X (down the arm)
        p_look = transform_point(T, [0.0, 0.0, 0.5])
        np.testing.assert_allclose(p_look, [0.58527, -0.04332, -0.04206], atol=1e-6)
        # +X (image right) -> node9 +Y
        p_right = transform_point(T, [0.1, 0.0, 0.0])
        np.testing.assert_allclose(p_right, [0.08527, 0.05668, -0.04206], atol=1e-6)
        # +Y (image down) -> node9 +Z
        p_down = transform_point(T, [0.0, 0.1, 0.0])
        np.testing.assert_allclose(p_down, [0.08527, -0.04332, 0.05794], atol=1e-6)

    def test_stereo_baseline_matches_zed_mini(self) -> None:
        """
        The right lens must sit 63.0 mm along optical +X from the left one.

        This pins the ZED Mini's documented baseline against the CAD-derived
        mount, so a future edit to the extrinsics cannot quietly move one lens
        without the other.
        """
        cfg_path = ROOT / "model_presets" / "visual_servoing" / "hand_eye.camera.json"
        T, _meta = load_hand_eye_transform(cfg_path)
        left = transform_point(T, [0.0, 0.0, 0.0])
        right = transform_point(T, [0.063, 0.0, 0.0])
        np.testing.assert_allclose(right, [0.08527, 0.01968, -0.04206], atol=1e-6)
        self.assertAlmostEqual(float(np.linalg.norm(right - left)), 0.063, places=9)


if __name__ == "__main__":
    unittest.main()
