from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "AGENTS.md").exists())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from elesim_controller.vision.perception_bridge.hand_eye import load_hand_eye_transform
from elesim_controller.vision.perception_bridge.transforms import transform_point


class TestHandEye(unittest.TestCase):
    def test_node9_mount_axes(self) -> None:
        cfg_path = ROOT / "controller" / "config" / "calibration" / "hand_eye.camera.json"
        T, meta = load_hand_eye_transform(cfg_path)
        self.assertEqual(meta["parent_frame"], "node9")
        p_look = transform_point(T, [0.0, 0.0, 0.5])
        np.testing.assert_allclose(p_look, [0.58, 0.025, -0.025], atol=1e-6)
        p_right = transform_point(T, [0.1, 0.0, 0.0])
        np.testing.assert_allclose(p_right, [0.08, -0.075, -0.025], atol=1e-6)


if __name__ == "__main__":
    unittest.main()
