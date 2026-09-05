from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "payload").is_dir())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from elesim_pilot.vision.perception_bridge.hand_eye import load_hand_eye_transform
from elesim_pilot.vision.perception_bridge.transforms import transform_point


class TestHandEye(unittest.TestCase):
    def test_node9_mount_axes(self) -> None:
        cfg_path = ROOT / "payload/data/calibration/cameras/zed_mini.hand_eye.json"
        T, meta = load_hand_eye_transform(cfg_path)
        self.assertEqual(meta["parent_frame"], "node9")
        p_look = transform_point(T, [0.0, 0.0, 0.5])
        np.testing.assert_allclose(p_look, [0.58527, -0.04332, -0.04206], atol=1e-6)
        p_right = transform_point(T, [0.1, 0.0, 0.0])
        np.testing.assert_allclose(p_right, [0.08527, 0.05668, -0.04206], atol=1e-6)


if __name__ == "__main__":
    unittest.main()
