from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from addons.perception_bridge.hand_eye import load_hand_eye_transform
from addons.perception_bridge.transforms import transform_point


class TestHandEye(unittest.TestCase):
    def test_node9_mount_axes(self) -> None:
        cfg_path = ROOT / "configs" / "hand_eye.node9_mount.json"
        T, meta = load_hand_eye_transform(cfg_path)
        self.assertEqual(meta["parent_frame"], "node9")
        p_look = transform_point(T, [0.0, 0.0, 0.5])
        np.testing.assert_allclose(p_look, [0.58, 0.03, -0.04], atol=1e-6)
        p_right = transform_point(T, [0.1, 0.0, 0.0])
        np.testing.assert_allclose(p_right, [0.08, -0.07, -0.04], atol=1e-6)


if __name__ == "__main__":
    unittest.main()
