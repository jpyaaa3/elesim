"""Tests for YOLO detector helpers (no model download)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from perception.yolo_detector import _bbox_mask, resolve_yolo_device


class TestYoloHelpers(unittest.TestCase):
    def test_resolve_yolo_device(self) -> None:
        self.assertEqual(resolve_yolo_device(2), 2)
        self.assertEqual(resolve_yolo_device("1"), 1)
        self.assertEqual(resolve_yolo_device("cuda:2"), "cuda:2")
        self.assertEqual(resolve_yolo_device("cpu"), "cpu")
        self.assertEqual(resolve_yolo_device(None), 0)

    def test_bbox_mask_fills_rectangle(self) -> None:
        mask = _bbox_mask(100, 200, (10, 20, 50, 60))
        self.assertEqual(mask.shape, (100, 200))
        self.assertEqual(int(mask[20, 10]), 255)
        self.assertEqual(int(mask[19, 9]), 0)
        self.assertEqual(int(np.count_nonzero(mask)), (50 - 10 + 1) * (60 - 20 + 1))


if __name__ == "__main__":
    unittest.main()
