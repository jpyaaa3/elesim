"""Tests for detection bbox helpers."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from perception.detection_utils import (  # noqa: E402
    bbox_xyxy_area,
    detection_center_pixel,
    detection_init_bbox,
    pad_bbox_xyxy,
)
from perception.detector import DetectionResult


class DetectionUtilsTests(unittest.TestCase):
    def test_pad_bbox_expands(self) -> None:
        bbox = (100, 100, 200, 200)
        padded = pad_bbox_xyxy(bbox, padding=1.5, image_width=640, image_height=480)
        self.assertGreater(bbox_xyxy_area(padded), bbox_xyxy_area(bbox))

    def test_detection_init_bbox_uses_mask(self) -> None:
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[20:80, 30:70] = 255
        det = DetectionResult(
            mask=mask,
            bbox_xyxy=(40, 40, 50, 50),
            label="ball",
            confidence=0.9,
        )
        init_bbox = detection_init_bbox(det, image_width=100, image_height=100, padding=1.0)
        self.assertGreaterEqual(bbox_xyxy_area(init_bbox), bbox_xyxy_area((30, 20, 70, 80)))

    def test_detection_center_pixel_mask_centroid(self) -> None:
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[40:60, 70:90] = 255
        det = DetectionResult(
            mask=mask,
            bbox_xyxy=(0, 0, 10, 10),
            label="ball",
            confidence=0.9,
        )
        cx, cy = detection_center_pixel(det, image_width=100, image_height=100)
        self.assertGreater(cx, 50.0)
        self.assertGreater(cy, 30.0)


if __name__ == "__main__":
    unittest.main()
