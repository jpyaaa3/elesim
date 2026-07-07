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
    detection_mask_coast_aligned,
    detection_mask_translated,
    pad_bbox_xyxy,
)
from main import detection_scale  # noqa: E402
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

    def test_refine_detection_mask_erode_shrinks_area(self) -> None:
        from perception.detection_utils import refine_detection_mask

        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[30:70, 30:70] = 255
        det = DetectionResult(
            mask=mask,
            bbox_xyxy=(30, 30, 70, 70),
            label="ball",
            confidence=0.9,
        )
        refined = refine_detection_mask(det, erode_px=4)
        self.assertLess(int(np.count_nonzero(refined.mask)), int(np.count_nonzero(det.mask)))
        cx0, _ = detection_center_pixel(det, image_width=100, image_height=100)
        cx1, _ = detection_center_pixel(refined, image_width=100, image_height=100)
        self.assertAlmostEqual(cx0, cx1, places=0)

    def test_detection_mask_coast_aligned_grows_with_csrt_bbox(self) -> None:
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[40:60, 40:60] = 255
        det = DetectionResult(
            mask=mask,
            bbox_xyxy=(40, 40, 60, 60),
            label="ball",
            confidence=0.9,
        )
        aligned = detection_mask_coast_aligned(
            det,
            csrt_bbox=(30, 30, 80, 80),
            anchor_center=(50.0, 50.0),
            image_width=100,
            image_height=100,
        )
        self.assertIsNotNone(aligned)
        assert aligned is not None
        s0 = detection_scale(det, image_width=100, image_height=100)
        s1 = detection_scale(aligned, image_width=100, image_height=100)
        self.assertGreater(s1, s0)

    def test_detection_mask_translated_shifts_centroid(self) -> None:
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[40:60, 40:60] = 255
        det = DetectionResult(
            mask=mask,
            bbox_xyxy=(40, 40, 60, 60),
            label="ball",
            confidence=0.9,
        )
        shifted = detection_mask_translated(det, dx=5, dy=-3, image_width=100, image_height=100)
        self.assertIsNotNone(shifted)
        assert shifted is not None
        cx0, cy0 = detection_center_pixel(det, image_width=100, image_height=100)
        cx1, cy1 = detection_center_pixel(shifted, image_width=100, image_height=100)
        self.assertAlmostEqual(cx1 - cx0, 5.0, places=0)
        self.assertAlmostEqual(cy1 - cy0, -3.0, places=0)
        self.assertEqual(int(np.count_nonzero(shifted.mask)), int(np.count_nonzero(det.mask)))

    def test_detection_mask_translated_empty_returns_none(self) -> None:
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[0:5, 0:5] = 255
        det = DetectionResult(
            mask=mask,
            bbox_xyxy=(0, 0, 5, 5),
            label="ball",
            confidence=0.9,
        )
        self.assertIsNone(
            detection_mask_translated(det, dx=200, dy=200, image_width=100, image_height=100)
        )


if __name__ == "__main__":
    unittest.main()
