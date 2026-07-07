#!/usr/bin/env python3
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

_APP_ROOT = Path(__file__).resolve().parents[1]
if str(_APP_ROOT) not in sys.path:
    sys.path.insert(0, str(_APP_ROOT))

from perception.visual_tracker import BboxTracker, detection_from_bbox


class VisualTrackerTests(unittest.TestCase):
    def test_detection_from_bbox(self) -> None:
        det = detection_from_bbox((100, 80, 200, 180), image_width=640, image_height=480, label="ball")
        self.assertEqual(det.label, "ball")
        self.assertGreater(int(np.count_nonzero(det.mask)), 0)

    def test_kcf_track_smoke(self) -> None:
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.rectangle(frame, (120, 90), (200, 150), (0, 0, 220), -1)
        tracker = BboxTracker(tracker_type="kcf")
        self.assertTrue(tracker.init(frame, (120, 90, 200, 150)))
        bbox = tracker.update(frame)
        self.assertIsNotNone(bbox)


if __name__ == "__main__":
    unittest.main()
