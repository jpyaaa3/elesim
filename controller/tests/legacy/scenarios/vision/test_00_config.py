from __future__ import annotations

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "AGENTS.md").exists())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from elesim_controller.config import load_app_config
from elesim_controller.vision.perception.detector import HsvDetector, create_detector, load_detector_config
from elesim_controller.vision.perception.pipeline import resolve_detector_cfg, run_mock_frame


class TestDetectorConfig(unittest.TestCase):
    def test_config_yaml_resolves_detector_preset_path(self) -> None:
        bundle = load_app_config(str(ROOT / "controller/config/default.yaml"))
        path = bundle.perception_config.resolved_detector_config_path()
        self.assertTrue(path.is_file(), msg=str(path))

    def test_sim_hsv_preset_uses_target_label(self) -> None:
        cfg = load_detector_config(
            ROOT / "controller" / "config" / "perception" / "detector.sim_hsv.json"
        )
        self.assertEqual(str(cfg["type"]), "hsv")
        self.assertEqual(str(cfg["target_label"]), "sim_sphere")
        self.assertNotIn("label", cfg)

        detector = create_detector(cfg)
        self.assertIsInstance(detector, HsvDetector)
        color, _depth, _intr, _scale = run_mock_frame(cfg)
        det = detector.detect(color)
        self.assertIsNotNone(det)
        assert det is not None
        self.assertEqual(str(det.label), "sim_sphere")

    def test_real_green_hsv_preset_accepts_shadow_green_largest_blob(self) -> None:
        cfg = load_detector_config(
            ROOT / "controller" / "config" / "perception" / "detector.real_green_hsv.json"
        )
        detector = create_detector(cfg)
        self.assertIsInstance(detector, HsvDetector)

        hsv = np.zeros((120, 160, 3), dtype=np.uint8)
        hsv[42:78, 50:70] = (32, 180, 130)  # bright green face
        hsv[42:78, 70:96] = (32, 170, 60)  # shadowed green face
        hsv[8:16, 8:16] = (32, 180, 60)  # separate background-like green hit
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        det = detector.detect(bgr)
        self.assertIsNotNone(det)
        assert det is not None
        self.assertGreater(int(det.mask[60, 84]), 0)
        x0, _y0, x1, _y1 = det.bbox_xyxy
        self.assertGreaterEqual(x0, 45)
        self.assertLessEqual(x1, 100)

    def test_yolo_example_has_no_mock_fallback_fields(self) -> None:
        cfg = load_detector_config(
            ROOT / "controller" / "config" / "perception" / "detector.yolo.example.json"
        )
        self.assertEqual(str(cfg["type"]), "yolo")
        self.assertNotIn("mock_fallback_type", cfg)
        self.assertNotIn("center_fraction", cfg)
        self.assertNotIn("mock_depth_m", cfg)

        resolved = resolve_detector_cfg(
            cfg,
            detector_cli="config",
            target_label_cli=None,
            yolo_device_cli=None,
        )
        self.assertEqual(str(resolved["type"]), "yolo")

    def test_detector_type_is_required(self) -> None:
        with self.assertRaisesRegex(ValueError, "missing required 'type'"):
            create_detector({})

    def test_external_detector_cli_keeps_file_type(self) -> None:
        cfg = {"type": "hsv", "target_label": "x"}
        resolved = resolve_detector_cfg(
            cfg,
            detector_cli="external",
            target_label_cli="override",
            yolo_device_cli=None,
        )
        self.assertEqual(str(resolved["type"]), "hsv")
        self.assertEqual(str(resolved["target_label"]), "override")

    def test_hsv_detector_supports_multi_ranges(self) -> None:
        cfg = {
            "type": "hsv",
            "target_label": "sim_sphere",
            "min_area_px": 10,
            "hsv": {
                "ranges": [
                    {"lower": [0, 80, 80], "upper": [12, 255, 255]},
                    {"lower": [168, 80, 80], "upper": [179, 255, 255]},
                ]
            },
        }
        detector = create_detector(cfg)
        self.assertIsInstance(detector, HsvDetector)
        color, _depth, _intr, _scale = run_mock_frame(cfg)
        det = detector.detect(color)
        self.assertIsNotNone(det)
        assert det is not None
        self.assertEqual(str(det.label), "sim_sphere")


if __name__ == "__main__":
    unittest.main()
