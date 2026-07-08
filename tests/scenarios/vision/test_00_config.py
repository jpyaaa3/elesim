from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "host.py").exists())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.core.config_loader import load_app_config_from_ini
from engine.vision.perception.detector import HsvDetector, create_detector, load_detector_config
from engine.vision.perception.pipeline import resolve_detector_cfg, run_mock_frame


class TestDetectorConfig(unittest.TestCase):
    def test_config_ini_resolves_detector_preset_path(self) -> None:
        bundle = load_app_config_from_ini(str(ROOT / "config.ini"))
        path = bundle.perception_config.resolved_detector_config_path()
        self.assertTrue(path.is_file(), msg=str(path))

    def test_sim_hsv_preset_uses_target_label(self) -> None:
        cfg = load_detector_config(ROOT / "model_presets" / "visual_servoing" / "detector.sim_hsv.json")
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

    def test_yolo_example_has_no_mock_fallback_fields(self) -> None:
        cfg = load_detector_config(ROOT / "model_presets" / "visual_servoing" / "detector.yolo.example.json")
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
