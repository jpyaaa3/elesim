from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from engine.behaviors.gaze.stabilizer import GazeStabilizerConfig
from tools.walking_baseline import _parse_gaze, _validate_gaze_config


class WalkingBaselinePreviewTests(unittest.TestCase):
    def test_parse_gaze_includes_preview(self) -> None:
        self.assertEqual(_parse_gaze("preview"), "preview")

    def test_validate_preview_requires_gait_enable(self) -> None:
        cfg = GazeStabilizerConfig(gait_preview_enable=False, gait_template_path="missing.json")
        with self.assertRaises(SystemExit):
            _validate_gaze_config("preview", cfg)

    def test_validate_preview_requires_template_file(self) -> None:
        cfg = GazeStabilizerConfig(gait_preview_enable=True, gait_template_path="missing.json")
        with self.assertRaises(SystemExit):
            _validate_gaze_config("preview", cfg)

    def test_validate_preview_ok_when_enabled(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmpl = Path(td) / "t.json"
            tmpl.write_text("{}", encoding="utf-8")
            cfg = GazeStabilizerConfig(gait_preview_enable=True, gait_template_path=str(tmpl))
            _validate_gaze_config("preview", cfg)


if __name__ == "__main__":
    unittest.main()
