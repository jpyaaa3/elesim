from __future__ import annotations

import unittest

from elesim_pilot.gaze.stabilizer import GazeStabilizerConfig
from research.experiments.walking_baseline import _parse_gaze, _validate_gaze_config


class WalkingBaselinePreviewTests(unittest.TestCase):
    def test_parse_gaze_includes_pitch_preview(self) -> None:
        self.assertEqual(_parse_gaze("pitch_preview"), "pitch_preview")

    def test_validate_pitch_preview_requires_enable(self) -> None:
        cfg = GazeStabilizerConfig(preview_enable=False)
        with self.assertRaises(SystemExit):
            _validate_gaze_config("pitch_preview", cfg)

    def test_validate_pitch_preview_ok_when_enabled(self) -> None:
        cfg = GazeStabilizerConfig(preview_enable=True)
        _validate_gaze_config("pitch_preview", cfg)


if __name__ == "__main__":
    unittest.main()
