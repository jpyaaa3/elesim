from __future__ import annotations

import unittest

from elesim_pilot.gaze.stabilizer import GazeStabilizerConfig, resolve_walking_gaze_mode


class ResolveWalkingGazeModeTests(unittest.TestCase):
    def test_default_from_config(self) -> None:
        cfg = GazeStabilizerConfig(preview_enable=True, walking_gaze_mode="pitch_preview")
        self.assertEqual(resolve_walking_gaze_mode(cfg), "pitch_preview")

    def test_explicit_override(self) -> None:
        cfg = GazeStabilizerConfig(walking_gaze_mode="pitch_preview")
        self.assertEqual(resolve_walking_gaze_mode(cfg, "uv_ff"), "uv_ff")

    def test_pitch_preview_requires_enable(self) -> None:
        cfg = GazeStabilizerConfig(preview_enable=False, walking_gaze_mode="pitch_preview")
        with self.assertRaises(ValueError):
            resolve_walking_gaze_mode(cfg)


if __name__ == "__main__":
    unittest.main()
