"""Unit tests for perception track watchdog helpers."""

from __future__ import annotations

import unittest

from engine.config_loader import PerceptionConfig
from engine.controller.perception_capture import PerceptionCapture


class TrackWatchdogTests(unittest.TestCase):
    def test_needs_redetect_low_scale(self) -> None:
        cap = PerceptionCapture(
            PerceptionConfig(track_watchdog_min_frames=5, track_scale_min=0.05),
            publish_fn=lambda **_: None,
        )
        need, streak = cap._track_needs_redetect(
            track_ok=10,
            current_scale=0.02,
            bbox_area=500,
            init_bbox_area=2000,
            last_scale=0.02,
            scale_stale_streak=0,
        )
        self.assertTrue(need)

    def test_stale_scale_streak(self) -> None:
        cap = PerceptionCapture(
            PerceptionConfig(
                track_watchdog_min_frames=5,
                track_scale_stale_eps=0.001,
                track_redetect_stale_frames=3,
            ),
            publish_fn=lambda **_: None,
        )
        streak = 0
        need = False
        for _ in range(3):
            need, streak = cap._track_needs_redetect(
                track_ok=10,
                current_scale=0.12,
                bbox_area=800,
                init_bbox_area=800,
                last_scale=0.12,
                scale_stale_streak=streak,
            )
        self.assertTrue(need)
        self.assertGreaterEqual(streak, 3)


if __name__ == "__main__":
    unittest.main()
