from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.config_loader import PerceptionConfig
from engine.controller.perception_capture import PerceptionCapture


class TestResolveMockWorld(unittest.TestCase):
    def _capture(self, *, mode: str) -> PerceptionCapture:
        return PerceptionCapture(
            PerceptionConfig(mode=mode),
            publish_fn=MagicMock(),
            mock_world_xyz_fn=lambda: (0.5, 0.0, 1.2),
        )

    def test_camera_mode_does_not_force_mock_world(self) -> None:
        cap = self._capture(mode="camera")
        self.assertIsNone(cap._resolve_mock_world({"mock_world_xyz": [0.1, 0.2, 0.3]}))

    def test_mock_mode_uses_panel_override(self) -> None:
        cap = self._capture(mode="mock")
        self.assertEqual(cap._resolve_mock_world({}), (0.5, 0.0, 1.2))

    def test_mock_mode_falls_back_to_detector_json(self) -> None:
        cap = PerceptionCapture(
            PerceptionConfig(mode="mock"),
            publish_fn=MagicMock(),
            mock_world_xyz_fn=lambda: None,
        )
        self.assertEqual(
            cap._resolve_mock_world({"mock_world_xyz": [0.4, 0.1, 0.9]}),
            (0.4, 0.1, 0.9),
        )


if __name__ == "__main__":
    unittest.main()
