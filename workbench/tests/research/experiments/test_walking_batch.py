from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from workbench.research.experiments.batch_readiness import sim_log_ready


class WalkingBatchReadinessTests(unittest.TestCase):
    def test_accepts_current_scene_media_readiness_marker(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "sim.log"
            log.write_text("[runtime] scene/media readiness gate opened\n", encoding="utf-8")
            self.assertTrue(sim_log_ready(log))

    def test_accepts_legacy_publisher_marker(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "sim.log"
            log.write_text("[sim_camera] publisher bound\n", encoding="utf-8")
            self.assertTrue(sim_log_ready(log))

    def test_rejects_log_without_readiness_marker(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "sim.log"
            log.write_text("[runtime] building scene\n", encoding="utf-8")
            self.assertFalse(sim_log_ready(log))


if __name__ == "__main__":
    unittest.main()
