from __future__ import annotations

import tempfile
import unittest
import sys
import types
from unittest.mock import Mock, patch
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


class WalkingBatchLaunchTests(unittest.TestCase):
    def test_empty_contact_csv_is_not_a_completed_trial(self):
        from workbench.research.experiments import run_walking_baseline_batch as batch
        with tempfile.TemporaryDirectory() as directory, patch.object(batch, "ROOT", Path(directory)):
            contact = Path(directory) / "logs/walking_baseline/test_contact.csv"
            contact.parent.mkdir(parents=True)
            contact.write_text("sim_time_s,leg\n")
            with self.assertRaisesRegex(SystemExit, "No contact samples"):
                batch._require_contact_rows("test", Path("sim.log"))
            contact.write_text("sim_time_s,leg\n1.0,FL\n")
            batch._require_contact_rows("test", Path("sim.log"))

    def test_batch_disables_native_viewer_in_child_command(self):
        from workbench.research.experiments import run_walking_baseline_batch as batch
        with tempfile.TemporaryDirectory() as directory, patch.object(batch.subprocess, "Popen") as launch:
            batch._start_sim("sim.yaml", "test_001", Path(directory) / "sim.log")
            argv = launch.call_args.args[0]
            self.assertIn("--no-viewer", argv)
            self.assertEqual(argv[argv.index("--config") + 1], "sim.yaml")

    def test_child_exit_reports_traceback_without_waiting(self):
        from workbench.research.experiments import run_walking_baseline_batch as batch
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "sim.log"
            log.write_text("ConfigValidationError: unknown config key")
            proc = Mock(returncode=1)
            proc.poll.return_value = 1
            with patch.object(batch.time, "sleep") as sleep:
                with self.assertRaisesRegex(SystemExit, "ConfigValidationError"):
                    batch._wait_sim_ready(log, timeout_s=180, proc=proc)
                sleep.assert_not_called()

    def test_default_batch_passes_separate_configs_and_skips_perception(self):
        from workbench.research.experiments import run_walking_baseline_batch as batch
        config = types.ModuleType("elesim_pilot.config")
        config.load_app_config = Mock()
        walking = types.ModuleType("workbench.research.experiments.walking_baseline")
        walking._run_trial = Mock()
        walking._trial_run_id = Mock(return_value="test_001")
        walking._validate_gaze_config = Mock()
        with tempfile.TemporaryDirectory() as directory, patch.dict(sys.modules, {
            "elesim_pilot.config": config,
            "workbench.research.experiments.walking_baseline": walking,
        }), patch.object(sys, "argv", ["batch", "--trials", "1", "--gaze", "off",
                                     "--batch-log-dir", directory]), \
                patch.object(batch, "_start_sim") as start, \
                patch.object(batch, "_wait_sim_ready", return_value=True), \
                patch.object(batch, "_connect_service"), \
                patch.object(batch, "_wait_perception") as perception, \
                patch.object(batch, "_stop_proc"), patch.object(batch, "_require_contact_rows"), patch.dict(batch.os.environ):
            batch.main()
            self.assertEqual(start.call_args.args[0], str(batch.ROOT / "payload/config/sim/config.yaml"))
            config.load_app_config.assert_called_once_with(str(batch.ROOT / "payload/config/pilot/config.yaml"))
            perception.assert_not_called()
            walking._run_trial.assert_called_once()


if __name__ == "__main__":
    unittest.main()
