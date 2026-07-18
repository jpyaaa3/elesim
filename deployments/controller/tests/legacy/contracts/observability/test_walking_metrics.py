from __future__ import annotations

import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from elesim_controller.robot.go2.mpc.control_rate import ControlRateInfo
from elesim_controller.observability.walking_metrics import (
    CAMERA_CSV_FIELDS,
    WALKING_CSV_FIELDS,
    CameraMetricsLogger,
    WalkingMetricsLogger,
    WalkingMetricsMeta,
    _env_run_id,
)


class WalkingMetricsTests(unittest.TestCase):
    def test_from_env_gated(self) -> None:
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertIsNone(WalkingMetricsLogger.from_env())
        with mock.patch.dict(os.environ, {"ELESIM_WALKING_METRICS": "1", "ELESIM_RUN_ID": "exp_test"}):
            logger = WalkingMetricsLogger.from_env()
            assert logger is not None
            self.assertEqual(logger.run_id, "exp_test")
            logger.close()

    def test_env_run_id_priority(self) -> None:
        with mock.patch.dict(os.environ, {"ELESIM_RUN_ID": "from_env"}):
            self.assertEqual(_env_run_id("explicit"), "explicit")
            self.assertEqual(_env_run_id(), "from_env")

    def test_close_flushes_counters(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            meta = WalkingMetricsMeta(run_id="r1")
            logger = WalkingMetricsLogger(run_id="r1", log_dir=td, meta=meta)
            logger.record_torque_step(recomputed=True, hold=False)
            logger.record_torque_step(recomputed=False, hold=True)
            rate = ControlRateInfo(sim_hz=200.0, ctrl_hz_config=50.0, ctrl_decim=4, ctrl_hz_effective=50.0)
            logger.set_control_rate_info(rate)
            logger.close()
            meta_path = Path(td) / "r1_meta.json"
            payload = json.loads(meta_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["extra"]["total_sim_step_count"], 2)
            self.assertEqual(payload["extra"]["total_torque_update_count"], 1)
            self.assertAlmostEqual(payload["extra"]["effective_ctrl_hz_mean"], 50.0)

    def test_camera_lost_frame_and_event_counts(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            with mock.patch.dict(os.environ, {"ELESIM_WALKING_METRICS": "1"}):
                cam = CameraMetricsLogger(run_id="cam1", log_dir=td)
            cam.sample(target_visible=True, u_err=0.1, v_err=-0.1, wall_time_s=0.0)
            cam.sample(target_visible=False, wall_time_s=0.05)
            cam.sample(target_visible=False, wall_time_s=0.10)
            cam.sample(target_visible=True, u_err=0.0, v_err=0.0, wall_time_s=0.15)
            cam.sample(target_visible=False, wall_time_s=0.20)
            cam.close()
            rows = list(Path(td).joinpath("cam1_camera.csv").read_text(encoding="utf-8").strip().splitlines())
            self.assertGreaterEqual(len(rows), 3)
            last = rows[-1].split(",")
            header = rows[0].split(",")
            data = dict(zip(header, last))
            self.assertEqual(int(data["target_lost_frame_count"]), 3)
            self.assertEqual(int(data["target_lost_event_count"]), 2)

    def test_field_lists_non_empty(self) -> None:
        self.assertIn("wall_time_s", WALKING_CSV_FIELDS)
        self.assertIn("target_lost_event_count", CAMERA_CSV_FIELDS)
        self.assertIn("preview_used", CAMERA_CSV_FIELDS)
        self.assertIn("gait_phase", CAMERA_CSV_FIELDS)
        self.assertIn("preview_term_u", CAMERA_CSV_FIELDS)
        self.assertIn("go2_gait_phase", WALKING_CSV_FIELDS)

    def test_camera_preview_columns(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            cam = CameraMetricsLogger(run_id="cam_prev", log_dir=td)
            cam.sample(
                target_visible=True,
                u_err=0.1,
                v_err=-0.1,
                gait_phase=0.25,
                gait_phase_future=0.35,
                preview_term_u=0.01,
                preview_term_v=-0.02,
            )
            cam.close()
            header = Path(td, "cam_prev_camera.csv").read_text(encoding="utf-8").splitlines()[0]
            self.assertIn("gait_phase", header)
            self.assertIn("preview_term_u", header)


if __name__ == "__main__":
    unittest.main()
