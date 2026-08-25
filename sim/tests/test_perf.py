from __future__ import annotations

import csv
import math
import time

from elesim_sim.runtime import PerfLogger, _advance_capture_deadline


def test_capture_deadline_preserves_fractional_frame_cadence() -> None:
    period = 1.0 / 30.0
    deadline = _advance_capture_deadline(0.0, 0.0, period)
    assert math.isclose(deadline, period)
    deadline = _advance_capture_deadline(deadline, 0.04, period)
    assert math.isclose(deadline, 2.0 * period)
    deadline = _advance_capture_deadline(deadline, 0.08, period)
    assert math.isclose(deadline, 3.0 * period)


def test_perf_logger_empty_path_reports_without_creating_a_file() -> None:
    logger = PerfLogger(enabled=True, interval_s=0.25, log_path="")
    try:
        assert logger._log_file is None
        assert logger._writer is None
    finally:
        logger.close()


def test_perf_logger_writes_camera_substage_metrics(tmp_path) -> None:
    path = tmp_path / "perf.csv"
    logger = PerfLogger(enabled=True, interval_s=0.25, log_path=str(path))
    try:
        logger.reset_loop()
        logger.observe("render", 0.001)
        logger.observe("render", 0.003)
        logger.observe("rgb_convert", 0.002)
        logger.observe("go2_mpc_solve", 0.003)
        logger.observe("go2_bridge_sync", 0.004)
        logger.observe("unknown", 0.003)
        logger.section("camera", time.perf_counter() - 0.004)
        logger.mark_step(True, 0.02)
        logger.mark_step(False, 0.02)
        logger._last_report_t = time.perf_counter() - 1.0
        logger.report_if_due()
    finally:
        logger.close()

    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 1
    row = rows[0]
    assert float(row["process_cpu_pct"]) >= 0.0
    assert float(row["camera_render_avg_ms"]) > 0.0
    assert math.isclose(float(row["camera_render_avg_ms"]), 2.0, rel_tol=0.05)
    assert float(row["camera_rgb_convert_avg_ms"]) > 0.0
    assert row["camera_depth_convert_avg_ms"] == "0.0"
    assert float(row["go2_mpc_solve_avg_ms"]) > 0.0
    assert float(row["go2_bridge_sync_avg_ms"]) > 0.0
    assert row["steps"] == "1"
    assert row["skipped_steps"] == "1"
    assert float(row["step_hz"]) > 0.0
    assert float(row["sim_realtime_factor"]) > 0.0
