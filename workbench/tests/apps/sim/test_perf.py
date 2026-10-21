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


def test_capture_deadline_advances_when_rounding_places_it_just_after_now() -> None:
    deadline = 0.1 + 0.2
    assert deadline > 0.3
    # The camera due check accepts this rounding difference. Advancing must
    # consume that slot rather than returning the same deadline a second time.
    advanced = _advance_capture_deadline(deadline, 0.3, 1.0 / 30.0)
    assert advanced > deadline
    assert math.isclose(advanced, 1.0 / 3.0)


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


def test_perf_logger_includes_pympc_solver_events_in_canonical_metrics(tmp_path) -> None:
    path = tmp_path / "perf.csv"
    logger = PerfLogger(enabled=True, interval_s=0.25, log_path=str(path))
    try:
        for index in range(5):
            logger.reset_loop()
            if index == 0:
                logger.observe("go2_pympc_solve", 0.006)
            elif index == 3:
                logger.observe("go2_pympc_solve", 0.010)
            logger.mark_step(True, 0.01)
            if index == 4:
                logger._last_report_t = time.perf_counter() - 1.0
            logger.report_if_due()
    finally:
        logger.close()

    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 1
    assert rows[0]["samples"] == "5"
    # Solves run less frequently than the loop: divide by two events, not five steps.
    assert math.isclose(float(rows[0]["go2_mpc_solve_avg_ms"]), 8.0)
    assert math.isclose(float(rows[0]["go2_mpc_solve_max_ms"]), 10.0)


def test_slow_steps_report_actual_realtime_factor_without_changing_sim_dt(
    tmp_path, monkeypatch,
) -> None:
    now = [100.0]
    monkeypatch.setattr(time, "perf_counter", lambda: now[0])
    path = tmp_path / "slow.csv"
    logger = PerfLogger(enabled=True, interval_s=0.9, log_path=str(path))
    try:
        for step in range(60):
            logger.reset_loop()
            now[0] = 100.0 + (step + 1) / 60.0
            logger.mark_step(True, 0.01)
            logger.report_if_due()
    finally:
        logger.close()

    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 1
    row = rows[0]
    assert math.isclose(float(row["step_hz"]), 60.0)
    assert math.isclose(float(row["sim_realtime_factor"]), 0.6)
    assert math.isclose(float(row["loop_avg_ms"]), 1000.0 / 60.0)
    assert row["skipped_steps"] == "0"
