from __future__ import annotations

import tempfile
from pathlib import Path

from elesim_controller.observability.camera_metrics import (
    CAMERA_CSV_FIELDS,
    CameraMetricsLogger,
    env_run_id,
)


def test_explicit_run_id_has_priority(monkeypatch) -> None:
    monkeypatch.setenv("ELESIM_RUN_ID", "from-env")
    assert env_run_id("explicit") == "explicit"
    assert env_run_id() == "from-env"


def test_camera_loss_counts_and_preview_columns() -> None:
    with tempfile.TemporaryDirectory() as directory:
        logger = CameraMetricsLogger(run_id="camera", log_dir=directory)
        logger.sample(target_visible=True, u_err=0.1, v_err=-0.1, wall_time_s=0.0)
        logger.sample(target_visible=False, wall_time_s=0.05)
        logger.sample(target_visible=False, wall_time_s=0.10)
        logger.sample(target_visible=True, gait_phase=0.25, preview_term_u=0.01, wall_time_s=0.15)
        logger.sample(target_visible=False, wall_time_s=0.20)
        logger.close()

        lines = Path(directory, "camera_camera.csv").read_text(encoding="utf-8").splitlines()
        row = dict(zip(lines[0].split(","), lines[-1].split(",")))
        assert int(row["target_lost_frame_count"]) == 3
        assert int(row["target_lost_event_count"]) == 2
        assert "gait_phase" in CAMERA_CSV_FIELDS
        assert "preview_term_u" in CAMERA_CSV_FIELDS
