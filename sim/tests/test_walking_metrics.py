from __future__ import annotations

import json
import csv
from pathlib import Path

import numpy as np

from elesim_sim.observability.walking_metrics import WalkingMetricsLogger, WalkingMetricsMeta
from elesim_sim.robot.go2.mpc.control_rate import ControlRateInfo
from elesim_sim.robot.go2.locomotion.types import ALL_LEGS
from elesim_sim.robot.go2.mpc.contact_diagnostics import (
    ContactDiagnosticsSample,
    FootContactDiagnostic,
)


def test_walking_metrics_flushes_rate_and_counters(tmp_path: Path) -> None:
    logger = WalkingMetricsLogger(
        run_id="walking",
        log_dir=tmp_path,
        meta=WalkingMetricsMeta(run_id="walking"),
    )
    logger.record_torque_step(recomputed=True, hold=False)
    logger.record_torque_step(recomputed=False, hold=True)
    logger.set_control_rate_info(
        ControlRateInfo(sim_hz=200.0, ctrl_hz_config=50.0, ctrl_decim=4, ctrl_hz_effective=50.0)
    )
    logger.close()
    payload = json.loads((tmp_path / "walking_meta.json").read_text(encoding="utf-8"))
    assert payload["extra"]["total_sim_step_count"] == 2
    assert payload["extra"]["total_torque_update_count"] == 1
    assert payload["extra"]["effective_ctrl_hz_mean"] == 50.0


def test_walking_metrics_writes_per_foot_contact_diagnostics(tmp_path: Path) -> None:
    logger = WalkingMetricsLogger(run_id="contact", log_dir=tmp_path)
    sample = ContactDiagnosticsSample(
        step_index=10,
        elapsed_s=0.1,
        feet=tuple(
            FootContactDiagnostic(
                leg=leg,
                position_world=np.array([1.0, 2.0, 3.0]),
                velocity_world=np.array([0.1, 0.2, 0.0]),
                net_contact_force_world=np.array([5.0, 6.0, 70.0]),
                stance=True,
                desired_grf_world=np.array([4.0, 3.0, 60.0]),
                slip_speed_mps=0.2236,
                slip_distance_m=0.02,
                friction_ratio=0.1042,
            )
            for leg in ALL_LEGS
        ),
    )
    logger.sample_contact(
        sample,
        sim_time_s=1.25,
        raw_grf=np.arange(12, dtype=float),
        tau_raw=np.arange(12, dtype=float) + 20.0,
        tau_limited=np.arange(12, dtype=float) + 10.0,
        tau_applied=np.arange(12, dtype=float),
    )
    logger.close()

    with (tmp_path / "contact_contact.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 4
    assert rows[0]["leg"] == "FL"
    assert float(rows[0]["actual_fz"]) == 70.0
    assert float(rows[0]["desired_fz"]) == 60.0
    assert float(rows[0]["raw_fz"]) == 2.0
    assert float(rows[0]["tau_applied_calf"]) == 2.0
