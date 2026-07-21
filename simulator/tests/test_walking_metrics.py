from __future__ import annotations

import json
from pathlib import Path

from elesim_simulator.observability.walking_metrics import WalkingMetricsLogger, WalkingMetricsMeta
from elesim_simulator.robot.go2.mpc.control_rate import ControlRateInfo


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
