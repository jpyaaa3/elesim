"""Data-driven walking migration and honest execution gates."""
import csv
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from elesim_protocol import SimMappingConfig
from workbench.research.experiments.run_walking_baseline_batch import assess_trial
from workbench.research.experiments.walking_baseline import _run_trial
from workbench.research.experiments.walking_scenarios import validate_arm_pose
from workbench.research.experiments.walking_suite import runtime_configs, suite_cases


def test_suites_preserve_gaze_conditions_and_preview_signs():
    assert [r["gaze"] for r in suite_cases("full")] == ["uv", "off", "uv_ff", "pitch_preview"]
    assert [r["gaze"] for r in suite_cases("ablation")] == ["off", "uv_ff"]
    assert [r["b_pitch"] for r in suite_cases("preview-sign")] == [.05, -.05]


def test_bench_identities_are_local_and_never_target_robot(tmp_path):
    import yaml
    paths = runtime_configs(tmp_path, "1234abcd")
    pilot, sim = (yaml.safe_load(paths[role].read_text()) for role in ("pilot", "sim"))
    assert pilot["runtime"]["active_target"] == sim["runtime"]["endpoint_id"]
    assert sim["runtime"]["mirror_robot_id"] == ""
    for document in (pilot, sim):
        assert document["dds"]["system_id"] == "bench-1234abcd"
        assert document["dds"]["network_interface"] == "lo"
        assert document["dds"]["static_peers"] == []
    assert pilot["rgbd"]["wire"]["topic"] != sim["runtime"]["streams"]["rgbd_topic"]
    assert pilot["rgbd"]["wire"]["topic"] == sim["rgbd"]["wire"]["topic"]
    from elesim_pilot.config import load_runtime_role_config as pilot_role
    from elesim_sim.config import load_runtime_role_config as sim_role
    assert pilot_role(paths["pilot"]).active_target == sim_role(paths["sim"]).endpoint_id


def test_historical_invalid_arm_presets_fail_before_any_command():
    validate_arm_pose("neutral", SimMappingConfig())
    for name in ("forward_extended", "bent_upward", "bent_side"):
        with pytest.raises(ValueError, match="exceeds"):
            validate_arm_pose(name, SimMappingConfig())


@pytest.mark.parametrize("progress,fall,truncated,accepted", [(1., 0, False, True), (0., 0, False, False),
                          (1., 1, False, False), (1., 0, True, False)])
def test_contact_existence_cannot_mask_fall_no_progress_or_truncated_logs(tmp_path, progress, fall, truncated, accepted):
    fields = ["sim_time_s", "base_pos_x", "base_pos_y", "base_pos_z", "base_roll", "base_pitch", "base_yaw", "fall_flag"]
    with (tmp_path / "run_walking.csv").open("w") as out:
        writer = csv.DictWriter(out, fieldnames=fields)
        writer.writeheader()
        for t, x in ((0., 0.), (1. if truncated else 3., progress)):
            writer.writerow(dict.fromkeys(fields, 0) | {"sim_time_s": t, "base_pos_x": x, "fall_flag": fall})
    result = assess_trial(tmp_path, "run", {"sim_start_s": 0., "sim_end_s": 3., "command_sim_s": 3.}, "forward")
    assert result["accepted"] is accepted


def test_trial_fault_still_sends_zero_velocity_even_if_gaze_shutdown_fails(tmp_path):
    service = Mock()
    service.client.refresh_state.return_value = SimpleNamespace(
        connected=True, host_state_age_s=0, safety_fault="", go2_controller_fault="contact timeout", sim_time_s=2.)
    # First stop is initial setup; the second occurs during error cleanup.
    service.stop_gaze_stabilizer.side_effect = [None, RuntimeError("stop gaze failed")]
    with patch("workbench.research.experiments.walking_baseline._apply_preset"), \
            patch("workbench.research.experiments.walking_baseline.RunContext"), \
            pytest.raises(RuntimeError, match="stop gaze"):
        _run_trial(service=service, run_id="run", preset="neutral", motion="forward", vx=.35, vy=0, wz=0,
                   duration=30, gaze="off", log_dir=str(tmp_path), notes="", strict_run_id=True)
    service.send_go2_velocity.assert_called_once_with(vx=0., vy=0., wz=0.)
