from __future__ import annotations

from pathlib import Path

from .pick_replay import analyze_pick_log


FIXTURES = Path(__file__).with_name("fixtures")


def test_problematic_recording_reproduces_the_observed_failure_chain() -> None:
    report = analyze_pick_log((FIXTURES / "problematic_grasp.log").read_text(encoding="utf-8"))

    assert report.perception_samples == 2
    assert report.control_samples == 3
    assert report.max_world_jump_m > 0.08
    assert report.measured_motion_stalls >= 1
    assert report.blind_handoff_remain_m == 0.098
    assert report.max_blind_look_error_deg == 19.8
    assert report.outcome == "aborted"
    assert {
        "world_pose_jump",
        "measured_motion_stall",
        "blind_handoff_long",
        "blind_direction_error",
        "grasp_aborted",
    }.issubset(report.issue_codes)


def test_healthy_recording_has_no_diagnostics() -> None:
    report = analyze_pick_log((FIXTURES / "healthy_grasp.log").read_text(encoding="utf-8"))

    assert report.outcome == "completed"
    assert report.issue_codes == frozenset()
    assert report.max_world_jump_m < 0.002
    assert report.measured_motion_stalls == 0
