from __future__ import annotations

import csv
import json
from pathlib import Path

import pytest

from workbench.research.analysis.analyze_contact_metrics import (
    main,
    render_markdown,
    summarize_contact,
)


def test_summary_uses_stance_only_and_reports_per_leg() -> None:
    rows = [
        {
            "sim_time_s": "1.0", "leg": "FL", "stance": "1",
            "slip_speed_mps": "0.01", "slip_distance_m": "0.01",
            "friction_ratio": "0.5", "grf_error_norm": "2.0",
        },
        {
            "sim_time_s": "1.1", "leg": "FL", "stance": "true",
            "slip_speed_mps": "0.03", "slip_distance_m": "0.04",
            "friction_ratio": "1.2", "grf_error_norm": "4.0",
        },
        {
            "sim_time_s": "1.2", "leg": "FL", "stance": "0",
            "slip_speed_mps": "9.0", "slip_distance_m": "0.04",
            "friction_ratio": "", "grf_error_norm": "100.0",
        },
    ]

    result = summarize_contact(rows)

    assert result["row_count"] == 3
    assert result["stance_sample_count"] == 2
    assert result["duration_s"] == pytest.approx(0.2)
    assert result["stance_slip_speed_mps"]["mean"] == pytest.approx(0.02)
    assert result["stance_slip_speed_mps"]["p95"] == pytest.approx(0.029)
    assert result["stance_slip_over_threshold_ratio"] == pytest.approx(0.5)
    assert result["stance_friction_over_one_ratio"] == pytest.approx(0.5)
    assert result["stance_grf_error_norm"]["mean"] == pytest.approx(3.0)
    assert result["per_leg"]["FL"]["slip_distance_m"] == pytest.approx(0.04)
    assert result["per_leg"]["FR"]["stance_samples"] == 0
    assert "Friction utilization" in render_markdown("trial-1", result)


def test_thresholds_must_be_ordered() -> None:
    with pytest.raises(ValueError, match="thresholds"):
        summarize_contact([], slip_threshold_mps=0.05, high_slip_threshold_mps=0.02)


def test_cli_writes_markdown_and_json_report(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    with (tmp_path / "trial_contact.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=("sim_time_s", "leg", "stance", "slip_speed_mps", "slip_distance_m", "friction_ratio", "grf_error_norm"),
        )
        writer.writeheader()
        writer.writerow({
            "sim_time_s": 0.1, "leg": "RR", "stance": 1,
            "slip_speed_mps": 0.06, "slip_distance_m": 0.02,
            "friction_ratio": 1.1, "grf_error_norm": 12,
        })
    monkeypatch.setattr("sys.argv", ["analyze_contact_metrics.py", "trial", "--log-dir", str(tmp_path)])

    main()

    report = json.loads((tmp_path / "trial_contact_report.json").read_text(encoding="utf-8"))
    markdown = (tmp_path / "trial_contact_report.md").read_text(encoding="utf-8")
    assert report["run_id"] == "trial"
    assert report["per_leg"]["RR"]["slip_speed_mps"]["max"] == pytest.approx(0.06)
    assert "MPC contact/slip report" in markdown


def test_cli_rejects_path_like_run_id(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("sys.argv", ["analyze_contact_metrics.py", "../escape"])
    with pytest.raises(SystemExit) as error:
        main()
    assert error.value.code == 2
