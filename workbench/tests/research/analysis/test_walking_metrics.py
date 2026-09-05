from __future__ import annotations

import csv
import json
import tempfile
import unittest
from pathlib import Path

from workbench.research.analysis.analyze_walking_metrics import _nearest_merge, effective_visibility_flags, evaluate_pitch_trim, summarize_run, _visibility_lost_counts


class AnalyzeWalkingMetricsTests(unittest.TestCase):
    def test_summarize_and_merge(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            log_dir = Path(td)
            run_id = "syn_run"
            walk_path = log_dir / f"{run_id}_walking.csv"
            cam_path = log_dir / f"{run_id}_camera.csv"
            with open(walk_path, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(
                    f,
                    fieldnames=["wall_time_s", "time_s", "base_pitch", "base_roll", "tau_max_abs", "tau_saturation_ratio", "fall_flag"],
                )
                w.writeheader()
                w.writerow(
                    {
                        "wall_time_s": 0.0,
                        "time_s": 0.0,
                        "base_pitch": 0.1,
                        "base_roll": 0.05,
                        "tau_max_abs": 5.0,
                        "tau_saturation_ratio": 0.0,
                        "fall_flag": 0,
                    }
                )
                w.writerow(
                    {
                        "wall_time_s": 0.05,
                        "time_s": 0.05,
                        "base_pitch": 0.2,
                        "base_roll": 0.0,
                        "tau_max_abs": 6.0,
                        "tau_saturation_ratio": 0.1,
                        "fall_flag": 0,
                    }
                )
            with open(cam_path, "w", newline="", encoding="utf-8") as f:
                c = csv.DictWriter(
                    f,
                    fieldnames=[
                        "wall_time_s",
                        "time_s",
                        "target_visible",
                        "u_err",
                        "v_err",
                        "target_lost_frame_count",
                        "target_lost_event_count",
                    ],
                )
                c.writeheader()
                c.writerow(
                    {
                        "wall_time_s": 0.0,
                        "time_s": 0.0,
                        "target_visible": 1,
                        "u_err": 0.05,
                        "v_err": -0.02,
                        "target_lost_frame_count": 0,
                        "target_lost_event_count": 0,
                    }
                )
                c.writerow(
                    {
                        "wall_time_s": 0.05,
                        "time_s": 0.05,
                        "target_visible": 0,
                        "u_err": "",
                        "v_err": "",
                        "target_lost_frame_count": 1,
                        "target_lost_event_count": 1,
                    }
                )
            (log_dir / f"{run_id}_meta.json").write_text(json.dumps({"run_id": run_id}), encoding="utf-8")
            summary = summarize_run(run_id, log_dir, write_plots=False)
            self.assertGreater(summary["pitch_rms_deg"], 0.0)
            self.assertEqual(summary["target_lost_event_count"], 1)
            walking = list(csv.DictReader(walk_path.open(encoding="utf-8")))
            camera = list(csv.DictReader(cam_path.open(encoding="utf-8")))
            merged = _nearest_merge(walking, camera)
            self.assertEqual(len(merged), 2)

    def test_evaluate_pitch_trim(self) -> None:
        before = {"pitch_rms_deg": 10.0, "u_rms": 0.1, "v_rms": 0.1, "visible_time_ratio": 0.9, "target_lost_event_count": 0, "fall_detected": False}
        after = {"pitch_rms_deg": 6.0, "u_rms": 0.11, "v_rms": 0.1, "visible_time_ratio": 0.88, "target_lost_event_count": 0, "fall_detected": False}
        ev = evaluate_pitch_trim(before, after)
        self.assertTrue(ev["pitch_trim_pass_30pct"])
        self.assertTrue(ev["overall_pass"])

    def test_effective_visibility_rejects_frozen_tracker(self) -> None:
        rows = []
        for i in range(10):
            rows.append(
                {
                    "target_visible": 1,
                    "u_err": -0.36 if i >= 3 else -0.40 + 0.01 * i,
                    "v_err": 0.95,
                    "bbox_scale": 0.001,
                }
            )
        flags = effective_visibility_flags(rows, frozen_samples=3)
        self.assertEqual(flags[:3], [1, 1, 1])
        self.assertEqual(flags[3:5], [1, 1])
        self.assertEqual(flags[5:], [0] * 5)

    def test_visibility_lost_counts(self) -> None:
        flags = [1, 1, 1, 0, 0, 0]
        frames, events = _visibility_lost_counts(flags)
        self.assertEqual(frames, 3)
        self.assertEqual(events, 1)


if __name__ == "__main__":
    unittest.main()
