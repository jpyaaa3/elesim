from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from workbench.research.analysis.analyze_preview_b_pitch_sign import compare_sign_sweep


class AnalyzePreviewBPitchSignTests(unittest.TestCase):
    def test_compare_empty_groups(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            result = compare_sign_sweep(Path(td))
            self.assertEqual(result["b_pitch_positive"]["count"], 0)
            self.assertEqual(result["recommendation"], "")

    def test_recommendation_lower_v_rms(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            log_dir = Path(td)
            for prefix, v_rms in (("exp_gaze_preview_bp05_neutral_forward_001", 0.5), ("exp_gaze_preview_bn05_neutral_forward_001", 0.7)):
                rid = prefix
                (log_dir / f"{rid}_summary.json").write_text(
                    json.dumps({"run_id": rid, "v_rms": v_rms}),
                    encoding="utf-8",
                )
                (log_dir / f"{rid}_meta.json").write_text(
                    json.dumps(
                        {
                            "run_id": rid,
                            "preview_used_ratio": 0.9,
                            "preview_fallback_ratio": 0.1,
                        }
                    ),
                    encoding="utf-8",
                )
                (log_dir / f"{rid}_camera.csv").write_text(
                    "preview_term_v\n0.01\n",
                    encoding="utf-8",
                )
                (log_dir / f"{rid}_walking.csv").write_text("wall_time_s,base_pitch,base_roll,tau_max_abs,tau_saturation_ratio,fall_flag\n0,0,0,0,0,0\n", encoding="utf-8")
            result = compare_sign_sweep(log_dir)
            self.assertIn("+0.05", result["recommendation"])


if __name__ == "__main__":
    unittest.main()
