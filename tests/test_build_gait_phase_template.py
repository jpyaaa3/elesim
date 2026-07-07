from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from tools.build_gait_phase_template import build_template


class BuildGaitPhaseTemplateTests(unittest.TestCase):
    def _write_run(self, log_dir: Path, run_id: str) -> None:
        cam = log_dir / f"{run_id}_camera.csv"
        walk = log_dir / f"{run_id}_walking.csv"
        cam.write_text(
            "wall_time_s,sim_time_s,host_go2_base_timestamp_s,target_visible,u_err,v_err\n"
            "2.0,2.0,2.0,1,0.10,0.20\n"
            "2.2,2.2,2.2,1,0.15,0.25\n"
            "2.4,2.4,2.4,1,0.20,0.30\n"
            "2.6,2.6,2.6,1,0.15,0.25\n",
            encoding="utf-8",
        )
        walk.write_text(
            "wall_time_s,sim_time_s,go2_gait_phase,go2_gait_period_s,go2_cmd_vx\n"
            "2.0,2.0,0.0,0.4,0.35\n"
            "2.2,2.2,0.25,0.4,0.35\n"
            "2.4,2.4,0.5,0.4,0.35\n"
            "2.6,2.6,0.75,0.4,0.35\n",
            encoding="utf-8",
        )

    def test_build_template_metadata_and_bins(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            log_dir = Path(td)
            self._write_run(log_dir, "exp_gaze_off_neutral_forward_001")
            payload = build_template(
                runs=["exp_gaze_off_neutral_forward_001"],
                log_dir=log_dir,
                gait_period_s=0.4,
                phase_offset=0.0,
                num_bins=4,
                trim_start_s=0.0,
                trim_end_s=0.0,
                vx_nominal=0.35,
                vx_tol=0.05,
            )
            self.assertEqual(payload["metadata"]["phase_source"], "go2_gait_phase")
            self.assertEqual(len(payload["u_template"]), 4)
            self.assertGreater(payload["metadata"]["sample_total"], 0)


if __name__ == "__main__":
    unittest.main()
