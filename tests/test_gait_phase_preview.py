from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from engine.behaviors.gaze.gait_phase_preview import (
    GaitPhasePreviewModel,
    GaitPhaseTemplate,
    fill_empty_bins,
    load_template,
    phase_future,
    resolve_gait_phase,
)
from engine.behaviors.gaze.preview_mpc import solve_preview_du


class GaitPhasePreviewTests(unittest.TestCase):
    def _model(self, u_vals: list[float], v_vals: list[float]) -> GaitPhasePreviewModel:
        n = len(u_vals)
        tmpl = GaitPhaseTemplate(
            metadata={"gait_period_s": 0.4, "num_bins": n, "phase_source": "sim_time_mod_period"},
            u_template=np.asarray(u_vals, dtype=float),
            v_template=np.asarray(v_vals, dtype=float),
            sample_count=np.ones(n, dtype=float),
            u_std=np.zeros(n, dtype=float),
            v_std=np.zeros(n, dtype=float),
        )
        return GaitPhasePreviewModel(tmpl)

    def test_resolve_gait_phase_priority(self) -> None:
        p, src = resolve_gait_phase(
            host_gait_phase=0.25,
            sim_time_s=1.0,
            wall_time_s=10.0,
            wall_t0_s=0.0,
            gait_period_s=0.4,
            phase_offset=0.0,
        )
        self.assertAlmostEqual(p, 0.25)
        self.assertEqual(src, "go2_gait_phase")

        p2, src2 = resolve_gait_phase(
            host_gait_phase=None,
            sim_time_s=0.2,
            wall_time_s=10.0,
            wall_t0_s=0.0,
            gait_period_s=0.4,
            phase_offset=0.0,
        )
        self.assertAlmostEqual(p2, 0.5)
        self.assertEqual(src2, "sim_time_mod_period")

    def test_phase_future_wrap(self) -> None:
        self.assertAlmostEqual(phase_future(0.9, horizon_s=0.08, period_s=0.4), 0.1)

    def test_preview_delta_near_zero_when_horizon_tiny(self) -> None:
        model = self._model([0.0, 0.1, 0.2, 0.1], [0.0, -0.1, -0.2, -0.1])
        out = model.preview_delta(0.5, scale=1.0, horizon_s=0.0, period_s=0.4)
        self.assertTrue(out.ok)
        self.assertAlmostEqual(float(out.preview_term[0]), 0.0, places=5)
        self.assertAlmostEqual(float(out.preview_term[1]), 0.0, places=5)

    def test_preview_delta_nonzero_on_phase_advance(self) -> None:
        model = self._model([0.0, 0.2, 0.4, 0.2], [0.0, -0.2, -0.4, -0.2])
        out = model.preview_delta(0.0, scale=1.0, horizon_s=0.1, period_s=0.4)
        self.assertTrue(out.ok)
        self.assertFalse(np.allclose(out.preview_term, 0.0))

    def test_fill_empty_bins_neighbor(self) -> None:
        vals = np.array([0.0, 0.0, 0.4, 0.0], dtype=float)
        counts = np.array([0, 0, 1, 0], dtype=float)
        filled = fill_empty_bins(vals, counts)
        self.assertAlmostEqual(float(filled[1]), 0.4, places=5)

    def test_solve_with_gait_preview_delta(self) -> None:
        j = np.eye(2, 3, dtype=float)
        result = solve_preview_du(
            jacobian=j,
            s=np.array([0.1, 0.2], dtype=float),
            preview_term=np.array([0.05, -0.03], dtype=float),
            q_u=1.0,
            q_v=1.0,
            r_roll=0.1,
            r_s1=0.1,
            r_s2=0.1,
        )
        self.assertTrue(result.ok)

    def test_load_template_roundtrip(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "t.json"
            payload = {
                "metadata": {"gait_period_s": 0.4, "num_bins": 4},
                "u_template": [0.0, 0.1, 0.0, -0.1],
                "v_template": [0.0, -0.1, 0.0, 0.1],
                "sample_count": [1, 1, 1, 1],
                "u_std": [0, 0, 0, 0],
                "v_std": [0, 0, 0, 0],
            }
            path.write_text(json.dumps(payload), encoding="utf-8")
            tmpl = load_template(path)
            self.assertEqual(tmpl.num_bins, 4)


if __name__ == "__main__":
    unittest.main()
