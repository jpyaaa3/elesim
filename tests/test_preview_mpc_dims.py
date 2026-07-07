from __future__ import annotations

import tempfile
import unittest

import numpy as np

from engine.behaviors.gaze.preview_model import PreviewGazeModel, PreviewGazeState
from engine.behaviors.gaze.preview_mpc import PreviewMpcController, PreviewMpcWeights


class PreviewMpcDimTests(unittest.TestCase):
    def test_model_step_dimensions(self) -> None:
        j = np.eye(2, 3, dtype=float)
        b = np.eye(2, 1, dtype=float)
        model = PreviewGazeModel(jacobian_uv=j, b_base=b)
        state = PreviewGazeState(u_err=0.1, v_err=-0.05, d_hat=np.zeros(1, dtype=float))
        du = np.array([0.01, 0.02, 0.0], dtype=float)
        nxt = model.step(state, du=du, d_disturbance=np.array([0.1], dtype=float))
        self.assertAlmostEqual(nxt.u_err, 0.21, places=6)
        self.assertAlmostEqual(nxt.v_err, -0.03, places=6)

    def test_mpc_solve_one_step(self) -> None:
        j = np.eye(2, 3, dtype=float)
        b = np.zeros((2, 1), dtype=float)
        model = PreviewGazeModel(jacobian_uv=j, b_base=b)
        w = PreviewMpcWeights(Q=np.eye(2), R=np.eye(3), S=np.eye(1))
        ctrl = PreviewMpcController(model, w)
        state = PreviewGazeState(u_err=0.1, v_err=-0.05, d_hat=np.zeros(1))
        du = ctrl.solve(state, disturbance_horizon=np.array([0.2]))
        self.assertEqual(du.shape, (3,))
        self.assertTrue(np.all(np.isfinite(du)))


if __name__ == "__main__":
    unittest.main()
