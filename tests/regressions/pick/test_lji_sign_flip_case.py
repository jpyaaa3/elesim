from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "host.py").exists())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.vision.visual_servoing.local_image_jacobian import (
    compute_dq_lji,
    default_j_lji_seed,
)


class TestLjiSignFlipRegression(unittest.TestCase):
    def test_stacked_solver_reduces_v_and_z_with_same_sign_z_seg_coupling(self) -> None:
        j = default_j_lji_seed(
            center_u_gain=0.1,
            center_v_gain=0.1,
            command_direction=(-1, -1, 1, -1),
        )
        j[2, :] = [-0.3, 0.0, -0.18, -0.18]
        s = np.array([0.0, -0.5, 0.25], dtype=float)

        dq, _ = compute_dq_lji(
            j_lji=j,
            s_lji=s,
            damping=0.05,
            gain_u=0.35,
            gain_v=0.55,
            gain_z=0.45,
            max_dq_linear=0.01,
            max_dq_angle=0.012,
        )
        ds = j @ dq

        self.assertLess(float(s[1] * ds[1]), 0.0)
        self.assertLess(float(s[2] * ds[2]), 0.0)
        self.assertLessEqual(abs(float(dq[0])), 0.01)
        self.assertLessEqual(float(np.max(np.abs(dq[1:]))), 0.012)


if __name__ == "__main__":
    unittest.main()
