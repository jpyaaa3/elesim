from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "AGENTS.md").exists())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from elesim_controller.vision.visual_servoing.uv_jacobian import default_uv_jacobian, solve_uv_control_delta


class TestUvJacobianSign(unittest.TestCase):
    def test_roll_moves_u_left_when_display_u_increases(self) -> None:
        j = default_uv_jacobian(center_u_gain=12.0, center_v_gain=12.0)
        self.assertLess(float(j[0, 0]), 0.0)

    def test_negative_u_error_commands_negative_roll(self) -> None:
        j = default_uv_jacobian(center_u_gain=12.0, center_v_gain=12.0)
        du3 = solve_uv_control_delta(
            uv_error=(-0.3, 0.0),
            jacobian=j,
            max_abs_delta=(2.1, 2.1, 2.1),
        )
        self.assertLess(float(du3[0]), 0.0)

    def test_positive_v_error_commands_opposite_seg_directions(self) -> None:
        j = default_uv_jacobian(center_u_gain=12.0, center_v_gain=12.0)
        du3 = solve_uv_control_delta(
            uv_error=(0.0, 0.3),
            jacobian=j,
            max_abs_delta=(2.1, 2.1, 2.1),
        )
        self.assertGreater(float(du3[1]), 0.0)
        self.assertLess(float(du3[2]), 0.0)


if __name__ == "__main__":
    unittest.main()
