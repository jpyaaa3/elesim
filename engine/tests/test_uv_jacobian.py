from __future__ import annotations

import unittest

import numpy as np

from engine.visual_servoing.uv_jacobian import (
    broyden_update_uv_jacobian,
    default_uv_jacobian,
    solve_uv_control_delta,
)


class UvJacobianTests(unittest.TestCase):
    def test_default_jacobian_matches_legacy_signs(self) -> None:
        jac = default_uv_jacobian(center_u_gain=12.0, center_v_gain=12.0)

        delta = solve_uv_control_delta(
            uv_error=(0.5, 0.5),
            jacobian=jac,
            damping=1e-6,
            max_abs_delta=(6.0, 6.0, 6.0),
        )

        self.assertLess(delta[0], 0.0)
        self.assertGreater(delta[1], 0.0)
        self.assertGreater(delta[2], 0.0)

    def test_broyden_update_learns_cross_coupling(self) -> None:
        jac = default_uv_jacobian(center_u_gain=12.0, center_v_gain=12.0)

        updated = broyden_update_uv_jacobian(
            jac,
            control_delta=(2.0, 0.0, 0.0),
            uv_delta=(0.10, 0.04),
            alpha=1.0,
            min_control_norm=0.0,
        )

        self.assertAlmostEqual(updated[0, 0], 0.05, places=6)
        self.assertAlmostEqual(updated[1, 0], 0.02, places=6)

    def test_solver_uses_coupled_columns(self) -> None:
        jac = np.array(
            [
                [0.10, 0.05, 0.00],
                [0.02, -0.03, -0.04],
            ],
            dtype=float,
        )

        delta = solve_uv_control_delta(
            uv_error=(0.20, -0.10),
            jacobian=jac,
            damping=1e-6,
            max_abs_delta=(99.0, 99.0, 99.0),
        )

        residual = jac @ delta + np.array([0.20, -0.10], dtype=float)
        self.assertLess(float(np.linalg.norm(residual)), 1e-5)


if __name__ == "__main__":
    unittest.main()
