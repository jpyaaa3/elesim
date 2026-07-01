from __future__ import annotations

import sys
import unittest
from pathlib import Path
from typing import Sequence

import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "host.py").exists())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.vision.visual_servoing.local_image_jacobian import (
    ImageJacobianEstimator3D,
    check_sample_quality,
    compute_dq_lji,
    default_j_lji_seed,
    display_v_seg_coupling,
    estimate_j_img_from_stacks,
    joint_saturated,
    null_space_projector,
    patch_lji_jacobian_for_control,
    z_jacobian_row_from_position_jacobian,
)


class TestJacobianEstimate(unittest.TestCase):
    def test_j_img_is_b_transpose(self) -> None:
        j_true = np.array(
            [
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.5, 0.5],
                [-1.0, 0.0, 0.1, 0.1],
            ],
            dtype=float,
        )
        rng = np.random.default_rng(0)
        q_stack = []
        s_stack = []
        for _ in range(8):
            dq = rng.normal(size=4)
            ds = j_true @ dq
            q_stack.append(dq)
            s_stack.append(ds)
        q_arr = np.stack(q_stack, axis=0)
        s_arr = np.stack(s_stack, axis=0)
        j_est, rank, _ = estimate_j_img_from_stacks(q_arr, s_arr)
        self.assertEqual(j_est.shape, (3, 4))
        self.assertGreaterEqual(rank, 3)
        self.assertTrue(np.allclose(j_est, j_true, atol=1e-5))

    def test_estimator_rank_gate(self) -> None:
        est = ImageJacobianEstimator3D(window_size=8)
        est.push([0.01, 0, 0, 0], [0.001, 0, -0.01])
        est.push([0.02, 0, 0, 0], [0.002, 0, -0.02])
        self.assertFalse(
            est.is_usable(min_samples=4, condition_max=100.0, min_rank=3)
        )

    def test_seed_used_until_min_samples(self) -> None:
        est = ImageJacobianEstimator3D(
            window_size=8,
            seed_j=default_j_lji_seed(center_u_gain=0.1, center_v_gain=0.1),
            min_measured_samples=4,
        )
        est.push([0.005, 0.03, -0.03, -0.03], [0.001, -0.002, -0.004])
        j, rank, _ = est.estimate()
        self.assertGreaterEqual(rank, 2)
        self.assertTrue(np.allclose(j[2, :], 0.0))

    def test_seed_fallback_when_measured_j_ill_conditioned_keeps_seed_rank(self) -> None:
        est = ImageJacobianEstimator3D(
            window_size=8,
            seed_j=default_j_lji_seed(center_u_gain=0.1, center_v_gain=0.1),
            min_measured_samples=4,
            condition_max=100.0,
            min_rank=3,
        )
        for _ in range(4):
            est.push([0.005, 0.03, -0.03, -0.03], [0.001, -0.002, -0.004])
        j, rank, cond = est.estimate()
        self.assertEqual(rank, 2)
        self.assertFalse(np.isfinite(cond))
        self.assertFalse(
            est.is_usable(min_samples=4, condition_max=100.0, min_rank=3)
        )
        self.assertTrue(np.allclose(j[2, :], 0.0))


class TestJacobianPatch(unittest.TestCase):
    def test_patch_restores_display_opposite_v_seg(self) -> None:
        seed = default_j_lji_seed(
            center_u_gain=0.1,
            center_v_gain=0.1,
            command_direction=(-1, -1, 1, -1),
        )
        wrong = seed.copy()
        wrong[1, 2] = 2.0
        wrong[1, 3] = 1.5
        z_row = np.array([-0.8, 0.0, -0.1, 0.15], dtype=float)
        j = patch_lji_jacobian_for_control(
            wrong,
            z_row=z_row,
            seed_j=seed,
            command_direction=(-1, -1, 1, -1),
        )
        self.assertTrue(np.allclose(j[2, :], z_row))
        self.assertTrue(np.allclose(j[1, :], seed[1, :]))
        eff1, eff2 = display_v_seg_coupling(j[1, :], (-1, -1, 1, -1))
        self.assertLess(eff1 * eff2, 0.0)


class TestFkZRow(unittest.TestCase):
    def test_z_row_couples_all_q_axes(self) -> None:
        j_pos = np.array(
            [
                [0.9, 0.1, 0.05, 0.08],
                [0.0, 0.2, 0.3, 0.4],
                [0.0, 0.1, -0.2, 0.3],
            ],
            dtype=float,
        )
        z_row = z_jacobian_row_from_position_jacobian(j_pos, [1.0, 0.0, 0.0])
        self.assertAlmostEqual(float(z_row[0]), -0.9)
        self.assertNotAlmostEqual(float(z_row[1]), 0.0)
        self.assertNotAlmostEqual(float(z_row[2]), 0.0)
        self.assertNotAlmostEqual(float(z_row[3]), 0.0)


class TestDefaultSeed(unittest.TestCase):
    def test_z_row_is_placeholder_until_fk_patch(self) -> None:
        j = default_j_lji_seed(center_u_gain=0.1, center_v_gain=0.1, z_bend_gain=0.2)
        self.assertEqual(j.shape, (3, 4))
        self.assertTrue(np.allclose(j[2, :], 0.0))

    def test_v_row_q_space_matches_hardware_direction(self) -> None:
        # Display s1/s2 oppose; seg2 command_direction=-1 maps both to same q sign.
        j = default_j_lji_seed(
            center_u_gain=0.1,
            center_v_gain=0.1,
            z_bend_gain=0.2,
            command_direction=(-1, -1, 1, -1),
        )
        self.assertGreater(float(j[1, 2]) * float(j[1, 3]), 0.0)


class TestComputeDq3D(unittest.TestCase):
    @staticmethod
    def _j_with_z_row(z_row: Sequence[float]) -> np.ndarray:
        j = default_j_lji_seed(center_u_gain=0.1, center_v_gain=0.1)
        j[2, :] = np.asarray(z_row, dtype=float).reshape(4)
        return j

    def test_stacked_z_and_v_use_bend_joints(self) -> None:
        j = self._j_with_z_row([-0.2, 0.0, -0.15, 0.25])
        dq, _ = compute_dq_lji(
            j_lji=j,
            s_lji=[0.02, -0.03, 0.25],
            damping=0.05,
            gain_u=0.35,
            gain_v=0.35,
            gain_z=0.45,
            max_dq_linear=0.01,
            max_dq_angle=0.012,
        )
        self.assertGreater(float(dq[0]), 0.0)
        self.assertNotEqual(float(dq[2]), 0.0)
        self.assertNotEqual(float(dq[3]), 0.0)

    def test_negative_v_err_commands_same_theta_sign_in_q(self) -> None:
        """Match pick center: object above center (v_d<0) -> same q sign on both segs."""
        j = default_j_lji_seed(
            center_u_gain=0.1,
            center_v_gain=0.1,
            command_direction=(-1, -1, 1, -1),
        )
        j[2, :] = [-0.5, 0.0, 0.0, 0.0]
        dq, _ = compute_dq_lji(
            j_lji=j,
            s_lji=[0.0, -0.4, 0.0],
            damping=0.05,
            gain_u=0.35,
            gain_v=0.35,
            gain_z=0.45,
            max_dq_linear=0.01,
            max_dq_angle=0.012,
        )
        self.assertLess(float(dq[2]), 0.0)
        self.assertLess(float(dq[3]), 0.0)

    def test_positive_v_delta_commands_same_theta_sign_in_q(self) -> None:
        """Match pick center: object below (v_d>0) -> +theta1 and +theta2 in q."""
        j = default_j_lji_seed(
            center_u_gain=0.1,
            center_v_gain=0.1,
            command_direction=(-1, -1, 1, -1),
        )
        j[2, :] = [-1.0, 0.0, -0.1, 0.1]
        dq, _ = compute_dq_lji(
            j_lji=j,
            s_lji=[0.0, 0.4, 0.33],
            damping=0.05,
            gain_u=0.35,
            gain_v=0.35,
            gain_z=0.45,
            max_dq_linear=0.01,
            max_dq_angle=0.012,
        )
        self.assertGreater(float(dq[2]), 0.0)
        self.assertGreater(float(dq[3]), 0.0)

    def test_stacked_solve_reduces_v_and_z_when_z_seg_coupling_matches(self) -> None:
        """Stacked solve may flip individual q signs but should reduce v and z."""
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

    def test_stacked_solve_uses_weighted_solution_even_when_linear_extends(self) -> None:
        """No mode switch: weighted z/u/v solve can prefer extend + bend."""
        j = default_j_lji_seed(
            center_u_gain=12.0,
            center_v_gain=12.0,
            command_direction=(-1, -1, 1, -1),
        )
        j[2, :] = [0.05, 0.0, -0.2, 0.35]
        dq, dq_raw = compute_dq_lji(
            j_lji=j,
            s_lji=[0.02, -0.3, 0.14],
            damping=0.05,
            gain_u=0.35,
            gain_v=0.35,
            gain_z=0.45,
            max_dq_linear=0.01,
            max_dq_angle=0.012,
        )
        self.assertGreater(float(dq_raw[0]), 0.0)
        self.assertGreater(float(dq[0]), 0.0)
        self.assertNotEqual(float(dq[2]), 0.0)
        self.assertNotEqual(float(dq[3]), 0.0)

    def test_seg1_moves_less_than_seg2_for_v_correction(self) -> None:
        j = default_j_lji_seed(
            center_u_gain=12.0,
            center_v_gain=12.0,
            z_bend_gain=0.2,
            command_direction=(-1, -1, 1, -1),
            seg1_jacobian_scale=0.30,
            seg2_jacobian_scale=1.0,
        )
        dq, _ = compute_dq_lji(
            j_lji=j,
            s_lji=[0.0, 0.4, 0.33],
            damping=0.05,
            gain_u=0.35,
            gain_v=0.35,
            gain_z=0.45,
            max_dq_linear=0.01,
            max_dq_angle=0.012,
            max_dq_theta1=0.004,
            max_dq_theta2=0.012,
        )
        self.assertGreater(abs(float(dq[3])), abs(float(dq[2])))

    def test_dq_reduces_positive_z_err_via_linear(self) -> None:
        j = np.zeros((3, 4), dtype=float)
        j[2, 0] = -1.0
        s = np.array([0.0, 0.0, 0.2], dtype=float)
        dq, _ = compute_dq_lji(
            j_lji=j,
            s_lji=s,
            damping=0.05,
            gain_u=0.5,
            gain_v=0.5,
            gain_z=0.5,
            max_dq_linear=0.01,
            max_dq_angle=0.05,
        )
        self.assertGreater(float(dq[0]), 0.0)


class TestNullSpace(unittest.TestCase):
    def test_approach_does_not_break_uv_first_order(self) -> None:
        j = np.array(
            [
                [0.0, 2.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 1.0],
            ],
            dtype=float,
        )
        seed = np.array([0.01, 0.02, 0.03, 0.04], dtype=float)
        n_proj = null_space_projector(j, damping=0.05)
        projected = n_proj @ seed
        self.assertTrue(np.allclose(j @ projected, np.zeros(2), atol=1e-4))


class TestSampleQuality(unittest.TestCase):
    def test_reject_small_dq(self) -> None:
        ok, reason = check_sample_quality(
            delta_q=[1e-8, 0, 0, 0],
            min_dq_norm=1e-4,
            object_lost=False,
            settle_ok=True,
            joint_saturated=False,
        )
        self.assertFalse(ok)
        self.assertEqual(reason.value, "dq_too_small")

    def test_joint_saturated_detected(self) -> None:
        before = np.array([0.1, 0.0, 0.0, 0.0])
        cmd = np.array([0.5, 0.0, 0.0, 0.0])
        after = np.array([0.11, 0.0, 0.0, 0.0])
        self.assertTrue(joint_saturated(before, cmd, after))

    def test_joint_saturated_partial_motion_ok(self) -> None:
        before = np.array([-0.1428, -0.3448, -0.4170, 0.6228])
        cmd = np.array([0.0, 0.03, 0.03, 0.03])
        after = before + np.array([0.0, 0.0238, 0.0310, 0.0055])
        self.assertFalse(joint_saturated(before, cmd, after))


if __name__ == "__main__":
    unittest.main()
