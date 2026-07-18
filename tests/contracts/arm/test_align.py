from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "AGENTS.md").exists())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.robot.arm import ik as ik_pipeline
from engine.robot.arm.iklib import aligner as ik_aligner


class TestAlignSkip(unittest.TestCase):
    def test_skips_refine_when_direction_already_good(self) -> None:
        q = np.array([0.1, 0.0, 0.2, 0.3], dtype=float)
        target = np.array([0.3, 0.0, 0.2], dtype=float)
        direction = np.array([1.0, 0.0, 0.0], dtype=float)
        context = {"limit": object()}

        with patch("engine.robot.arm.ik.ik_solver.solve_ik") as solve_ik, patch(
            "engine.robot.arm.ik.ik_kin._forward_grasp_direction_world",
            return_value=direction,
        ), patch("engine.robot.arm.ik._refine_orientation") as refine:
            solve_ik.return_value = MagicMock(
                success=True,
                q=q.copy(),
                position_error_m=0.001,
                seed_name="seed",
                iterations=3,
                reason="ok",
            )
            result = ik_pipeline.solve_then_align(
                target_world=target,
                target_dir_world=direction,
                context=context,
                position_tol_m=0.01,
                max_iters=10,
                current_seed=q,
                align_skip_under_deg=10.0,
            )
            refine.assert_not_called()
            self.assertFalse(result.align_attempted)
            self.assertEqual(result.reason, "position_converged_align_skipped")
            self.assertAlmostEqual(result.direction_angle_rad, 0.0, places=6)

    def test_refines_when_direction_exceeds_skip_threshold(self) -> None:
        q = np.array([0.1, 0.0, 0.2, 0.3], dtype=float)
        target = np.array([0.3, 0.0, 0.2], dtype=float)
        direction = np.array([1.0, 0.0, 0.0], dtype=float)
        context = {"limit": object()}
        bad_dir = np.array([0.0, 1.0, 0.0], dtype=float)

        refine_result = ik_aligner.OrientationRefineResult(
            q=q.copy(),
            position_error_m=0.001,
            direction_error=0.01,
            direction_angle_rad=math.radians(2.0),
            initial_direction_error=0.5,
            initial_direction_angle_rad=math.radians(20.0),
            iterations=1,
            accepted_steps=0,
            position_kept=True,
            direction_improved=True,
            converged=True,
        )

        with patch("engine.robot.arm.ik.ik_solver.solve_ik") as solve_ik, patch(
            "engine.robot.arm.ik.ik_kin._forward_grasp_direction_world",
            return_value=bad_dir,
        ), patch("engine.robot.arm.ik.ik_kin._forward_grasp_world", return_value=target), patch(
            "engine.robot.arm.ik._refine_orientation",
            return_value=refine_result,
        ) as refine:
            solve_ik.return_value = MagicMock(
                success=True,
                q=q.copy(),
                position_error_m=0.001,
                seed_name="seed",
                iterations=3,
                reason="ok",
            )
            result = ik_pipeline.solve_then_align(
                target_world=target,
                target_dir_world=direction,
                context=context,
                position_tol_m=0.01,
                max_iters=10,
                current_seed=q,
                align_skip_under_deg=5.0,
                align_mode="lite",
            )
            refine.assert_called_once()
            self.assertTrue(result.align_attempted)
            kwargs = refine.call_args.kwargs
            self.assertEqual(kwargs["align_mode"], "lite")


class TestLiteAlignMode(unittest.TestCase):
    def test_refine_orientation_uses_lite_path(self) -> None:
        q = np.array([0.1, 0.0, 0.2, 0.3], dtype=float)
        target = np.array([0.3, 0.0, 0.2], dtype=float)
        direction = np.array([1.0, 0.0, 0.0], dtype=float)
        context = {"limit": object()}
        lite_result = ik_aligner.OrientationRefineResult(
            q=q.copy(),
            position_error_m=0.001,
            direction_error=0.01,
            direction_angle_rad=math.radians(1.0),
            initial_direction_error=0.2,
            initial_direction_angle_rad=math.radians(8.0),
            iterations=1,
            accepted_steps=0,
            position_kept=True,
            direction_improved=True,
            converged=True,
        )

        with patch(
            "engine.robot.arm.ik.ik_aligner.refine_direction_lite",
            return_value=lite_result,
        ) as lite_fn, patch(
            "engine.robot.arm.ik.ik_aligner.refine_direction_with_position_hold",
        ) as full_fn:
            out = ik_pipeline._refine_orientation(
                q=q,
                hold_target=target,
                unit_dir=direction,
                context=context,
                position_hold_tol_m=0.01,
                tweak_rounds=4,
                align_mode="lite",
                timing=None,
            )
            lite_fn.assert_called_once()
            full_fn.assert_not_called()
            self.assertEqual(int(out.iterations), 1)


class TestSolvePositionOnly(unittest.TestCase):
    def test_measures_direction_without_align(self) -> None:
        q = np.array([0.1, 0.0, 0.2, 0.3], dtype=float)
        target = np.array([0.3, 0.0, 0.2], dtype=float)
        desired = np.array([1.0, 0.0, 0.0], dtype=float)
        actual = np.array([0.0, 1.0, 0.0], dtype=float)
        context = {"limit": object()}

        with patch("engine.robot.arm.ik.ik_solver.solve_ik") as solve_ik, patch(
            "engine.robot.arm.ik.ik_kin._forward_grasp_direction_world",
            return_value=actual,
        ), patch("engine.robot.arm.ik._refine_orientation") as refine:
            solve_ik.return_value = MagicMock(
                success=True,
                q=q.copy(),
                position_error_m=0.001,
                seed_name="seed",
                iterations=2,
                reason="ok",
            )
            result = ik_pipeline.solve_position_only(
                target_world=target,
                target_dir_world=desired,
                context=context,
                position_tol_m=0.01,
                max_iters=10,
                current_seed=q,
            )
            refine.assert_not_called()
            self.assertTrue(result.success)
            self.assertEqual(result.reason, "position_only")
            self.assertAlmostEqual(result.direction_angle_rad, math.pi / 2.0, places=6)


if __name__ == "__main__":
    unittest.main()
