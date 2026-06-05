from __future__ import annotations

import math
import sys
import unittest
from dataclasses import dataclass
from pathlib import Path
from typing import Optional
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.visual_servoing.feasible_look_pose import resolve_feasible_look_pose
from engine.visual_servoing.pick_view_pregrasp import ViewPregraspCandidate


@dataclass
class _StubSolveResult:
    success: bool
    q: Optional[np.ndarray]
    position_error_m: float
    direction_angle_rad: float


def _ok_result(*, dir_deg: float, q: Optional[np.ndarray] = None, pos_err: float = 0.001) -> _StubSolveResult:
    if q is None:
        q = np.array([0.1, 0.0, 0.2, 0.3], dtype=float)
    return _StubSolveResult(
        success=True,
        q=q,
        position_error_m=pos_err,
        direction_angle_rad=math.radians(float(dir_deg)),
    )


class TestFeasibleLookPose(unittest.TestCase):
    def test_fast_path_returns_user_preferred(self) -> None:
        desired = (1.0, 0.0, 0.0)
        user_cand = ViewPregraspCandidate(
            pregrasp_world=(0.3, 0.0, 0.2),
            look_dir_world=(1.0, 0.0, 0.0),
            tag="user_preferred",
        )
        other_cand = ViewPregraspCandidate(
            pregrasp_world=(0.3, 0.0, 0.2),
            look_dir_world=(0.6, 0.8, 0.0),
            tag="other",
        )

        def solve_fn(**kwargs) -> _StubSolveResult:
            td = np.asarray(kwargs["target_dir_world"], dtype=float).reshape(3)
            if np.allclose(td, np.array(desired, dtype=float), atol=1e-8):
                return _ok_result(dir_deg=2.0)
            return _ok_result(dir_deg=8.0)

        with patch(
            "engine.visual_servoing.feasible_look_pose._build_candidates",
            return_value=[user_cand, other_cand],
        ):
            result = resolve_feasible_look_pose(
                tip_world=(0.0, 0.0, 0.0),
                object_world=(0.5, 0.0, 0.2),
                desired_look_dir=desired,
                standoff_m=0.2,
                ik_context={},
                current_seed=(0.0, 0.0, 0.0, 0.0),
                position_tol_m=0.01,
                max_iters=10,
                max_dir_error_deg=10.0,
                skip_search_under_deg=5.0,
                lateral_offsets_m=(0.0,),
                height_offsets_m=(0.0,),
                hand_eye_transform=None,
                solve_fn=solve_fn,
            )

        self.assertTrue(result.success)
        self.assertEqual(result.reason, "fast_path")
        self.assertEqual(result.candidate_tag, "user_preferred")
        self.assertEqual(result.evaluated_count, 1)

    def test_grid_search_picks_better_dir(self) -> None:
        desired = (1.0, 0.0, 0.0)
        user_cand = ViewPregraspCandidate(
            pregrasp_world=(0.3, 0.0, 0.2),
            look_dir_world=(1.0, 0.0, 0.0),
            tag="user_preferred",
        )
        other_cand = ViewPregraspCandidate(
            pregrasp_world=(0.3, 0.0, 0.2),
            look_dir_world=(0.9, 0.1, 0.0),
            tag="other",
        )

        def solve_fn(**kwargs) -> _StubSolveResult:
            td = np.asarray(kwargs["target_dir_world"], dtype=float).reshape(3)
            if np.allclose(td, np.array(desired, dtype=float), atol=1e-8):
                return _ok_result(dir_deg=8.0)
            return _ok_result(dir_deg=3.0)

        with patch(
            "engine.visual_servoing.feasible_look_pose._build_candidates",
            return_value=[user_cand, other_cand],
        ):
            result = resolve_feasible_look_pose(
                tip_world=(0.0, 0.0, 0.0),
                object_world=(0.5, 0.0, 0.2),
                desired_look_dir=desired,
                standoff_m=0.2,
                ik_context={},
                current_seed=(0.0, 0.0, 0.0, 0.0),
                position_tol_m=0.01,
                max_iters=10,
                max_dir_error_deg=10.0,
                skip_search_under_deg=1.0,  # fast_path disabled
                lateral_offsets_m=(0.0,),
                height_offsets_m=(0.0,),
                hand_eye_transform=None,
                solve_fn=solve_fn,
            )

        self.assertTrue(result.success)
        self.assertEqual(result.reason, "grid_search")
        self.assertEqual(result.candidate_tag, "other")
        self.assertEqual(result.evaluated_count, 2)
        self.assertLessEqual(math.degrees(result.direction_angle_rad), 10.0)

    def test_failure_when_no_candidate_passes_gate(self) -> None:
        desired = (1.0, 0.0, 0.0)
        user_cand = ViewPregraspCandidate(
            pregrasp_world=(0.3, 0.0, 0.2),
            look_dir_world=(1.0, 0.0, 0.0),
            tag="user_preferred",
        )
        other_cand = ViewPregraspCandidate(
            pregrasp_world=(0.3, 0.0, 0.2),
            look_dir_world=(0.9, 0.1, 0.0),
            tag="other",
        )

        def solve_fn(**_kwargs) -> _StubSolveResult:
            return _ok_result(dir_deg=25.0)

        with patch(
            "engine.visual_servoing.feasible_look_pose._build_candidates",
            return_value=[user_cand, other_cand],
        ):
            result = resolve_feasible_look_pose(
                tip_world=(0.0, 0.0, 0.0),
                object_world=(0.5, 0.0, 0.2),
                desired_look_dir=desired,
                standoff_m=0.2,
                ik_context={},
                current_seed=(0.0, 0.0, 0.0, 0.0),
                position_tol_m=0.01,
                max_iters=10,
                max_dir_error_deg=10.0,
                skip_search_under_deg=5.0,
                lateral_offsets_m=(0.0,),
                height_offsets_m=(0.0,),
                hand_eye_transform=None,
                solve_fn=solve_fn,
            )

        self.assertFalse(result.success)
        self.assertEqual(result.reason, "no feasible look dir")
        self.assertAlmostEqual(result.best_rejected_dir_err_deg, 25.0, places=3)


if __name__ == "__main__":
    unittest.main()

