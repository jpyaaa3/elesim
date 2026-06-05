from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from dataclasses import dataclass
from typing import Optional

from engine.visual_servoing.feasible_ready_pose import (
    _build_candidates,
    resolve_feasible_ready_pose,
)
from engine.visual_servoing.pick_view_pregrasp import ViewPregraspCandidate


@dataclass
class _StubIkResult:
    success: bool
    q: Optional[np.ndarray]
    position_error_m: float
    direction_angle_rad: float = 0.0
    reason: str = ""


def _ok_result(*, q: np.ndarray, dir_deg: float, pos_err: float = 0.001) -> _StubIkResult:
    return _StubIkResult(
        success=True,
        q=np.asarray(q, dtype=float).reshape(4).copy(),
        position_error_m=float(pos_err),
        direction_angle_rad=math.radians(float(dir_deg)),
        reason="position_converged_align_improved",
    )


class TestFeasibleReadyPoseCandidates(unittest.TestCase):
    def test_build_candidates_include_seed_preferred(self) -> None:
        obj = (0.5, 0.0, 0.2)
        preferred = (1.0, 0.0, 0.0)
        cands = _build_candidates(
            obj,
            preferred,
            standoff_m=0.20,
            lateral_offsets_m=(0.0,),
            height_offsets_m=(0.0,),
        )
        self.assertGreaterEqual(len(cands), 1)
        self.assertEqual(cands[0].tag, "seed_preferred")
        ready = np.asarray(cands[0].pregrasp_world, dtype=float)
        self.assertAlmostEqual(float(ready[0]), 0.30, places=3)
        self.assertAlmostEqual(float(ready[1]), 0.0, places=3)
        self.assertAlmostEqual(float(ready[2]), 0.2, places=3)


class TestResolveFeasibleReadyPose(unittest.TestCase):
    def test_fast_path_skips_grid_when_user_dir_ok(self) -> None:
        obj = (0.5, 0.0, 0.2)
        preferred = (1.0, 0.0, 0.0)
        calls: list[str] = []

        def solve_fn(**kwargs) -> _StubIkResult:
            target = tuple(float(v) for v in np.asarray(kwargs["target_world"], dtype=float).reshape(3))
            if abs(target[0] - 0.30) < 1e-6:
                calls.append("user")
                return _ok_result(q=np.array([0.1, 0.0, 0.2, 0.3]), dir_deg=2.0)
            calls.append("grid")
            return _ok_result(q=np.array([0.2, 0.0, 0.1, 0.4]), dir_deg=1.0)

        result = resolve_feasible_ready_pose(
            object_world=obj,
            preferred_dir=preferred,
            standoff_m=0.20,
            ik_context={},
            current_seed=(0.0, 0.0, 0.0, 0.0),
            position_tol_m=0.01,
            max_iters=10,
            max_dir_error_deg=10.0,
            skip_search_under_deg=5.0,
            lateral_offsets_m=(-0.05, 0.0, 0.05),
            height_offsets_m=(0.0, 0.05),
            solve_fn=solve_fn,
        )
        self.assertTrue(result.success)
        self.assertEqual(result.reason, "fast_path")
        self.assertEqual(result.candidate_tag, "seed_preferred")
        self.assertEqual(calls, ["user"])

    def test_grid_selects_best_ranked_candidate(self) -> None:
        obj = (0.5, 0.0, 0.2)
        preferred = (1.0, 0.0, 0.0)

        def solve_fn(**kwargs) -> _StubIkResult:
            target = np.asarray(kwargs["target_world"], dtype=float).reshape(3)
            if abs(float(target[0]) - 0.30) < 1e-6:
                return _ok_result(q=np.array([0.1, 0.0, 0.2, 0.3]), dir_deg=8.0)
            return _ok_result(q=np.array([0.2, 0.0, 0.1, 0.4]), dir_deg=3.0)

        result = resolve_feasible_ready_pose(
            object_world=obj,
            preferred_dir=preferred,
            standoff_m=0.20,
            ik_context={},
            current_seed=(0.0, 0.0, 0.0, 0.0),
            position_tol_m=0.01,
            max_iters=10,
            max_dir_error_deg=10.0,
            skip_search_under_deg=1.0,
            lateral_offsets_m=(0.0,),
            height_offsets_m=(0.0,),
            solve_fn=solve_fn,
        )
        self.assertTrue(result.success)
        self.assertEqual(result.reason, "grid_search")
        self.assertGreater(result.evaluated_count, 1)
        self.assertLessEqual(math.degrees(result.direction_angle_rad), 10.0)

    def test_failure_when_no_candidate_passes_gate(self) -> None:
        obj = (0.5, 0.0, 0.2)
        preferred = (1.0, 0.0, 0.0)

        def solve_fn(**_kwargs) -> _StubIkResult:
            return _ok_result(q=np.array([0.1, 0.0, 0.2, 0.3]), dir_deg=25.0)

        result = resolve_feasible_ready_pose(
            object_world=obj,
            preferred_dir=preferred,
            standoff_m=0.20,
            ik_context={},
            current_seed=(0.0, 0.0, 0.0, 0.0),
            position_tol_m=0.01,
            max_iters=10,
            max_dir_error_deg=10.0,
            skip_search_under_deg=5.0,
            lateral_offsets_m=(0.0,),
            height_offsets_m=(0.0,),
            solve_fn=solve_fn,
        )
        self.assertFalse(result.success)
        self.assertEqual(result.reason, "no feasible ready dir")
        self.assertAlmostEqual(result.best_rejected_dir_err_deg, 25.0, places=3)

    def test_camera_score_tiebreak_prefers_higher_score(self) -> None:
        obj = (0.5, 0.0, 0.2)
        preferred = (1.0, 0.0, 0.0)
        cand_a = ViewPregraspCandidate(
            pregrasp_world=(0.30, 0.0, 0.2),
            look_dir_world=(1.0, 0.0, 0.0),
            tag="a",
        )
        cand_b = ViewPregraspCandidate(
            pregrasp_world=(0.31, 0.0, 0.2),
            look_dir_world=(1.0, 0.0, 0.0),
            tag="b",
        )

        def solve_fn(**kwargs) -> _StubIkResult:
            target_x = float(np.asarray(kwargs["target_world"], dtype=float).reshape(3)[0])
            if abs(target_x - 0.31) < 1e-6:
                return _ok_result(q=np.array([0.2, 0.0, 0.1, 0.4]), dir_deg=4.0)
            return _ok_result(q=np.array([0.1, 0.0, 0.2, 0.3]), dir_deg=4.0)

        def _eval_side_effect(q4, object_world, **kwargs):
            _ = (object_world, kwargs)
            if float(q4[0]) > 0.15:
                return type("M", (), {"score": 5.0, "look_dot": 0.95})()
            return type("M", (), {"score": 1.0, "look_dot": 0.95})()

        with patch(
            "engine.visual_servoing.feasible_ready_pose._build_candidates",
            return_value=[cand_a, cand_b],
        ), patch(
            "engine.visual_servoing.feasible_ready_pose.evaluate_view_candidate",
            side_effect=_eval_side_effect,
        ):
            result = resolve_feasible_ready_pose(
                object_world=obj,
                preferred_dir=preferred,
                standoff_m=0.20,
                ik_context={"limit": {}},
                current_seed=(0.0, 0.0, 0.0, 0.0),
                position_tol_m=0.01,
                max_iters=10,
                max_dir_error_deg=10.0,
                skip_search_under_deg=0.0,
                hand_eye_transform=np.eye(4),
                solve_fn=solve_fn,
            )

        self.assertTrue(result.success)
        self.assertEqual(result.candidate_tag, "b")


if __name__ == "__main__":
    unittest.main()
