from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from dataclasses import dataclass
from typing import Optional

from engine.controller.actions import ControlService
from engine.controller.state import PanelState
from engine.visual_servoing.feasible_ready_pose import resolve_feasible_ready_pose


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


class TestLookViewPoseResolver(unittest.TestCase):
    def test_look_standoff_moves_to_pregrasp_not_tip(self) -> None:
        obj = (0.5, 0.0, 0.2)
        preferred = (1.0, 0.0, 0.0)
        tip = np.array([0.1, 0.0, 1.0], dtype=float)
        targets: list[np.ndarray] = []

        def solve_fn(**kwargs) -> _StubIkResult:
            target = np.asarray(kwargs["target_world"], dtype=float).reshape(3)
            targets.append(target.copy())
            if abs(float(target[0]) - 0.20) < 1e-6 and abs(float(target[2]) - 0.2) < 1e-6:
                return _ok_result(q=np.array([0.1, 0.0, 0.2, 0.3]), dir_deg=2.0)
            return _StubIkResult(
                success=False,
                q=None,
                position_error_m=1.0,
                reason="miss",
            )

        result = resolve_feasible_ready_pose(
            object_world=obj,
            preferred_dir=preferred,
            standoff_m=0.30,
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
        self.assertTrue(result.success)
        self.assertIsNotNone(result.resolved_target)
        assert result.resolved_target is not None
        self.assertAlmostEqual(float(result.resolved_target[0]), 0.20, places=3)
        self.assertTrue(targets)
        for target in targets:
            self.assertFalse(np.allclose(target, tip, atol=1e-3))

    def test_start_ready_requires_look_latch(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc.start_ready_pose()
        self.assertTrue(svc.state.pick_failed)
        self.assertIn("run Look first", str(svc.state.pick_status_msg))

    def test_start_ready_uncorrected_uses_ready_standoff(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc._pick_look_object_world_xyz = (0.5, 0.0, 0.2)
        svc._pick_look_ready_pose_world_xyz = (0.20, 0.0, 0.2)
        svc._pick_look_dir_world = (1.0, 0.0, 0.0)
        with patch.object(svc, "_start_ready_pose_resolve_and_solve") as mock_solve:
            svc.start_ready_pose()
        mock_solve.assert_called_once()
        kwargs = mock_solve.call_args.kwargs
        self.assertFalse(kwargs["resolve_dir"])
        self.assertEqual(kwargs["label"], "pre-grasp")
        # Look latch is at look standoff (0.30m); Ready approaches to ready standoff (0.20m).
        self.assertEqual(kwargs["target_world"], (0.30, 0.0, 0.2))
        self.assertEqual(tuple(float(v) for v in kwargs["preferred_dir"]), (1.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
