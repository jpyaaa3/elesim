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

from dataclasses import dataclass
from typing import Optional

from engine.pick.actions import ControlService
from engine.pick.state import PanelState
from engine.vision.visual_servoing.feasible_ready_pose import resolve_feasible_ready_pose


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

    def test_start_aim_sends_look_object_anchor_marker(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc._pick_look_object_world_xyz = (0.239, -0.045, 1.444)
        svc._pick_look_ready_pose_world_xyz = (-0.009, -0.067, 1.465)
        svc._pick_look_dir_world = (0.993, 0.085, -0.084)
        with patch.object(svc, "_wait_for_track_lock", return_value=False):
            with patch.object(svc, "start_perception_capture"):
                with patch.object(svc, "current_visual_observation", return_value=None):
                    svc.start_aim()
                    if svc._pick_worker is not None:
                        svc._pick_worker.join(timeout=1.0)
        svc.client.send_debug_markers.assert_called_once()
        markers = svc.client.send_debug_markers.call_args.args[0]
        anchor = next(m for m in markers if m["name"] == "look_object_anchor")
        self.assertEqual(anchor["pos"], [0.239, -0.045, 1.444])
        self.assertEqual(anchor["frame"], "world")

    def test_start_ready_uncorrected_uses_ready_standoff(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        svc._pick_look_object_world_xyz = (0.5, 0.0, 0.2)
        svc._pick_look_ready_pose_world_xyz = (0.30, 0.0, 0.2)
        svc._pick_look_dir_world = (1.0, 0.0, 0.0)
        with patch.object(svc, "_start_ready_pose_resolve_and_solve") as mock_solve:
            svc.start_ready_pose()
        mock_solve.assert_called_once()
        kwargs = mock_solve.call_args.kwargs
        self.assertTrue(kwargs["resolve_dir"])
        self.assertEqual(kwargs["label"], "pre-grasp")
        # Look and Ready now share the same 0.20m standoff.
        self.assertEqual(kwargs["target_world"], (0.30, 0.0, 0.2))
        self.assertEqual(tuple(float(v) for v in kwargs["preferred_dir"]), (1.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
