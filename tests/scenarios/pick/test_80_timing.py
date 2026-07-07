from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from time import sleep
import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "host.py").exists())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from dataclasses import dataclass
from typing import Optional

from engine.observability.pick_timing import PickPhaseProfile, PickTimingCollector, format_report
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


class TestPickTimingCollector(unittest.TestCase):
    def test_span_accumulates(self) -> None:
        col = PickTimingCollector()
        with col.span("solve_position"):
            sleep(0.01)
        with col.span("align_direction"):
            sleep(0.005)
        self.assertGreater(col.get("solve_position"), 0.005)
        self.assertGreater(col.get("align_direction"), 0.002)

    def test_to_profile_maps_fields(self) -> None:
        col = PickTimingCollector()
        col.add("resolve_grid", 0.1)
        col.add("solve_position", 0.08)
        col.add("align_direction", 0.02)
        col.add("view_eval", 0.01)
        col.candidates_evaluated = 3
        col.ik_calls = 3
        col.resolve_reason = "fast_path"
        profile = col.to_profile(
            phase="look",
            t_total_s=0.25,
            t_host_apply_s=0.05,
            t_settle_s=0.15,
            success=True,
        )
        self.assertIsInstance(profile, PickPhaseProfile)
        self.assertEqual(profile.phase, "look")
        self.assertAlmostEqual(profile.t_resolve_s, 0.1, places=3)
        self.assertAlmostEqual(profile.t_solve_position_s, 0.08, places=3)
        self.assertAlmostEqual(profile.t_align_s, 0.02, places=3)
        self.assertEqual(profile.candidates_evaluated, 3)
        self.assertEqual(profile.resolve_reason, "fast_path")
        text = format_report(profile)
        self.assertIn("[Profile] look", text)
        self.assertIn("resolve", text)
        self.assertIn("host_apply", text)


class TestResolveWithTiming(unittest.TestCase):
    def test_timing_passed_to_stub_solve(self) -> None:
        obj = (0.5, 0.0, 0.2)
        preferred = (1.0, 0.0, 0.0)
        col = PickTimingCollector()

        def solve_fn(**kwargs) -> _StubIkResult:
            timing = kwargs.get("timing")
            if timing is not None:
                with timing.span("solve_position"):
                    sleep(0.001)
                with timing.span("align_direction"):
                    sleep(0.001)
            return _ok_result(q=np.array([0.1, 0.0, 0.2, 0.3]), dir_deg=2.0)

        result = resolve_feasible_ready_pose(
            object_world=obj,
            preferred_dir=preferred,
            standoff_m=0.20,
            ik_context={},
            current_seed=(0.0, 0.0, 0.0, 0.0),
            position_tol_m=0.01,
            max_iters=10,
            lateral_offsets_m=(0.0,),
            height_offsets_m=(0.0,),
            solve_fn=solve_fn,
            timing=col,
        )
        self.assertTrue(result.success)
        self.assertGreater(col.get("resolve_grid"), 0.0)
        self.assertGreater(col.get("solve_position"), 0.0)
        self.assertGreater(col.get("align_direction"), 0.0)
        self.assertEqual(col.ik_calls, 1)


if __name__ == "__main__":
    unittest.main()
