from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.pick_view_pregrasp import (
    ViewPregraspLimits,
    camera_visibility_ok,
    camera_visibility_score,
    generate_view_pregrasp_candidates,
    pick_best_visible_candidate,
)


class TestViewPregraspCandidates(unittest.TestCase):
    def test_generate_unique_candidates(self) -> None:
        obj = (0.5, 0.0, 0.2)
        cands = generate_view_pregrasp_candidates(
            obj,
            base_offset_m=(-0.15, 0.0, 0.05),
            view_distance_m=0.45,
            lateral_offsets_m=(-0.05, 0.0, 0.05),
            height_offsets_m=(0.0, 0.05),
        )
        self.assertGreaterEqual(len(cands), 3)
        tags = {c.tag for c in cands}
        self.assertIn("base_offset", tags)
        self.assertIn("view_distance", tags)
        for cand in cands:
            look = np.asarray(cand.look_dir_world, dtype=float)
            self.assertAlmostEqual(float(np.linalg.norm(look)), 1.0, places=5)


class TestCameraVisibility(unittest.TestCase):
    def setUp(self) -> None:
        self.limits = ViewPregraspLimits()

    def test_visibility_ok_in_fov(self) -> None:
        self.assertTrue(camera_visibility_ok((0.02, -0.01, 0.50), self.limits))

    def test_visibility_rejects_behind_camera(self) -> None:
        self.assertFalse(camera_visibility_ok((0.0, 0.0, -0.1), self.limits))

    def test_visibility_rejects_far_lateral(self) -> None:
        self.assertFalse(camera_visibility_ok((0.25, 0.0, 0.50), self.limits))

    def test_score_prefers_center(self) -> None:
        center = camera_visibility_score((0.0, 0.0, 0.50), self.limits)
        off = camera_visibility_score((0.08, 0.05, 0.50), self.limits)
        self.assertGreater(center, off)

    def test_soft_score_finite_outside_fov(self) -> None:
        from engine.pick_view_pregrasp import camera_visibility_score_soft

        inside = camera_visibility_score_soft((0.0, 0.0, 0.50), self.limits)
        outside = camera_visibility_score_soft((0.30, 0.0, 0.50), self.limits)
        self.assertGreater(inside, outside)
        self.assertTrue(np.isfinite(outside))

    def test_pick_best_visible_candidate(self) -> None:
        cands = generate_view_pregrasp_candidates(
            (0.0, 0.0, 0.2),
            base_offset_m=(-0.15, 0.0, 0.05),
            view_distance_m=0.45,
            lateral_offsets_m=(0.0,),
            height_offsets_m=(0.0,),
        )
        scored = [(cands[0], -1.0), (cands[-1], 0.5)]
        best = pick_best_visible_candidate(scored)
        self.assertIsNotNone(best)
        assert best is not None
        self.assertEqual(best[0].tag, cands[-1].tag)
        self.assertAlmostEqual(best[1], 0.5)


if __name__ == "__main__":
    unittest.main()
