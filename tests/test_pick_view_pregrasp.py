from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.pick_view_pregrasp import (
    ViewCandidateMetrics,
    ViewPregraspCandidate,
    ViewPregraspLimits,
    camera_visibility_fail_reasons,
    camera_visibility_ok,
    camera_visibility_score,
    generate_view_pregrasp_candidates,
    pick_best_strict_candidate,
    pick_best_visible_candidate,
    view_candidate_passes,
    view_candidate_passes_strict,
    view_candidate_score,
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
        self.assertTrue(any(t.startswith("view_d") for t in tags))
        for cand in cands:
            look = np.asarray(cand.look_dir_world, dtype=float)
            self.assertAlmostEqual(float(np.linalg.norm(look)), 1.0, places=5)

    def test_view_distances_m_grid(self) -> None:
        obj = (0.5, 0.0, 0.2)
        cands = generate_view_pregrasp_candidates(
            obj,
            base_offset_m=(-0.15, 0.0, 0.05),
            view_distances_m=(0.45, 0.55),
            lateral_offsets_m=(0.0,),
            height_offsets_m=(0.0,),
        )
        tags = {c.tag for c in cands}
        self.assertIn("view_d0p45", tags)
        self.assertIn("view_d0p55", tags)


class TestCameraVisibility(unittest.TestCase):
    def setUp(self) -> None:
        self.limits = ViewPregraspLimits()

    def test_visibility_ok_in_fov(self) -> None:
        self.assertTrue(camera_visibility_ok((0.02, -0.01, 0.50), self.limits))

    def test_visibility_rejects_behind_camera(self) -> None:
        self.assertFalse(camera_visibility_ok((0.0, 0.0, -0.1), self.limits))

    def test_visibility_rejects_far_lateral(self) -> None:
        self.assertFalse(camera_visibility_ok((0.25, 0.0, 0.50), self.limits))

    def test_visibility_fail_reasons(self) -> None:
        reasons = camera_visibility_fail_reasons((0.0, 0.30, 0.12), self.limits)
        self.assertTrue(any("y_oob" in r for r in reasons))
        self.assertTrue(any("z_low" in r for r in reasons))

    def test_score_prefers_center(self) -> None:
        center = camera_visibility_score((0.0, 0.0, 0.50), self.limits)
        off = camera_visibility_score((0.08, 0.05, 0.50), self.limits)
        self.assertGreater(center, off)

    def test_view_candidate_score_ordering(self) -> None:
        center = view_candidate_score((0.0, 0.0, 0.50), desired_xy=(0.0, 0.0), limits=self.limits)
        off = view_candidate_score((0.08, 0.05, 0.50), desired_xy=(0.0, 0.0), limits=self.limits)
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


def _metrics(
    *,
    p_camera: tuple[float, float, float],
    visible_pred: bool,
    look_dot: float,
    score: float | None = None,
) -> ViewCandidateMetrics:
    sc = score if score is not None else view_candidate_score(
        p_camera, desired_xy=(0.0, 0.0), limits=ViewPregraspLimits(), visible_pred=visible_pred
    )
    return ViewCandidateMetrics(
        p_camera=p_camera,
        visible_pred=visible_pred,
        look_dot=look_dot,
        score=float(sc),
        camera_world=(0.0, 0.0, 0.0),
        camera_look=(0.0, 0.0, 1.0),
        object_dir=(0.0, 0.0, 1.0),
    )


class TestViewCandidateStrictPass(unittest.TestCase):
    def setUp(self) -> None:
        self.limits = ViewPregraspLimits()
        self.metrics_ok = ViewCandidateMetrics(
            p_camera=(0.0, 0.0, 0.50),
            visible_pred=True,
            look_dot=0.9,
            score=-0.01,
            camera_world=(0.0, 0.0, 0.0),
            camera_look=(0.0, 0.0, 1.0),
            object_dir=(0.0, 0.0, 1.0),
        )
        self.metrics_bad_look = ViewCandidateMetrics(
            p_camera=(0.0, 0.0, 0.50),
            visible_pred=True,
            look_dot=0.3,
            score=-0.01,
            camera_world=(0.0, 0.0, 0.0),
            camera_look=(0.0, 0.0, 1.0),
            object_dir=(0.0, 0.0, 1.0),
        )

    def test_live_hold_current_pose(self) -> None:
        live = (-0.023, -0.06, 0.395)
        self.assertTrue(
            view_candidate_passes_strict(
                self.metrics_bad_look,
                limits=self.limits,
                look_dot_min=0.85,
                tag="current_pose",
                live_p_camera=live,
                accept_current_if_live_visible=True,
            )
        )

    def test_grid_still_requires_fk(self) -> None:
        live = (-0.023, -0.06, 0.395)
        self.assertFalse(
            view_candidate_passes_strict(
                self.metrics_bad_look,
                limits=self.limits,
                look_dot_min=0.85,
                tag="grid_+0.00_+0.00_+0.00",
                live_p_camera=live,
                accept_current_if_live_visible=True,
            )
        )


class TestViewCandidateStrict(unittest.TestCase):
    def setUp(self) -> None:
        self.limits = ViewPregraspLimits()

    def test_look_dot_gate_pass(self) -> None:
        m = _metrics(p_camera=(0.0, 0.0, 0.50), visible_pred=True, look_dot=0.92)
        self.assertTrue(view_candidate_passes(m, limits=self.limits, look_dot_min=0.85))

    def test_look_dot_gate_reject(self) -> None:
        m = _metrics(p_camera=(0.0, 0.0, 0.50), visible_pred=True, look_dot=0.50)
        self.assertFalse(view_candidate_passes(m, limits=self.limits, look_dot_min=0.85))

    def test_visible_pred_gate_reject(self) -> None:
        m = _metrics(p_camera=(0.30, 0.0, 0.50), visible_pred=False, look_dot=0.99)
        self.assertFalse(view_candidate_passes(m, limits=self.limits, look_dot_min=0.85))

    def test_pick_best_strict_candidate_none_when_empty(self) -> None:
        self.assertIsNone(pick_best_strict_candidate([]))

    def test_pick_best_strict_candidate_prefers_higher_score(self) -> None:
        c0 = ViewPregraspCandidate(
            pregrasp_world=(0.0, 0.0, 0.0),
            look_dir_world=(1.0, 0.0, 0.0),
            tag="a",
        )
        c1 = ViewPregraspCandidate(
            pregrasp_world=(0.1, 0.0, 0.0),
            look_dir_world=(1.0, 0.0, 0.0),
            tag="b",
        )
        q = np.zeros(4)
        m0 = _metrics(p_camera=(0.05, 0.0, 0.50), visible_pred=True, look_dot=0.9, score=-0.05)
        m1 = _metrics(p_camera=(0.0, 0.0, 0.50), visible_pred=True, look_dot=0.9, score=-0.01)
        best = pick_best_strict_candidate([(c0, q, m0), (c1, q, m1)])
        self.assertIsNotNone(best)
        assert best is not None
        self.assertEqual(best[0].tag, "b")

    @patch("engine.pick_view_pregrasp.evaluate_view_candidate")
    def test_strict_only_no_visible_returns_none(self, mock_eval: object) -> None:
        del mock_eval
        rows: list[tuple[ViewPregraspCandidate, np.ndarray, ViewCandidateMetrics]] = []
        for tag, visible, look_dot in [("bad_fov", False, 0.99), ("bad_look", True, 0.5)]:
            cand = ViewPregraspCandidate(
                pregrasp_world=(0.0, 0.0, 0.0),
                look_dir_world=(1.0, 0.0, 0.0),
                tag=tag,
            )
            m = _metrics(p_camera=(0.3, 0.0, 0.50), visible_pred=visible, look_dot=look_dot)
            if view_candidate_passes(m, limits=self.limits, look_dot_min=0.85):
                rows.append((cand, np.zeros(4), m))
        self.assertEqual(len(rows), 0)
        self.assertIsNone(pick_best_strict_candidate(rows))


if __name__ == "__main__":
    unittest.main()
