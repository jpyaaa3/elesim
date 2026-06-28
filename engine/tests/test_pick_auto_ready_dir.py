from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.controller.actions import ControlService
from engine.controller.state import PanelState


class TestPickAutoReadyDir(unittest.TestCase):
    def _service(self) -> ControlService:
        return ControlService(PanelState())

    def test_resolved_latch_has_priority(self) -> None:
        svc = self._service()
        svc._pick_resolved_ready_dir_world = (0.0, 0.0, 1.0)
        svc._pick_look_dir_world = (1.0, 0.0, 0.0)
        result = svc._pick_auto_preferred_dir((0.5, 0.0, 0.2))
        self.assertEqual(result, (0.0, 0.0, 1.0))

    def test_look_latch_used_when_no_resolved(self) -> None:
        svc = self._service()
        svc._pick_look_dir_world = (0.0, 1.0, 0.0)
        result = svc._pick_auto_preferred_dir((0.5, 0.0, 0.2))
        self.assertEqual(result, (0.0, 1.0, 0.0))

    def test_object_minus_tip_when_no_latch(self) -> None:
        svc = self._service()
        obj = (0.5, 0.0, 0.2)
        tip = (0.3, 0.0, 0.2)
        result = svc._pick_auto_preferred_dir(obj, tip_world=tip)
        self.assertIsNotNone(result)
        assert result is not None
        expected = np.array([0.2, 0.0, 0.0], dtype=float)
        expected /= np.linalg.norm(expected)
        got = np.asarray(result, dtype=float)
        self.assertTrue(np.allclose(got, expected, atol=1e-6))

    def test_user_preferred_dir_has_look_seed_priority(self) -> None:
        svc = self._service()
        svc.state.set_mock_object_preferred_dir(0.0, 3.0, 0.0)
        result = svc._pick_look_seed_dir((0.5, 0.0, 0.2), tip_world=(0.3, 0.0, 0.2))
        self.assertEqual(result, (0.0, 1.0, 0.0))

    def test_zero_user_preferred_dir_falls_back_to_auto(self) -> None:
        svc = self._service()
        svc.state.set_mock_object_preferred_dir(0.0, 0.0, 0.0)
        result = svc._pick_look_seed_dir((0.5, 0.0, 0.2), tip_world=(0.3, 0.0, 0.2))
        self.assertEqual(result, (1.0, 0.0, 0.0))

    def test_degenerate_object_tip_returns_none(self) -> None:
        svc = self._service()
        obj = (0.5, 0.0, 0.2)
        tip = (0.5, 0.0, 0.2)
        result = svc._pick_auto_preferred_dir(obj, tip_world=tip)
        self.assertIsNone(result)

    def test_pick_ready_direction_delegates_to_latches(self) -> None:
        svc = self._service()
        svc._pick_look_dir_world = (0.707, 0.0, 0.707)
        result = svc._pick_ready_direction()
        self.assertEqual(result, (0.707, 0.0, 0.707))

    def test_compute_pick_ready_pose_uses_auto_dir(self) -> None:
        svc = self._service()
        svc._pick_look_dir_world = (1.0, 0.0, 0.0)
        ready = svc._compute_pick_ready_pose((0.5, 0.0, 0.2))
        self.assertIsNotNone(ready)
        assert ready is not None
        self.assertAlmostEqual(ready[0], 0.30, places=3)
        self.assertAlmostEqual(ready[1], 0.0, places=3)
        self.assertAlmostEqual(ready[2], 0.2, places=3)

    def test_pick_current_tip_world_from_reach_model(self) -> None:
        svc = self._service()
        mock_model = MagicMock()
        mock_model.grasp_position.return_value = np.array([0.1, 0.2, 0.3], dtype=float)
        with patch.object(svc, "_pick_reach_model", return_value=mock_model):
            with patch.object(svc, "_q_array_from_state", return_value=np.zeros(4)):
                tip = svc._pick_current_tip_world(host_state=MagicMock())
        self.assertEqual(tip, (0.1, 0.2, 0.3))


if __name__ == "__main__":
    unittest.main()
