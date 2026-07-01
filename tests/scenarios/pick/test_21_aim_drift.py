from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "host.py").exists())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.behaviors.pick.actions import ControlService
from engine.behaviors.pick.state import PanelState
from engine.vision.visual_servoing.ready_pose import compute_ready_pose_target


class TestAimDriftDirection(unittest.TestCase):
    def test_same_object_gives_zero_drift_with_pick_standoff(self) -> None:
        svc = ControlService(PanelState())
        direction = (1.0, 0.0, 0.0)
        obj = (0.5, 0.0, 0.2)

        with patch.object(svc, "_pick_ready_direction", return_value=direction):
            pack = svc._pick_ready_pose_drift_vectors(
                initial_object=obj,
                centered_object=obj,
            )

        self.assertIsNotNone(pack)
        _, _, drift = pack
        self.assertAlmostEqual(float(np.linalg.norm(drift)), 0.0, places=6)

    def test_look_standoff_must_not_be_used_for_drift(self) -> None:
        obj = (0.5, 0.0, 0.2)
        direction = (1.0, 0.0, 0.0)
        look_ready = compute_ready_pose_target(obj, direction, standoff_m=0.30)
        pick_ready = compute_ready_pose_target(obj, direction, standoff_m=0.20)
        wrong_drift = np.asarray(look_ready, dtype=float) - np.asarray(pick_ready, dtype=float)
        self.assertAlmostEqual(float(wrong_drift[0]), -0.10, places=6)

        svc = ControlService(PanelState())
        with patch.object(svc, "_pick_ready_direction", return_value=direction):
            pack = svc._pick_ready_pose_drift_vectors(
                initial_object=obj,
                centered_object=obj,
            )
        self.assertIsNotNone(pack)
        _, _, drift = pack
        np.testing.assert_allclose(drift, 0.0, atol=1e-6)

    def test_object_shift_yields_nonzero_drift(self) -> None:
        svc = ControlService(PanelState())
        direction = (0.0, 1.0, 0.0)
        initial = (0.5, 0.0, 0.2)
        centered = (0.5, 0.01, 0.2)

        with patch.object(svc, "_pick_ready_direction", return_value=direction):
            pack = svc._pick_ready_pose_drift_vectors(
                initial_object=initial,
                centered_object=centered,
            )

        self.assertIsNotNone(pack)
        _, _, drift = pack
        self.assertAlmostEqual(float(drift[1]), -0.01, places=6)

    def test_pick_target_stays_at_centered_object_not_initial(self) -> None:
        svc = ControlService(PanelState())
        direction = (1.0, 0.0, 0.0)
        initial = (0.322, 0.026, 0.883)
        centered = (0.336, 0.030, 0.912)

        with patch.object(svc, "_pick_ready_direction", return_value=direction):
            pack = svc._pick_ready_pose_drift_vectors(
                initial_object=initial,
                centered_object=centered,
            )
        self.assertIsNotNone(pack)
        _, centered_ready, drift = pack

        svc._pick_centered_object_world_xyz = centered
        svc._pick_centered_ready_pose_world_xyz = centered_ready
        svc._pick_corrected_object_world_xyz = centered

        ready = svc._pick_corrected_ready_pose()
        self.assertIsNotNone(ready)
        assert ready is not None
        np.testing.assert_allclose(ready, centered_ready, atol=1e-6)
        np.testing.assert_allclose(svc._pick_corrected_object_world_xyz, centered, atol=1e-6)
        legacy_undo = np.asarray(centered, dtype=float) + drift
        np.testing.assert_allclose(legacy_undo, initial, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
