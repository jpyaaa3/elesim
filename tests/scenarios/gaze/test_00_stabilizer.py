from __future__ import annotations

import time
import tempfile
import unittest

import numpy as np

from engine.gaze.stabilizer import GazeStabilizer, GazeStabilizerConfig
from engine.pick.control_ownership import ControlOwner, ControlOwnership, ControlOwnershipError, ControlState


class GazeStabilizerTests(unittest.TestCase):
    def test_positive_u_err_reduces_with_negative_du_s(self) -> None:
        cfg = GazeStabilizerConfig(enable_feedback=True, enable_base_ff=False, uv_gain=1.0)
        stab = GazeStabilizer(cfg)
        jacobian = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], dtype=float)
        du = stab.compute_display_u_delta(
            uv_error=np.array([0.1, 0.0], dtype=float),
            jacobian=jacobian,
        )
        self.assertLess(float(du[1]), 0.0)

    def test_base_ff_additive(self) -> None:
        cfg = GazeStabilizerConfig(
            enable_feedback=False,
            enable_base_ff=True,
            base_ff_gain_pitch=0.5,
        )
        stab = GazeStabilizer(cfg)
        du = stab.compute_display_u_delta(
            uv_error=np.zeros(2),
            jacobian=np.eye(2, 3),
            base_ang_vel_body=np.array([0.0, 1.0, 0.0], dtype=float),
        )
        self.assertAlmostEqual(float(du[1]), -0.5, places=6)


class ControlOwnershipExtendedTests(unittest.TestCase):
    def test_heartbeat_timeout_fails(self) -> None:
        gate = ControlOwnership(heartbeat_timeout_s=0.01)
        gate.acquire(ControlOwner.GAZE_TRACK, state=ControlState.GAZE_TRACK)
        gate.heartbeat(ControlOwner.GAZE_TRACK)
        time.sleep(0.05)
        _ = gate.owner
        self.assertEqual(gate.current_state(), ControlState.FAILED)
        self.assertEqual(gate.owner, ControlOwner.NONE)

    def test_exception_release(self) -> None:
        gate = ControlOwnership()
        gate.acquire(ControlOwner.WALK_APPROACH, state=ControlState.WALK_APPROACH)
        gate.release(ControlOwner.WALK_APPROACH)
        self.assertEqual(gate.current_state(), ControlState.IDLE)


if __name__ == "__main__":
    unittest.main()
