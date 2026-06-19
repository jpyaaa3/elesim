from __future__ import annotations

import unittest

import numpy as np

from engine.go2_locomotion.config import Go2LocomotionConfig
from engine.go2_locomotion.gait import GaitScheduler
from engine.go2_locomotion.kinematics import HIP_OFFSET_BODY, TROT_PHASE_OFFSET
from engine.go2_locomotion.raibert import RaibertFootPlacement
from engine.go2_locomotion.swing import SwingTrajectory
from engine.go2_locomotion.types import Go2Command, LegId, LegPhase


class Go2RaibertMathTests(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = Go2LocomotionConfig()

    def test_trot_diagonal_pairs_share_phase(self) -> None:
        self.assertEqual(TROT_PHASE_OFFSET[LegId.FL], TROT_PHASE_OFFSET[LegId.RR])
        self.assertEqual(TROT_PHASE_OFFSET[LegId.FR], TROT_PHASE_OFFSET[LegId.RL])
        self.assertNotEqual(TROT_PHASE_OFFSET[LegId.FL], TROT_PHASE_OFFSET[LegId.FR])

    def test_gait_scheduler_stance_swing_split(self) -> None:
        gait = GaitScheduler(self.cfg)
        duty = self.cfg.stance_duty
        gait._phase = 0.0
        self.assertEqual(gait.leg_contact(LegId.FL), LegPhase.STANCE)
        gait._phase = duty + 0.01
        self.assertEqual(gait.leg_contact(LegId.FL), LegPhase.SWING)
        gait._phase = 0.5
        self.assertEqual(gait.leg_contact(LegId.FR), LegPhase.STANCE)

    def test_swing_progress_endpoints(self) -> None:
        gait = GaitScheduler(self.cfg)
        gait._phase = self.cfg.stance_duty
        self.assertAlmostEqual(gait.swing_progress(LegId.FL), 0.0)
        gait._phase = 1.0 - 1e-9
        self.assertAlmostEqual(gait.swing_progress(LegId.FL), 1.0, places=3)

    def test_raibert_forward_places_foot_ahead(self) -> None:
        raibert = RaibertFootPlacement(self.cfg)
        cmd = Go2Command(vx=0.35, vy=0.0, yaw_rate=0.0)
        v_body = np.array([0.0, 0.0, 0.0], dtype=float)
        p_fl = raibert.compute_foot_target(LegId.FL, v_body=v_body, cmd=cmd)
        p_nom = np.array([HIP_OFFSET_BODY[LegId.FL][0], HIP_OFFSET_BODY[LegId.FL][1], -self.cfg.nominal_body_height_m])
        self.assertGreater(p_fl[0], p_nom[0])

    def test_swing_trajectory_apex(self) -> None:
        p0 = np.array([0.0, 0.0, -0.30])
        p1 = np.array([0.10, 0.0, -0.30])
        mid = SwingTrajectory.sample(0.5, p0, p1, swing_height_m=0.08)
        self.assertAlmostEqual(mid[0], 0.05)
        self.assertAlmostEqual(mid[2], -0.30 + 0.08)


if __name__ == "__main__":
    unittest.main()
