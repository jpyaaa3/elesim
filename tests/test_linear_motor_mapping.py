from __future__ import annotations

import unittest

from engine.core.protocol import (
    ControlU,
    DEFAULT_START_CONTROL_U,
    PERCEPTION_READY_CONTROL_U,
    SimMappingConfig,
    SimQ,
    control_u_to_sim_q,
    default_start_sim_q,
    linear_motor_u_limit,
    perception_ready_sim_q,
    sim_q_to_control_u,
)


class LinearMotorMappingTests(unittest.TestCase):
    def _cfg(self) -> SimMappingConfig:
        return SimMappingConfig(
            command_direction=(1, -1, 1, -1),
            linear_u_max=360.0,
            linear_u_limit=250.0,
            linear_q_min_m=-0.230,
            linear_q_max_m=0.010,
        )

    def test_forward_is_zero_on_panel(self) -> None:
        cfg = self._cfg()
        u = sim_q_to_control_u(SimQ(0.010, 0.0, 0.0, 0.0), cfg)
        self.assertAlmostEqual(u.u_linear, 0.0, places=6)

    def test_full_backward_clamped_to_panel_limit(self) -> None:
        cfg = self._cfg()
        u = sim_q_to_control_u(SimQ(-0.230, 0.0, 0.0, 0.0), cfg)
        self.assertAlmostEqual(u.u_linear, linear_motor_u_limit(cfg), places=6)

    def test_mapping_uses_360_scale_not_250(self) -> None:
        cfg = self._cfg()
        from engine.protocol import sim_q_to_motor_deg

        motor = sim_q_to_motor_deg(SimQ(-0.230, 0.0, 0.0, 0.0), cfg)
        self.assertAlmostEqual(motor.u_linear, 360.0, places=6)

    def test_panel_round_trip(self) -> None:
        cfg = self._cfg()
        for u in (0.0, 125.0, 250.0):
            q = control_u_to_sim_q(ControlU(u, 0.0, 0.0, 0.0), cfg)
            back = sim_q_to_control_u(q, cfg)
            self.assertAlmostEqual(back.u_linear, u, places=4)

    def test_perception_ready_pose_display_u(self) -> None:
        cfg = self._cfg()
        back = sim_q_to_control_u(perception_ready_sim_q(cfg), cfg)
        self.assertAlmostEqual(back.u_linear, PERCEPTION_READY_CONTROL_U.u_linear, places=4)
        self.assertAlmostEqual(back.u_roll, PERCEPTION_READY_CONTROL_U.u_roll, places=4)
        self.assertAlmostEqual(back.u_s1, PERCEPTION_READY_CONTROL_U.u_s1, places=4)
        self.assertAlmostEqual(back.u_s2, PERCEPTION_READY_CONTROL_U.u_s2, places=4)

    def test_no_longer_110_at_forward(self) -> None:
        cfg = self._cfg()
        u = sim_q_to_control_u(SimQ(0.010, 0.0, 0.0, 0.0), cfg)
        self.assertNotAlmostEqual(u.u_linear, 110.0, places=3)

    def test_default_and_perception_ready_match(self) -> None:
        self.assertEqual(DEFAULT_START_CONTROL_U, PERCEPTION_READY_CONTROL_U)

    def test_default_start_pose_display_u(self) -> None:
        cfg = self._cfg()
        back = sim_q_to_control_u(default_start_sim_q(cfg), cfg)
        self.assertAlmostEqual(back.u_linear, DEFAULT_START_CONTROL_U.u_linear, places=4)
        self.assertAlmostEqual(back.u_roll, DEFAULT_START_CONTROL_U.u_roll, places=4)
        self.assertAlmostEqual(back.u_s1, DEFAULT_START_CONTROL_U.u_s1, places=4)
        self.assertAlmostEqual(back.u_s2, DEFAULT_START_CONTROL_U.u_s2, places=4)


if __name__ == "__main__":
    unittest.main()
