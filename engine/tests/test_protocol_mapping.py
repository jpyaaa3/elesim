from __future__ import annotations

import unittest

from engine.protocol import (
    ControlU,
    SimMappingConfig,
    SimQ,
    control_u_to_sim_q,
    linear_effective_q_bounds,
    pack_state,
    sim_q_to_control_u,
)


class ProtocolMappingTests(unittest.TestCase):
    def test_reversed_linear_control_u_roundtrips_through_q(self) -> None:
        cfg = SimMappingConfig(command_direction=(-1, -1, 1, -1), linear_u_limit=250.0)
        for u_linear in (0.0, 15.0, 110.0, 180.0, 250.0):
            with self.subTest(u_linear=u_linear):
                u0 = ControlU(u_linear=u_linear, u_roll=180.0, u_s1=180.0, u_s2=180.0)
                q = control_u_to_sim_q(u0, cfg)
                u1 = sim_q_to_control_u(q, cfg)
                self.assertAlmostEqual(u1.u_linear, u_linear, places=6)

    def test_linear_definition_uses_urdf_zero_as_user_zero(self) -> None:
        cfg = SimMappingConfig(command_direction=(-1, -1, 1, -1), linear_u_limit=250.0)
        q0 = control_u_to_sim_q(
            ControlU(u_linear=0.0, u_roll=180.0, u_s1=180.0, u_s2=180.0),
            cfg,
        )
        self.assertAlmostEqual(q0.linear_m, 0.0, places=6)
        self.assertAlmostEqual(
            sim_q_to_control_u(SimQ(linear_m=0.0, roll_rad=0.0, theta1_rad=0.0, theta2_rad=0.0), cfg).u_linear,
            0.0,
            places=6,
        )
        _lo, hi = linear_effective_q_bounds(cfg)
        self.assertAlmostEqual(hi, 0.0, places=6)

    def test_pack_state_keeps_command_q_and_sim_q_separate(self) -> None:
        command_q = SimQ(linear_m=-0.02, roll_rad=0.4, theta1_rad=0.5, theta2_rad=-0.6)
        sim_q = SimQ(linear_m=-0.12, roll_rad=0.1, theta1_rad=0.2, theta2_rad=-0.3)
        msg = pack_state(q=command_q, sim_q=sim_q, ts=1.0)
        self.assertEqual(msg["q"]["linear_m"], -0.02)
        self.assertEqual(msg["sim_q"]["linear_m"], -0.12)
        self.assertEqual(msg["q"]["roll_rad"], 0.4)
        self.assertEqual(msg["sim_q"]["roll_rad"], 0.1)


if __name__ == "__main__":
    unittest.main()
