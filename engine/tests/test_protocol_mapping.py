from __future__ import annotations

import unittest

from engine.protocol import ControlU, SimMappingConfig, control_u_to_sim_q, sim_q_to_control_u


class ProtocolMappingTests(unittest.TestCase):
    def test_reversed_linear_control_u_roundtrips_through_q(self) -> None:
        cfg = SimMappingConfig(command_direction=(-1, -1, 1, -1), linear_u_limit=250.0)
        for u_linear in (0.0, 15.0, 110.0, 180.0, 250.0):
            with self.subTest(u_linear=u_linear):
                u0 = ControlU(u_linear=u_linear, u_roll=180.0, u_s1=180.0, u_s2=180.0)
                q = control_u_to_sim_q(u0, cfg)
                u1 = sim_q_to_control_u(q, cfg)
                self.assertAlmostEqual(u1.u_linear, u_linear, places=6)


if __name__ == "__main__":
    unittest.main()
