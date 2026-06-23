from __future__ import annotations

import unittest

from engine.protocol import (
    ControlU,
    SimMappingConfig,
    SimQ,
    control_u_to_sim_q,
    linear_motor_u_limit,
    sim_q_to_control_u,
)


class LinearMotorMappingTests(unittest.TestCase):
    def _cfg(self) -> SimMappingConfig:
        return SimMappingConfig(
            command_direction=(1, -1, 1, -1),
            linear_u_limit=250.0,
            linear_q_min_m=-0.230,
            linear_q_max_m=0.010,
        )

    def test_forward_is_zero_backward_is_limit(self) -> None:
        cfg = self._cfg()
        u_fwd = sim_q_to_control_u(SimQ(0.010, 0.0, 0.0, 0.0), cfg)
        u_bwd = sim_q_to_control_u(SimQ(-0.230, 0.0, 0.0, 0.0), cfg)
        self.assertAlmostEqual(u_fwd.u_linear, 0.0, places=6)
        self.assertAlmostEqual(u_bwd.u_linear, linear_motor_u_limit(cfg), places=6)

    def test_round_trip_endpoints(self) -> None:
        cfg = self._cfg()
        for u in (0.0, 250.0):
            q = control_u_to_sim_q(ControlU(u, 0.0, 0.0, 0.0), cfg)
            back = sim_q_to_control_u(q, cfg)
            self.assertAlmostEqual(back.u_linear, u, places=6)

    def test_no_longer_110_at_forward(self) -> None:
        cfg = self._cfg()
        u = sim_q_to_control_u(SimQ(0.010, 0.0, 0.0, 0.0), cfg)
        self.assertNotAlmostEqual(u.u_linear, 110.0, places=3)


if __name__ == "__main__":
    unittest.main()
