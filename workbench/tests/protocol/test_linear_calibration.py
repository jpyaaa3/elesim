"""Keep the wire motor-angle conversion on the design rack scale."""

import pytest

from elesim_protocol.messages import (
    ControlU, SimMappingConfig, control_u_to_sim_q, sim_q_to_control_u,
)

@pytest.mark.parametrize("degrees, millimetres", [(0.0, 0.0), (15.0, 10.0), (150.0, 100.0), (213.0, 142.0), (250.0, 250/1.5), (259.0, 259/1.5)])
def test_design_rack_travel(degrees, millimetres):
    config = SimMappingConfig()
    q = control_u_to_sim_q(ControlU(degrees, 180.0, 180.0, 180.0), config)
    assert q.linear_m == pytest.approx(-millimetres / 1000.0)
    assert sim_q_to_control_u(q, config).u_linear == pytest.approx(degrees)


def test_motor_cap_does_not_expand_to_full_rack_length():
    q = control_u_to_sim_q(ControlU(360.0, 180.0, 180.0, 180.0), SimMappingConfig())
    assert q.linear_m == pytest.approx(-259/1.5/1000)
    assert abs(q.linear_m) < 0.29531
