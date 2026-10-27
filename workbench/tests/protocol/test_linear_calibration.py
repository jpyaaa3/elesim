"""Keep motor-angle conversion on the operator-selected oct27 stroke."""

import pytest

from elesim_protocol.messages import (
    ControlU, SimMappingConfig, control_u_to_sim_q, sim_q_to_control_u,
)

@pytest.mark.parametrize("degrees, millimetres", [
    (0.0, 0.0), (64.75, 57.5), (129.5, 115.0), (194.25, 172.5), (259.0, 230.0),
])
def test_oct27_linear_travel(degrees, millimetres):
    config = SimMappingConfig()
    q = control_u_to_sim_q(ControlU(degrees, 180.0, 180.0, 180.0), config)
    assert q.linear_m == pytest.approx(-millimetres / 1000.0)
    assert sim_q_to_control_u(q, config).u_linear == pytest.approx(degrees)


def test_motor_cap_does_not_expand_to_full_rack_length():
    q = control_u_to_sim_q(ControlU(360.0, 180.0, 180.0, 180.0), SimMappingConfig())
    assert q.linear_m == pytest.approx(-0.230)
    assert abs(q.linear_m) < 0.29531
