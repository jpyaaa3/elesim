from __future__ import annotations

import pytest

from elesim_protocol import SimMappingConfig
from elesim_simulator.control_state import SimulationStateSource


def test_canonical_target_updates_all_simulator_command_state() -> None:
    state = SimulationStateSource(SimMappingConfig())
    state.apply_target(
        {
            "command": "target",
            "q": [-0.1, 0.2, 0.3, -0.3],
            "target": [0.8, 0.0, 0.2],
            "target_dir": [1.0, 0.0, 0.0],
            "go2_vel": [0.2, -0.1, 0.3],
            "claw_closed": True,
        }
    )

    assert state.estimate_q().linear_m == -0.1
    assert tuple(state.ik_target_xyz()) == (0.8, 0.0, 0.2)
    assert tuple(state.ik_target_dir()) == (1.0, 0.0, 0.0)
    assert state.go2_vel() == (0.2, -0.1, 0.3)
    assert state.claw_closed() is True


def test_legacy_u_is_rejected_at_simulator_boundary() -> None:
    state = SimulationStateSource(SimMappingConfig())
    with pytest.raises(ValueError, match="legacy u"):
        state.apply_target({"command": "target", "u": {"linear": 20.0}})


def test_lease_revocation_stops_mobile_base_without_resetting_arm() -> None:
    state = SimulationStateSource(SimMappingConfig())
    state.apply_target({"command": "target", "q": [-0.1, 0.2, 0.3, -0.3], "go2_vel": [0.2, 0.0, 0.0]})
    state.revoke_control()

    assert state.go2_vel() == (0.0, 0.0, 0.0)
    assert state.estimate_q().linear_m == -0.1

