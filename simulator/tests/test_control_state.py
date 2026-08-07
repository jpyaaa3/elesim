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


def test_planned_move_preview_waypoints_parse_and_advance_the_seq_counter() -> None:
    state = SimulationStateSource(SimMappingConfig())
    assert state.planned_move_preview_waypoints() == []
    assert state.planned_move_preview_seq() == 0

    waypoints = [[0.0, 0.0, 0.0, 0.0], [-0.1, 0.2, 0.3, -0.1]]
    state.apply_target({"command": "target", "planned_move_preview_waypoints": waypoints})

    assert state.planned_move_preview_waypoints() == [(0.0, 0.0, 0.0, 0.0), (-0.1, 0.2, 0.3, -0.1)]
    assert state.planned_move_preview_seq() == 1


def test_planned_move_preview_seq_advances_even_on_an_identical_repeated_payload() -> None:
    """Each Preview click must restart playback from the start -- including a
    click that resends the exact same plan -- so the seq counter (which the
    Simulator watches for a rising edge) has to advance unconditionally
    whenever the field is present, not just when the value changes."""
    state = SimulationStateSource(SimMappingConfig())
    waypoints = [[0.0, 0.0, 0.0, 0.0], [-0.1, 0.2, 0.3, -0.1]]
    payload = {"command": "target", "planned_move_preview_waypoints": waypoints}

    state.apply_target(payload)
    state.apply_target(payload)

    assert state.planned_move_preview_seq() == 2


def test_planned_move_preview_waypoints_rejects_malformed_entries() -> None:
    state = SimulationStateSource(SimMappingConfig())
    with pytest.raises(ValueError):
        state.apply_target(
            {"command": "target", "planned_move_preview_waypoints": [[0.0, 0.0, 0.0]]}
        )
    with pytest.raises(ValueError):
        state.apply_target({"command": "target", "planned_move_preview_waypoints": "not-a-list"})

