from __future__ import annotations

import pytest
from pathlib import Path

from elesim_protocol import SimMappingConfig
from elesim_sim.control_state import SimulationStateSource
from elesim_sim.simulation.mock_objects import MockObjectCatalog
from elesim_sim.simulation.mock_object_state import MockObjectState


def test_canonical_target_updates_all_sim_command_state() -> None:
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


def test_legacy_u_is_rejected_at_sim_boundary() -> None:
    state = SimulationStateSource(SimMappingConfig())
    with pytest.raises(ValueError, match="legacy u"):
        state.apply_target({"command": "target", "u": {"linear": 20.0}})


def test_lease_revocation_stops_mobile_base_without_resetting_arm() -> None:
    state = SimulationStateSource(SimMappingConfig())
    state.apply_target({"command": "target", "q": [-0.1, 0.2, 0.3, -0.3], "go2_vel": [0.2, 0.0, 0.0]})
    state.revoke_control()

    assert state.go2_vel() == (0.0, 0.0, 0.0)
    assert state.estimate_q().linear_m == -0.1


def test_mock_hug_waypoint_is_identity_checked_before_q_is_applied() -> None:
    mock = MockObjectState(
        MockObjectCatalog(Path(__file__).resolve().parents[1] / "config/mock_objects")
    )
    spawned = mock.spawn("demo_box", (0.5, 0.0, 0.4), (0.0, 0.0, 0.0))
    state = SimulationStateSource(SimMappingConfig(), mock_object_state=mock)
    q = [-0.1, 0.2, 0.3, 0.3]
    state.apply_target(
        {
            "command": "target",
            "q": q,
            "mock_hug": {
                "solution_id": "solution-1",
                "object_revision": spawned["revision"],
                "object_sha256": spawned["sha256"],
                "final_q": q,
            },
        }
    )
    assert state.estimate_q().linear_m == q[0]
    assert mock.snapshot()["solution_id"] == "solution-1"

    state.revoke_control()
    assert mock.snapshot()["state"] == "error"
    assert mock.snapshot()["reason"] == "motion lease revoked"


def test_stale_mock_hug_waypoint_cannot_move_the_arm() -> None:
    mock = MockObjectState(
        MockObjectCatalog(Path(__file__).resolve().parents[1] / "config/mock_objects")
    )
    mock.spawn("demo_box", (0.5, 0.0, 0.4), (0.0, 0.0, 0.0))
    state = SimulationStateSource(SimMappingConfig(), mock_object_state=mock)
    before = state.estimate_q()
    q = [-0.1, 0.0, 0.2, 0.2]
    with pytest.raises(ValueError, match="stale"):
        state.apply_target(
            {
                "command": "target",
                "q": q,
                "mock_hug": {
                    "solution_id": "stale",
                    "object_revision": 2,
                    "object_sha256": "a" * 64,
                    "final_q": q,
                },
            }
        )
    assert state.estimate_q() == before
