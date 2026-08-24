from __future__ import annotations

import threading

import pytest

from elesim_protocol import MockObjectStatePayload, SimMappingConfig, SimQ, SimulationStatusPayload
from elesim_pilot.pick.mock_hug import MockHugCoordinator, MockHugError, solve_mock_hug


def status(*, revision: int = 1, size: float = 0.08) -> SimulationStatusPayload:
    return SimulationStatusPayload(
        epoch=0,
        paused=False,
        speed=1.0,
        debug_visible=True,
        sim_time_s=0.0,
        mock_object=MockObjectStatePayload(
            available_assets=("box",),
            state="spawned",
            asset_id="box",
            revision=revision,
            sha256="a" * 64,
            position=(0.5, 0.0, 0.4),
            silhouette_xz=((-size, -size), (size, -size), (size, size), (-size, size)),
        ),
    )


def test_solver_generates_bounded_directional_open_space_path() -> None:
    cfg = SimMappingConfig()
    result = solve_mock_hug(
        status(), current_q=SimQ(-0.2, 0.0, 0.0, 0.0), mapping=cfg
    )
    assert result.clearance_mode == "open-space"
    assert len(result.waypoints) == 12
    assert result.final_q == result.waypoints[-1]
    assert cfg.seg1_q_min_rad <= result.final_q[2] <= cfg.seg1_q_max_rad
    assert cfg.seg2_q_min_rad <= result.final_q[3] <= cfg.seg2_q_max_rad


def test_solver_rejects_absent_attached_and_oversized_objects() -> None:
    empty = SimulationStatusPayload(0, False, 1.0, True, 0.0)
    with pytest.raises(MockHugError, match="no spawned"):
        solve_mock_hug(empty, current_q=SimQ(0, 0, 0, 0), mapping=SimMappingConfig())
    with pytest.raises(MockHugError, match="too large"):
        solve_mock_hug(status(size=0.3), current_q=SimQ(0, 0, 0, 0), mapping=SimMappingConfig())


def test_solution_identity_changes_with_spawn_revision() -> None:
    q = SimQ(-0.1, 0.0, 0.0, 0.0)
    first = solve_mock_hug(status(revision=1), current_q=q, mapping=SimMappingConfig())
    second = solve_mock_hug(status(revision=2), current_q=q, mapping=SimMappingConfig())
    assert first.solution_id != second.solution_id


def test_coordinator_stops_before_sending_again_when_lifecycle_revision_changes() -> None:
    latest = [status(revision=1)]
    sent: list[object] = []
    first_sent = threading.Event()

    class Client:
        def send_mock_hug_target(self, *, q, solution, execution_context=None) -> None:
            sent.append((q, solution.solution_id))
            latest[0] = status(revision=2)
            first_sent.set()

    coordinator = MockHugCoordinator(
        Client(),
        lambda: latest[0],
        lambda: SimQ(-0.2, 0.0, 0.0, 0.0),
        mapping=SimMappingConfig(),
        period_s=0.01,
    )
    solution = coordinator.compute()
    coordinator.execute(str(solution["solution_id"]))
    assert first_sent.wait(timeout=1.0)
    assert coordinator._thread is not None
    coordinator._thread.join(timeout=1.0)
    coordinator.close()

    assert not coordinator._thread.is_alive()
    assert len(sent) == 1
    assert "changed" in coordinator._error


def test_coordinator_fences_every_waypoint_to_exact_target_boot_and_lease() -> None:
    latest = [status()]
    context = [["sim-a", "boot-a", "lease-a"]]
    sent: list[object] = []
    first_sent = threading.Event()

    class Client:
        def send_mock_hug_target(self, *, q, solution, execution_context=None) -> None:
            sent.append(q)
            context[0][2] = "lease-b"
            first_sent.set()

    coordinator = MockHugCoordinator(
        Client(),
        lambda: latest[0],
        lambda: SimQ(-0.2, 0.0, 0.0, 0.0),
        mapping=SimMappingConfig(),
        period_s=0.01,
        execution_context=lambda: tuple(context[0]),
    )
    solution = coordinator.compute()
    coordinator.execute(str(solution["solution_id"]))
    assert first_sent.wait(timeout=1.0)
    assert coordinator._thread is not None
    coordinator._thread.join(timeout=1.0)
    coordinator.close()

    assert len(sent) == 1
    assert "lease changed" in coordinator._error
