from __future__ import annotations

from pathlib import Path

import pytest

from elesim_sim.simulation.mock_object_state import MockObjectState, MockObjectStateError
from elesim_sim.simulation.mock_objects import MockObjectCatalog


def _cube() -> str:
    return """\
v -1 0 -1
v  1 0 -1
v  1 0  1
v -1 0  1
v -1 2 -1
v  1 2 -1
v  1 2  1
v -1 2  1
f 1 2 3 4
f 5 8 7 6
f 1 5 6 2
f 2 6 7 3
f 3 7 8 4
f 4 8 5 1
"""


def _state(tmp_path: Path) -> MockObjectState:
    (tmp_path / "cube.obj").write_text(_cube(), encoding="utf-8")
    return MockObjectState(MockObjectCatalog(tmp_path), settle_samples=3, q_tolerance=1e-3)


def test_spawn_consumes_artifact_and_increments_generation(tmp_path: Path) -> None:
    state = _state(tmp_path)

    snapshot = state.spawn("cube", (1.0, 2.0, 3.0), (0.0, 90.0, 0.0))

    assert snapshot["state"] == "spawned"
    assert snapshot["asset_id"] == "cube"
    assert snapshot["revision"] == 1
    assert len(snapshot["silhouette_xz"]) == 4
    state.remove()
    state.reset()
    assert state.spawn("cube.obj", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))["revision"] == 2


def test_spawn_status_contains_world_oriented_planning_silhouette(tmp_path: Path) -> None:
    state = _state(tmp_path)

    snapshot = state.spawn("cube", (1.0, 2.0, 3.0), (90.0, 0.0, 0.0))

    assert snapshot["silhouette_xz"] == (
        (-1.0, 0.0),
        (1.0, 0.0),
        (1.0, 2.0),
        (-1.0, 2.0),
    )


def test_spawn_rejects_nonfinite_pose_and_preserves_previous_object(tmp_path: Path) -> None:
    state = _state(tmp_path)
    state.spawn("cube", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    with pytest.raises(MockObjectStateError, match="finite"):
        state.spawn("cube", (float("nan"), 0.0, 0.0), (0.0, 0.0, 0.0))
    assert state.snapshot()["revision"] == 1
    assert state.snapshot()["state"] == "spawned"


@pytest.mark.parametrize(
    ("position", "euler"),
    [((10.01, 0.0, 0.0), (0.0, 0.0, 0.0)), ((0.0, 0.0, 0.0), (0.0, 361.0, 0.0))],
)
def test_spawn_rejects_pose_outside_the_mock_workspace(tmp_path: Path, position, euler) -> None:
    state = _state(tmp_path)
    with pytest.raises(MockObjectStateError, match="within"):
        state.spawn("cube", position, euler)


def test_execution_requires_current_revision_and_hash(tmp_path: Path) -> None:
    state = _state(tmp_path)
    spawned = state.spawn("cube", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    digest = str(spawned["sha256"])

    with pytest.raises(MockObjectStateError, match="stale.*revision"):
        state.begin_execution("hug-1", 2, digest, (0.0, 0.0, 0.0, 0.0))
    with pytest.raises(MockObjectStateError, match="sha256"):
        state.begin_execution("hug-1", 1, "0" * 64, (0.0, 0.0, 0.0, 0.0))
    with pytest.raises(MockObjectStateError, match="finite"):
        state.begin_execution("hug-1", 1, digest, (0.0, float("inf"), 0.0, 0.0))
    assert state.snapshot()["state"] == "spawned"


def test_execution_settles_only_after_consecutive_target_samples(tmp_path: Path) -> None:
    state = _state(tmp_path)
    spawned = state.spawn("cube", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    state.begin_execution("hug-1", 1, str(spawned["sha256"]), (1.0, 2.0, 3.0, 4.0))

    assert state.observe_q((1.0005, 2.0, 3.0, 4.0))["state"] == "executing"
    assert state.observe_q((0.0, 0.0, 0.0, 0.0))["state"] == "executing"
    assert state.observe_q((1.0, 2.0, 3.0, 4.0))["state"] == "executing"
    assert state.observe_q((1.0, 2.0, 3.0, 4.0))["state"] == "executing"
    attached = state.observe_q((1.0, 2.0, 3.0, 4.0))
    assert attached["state"] == "attached"
    assert attached["attached"] is True
    assert attached["solution_id"] == "hug-1"


def test_observe_rejects_without_active_execution_or_with_nonfinite_q(tmp_path: Path) -> None:
    state = _state(tmp_path)
    with pytest.raises(MockObjectStateError, match="active mock object"):
        state.observe_q((0.0, 0.0, 0.0, 0.0))
    spawned = state.spawn("cube", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    with pytest.raises(MockObjectStateError, match="execution"):
        state.observe_q((0.0, 0.0, 0.0, 0.0))
    state.begin_execution("hug-1", 1, str(spawned["sha256"]), (0.0, 0.0, 0.0, 0.0))
    with pytest.raises(MockObjectStateError, match="finite"):
        state.observe_q((0.0, 0.0, float("nan"), 0.0))


def test_wrong_target_is_rejected_while_executing(tmp_path: Path) -> None:
    state = _state(tmp_path)
    spawned = state.spawn("cube", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    digest = str(spawned["sha256"])
    state.begin_execution("hug-1", 1, digest, (0.0, 0.0, 0.0, 0.0))

    with pytest.raises(MockObjectStateError, match="already in progress"):
        state.begin_execution("hug-2", 1, digest, (1.0, 1.0, 1.0, 1.0))


def test_same_fenced_execution_is_idempotent_for_each_waypoint(tmp_path: Path) -> None:
    state = _state(tmp_path)
    spawned = state.spawn("cube", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    args = ("hug-1", 1, str(spawned["sha256"]), (0.0, 0.0, 0.2, 0.2))
    state.accept_execution(*args)
    assert state.accept_execution(*args)["state"] == "executing"


def test_detach_returns_to_spawned_and_remove_clears_state(tmp_path: Path) -> None:
    state = _state(tmp_path)
    spawned = state.spawn("cube", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    digest = str(spawned["sha256"])
    state.begin_execution("hug-1", 1, digest, (0.0, 0.0, 0.0, 0.0))
    for _ in range(3):
        state.observe_q((0.0, 0.0, 0.0, 0.0))
    detached = state.detach()
    assert detached["state"] == "spawned"
    assert detached["revision"] == 2
    with pytest.raises(MockObjectStateError, match="stale.*revision"):
        state.accept_execution("hug-1", 1, digest, (0.0, 0.0, 0.0, 0.0))
    removed = state.remove()
    assert removed["state"] == "empty"
    assert removed["asset_id"] == ""
    assert removed["sha256"] == ""
    assert removed["available_assets"] == ("cube.obj",)


def test_snapshot_is_bounded_and_compatible_with_protocol_shape(tmp_path: Path) -> None:
    state = _state(tmp_path)
    snapshot = state.spawn("cube", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert set(snapshot) == {
        "available_assets", "state", "asset_id", "revision", "sha256",
        "position", "euler_deg", "silhouette_xz", "solution_id", "attached", "reason",
    }
    assert len(snapshot["available_assets"]) <= 16
    assert len(snapshot["silhouette_xz"]) <= 64


def test_failure_keeps_artifact_identity_for_operator_recovery(tmp_path: Path) -> None:
    state = _state(tmp_path)
    spawned = state.spawn("cube", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    failed = state.fail("stale plan")
    assert failed["state"] == "error"
    assert failed["sha256"] == spawned["sha256"]
