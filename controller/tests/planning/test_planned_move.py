from __future__ import annotations

import threading
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from elesim_controller.pick.control_ownership import ControlOwner, ControlOwnership
from elesim_controller.pick.planned_move import (
    STUCK_WAYPOINT_MIN_EXTRA_S,
    STUCK_WAYPOINT_TIMEOUT_SCALE,
    PlannedMoveExecutor,
    _waypoint_markers,
)
from elesim_controller.robot.arm.iklib import kinematics as ik_kin
from elesim_controller.robot.arm.iklib.kinematics import Q_NEUTRAL, with_base_world_transform
from elesim_controller.robot.arm.iklib.solver import load_solver_context
from elesim_controller.robot.arm.planning.collision import CollisionModel
from elesim_controller.robot.arm.planning.rrt import RrtConfig
from elesim_controller.robot.arm.planning.trajectory import JointRateLimits, resample, time_parameterize

CONFIG_PATH = Path(__file__).parents[2] / "config" / "config.yaml"
COLLISION_MODEL_PATH = Path(__file__).parents[2] / "config" / "collision_model.json"

# A random pose verified earlier (self-collision-free) to reach from Q_NEUTRAL
# without needing an obstacle detour -- keeps the "happy path" tests fast.
VALID_GOAL_Q = np.array([-0.0429, 1.2967, 0.134, 0.2884])

# theta1/theta2 bent ~32/26 degrees the *same* direction -- confirmed (live,
# against a running Simulator) to visibly self-intersect at wedge-node9.
# The pre-chain-axis-fix capsule model reported this as clear (+3.8cm); this
# is exactly the regression the chain-axis capsule fit was built to catch.
SELF_COLLIDING_GOAL_Q = np.array([-0.1711, -0.7156, 0.5585, 0.4503])

FAST_TICK_HZ = 1000.0  # only affects wall-clock sleep granularity, not physical correctness


class _RecordingMarkerSink:
    def __init__(self) -> None:
        self.calls: list[list[dict[str, Any]]] = []

    def __call__(self, markers: list[dict[str, Any]]) -> None:
        self.calls.append([dict(m) for m in markers])


class _RecordingClient:
    def __init__(self, *, cancel_after: int | None = None, cancel_event: threading.Event | None = None) -> None:
        self.calls: list[dict[str, float]] = []
        self._cancel_after = cancel_after
        self._cancel_event = cancel_event

    def send_target_values(self, *, linear_m, roll_rad, theta1_rad, theta2_rad, source, force: bool = False) -> None:
        self.calls.append(
            {"linear_m": linear_m, "roll_rad": roll_rad, "theta1_rad": theta1_rad, "theta2_rad": theta2_rad}
        )
        if self._cancel_after is not None and len(self.calls) >= self._cancel_after and self._cancel_event:
            self._cancel_event.set()


class _StuckFeedbackHostState:
    """Fake HostState whose actual_tip_xyz never moves from the start pose."""

    def __init__(
        self, actual_tip_xyz: tuple[float, float, float], *, sim_realtime_factor: float | None = None
    ) -> None:
        self.actual_tip_xyz = actual_tip_xyz
        self.sim_realtime_factor = sim_realtime_factor


class _RecordingClientWithStuckFeedback(_RecordingClient):
    """Simulates real tracking that never catches up to the commanded stream --
    e.g. cable sag/inertia holding the tip at the start pose indefinitely."""

    def __init__(
        self,
        *,
        stuck_tip_xyz: tuple[float, float, float],
        sim_realtime_factor: float | None = None,
        cancel_after: int | None = None,
        cancel_event: threading.Event | None = None,
    ) -> None:
        super().__init__(cancel_after=cancel_after, cancel_event=cancel_event)
        self._stuck_tip_xyz = stuck_tip_xyz
        self._sim_realtime_factor = sim_realtime_factor

    def refresh_state(self) -> _StuckFeedbackHostState:
        return _StuckFeedbackHostState(self._stuck_tip_xyz, sim_realtime_factor=self._sim_realtime_factor)


@pytest.fixture(scope="module")
def ik_context() -> dict:
    _bundle, context = load_solver_context(str(CONFIG_PATH))
    return context


@pytest.fixture(scope="module")
def collision_model() -> CollisionModel:
    return CollisionModel.from_json(str(COLLISION_MODEL_PATH))


def _executor(ik_context: dict, collision_model: CollisionModel | None, *, ownership: ControlOwnership | None = None) -> PlannedMoveExecutor:
    return PlannedMoveExecutor(
        ownership=ownership or ControlOwnership(),
        ik_context=ik_context,
        collision_model=collision_model,
        tick_hz=FAST_TICK_HZ,
        rrt_config=RrtConfig(max_iters=2000, seed=0),
    )


def test_waypoint_markers_use_distinct_keys_and_fk_positions(ik_context: dict) -> None:
    markers = _waypoint_markers(ik_context, [Q_NEUTRAL, Q_NEUTRAL])
    assert [m["key"] for m in markers] == ["0", "1"]
    assert all(m["name"] == "planned_waypoint" for m in markers)
    assert all(m["frame"] == "world" for m in markers)
    assert all(len(m["pos"]) == 3 for m in markers)


def test_mark_planning_sets_phase_immediately(ik_context: dict, collision_model: CollisionModel) -> None:
    """Regression: ``start_planned_move_generate_task_space`` runs a
    potentially long IK-seed search before it can call ``generate()`` --
    without an immediate status update, the UI's status line (and the
    Generate button's disabled state) stay stale for that whole search,
    reading as "Generate did nothing" even though a request was accepted."""
    executor = _executor(ik_context, collision_model)
    assert executor.status().phase == "idle"
    executor.mark_planning()
    status = executor.status()
    assert status.phase == "planning"
    assert status.waypoint_count == 0


def test_generate_without_collision_model_fails_immediately(ik_context: dict) -> None:
    executor = _executor(ik_context, None)
    sink = _RecordingMarkerSink()
    outcome = executor.generate(
        current_q=Q_NEUTRAL, target_q=VALID_GOAL_Q, send_debug_markers=sink
    )
    assert not outcome.success
    assert outcome.reason == "no_collision_model"
    assert executor.status().phase == "failed"
    assert not executor.available


def test_generate_success_publishes_markers_and_reports_planned_status(
    ik_context: dict, collision_model: CollisionModel
) -> None:
    executor = _executor(ik_context, collision_model)
    sink = _RecordingMarkerSink()
    outcome = executor.generate(
        current_q=Q_NEUTRAL, target_q=VALID_GOAL_Q, send_debug_markers=sink
    )
    assert outcome.success
    status = executor.status()
    assert status.phase == "planned"
    assert status.waypoint_count >= 1
    assert len(sink.calls) == 1
    markers = sink.calls[0]
    assert len(markers) == status.waypoint_count
    assert markers[-1]["pos"] != markers[0]["pos"] or len(markers) == 1


def test_generate_reports_cancelled_when_cancel_is_already_set(
    ik_context: dict, collision_model: CollisionModel
) -> None:
    """Regression: Cancel must be able to interrupt an in-progress *plan*
    (RRT search / shortcut / clearance smoothing), not just an in-progress
    *execute* -- see PlannedMoveExecutor.generate's ``cancel`` docstring."""
    executor = _executor(ik_context, collision_model)
    sink = _RecordingMarkerSink()
    cancel = threading.Event()
    cancel.set()

    outcome = executor.generate(
        current_q=Q_NEUTRAL, target_q=VALID_GOAL_Q, send_debug_markers=sink, cancel=cancel
    )

    assert not outcome.success
    assert outcome.reason == "cancelled"
    assert executor.status().phase == "cancelled"
    assert executor.preview_waypoints() == []
    assert sink.calls[-1] == []


def test_mark_cancelled_reports_cancelled_status(ik_context: dict, collision_model: CollisionModel) -> None:
    executor = _executor(ik_context, collision_model)
    outcome = executor.mark_cancelled()
    assert not outcome.success
    assert outcome.reason == "cancelled"
    assert executor.status().phase == "cancelled"


def test_preview_waypoints_empty_before_generate(ik_context: dict, collision_model: CollisionModel) -> None:
    executor = _executor(ik_context, collision_model)
    assert executor.preview_waypoints() == []


def test_preview_waypoints_returns_generated_path_and_clears_once_executed(
    ik_context: dict, collision_model: CollisionModel
) -> None:
    executor = _executor(ik_context, collision_model)
    sink = _RecordingMarkerSink()
    outcome = executor.generate(current_q=Q_NEUTRAL, target_q=VALID_GOAL_Q, send_debug_markers=sink)
    assert outcome.success

    waypoints = executor.preview_waypoints()
    assert len(waypoints) == executor.status().waypoint_count + 1
    assert np.allclose(waypoints[0], Q_NEUTRAL)
    assert np.allclose(waypoints[-1], VALID_GOAL_Q)

    executor.execute(client=_RecordingClient(), cancel=threading.Event(), send_debug_markers=_RecordingMarkerSink())
    assert executor.preview_waypoints() == []


def test_generate_uses_the_supplied_live_context_not_the_static_construction_context(
    ik_context: dict, collision_model: CollisionModel
) -> None:
    """Regression: markers must track the arm's *current* base pose (e.g. GO2
    having moved from its spawn point), not the static context frozen at
    executor-construction time -- reported as "orange balls drawn in a
    completely unrelated place" when this was missing."""
    shifted_base_T = np.eye(4)
    shifted_base_T[:3, 3] = [5.0, 5.0, 5.0]
    live_context = with_base_world_transform(ik_context, shifted_base_T)

    executor = _executor(ik_context, collision_model)
    sink = _RecordingMarkerSink()
    outcome = executor.generate(
        current_q=Q_NEUTRAL,
        target_q=VALID_GOAL_Q,
        send_debug_markers=sink,
        context=live_context,
    )
    assert outcome.success
    markers = sink.calls[0]
    assert markers, "expected at least one waypoint marker"
    # Every marker position must land near the shifted base, not the
    # construction-time context's own (near-origin) base.
    for marker in markers:
        pos = np.asarray(marker["pos"], dtype=float)
        assert np.linalg.norm(pos - np.array([5.0, 5.0, 5.0])) < 1.0


def test_generate_fails_when_goal_in_self_collision_and_shows_diagnostic_markers(
    ik_context: dict, collision_model: CollisionModel
) -> None:
    """A failed generate() must still give the operator something to look at --
    the (invalid) target pose and the two conflicting links -- instead of
    silently clearing every marker and leaving only a status string."""
    executor = _executor(ik_context, collision_model)
    sink = _RecordingMarkerSink()
    outcome = executor.generate(
        current_q=Q_NEUTRAL, target_q=SELF_COLLIDING_GOAL_Q, send_debug_markers=sink
    )
    assert not outcome.success
    assert outcome.reason == "goal_in_collision"
    assert executor.status().phase == "failed"
    assert len(sink.calls) == 1
    markers = sink.calls[0]
    assert markers, "expected diagnostic markers instead of an empty clear"
    keys = {m["key"] for m in markers}
    assert "collision_target" in keys
    assert {"collision_link_a", "collision_link_b"} & keys


def test_execute_without_prior_generate_fails(ik_context: dict, collision_model: CollisionModel) -> None:
    executor = _executor(ik_context, collision_model)
    sink = _RecordingMarkerSink()
    outcome = executor.execute(client=_RecordingClient(), cancel=threading.Event(), send_debug_markers=sink)
    assert not outcome.success
    assert outcome.reason == "nothing_generated"
    assert executor.status().phase == "failed"


def test_execute_streams_to_goal_and_clears_markers_on_completion(
    ik_context: dict, collision_model: CollisionModel
) -> None:
    executor = _executor(ik_context, collision_model)
    sink = _RecordingMarkerSink()
    generate_outcome = executor.generate(current_q=Q_NEUTRAL, target_q=VALID_GOAL_Q, send_debug_markers=sink)
    assert generate_outcome.success

    client = _RecordingClient()
    exec_sink = _RecordingMarkerSink()
    outcome = executor.execute(client=client, cancel=threading.Event(), send_debug_markers=exec_sink)

    assert outcome.success
    assert executor.status().phase == "done"
    assert executor.status().waypoint_count == 0
    assert len(client.calls) >= 1
    last = client.calls[-1]
    assert last["linear_m"] == pytest.approx(float(VALID_GOAL_Q[0]), abs=1e-6)
    assert last["roll_rad"] == pytest.approx(float(VALID_GOAL_Q[1]), abs=1e-6)
    assert last["theta1_rad"] == pytest.approx(float(VALID_GOAL_Q[2]), abs=1e-6)
    assert last["theta2_rad"] == pytest.approx(float(VALID_GOAL_Q[3]), abs=1e-6)
    # Markers must always end cleared, success or not.
    assert exec_sink.calls[-1] == []


def test_execute_marker_stays_until_real_feedback_confirms_arrival_not_on_a_time_schedule(
    ik_context: dict, collision_model: CollisionModel
) -> None:
    """Regression: markers must clear because the *real, observed* tip got
    there, not because the commanded trajectory's clock says it should have
    by now -- the real arm (cable sag, inertia) tracks with lag, so a
    clock-only clear can fire long before the arm is actually near that
    point (reported live: markers vanished almost as soon as motion started).
    """
    executor = _executor(ik_context, collision_model)
    gen_sink = _RecordingMarkerSink()
    generate_outcome = executor.generate(current_q=Q_NEUTRAL, target_q=VALID_GOAL_Q, send_debug_markers=gen_sink)
    assert generate_outcome.success

    # Overwrite with a hand-built path (a real *middle* waypoint, not just
    # start+goal) so its arrival time is well before the stream ends --
    # isolates the marker-timing logic from RRT's own routing, which is
    # already covered elsewhere. The first leg (Q_NEUTRAL -> mid_q) is what
    # the assertions below key off of; the pattern is then repeated back
    # and forth several more times purely to stretch the *total* trajectory
    # well past STUCK_WAYPOINT_MIN_EXTRA_S's flat floor -- otherwise the
    # per-waypoint timeout fallback would never get a chance to fire before
    # the stream itself ends, and every clear would come from the
    # unconditional "clear everything" call in execute()'s ``finally``.
    q_neutral = np.asarray(Q_NEUTRAL, dtype=float)
    mid_q = (q_neutral + VALID_GOAL_Q) / 2.0
    leg = [mid_q, VALID_GOAL_Q, mid_q, q_neutral]
    waypoints = [q_neutral, *leg, *leg]
    with executor._lock:
        executor._waypoints = waypoints
    trajectory = time_parameterize(waypoints, rates=JointRateLimits())
    first_waypoint_arrival_s = trajectory.samples[1].t_s
    assert 0.0 < first_waypoint_arrival_s < trajectory.duration_s

    # Feedback pinned at the *start* pose the whole time -- simulates real
    # tracking that never catches up to the commanded stream at all.
    stuck_tip = ik_kin._forward_grasp_world(ik_context, Q_NEUTRAL)
    client = _RecordingClientWithStuckFeedback(stuck_tip_xyz=tuple(float(x) for x in stuck_tip))

    tick_of_each_clear: list[int] = []

    def _sink(markers: list[dict[str, Any]]) -> None:
        tick_of_each_clear.append(len(client.calls))

    outcome = executor.execute(client=client, cancel=threading.Event(), send_debug_markers=_sink)
    assert outcome.success

    period_s = 1.0 / FAST_TICK_HZ
    naive_schedule_tick = first_waypoint_arrival_s / period_s
    timeout_fallback_s = max(
        STUCK_WAYPOINT_TIMEOUT_SCALE * first_waypoint_arrival_s,
        first_waypoint_arrival_s + STUCK_WAYPOINT_MIN_EXTRA_S,
    )
    timeout_fallback_tick = timeout_fallback_s / period_s

    first_clear_tick = tick_of_each_clear[0]
    # Must NOT clear anywhere near the naive (unscaled) schedule tick --
    # that would mean it's still just following the clock, feedback ignored.
    assert first_clear_tick > naive_schedule_tick * 1.5
    # Must eventually clear via the stuck-safeguard, not hang forever.
    assert first_clear_tick == pytest.approx(timeout_fallback_tick, rel=0.05)


def test_execute_stuck_waypoint_timeout_stretches_when_sim_reports_running_below_realtime(
    ik_context: dict, collision_model: CollisionModel
) -> None:
    """Regression: reported live -- a Genesis sim under heavy render/physics
    load can fall well behind real time (measured sim_realtime_factor~0.125,
    i.e. running at ~1/8 speed), so a waypoint's *rated* arrival time (which
    assumes the arm tracks commands in real time) is reached in real seconds
    long before the arm -- correctly, if slowly -- actually gets there. The
    stuck-safeguard must stretch by 1/sim_realtime_factor before firing, or
    it force-clears a waypoint the real arm was still genuinely en route to.
    """
    executor = _executor(ik_context, collision_model)
    gen_sink = _RecordingMarkerSink()
    generate_outcome = executor.generate(current_q=Q_NEUTRAL, target_q=VALID_GOAL_Q, send_debug_markers=gen_sink)
    assert generate_outcome.success

    q_neutral = np.asarray(Q_NEUTRAL, dtype=float)
    mid_q = (q_neutral + VALID_GOAL_Q) / 2.0
    leg = [mid_q, VALID_GOAL_Q, mid_q, q_neutral]
    waypoints = [q_neutral, *leg, *leg]
    with executor._lock:
        executor._waypoints = waypoints
    trajectory = time_parameterize(waypoints, rates=JointRateLimits())
    first_waypoint_arrival_s = trajectory.samples[1].t_s
    assert 0.0 < first_waypoint_arrival_s < trajectory.duration_s

    stuck_tip = ik_kin._forward_grasp_world(ik_context, Q_NEUTRAL)
    sim_realtime_factor = 0.5
    client = _RecordingClientWithStuckFeedback(
        stuck_tip_xyz=tuple(float(x) for x in stuck_tip), sim_realtime_factor=sim_realtime_factor
    )

    tick_of_each_clear: list[int] = []

    def _sink(markers: list[dict[str, Any]]) -> None:
        tick_of_each_clear.append(len(client.calls))

    outcome = executor.execute(client=client, cancel=threading.Event(), send_debug_markers=_sink)
    assert outcome.success

    period_s = 1.0 / FAST_TICK_HZ
    effective_arrival_s = first_waypoint_arrival_s / sim_realtime_factor
    timeout_fallback_s = max(
        STUCK_WAYPOINT_TIMEOUT_SCALE * effective_arrival_s,
        effective_arrival_s + STUCK_WAYPOINT_MIN_EXTRA_S,
    )
    timeout_fallback_tick = timeout_fallback_s / period_s

    first_clear_tick = tick_of_each_clear[0]
    # The unscaled (sim_realtime_factor=1.0) timeout is a *lower bound* here --
    # confirms the safeguard actually stretched, not just coincidentally
    # landed near the same place. It's not necessarily double: both terms of
    # the max(scale*arrival, arrival+floor) safeguard get the same effective
    # (scaled) arrival time, so when the flat +floor term dominates (a small
    # arrival time, as here), stretching the arrival barely moves the total.
    unscaled_timeout_s = max(
        STUCK_WAYPOINT_TIMEOUT_SCALE * first_waypoint_arrival_s,
        first_waypoint_arrival_s + STUCK_WAYPOINT_MIN_EXTRA_S,
    )
    assert first_clear_tick > unscaled_timeout_s / period_s
    assert first_clear_tick == pytest.approx(timeout_fallback_tick, rel=0.05)


def test_execute_paces_the_commanded_stream_by_measured_sim_realtime_factor(
    ik_context: dict, collision_model: CollisionModel
) -> None:
    """Regression: the commanded reference must advance through the resampled
    stream only as fast as the sim can actually realize it, not on a fixed
    real-time schedule -- otherwise the reference races through every
    intermediate waypoint's q within the first few real seconds while a
    heavily-loaded sim (measured live at sim_realtime_factor~0.125-0.18)
    is still far behind, so the real arm never gets a chance to actually
    pass near an intermediate waypoint before the reference has already
    moved on to the goal (reported live: an intermediate waypoint missed by
    16cm and only cleared via timeout_fallback, immediately followed by the
    *final* waypoint clearing via a genuine 2mm position match).
    """
    executor = _executor(ik_context, collision_model)
    gen_sink = _RecordingMarkerSink()
    generate_outcome = executor.generate(current_q=Q_NEUTRAL, target_q=VALID_GOAL_Q, send_debug_markers=gen_sink)
    assert generate_outcome.success

    with executor._lock:
        waypoints = list(executor._waypoints)
    trajectory = time_parameterize(waypoints, rates=executor._rates)
    stream = resample(trajectory, tick_hz=executor._tick_hz)
    assert len(stream) > 10  # otherwise this scenario can't demonstrate pacing at all

    stuck_tip = ik_kin._forward_grasp_world(ik_context, Q_NEUTRAL)
    sim_realtime_factor = 0.5
    n_ticks_before_cancel = 50
    cancel_event = threading.Event()
    client = _RecordingClientWithStuckFeedback(
        stuck_tip_xyz=tuple(float(x) for x in stuck_tip),
        sim_realtime_factor=sim_realtime_factor,
        cancel_after=n_ticks_before_cancel,
        cancel_event=cancel_event,
    )

    outcome = executor.execute(client=client, cancel=cancel_event, send_debug_markers=_RecordingMarkerSink())
    assert not outcome.success
    assert outcome.reason == "cancelled"
    assert len(client.calls) == n_ticks_before_cancel

    # stream_pos before the *last* send had accumulated sim_realtime_factor
    # once per completed prior tick (n_ticks_before_cancel - 1 of them).
    expected_stream_idx = min(
        int(sim_realtime_factor * (n_ticks_before_cancel - 1)), len(stream) - 1
    )
    expected_q = stream[expected_stream_idx]
    last_call = client.calls[-1]
    assert last_call["linear_m"] == pytest.approx(float(expected_q[0]), abs=1e-6)
    assert last_call["roll_rad"] == pytest.approx(float(expected_q[1]), abs=1e-6)
    assert last_call["theta1_rad"] == pytest.approx(float(expected_q[2]), abs=1e-6)
    assert last_call["theta2_rad"] == pytest.approx(float(expected_q[3]), abs=1e-6)

    # Must NOT match what an unscaled (sim_realtime_factor=1.0) schedule
    # would have sent by the same real tick -- that would mean pacing never
    # actually kicked in.
    unscaled_q = stream[min(n_ticks_before_cancel - 1, len(stream) - 1)]
    assert not np.allclose(
        [last_call["linear_m"], last_call["roll_rad"], last_call["theta1_rad"], last_call["theta2_rad"]],
        unscaled_q,
        atol=1e-6,
    )


def test_execute_cancelled_midway_reports_cancelled_and_clears_markers(
    ik_context: dict, collision_model: CollisionModel
) -> None:
    executor = _executor(ik_context, collision_model)
    sink = _RecordingMarkerSink()
    generate_outcome = executor.generate(current_q=Q_NEUTRAL, target_q=VALID_GOAL_Q, send_debug_markers=sink)
    assert generate_outcome.success

    cancel = threading.Event()
    client = _RecordingClient(cancel_after=1, cancel_event=cancel)
    exec_sink = _RecordingMarkerSink()
    outcome = executor.execute(client=client, cancel=cancel, send_debug_markers=exec_sink)

    assert not outcome.success
    assert outcome.reason == "cancelled"
    assert executor.status().phase == "cancelled"
    assert exec_sink.calls[-1] == []
    # Cancellation must stop well before reaching the final goal command.
    assert len(client.calls) < 5000


def test_execute_denied_when_ownership_already_held_by_another_owner(
    ik_context: dict, collision_model: CollisionModel
) -> None:
    ownership = ControlOwnership()
    ownership.acquire(ControlOwner.GAZE_TRACK)
    executor = _executor(ik_context, collision_model, ownership=ownership)
    sink = _RecordingMarkerSink()
    generate_outcome = executor.generate(current_q=Q_NEUTRAL, target_q=VALID_GOAL_Q, send_debug_markers=sink)
    assert generate_outcome.success

    outcome = executor.execute(client=_RecordingClient(), cancel=threading.Event(), send_debug_markers=_RecordingMarkerSink())
    assert not outcome.success
    assert outcome.reason.startswith("ownership_denied")
    assert executor.status().phase == "failed"
