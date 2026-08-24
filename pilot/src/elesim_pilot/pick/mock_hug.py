"""Mock-object hug planning owned by Pilot.

This is deliberately an open-space local planner.  It consumes Sim's bounded
2-D planning projection and leaves obstacle-aware approach planning behind a
small planner interface for the later reconstruction/RRT stage.  The target-
first identity and two-section closure are adapted from ``grasp/hug/hug.py``;
its Shapely contact-pair optimizer is not imported across repositories.  The
present bounded approximation is therefore mock-only, not a physical grasp
certificate.
"""

from __future__ import annotations

import hashlib
import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, Protocol

from elesim_protocol import SimMappingConfig, SimQ, SimulationStatusPayload


class MockHugError(RuntimeError):
    pass


@dataclass(frozen=True)
class MockHugSolution:
    solution_id: str
    object_revision: int
    object_sha256: str
    final_q: tuple[float, float, float, float]
    waypoints: tuple[tuple[float, float, float, float], ...]
    clearance_mode: str = "open-space"

    def to_payload(self) -> dict[str, object]:
        return {
            "solution_id": self.solution_id,
            "object_revision": self.object_revision,
            "object_sha256": self.object_sha256,
            "final_q": list(self.final_q),
            "waypoints": [list(value) for value in self.waypoints],
            "clearance_mode": self.clearance_mode,
        }


class LocalTrajectoryPlanner(Protocol):
    def plan(self, start: SimQ, goal: SimQ) -> tuple[tuple[float, float, float, float], ...]: ...


class OpenSpaceTrajectoryPlanner:
    """Bounded interpolation seam to be replaced by environment-aware planning."""

    def __init__(self, *, steps: int = 12) -> None:
        self.steps = max(2, min(int(steps), 64))

    def plan(self, start: SimQ, goal: SimQ) -> tuple[tuple[float, float, float, float], ...]:
        left = (start.linear_m, start.roll_rad, start.theta1_rad, start.theta2_rad)
        right = (goal.linear_m, goal.roll_rad, goal.theta1_rad, goal.theta2_rad)
        return tuple(
            tuple(a + (b - a) * index / self.steps for a, b in zip(left, right))
            for index in range(1, self.steps + 1)
        )


def solve_mock_hug(
    status: SimulationStatusPayload,
    *,
    current_q: SimQ,
    mapping: SimMappingConfig,
    planner: LocalTrajectoryPlanner | None = None,
) -> MockHugSolution:
    obj = status.mock_object
    if obj is None or obj.state == "empty":
        raise MockHugError("Sim has no spawned mock object")
    if obj.attached:
        raise MockHugError("mock object is already attached")
    if obj.state != "spawned":
        raise MockHugError(f"mock object is not available for planning ({obj.state})")
    if len(obj.silhouette_xz) < 3:
        raise MockHugError("mock object has no usable XZ silhouette")
    xs = [point[0] for point in obj.silhouette_xz]
    zs = [point[1] for point in obj.silhouette_xz]
    radius = 0.5 * math.hypot(max(xs) - min(xs), max(zs) - min(zs))
    if radius > 0.18:
        raise MockHugError(f"mock object is too large for bounded mock hug ({radius:.3f} m)")

    # The present mock is a target-local closure estimate, not global path
    # planning.  World X chooses prismatic reach; Y/Z choose the curl plane.
    linear = min(mapping.linear_q_max_m, max(mapping.linear_q_min_m, 0.45 - obj.position[0]))
    roll = math.atan2(obj.position[1], max(0.05, obj.position[2]))
    roll = min(mapping.roll_q_max_rad, max(mapping.roll_q_min_rad, roll))
    closure = min(math.radians(34.0), max(math.radians(12.0), math.radians(15.0) + radius * 1.6))
    theta1 = min(mapping.seg1_q_max_rad, max(mapping.seg1_q_min_rad, closure))
    theta2 = min(mapping.seg2_q_max_rad, max(mapping.seg2_q_min_rad, closure))
    goal = SimQ(linear, roll, theta1, theta2)
    waypoints = (planner or OpenSpaceTrajectoryPlanner()).plan(current_q, goal)
    identity = {
        "revision": obj.revision,
        "sha256": obj.sha256,
        "final_q": list(waypoints[-1]),
    }
    solution_id = hashlib.sha256(
        json.dumps(identity, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()[:24]
    return MockHugSolution(solution_id, obj.revision, obj.sha256, waypoints[-1], waypoints)


class MockHugCoordinator:
    """Own one latest solution and a cancellable, non-blocking execution."""

    def __init__(
        self,
        client,
        latest_status: Callable[[], Optional[SimulationStatusPayload]],
        current_q: Callable[[], SimQ],
        *,
        mapping: SimMappingConfig,
        period_s: float = 0.05,
        execution_context: Callable[[], tuple[str, str, str]] | None = None,
    ) -> None:
        self.client = client
        self.latest_status = latest_status
        self.current_q = current_q
        self.mapping = mapping
        self.period_s = max(0.01, float(period_s))
        self.execution_context = execution_context
        self._lock = threading.Lock()
        self._solution: MockHugSolution | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._error = ""

    def compute(self) -> dict[str, object]:
        status = self.latest_status()
        if status is None:
            raise MockHugError("no simulation status has been received")
        solution = solve_mock_hug(
            status, current_q=self.current_q(), mapping=self.mapping
        )
        with self._lock:
            self._solution = solution
            self._error = ""
        return solution.to_payload()

    def execute(self, solution_id: str) -> dict[str, object]:
        with self._lock:
            solution = self._solution
            running = self._thread is not None and self._thread.is_alive()
        if solution is None or solution.solution_id != str(solution_id):
            raise MockHugError("mock hug solution is missing or stale")
        if running:
            raise MockHugError("mock hug execution is already running")
        self._assert_fresh(solution)
        context = self._current_execution_context()
        self._stop.clear()
        thread = threading.Thread(
            target=self._run,
            args=(solution, context),
            name="pilot-mock-hug",
            daemon=True,
        )
        with self._lock:
            self._thread = thread
        thread.start()
        return {"solution_id": solution.solution_id, "state": "executing"}

    def _assert_fresh(self, solution: MockHugSolution) -> None:
        status = self.latest_status()
        obj = None if status is None else status.mock_object
        if obj is None or obj.revision != solution.object_revision or obj.sha256 != solution.object_sha256:
            raise MockHugError("spawned mock object changed after hug computation")

    def _current_execution_context(self) -> tuple[str, str, str] | None:
        if self.execution_context is None:
            return None
        context = tuple(str(value) for value in self.execution_context())
        if len(context) != 3 or not all(context):
            raise MockHugError("mock hug requires an exact Sim boot and motion lease")
        return context  # type: ignore[return-value]

    def _assert_execution_context(self, expected: tuple[str, str, str] | None) -> None:
        if expected is not None and self._current_execution_context() != expected:
            raise MockHugError("Sim target boot or motion lease changed during mock hug")

    def _run(
        self,
        solution: MockHugSolution,
        context: tuple[str, str, str] | None,
    ) -> None:
        try:
            for values in solution.waypoints:
                if self._stop.is_set():
                    return
                self._assert_execution_context(context)
                self._assert_fresh(solution)
                self.client.send_mock_hug_target(
                    q=SimQ(*values),
                    solution=solution,
                    execution_context=context,
                )
                time.sleep(self.period_s)
        except Exception as exc:
            with self._lock:
                self._error = str(exc) or type(exc).__name__

    def cancel(self, reason: str = "mock hug cancelled") -> None:
        self._stop.set()
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                self._error = str(reason)

    def close(self) -> None:
        self.cancel("mock hug coordinator closed")
        with self._lock:
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)


__all__ = [
    "LocalTrajectoryPlanner", "MockHugCoordinator", "MockHugError", "MockHugSolution",
    "OpenSpaceTrajectoryPlanner", "solve_mock_hug",
]
