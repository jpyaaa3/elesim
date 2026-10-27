"""Pilot-owned cuRobo planning and measured-state-gated path execution.

CUDA work stays off the DDS thread. Only tick/execute/cancel on that thread
may send samples. No automatic torque-on, gripper command or replanning.
"""
from __future__ import annotations

import hashlib
import threading
import time
import uuid
from pathlib import Path

import numpy as np
import yaml

from elesim_protocol import linear_effective_q_bounds
from elesim_pilot.robot.arm.iklib.kinematics import _ReachModel, base_world_transform_from_context


def q_values(q):
    return np.array([q.linear_m, q.roll_rad, q.theta1_rad, q.theta2_rad], dtype=float)


class ArmTrajectory:
    def __init__(self, connection, service, state, *, model_dir, world_path,
                 clock=time.monotonic, planner_factory=None):
        self.connection, self.service, self.state = connection, service, state
        self.model_dir = Path(model_dir).resolve()
        self.world_path = Path(world_path).resolve() if world_path else None
        self.clock, self.planner_factory = clock, planner_factory
        self._planner = None
        self._planner_digest = None
        self._worker = None
        self._generation = 0
        self._result = None
        self._lock = threading.Lock()
        self._cancel_requested = threading.Event()
        self._phase, self._reason = "idle", ""
        self._plan_id, self._plan = "", None
        self._index = 0
        self._route = None
        self._last = None
        self._stage = "idle"
        self._started_at = None
        self._finished_at = None

    def _progress(self, stage, generation):
        with self._lock:
            if generation != self._generation:
                return
            self._stage = stage
        print(f"[curobo-plan] stage={stage}", flush=True)

    @property
    def active(self):
        return self._phase in {"planning", "executing", "settling"}

    def status(self):
        plan = self._plan
        blocked = ""
        try:
            self.route()
            self._feedback()
        except RuntimeError as exc:
            blocked = str(exc)
        if self.world_path is None:
            blocked = "Configure curobo_world on the Pilot host."
        elif not blocked and not self.active:
            if self._worker is not None and self._worker.is_alive():
                blocked = "Previous CUDA task is still stopping; wait before planning again."
            elif self._busy():
                blocked = "Another arm workflow is active. Stop it before planning."
        try:
            host = self._feedback()
            context = self._context(host)
            current = _ReachModel(context=context, limit=context["limit"]).grasp_position(q_values(host.q))
            self._current_xyz = current.tolist()
        except (KeyError, TypeError, ValueError, RuntimeError, AttributeError):
            self._current_xyz = None
        return dict(available=self.world_path is not None, phase=self._phase,
                    stage=self._stage,
                    elapsed_s=(0.0 if self._started_at is None else max(0.0,
                        (self.clock() if self._finished_at is None else self._finished_at)-self._started_at)),
                    blocked_reason=blocked,
                    can_plan=not blocked and not self.active and not self._busy() and
                        not (self._worker is not None and self._worker.is_alive()),
                    reason=self._reason, plan_id=self._plan_id, waypoint=self._index,
                    waypoints=0 if plan is None else len(plan[0]),
                    duration_s=0.0 if plan is None else (len(plan[0])-1)*plan[1],
                    target_xyz=getattr(self, "_target", None),
                    current_xyz=self._current_xyz,
                    goal_q=None if plan is None else plan[0][-1].tolist(),
                    world="unconfigured" if self.world_path is None else (
                        "empty (self collision only)" if getattr(self, "_empty_world", False)
                        else "static file; no live Genesis sync"))

    def route(self):
        c = self.connection
        if not c.active_target:
            raise RuntimeError("Robot is not connected. Start Robot and select its endpoint.")
        d = next((v for v in c.endpoints if v.get("endpoint_id") == c.active_target), {})
        if d.get("role") != "robot" or not d.get("instance_id") or not c.lease_id:
            raise RuntimeError("trajectory requires an exact live Robot boot and motion lease")
        return c.active_target, d["instance_id"], c.lease_id

    def _feedback(self, *, torque=False):
        s = self.connection.state_sink.get_state()
        received = self.connection.telemetry_received_at
        if (not s.connected or received is None or self.clock()-received > 0.35
                or s.q is None or not np.isfinite(q_values(s.q)).all()):
            raise RuntimeError("fresh measured joint telemetry required (<=350 ms)")
        if s.safety_fault or not s.reply_ok:
            raise RuntimeError(s.safety_fault or s.reply_reason or "Robot rejected a command")
        raw = self.connection.trajectory_telemetry
        if raw.get("imu_model") != {"id": "identity", "version": 1}:
            raise RuntimeError("trajectory requires Robot identity/v1 correction model")
        if raw.get("hardware_read_failures", 0) or raw.get("read_error"):
            raise RuntimeError("Robot hardware read failure")
        if torque and not s.torque_enabled:
            raise RuntimeError("Torque must already be ON; executor never enables it")
        if s.claw_closed is not False:
            raise RuntimeError("planning model requires measured open claws")
        if max(abs(float(v)) for v in s.go2_vel) > 1e-4:
            raise RuntimeError("GO2 must be stationary for arm-only planning")
        for key in ("go2_base_lin_vel_body", "go2_base_ang_vel"):
            values = getattr(s, key, None)
            if values is not None and np.linalg.norm(values) > 0.01:
                raise RuntimeError("measured GO2 base must be stationary")
        return s

    def _digest(self):
        if self.world_path is None:
            raise RuntimeError("configure an explicit curobo_world file on the Pilot host")
        return tuple(hashlib.sha256(p.read_bytes()).hexdigest() for p in (
            self.model_dir/"metadata.json", self.model_dir/"arm.urdf",
            self.model_dir/"robot.yml", self.world_path))

    def _context(self, host):
        return self.service._ik_context_for_host(host)

    def _busy(self):
        return (self.service._visual_busy() or self.service._pick_busy()
                or self.service._gaze_busy() or self.service.pick_e2e_running()
                or self.service.wrap_grasp_running() or bool(self.state.ik_running)
                or bool(self.service.sim_reset_in_progress))

    def plan_arm_to_target(self):
        if not self.active:
            self._started_at, self._finished_at, self._stage = self.clock(), None, "preflight"
        try:
            return self._plan_arm_to_target()
        except Exception as exc:
            if not self.active:
                self._phase, self._stage, self._reason = "failed", "preflight failed", str(exc)
                self._plan, self._plan_id = None, ""
                self._finished_at = self.clock()
            raise

    def _plan_arm_to_target(self):
        if self.active or (self._worker is not None and self._worker.is_alive()) or self._busy():
            raise RuntimeError("another arm workflow is active or still stopping")
        host = self._feedback()
        route, digest = self.route(), self._digest()
        context = self._context(host)
        base = base_world_transform_from_context(context)
        target = np.array([self.state.target_x, self.state.target_y, self.state.target_z], dtype=float)
        if not np.isfinite(target).all():
            raise ValueError("Target XYZ must be finite world coordinates in metres")
        start = q_values(host.q)
        self._target, self._start, self._base = target.tolist(), start, base
        model = _ReachModel(context=context, limit=context["limit"])
        self._current_xyz = model.grasp_position(start).tolist()
        self._route, self._digest_at_plan = route, digest
        self._generation += 1
        generation = self._generation
        self._phase, self._reason, self._plan, self._plan_id = "planning", "", None, ""
        self._index = 0
        self.connection.clear_pending_motion()

        def worker():
            try:
                if self._planner is None or self._planner_digest != digest:
                    self._progress("Loading CUDA and collision model (first run may take 2-3 minutes)", generation)
                    from elesim_pilot.robot.arm.curobo import CuRoboPlanner
                    world = yaml.safe_load(self.world_path.read_text())
                    if not isinstance(world, dict):
                        raise ValueError("cuRobo world must be a mapping; {} explicitly means empty")
                    self._empty_world = not bool(world)
                    factory = self.planner_factory or CuRoboPlanner
                    self._planner = factory(self.model_dir, world=world)
                    self._planner_digest = digest
                planner = self._planner
                expected_linear = linear_effective_q_bounds(self.connection.mapping)
                limit = context["limit"]
                expected_bounds = [expected_linear, (limit.roll_min_rad(), limit.roll_max_rad()),
                                   (-limit.bend_lim_rad(), limit.bend_lim_rad()),
                                   (-limit.bend_lim_rad(), limit.bend_lim_rad())]
                if not np.allclose(planner.bounds, expected_bounds, atol=1e-9, rtol=0):
                    raise RuntimeError("planning model differs from configured joint bounds")
                local = base[:3, :3].T @ (target-base[:3, 3])
                self._progress("Solving position IK", generation)
                goal = np.asarray(planner.solve_position(local, seed=start, tolerance_m=0.001))
                if np.linalg.norm(model.error_vec(goal, target)) > 0.001:
                    raise RuntimeError("cuRobo/Pilot grasp FK mismatch")
                self._progress("Planning collision-checked joint path", generation)
                path = planner.plan_joint(start, goal)
                self._progress("Validating path and grasp FK", generation)
                positions, dt = np.asarray(path.positions, dtype=float), float(path.dt_s)
                if (positions.ndim != 2 or positions.shape[1] != 4 or not 2 <= len(positions) <= 4000
                        or not np.isfinite(positions).all() or not np.isfinite(dt) or not 0.03 <= dt <= 0.1
                        or (len(positions)-1)*dt > 120):
                    raise RuntimeError("invalid or oversized timed trajectory")
                # The CUDA adapter checks collisions, limits, endpoints and speed.
                # Repeat endpoint parity before making a live execution available.
                if not np.allclose(positions[0], start, atol=1e-4, rtol=0):
                    raise RuntimeError("trajectory start differs from measured start")
                for q in (start, positions[-1]):
                    gpu_world = base[:3, :3] @ np.asarray(planner.forward(q)["position"])+base[:3, 3]
                    if np.linalg.norm(gpu_world-model.grasp_position(q)) > 1e-4:
                        raise RuntimeError("cuRobo/Pilot FK calibration mismatch")
                result = (positions, dt)
            except Exception as exc:
                result = str(exc)
            with self._lock:
                self._result = generation, result

        self._worker = threading.Thread(target=worker, name="curobo-plan", daemon=True)
        self._worker.start()
        return self.status()

    def execute_arm_trajectory(self, plan_id):
        if self._phase != "ready" or not plan_id or plan_id != self._plan_id:
            raise RuntimeError("no matching unconsumed prepared trajectory")
        host = self._feedback(torque=True)
        if (self.route() != self._route or self._digest() != self._digest_at_plan
                or self.clock()-self._prepared_at > 120 or self._busy()):
            raise RuntimeError("plan expired, route/model/world changed or arm workflow busy; replan")
        if not np.allclose(self._base, base_world_transform_from_context(self._context(host)), atol=1e-5, rtol=0):
            raise RuntimeError("arm base moved; replan")
        if np.any(np.abs(q_values(host.q)-self._start) > [0.001, 0.015, 0.015, 0.015]):
            raise RuntimeError("measured start moved; replan from the current posture")
        current_target = [self.state.target_x, self.state.target_y, self.state.target_z]
        if not np.allclose(current_target, self._target, atol=1e-7, rtol=0):
            raise RuntimeError("Target XYZ changed; replan")
        self.connection.clear_pending_motion()
        self._phase, self._index, self._last = "executing", 0, self._start.copy()
        self._stage = "Streaming timed path"
        self._next_at = self.clock()
        return self.status()

    def cancel_arm_trajectory(self):
        return self._cancel("operator cancelled")

    def request_cancel(self):
        """Lifecycle thread requests; DDS thread owns lease release."""
        self._cancel_requested.set()

    def _cancel(self, reason, *, release=True):
        moving = self._phase in {"executing", "settling"}
        self._generation += 1
        self._phase, self._reason, self._plan_id = "cancelled", str(reason), ""
        self._stage, self._finished_at = "cancelled", self.clock()
        self.connection.clear_pending_motion()
        if moving and release:
            # Releasing the exact lease invokes Robot's local measured safe hold.
            # Do not replace a stale/lost target with a newly selected target.
            self.connection.release_trajectory_route(self._route)
        return self.status()

    def guard_operator(self, operation, name):
        if self.active and operation != "view_snapshot":
            if operation == "service_call" and name in {"cancel_arm_trajectory", "torque_off"}:
                self._cancel(name, release=name != "torque_off")
            else:
                raise RuntimeError("cuRobo workflow active; cancel it before other operator changes")

    def allow_motion(self, payload):
        if not self.active:
            return True
        if payload.get("command") in {"estop", "torque_off"}:
            self._cancel(payload["command"], release=False)
            return True
        return False

    def tick(self):
        if self._cancel_requested.is_set():
            self._cancel_requested.clear()
            self._cancel("Pilot stopping")
            return
        with self._lock:
            result, self._result = self._result, None
        if result is not None and result[0] == self._generation:
            self._finished_at = self.clock()
            if isinstance(result[1], str):
                self._phase, self._reason = "failed", result[1]
                self._stage = "failed"
            else:
                self._plan = result[1]
                self._phase, self._plan_id = "ready", uuid.uuid4().hex
                self._stage = "Ready to execute (no movement sent)"
                self._prepared_at = self.clock()
        if self._phase not in {"executing", "settling"}:
            return
        try:
            now = self.clock()
            if self.route() != self._route:
                raise RuntimeError("target boot/lease changed")
            host = self._feedback(torque=True)
            if not np.allclose(self._base, base_world_transform_from_context(self._context(host)), atol=1e-5, rtol=0):
                raise RuntimeError("arm base moved")
            measured = q_values(host.q)
            if np.any(np.abs(measured-self._last) > [0.004, 0.06, 0.06, 0.06]):
                raise RuntimeError("measured tracking error exceeds 4 mm / 0.06 rad")
            if now < self._next_at:
                return
            if now-self._next_at > 0.1:
                raise RuntimeError("trajectory scheduling overrun; no catch-up jump permitted")
            positions, dt = self._plan
            if self._phase == "settling":
                if np.all(np.abs(measured-positions[-1]) <= [0.001, 0.015, 0.015, 0.015]):
                    self._phase, self._plan_id = "completed", ""
                    self._stage = "Measured goal reached"
                    self.state.set_q(*map(float, measured))
                    return
                if now-self._settling_at > 3:
                    raise RuntimeError("goal did not settle within 3 seconds")
                q = positions[-1]
            else:
                q = positions[self._index]
                self._index += 1
                if self._index == len(positions):
                    self._phase, self._settling_at = "settling", now
            self.connection.send_trajectory_q(q.tolist(), self._route)
            self._last = q.copy()
            self.state.set_q(*map(float, q))
            self._next_at += dt
        except Exception as exc:
            self._cancel(str(exc))
            self._phase = "failed"
            self._stage = "failed"

