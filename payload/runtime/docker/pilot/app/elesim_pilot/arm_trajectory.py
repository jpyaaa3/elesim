"""Pilot-owned cuRobo planning and measured-state-gated path execution.

CUDA work stays off the DDS thread. Only tick/execute/cancel on that thread
may send samples. No automatic torque-on, gripper command or replanning.
"""
from __future__ import annotations

import hashlib
import multiprocessing
import threading
import time
import uuid
from pathlib import Path

import numpy as np
import yaml

from elesim_protocol import linear_effective_q_bounds
from elesim_pilot.robot.arm.iklib.kinematics import _ReachModel, base_world_transform_from_context

_GO2_STATIONARY_LINEAR_SPEED_MPS = 0.01
_GO2_STATIONARY_ANGULAR_SPEED_RADPS = 0.05
# The GO2 pose estimate jitters by roughly 0.6 mm / 0.1 deg while standing
# still. A 10 um element-wise matrix comparison treated that sensor noise as
# real base motion and made every prepared path unexecutable.
_ARM_BASE_POSITION_TOLERANCE_M = 0.002
_ARM_BASE_ANGLE_TOLERANCE_RAD = np.deg2rad(0.25)
_TRAJECTORY_REBASE_AFTER_S = 0.1
_TRAJECTORY_MAX_TICK_STALL_S = 0.25
_TRAJECTORY_GOAL_SETTLE_STABLE_SAMPLES = 3
_CUROBO_STAGE_TIMEOUTS_S = {
    "Starting CUDA worker": 30.0,
    "Loading CUDA and collision model": 900.0,
    "Initializing CUDA position IK solver": 900.0,
    "Solving position IK": 180.0,
    "Planning collision-checked joint path": 180.0,
    "Validating path and grasp FK": 180.0,
}


def _curobo_worker_process(connection, model_dir: str, world_path: str) -> None:
    """Keep the CUDA context in an isolated, restartable process.

    cuRobo/Torch calls can block inside native CUDA code, where Python thread
    cancellation cannot run. The Pilot can terminate this process without
    disturbing its DDS thread or the Robot lease.
    """
    planner = None
    planner_digest = None
    empty_world = False

    def send_stage(stage):
        connection.send({"type": "stage", "stage": stage})

    while True:
        try:
            request = connection.recv()
        except (EOFError, OSError):
            return
        if request.get("type") == "stop":
            return
        try:
            digest = tuple(request["digest"])
            if planner is None or planner_digest != digest:
                send_stage("Loading CUDA and collision model")
                world = yaml.safe_load(Path(world_path).read_text())
                if not isinstance(world, dict):
                    raise ValueError("cuRobo world must be a mapping; {} explicitly means empty")
                from elesim_pilot.robot.arm.curobo import CuRoboPlanner
                planner = CuRoboPlanner(model_dir, world=world)
                planner_digest = digest
                empty_world = not bool(world)

            expected_bounds = np.asarray(request["expected_bounds"], dtype=float)
            if not np.allclose(planner.bounds, expected_bounds, atol=1e-9, rtol=0):
                raise RuntimeError("planning model differs from configured joint bounds")

            start = np.asarray(request["start"], dtype=float)
            target = np.asarray(request["target"], dtype=float)
            base = np.asarray(request["base"], dtype=float)
            local = base[:3, :3].T @ (target-base[:3, 3])
            send_stage("Solving position IK")
            goal, path, goal_local = planner.plan_to_target(
                start, local, progress=send_stage)
            goal = np.asarray(goal, dtype=float)
            goal_local = np.asarray(goal_local, dtype=float)
            if (goal_local.shape != (3,) or not np.isfinite(goal_local).all()
                    or np.linalg.norm(goal_local-local) > 1e-6):
                raise RuntimeError("cuRobo goal differs from the target object centre")

            send_stage("Validating path and grasp FK")
            positions = np.asarray(path.positions, dtype=float)
            dt = float(path.dt_s)
            if (positions.ndim != 2 or positions.shape[1] != 4
                    or not 2 <= len(positions) <= 4000
                    or not np.isfinite(positions).all()
                    or not np.isfinite(dt) or not 0.03 <= dt <= 0.1
                    or (len(positions)-1)*dt > 120):
                raise RuntimeError("invalid or oversized timed trajectory")
            subdivisions = max(1, int(np.ceil(dt / 0.025)))
            if subdivisions > 1:
                dense = [positions[0]]
                for left, right in zip(positions[:-1], positions[1:]):
                    dense.extend(left + (right-left) * (step/subdivisions)
                                 for step in range(1, subdivisions+1))
                positions = np.asarray(dense, dtype=float)
                dt /= subdivisions
            if len(positions) > 5000:
                raise RuntimeError("resampled trajectory exceeds 5000 waypoints")
            planner.validate_path(positions, target_sphere_center=local)
            if not np.allclose(positions[0], start, atol=1e-4, rtol=0):
                raise RuntimeError("trajectory start differs from measured start")
            fk = [planner.forward(q)["position"] for q in (start, positions[-1])]
            if np.linalg.norm(np.asarray(fk[-1])-local) > 0.001:
                raise RuntimeError("trajectory endpoint does not place the target between the claws")
            response = {
                "type": "result", "ok": True, "goal": goal.tolist(),
                "positions": positions.tolist(), "dt": dt,
                "fk": fk, "goal_xyz": goal_local.tolist(),
                "empty_world": empty_world,
            }
        except Exception as exc:
            response = {"type": "result", "ok": False, "error": str(exc)}
        try:
            connection.send(response)
        except (BrokenPipeError, EOFError, OSError):
            return


def arm_base_pose_delta(reference, current):
    """Return translation and geodesic rotation delta for two world poses."""
    reference = np.asarray(reference, dtype=float)
    current = np.asarray(current, dtype=float)
    if (reference.shape != (4, 4) or current.shape != (4, 4)
            or not np.isfinite(reference).all() or not np.isfinite(current).all()):
        raise RuntimeError("invalid measured arm base transform")
    position_m = float(np.linalg.norm(reference[:3, 3] - current[:3, 3]))
    relative = reference[:3, :3].T @ current[:3, :3]
    cosine = float(np.clip((np.trace(relative) - 1.) / 2., -1., 1.))
    angle_rad = float(np.arccos(cosine))
    return position_m, angle_rad


def require_stable_arm_base(reference, current):
    """Reject meaningful base motion while tolerating measured pose jitter."""
    position_m, angle_rad = arm_base_pose_delta(reference, current)
    if (position_m > _ARM_BASE_POSITION_TOLERANCE_M
            or angle_rad > _ARM_BASE_ANGLE_TOLERANCE_RAD):
        raise RuntimeError(
            "arm base moved; replan "
            f"(translation {position_m * 1000:.2f} mm / "
            f"{_ARM_BASE_POSITION_TOLERANCE_M * 1000:.2f} mm, "
            f"rotation {np.rad2deg(angle_rad):.3f} deg / "
            f"{np.rad2deg(_ARM_BASE_ANGLE_TOLERANCE_RAD):.3f} deg)"
        )


def q_values(q):
    return np.array([q.linear_m, q.roll_rad, q.theta1_rad, q.theta2_rad], dtype=float)


def measured_planning_start(measured, bounds):
    """Project only sub-millimetre encoder/servo error at a joint boundary.

    Model limits, IK goals and every generated waypoint stay strictly bounded.
    Keep measured feedback unchanged for execution's start/route checks.
    """
    q = np.asarray(measured, dtype=float)
    bounds = np.asarray(bounds, dtype=float)
    if q.shape != (4,) or bounds.shape != (4, 2) or not np.isfinite(q).all():
        raise RuntimeError("invalid measured planning start or joint bounds")
    projected = np.clip(q, bounds[:, 0], bounds[:, 1])
    # Linear: 0.20 mm; rotary: 0.001 rad (~0.057 degrees).
    tolerance = np.array([0.0002, 0.001, 0.001, 0.001])
    excess = np.abs(q-projected)
    for i, name in enumerate(("Linear", "Roll", "Bend1", "Bend2")):
        if excess[i] > tolerance[i] + 1e-12:
            unit, scale = ("mm", 1000.) if i == 0 else ("rad", 1.)
            raise RuntimeError(f"Measured start {name}={q[i]*scale:.6f} {unit} outside "
                               f"[{bounds[i,0]*scale:.6f}, {bounds[i,1]*scale:.6f}]; "
                               f"excess {excess[i]*scale:.6f} {unit} exceeds "
                               f"boundary tolerance {tolerance[i]*scale:.6f} {unit}")
    return projected


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
        self._cuda_process = None
        self._cuda_connection = None
        self._cuda_terminating = False
        self._cuda_process_lock = threading.Lock()
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
        self._execution_started_at = None
        self._wait_started_at = None
        self._settle_stable_samples = 0
        self._tracking = None

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
            self._refresh_cuda_process_state()
            if self._cuda_terminating:
                blocked = "CUDA worker has not exited after cancellation; restart Pilot before planning."
            elif self._worker is not None and self._worker.is_alive():
                blocked = "Previous CUDA task is still stopping; wait before planning again."
            elif self._busy():
                blocked = "Another arm workflow is active. Stop it before planning."
        try:
            # Reporting the measured end-effector pose is a read-only FK
            # operation. It needs fresh joint feedback and, for a mounted
            # arm, the measured base pose, but it must not inherit planning
            # interlocks such as open claws or a stationary GO2.
            host = self._measured_state()
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
                    preview=self.preview(),
                    tracking=self._tracking,
                    execution_elapsed_s=(0. if self._execution_started_at is None else
                        max(0., self.clock()-self._execution_started_at)) if self.active else 0.,
                    target_xyz=getattr(self, "_target", None),
                    planned_xyz=getattr(self, "_planned_xyz", None),
                    current_xyz=self._current_xyz,
                    goal_q=None if plan is None else plan[0][-1].tolist(),
                    world="unconfigured" if self.world_path is None else (
                        "20 mm target + self collision; no other world obstacles"
                        if getattr(self, "_empty_world", False)
                        else "static file + 20 mm target; no live Genesis sync"))

    def preview(self):
        if (self._phase != "ready" or not self._plan_id or self._plan is None
                or self.clock()-self._prepared_at > 120):
            return None
        positions, dt = self._plan
        # Bounded visual samples; the full execution path remains private to Pilot.
        indices = np.unique(np.linspace(0, len(positions)-1, min(96, len(positions))).astype(int))
        return {"plan_id": self._plan_id,
                "times_s": (indices*dt).tolist(),
                "positions": positions[indices].tolist(), "base": self._base.tolist()}

    def route(self):
        c = self.connection
        if not c.active_target:
            raise RuntimeError("Robot is not connected. Start Robot and select its endpoint.")
        d = next((v for v in c.endpoints if v.get("endpoint_id") == c.active_target), {})
        if d.get("role") != "robot" or not d.get("instance_id") or not c.lease_id:
            raise RuntimeError("trajectory requires an exact live Robot boot and motion lease")
        return c.active_target, d["instance_id"], c.lease_id

    def _measured_state(self):
        s = self.connection.state_sink.get_state()
        received = self.connection.telemetry_received_at
        if (not s.connected or received is None or self.clock()-received > 0.35
                or s.q is None or not np.isfinite(q_values(s.q)).all()):
            raise RuntimeError("fresh measured joint telemetry required (<=350 ms)")
        return s

    def _feedback(self, *, torque=False):
        """Return fresh measured state only when arm-only planning is valid."""
        s = self._measured_state()
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
        if getattr(self.service, "_go2_arm_mount", None) is not None:
            base_pos = getattr(s, "go2_base_pos", None)
            base_rpy = getattr(s, "go2_base_rpy", None)
            if base_pos is None or base_rpy is None:
                raise RuntimeError(
                    "measured GO2 body position and orientation are required for arm planning"
                )
            leg_q = getattr(s, "go2_leg_q", None)
            if leg_q is None or len(leg_q) != 12:
                raise RuntimeError("measured 12-joint GO2 state is required for arm planning")
        for key, label, limit, unit in (
            ("go2_base_lin_vel_body", "linear", _GO2_STATIONARY_LINEAR_SPEED_MPS, "m/s"),
            ("go2_base_ang_vel", "angular", _GO2_STATIONARY_ANGULAR_SPEED_RADPS, "rad/s"),
        ):
            values = getattr(s, key, None)
            if values is not None:
                speed = float(np.linalg.norm(values))
                if speed > limit:
                    raise RuntimeError(
                        f"measured GO2 base {label} speed {speed:.4f} {unit} "
                        f"exceeds stationary limit {limit:.4f} {unit}"
                    )
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
        measured = q_values(host.q)
        limit = context["limit"]
        expected_bounds = [linear_effective_q_bounds(self.connection.mapping),
                           (limit.roll_min_rad(), limit.roll_max_rad()),
                           (-limit.bend_lim_rad(), limit.bend_lim_rad()),
                           (-limit.bend_lim_rad(), limit.bend_lim_rad())]
        start = measured_planning_start(measured, expected_bounds)
        self._target, self._start, self._base = target.tolist(), measured.copy(), base
        model = _ReachModel(context=context, limit=context["limit"])
        self._current_xyz = model.grasp_position(start).tolist()
        self._route, self._digest_at_plan = route, digest
        self._generation += 1
        generation = self._generation
        self._phase, self._reason, self._plan, self._plan_id = "planning", "", None, ""
        self._planned_xyz = None
        self._index = 0
        self._tracking = None
        self._execution_started_at = None
        self.connection.clear_pending_motion()

        def thread_plan():
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
                goal, path, goal_local = planner.plan_to_target(
                    start, local, progress=lambda stage: self._progress(stage, generation))
                goal, goal_local = np.asarray(goal, dtype=float), np.asarray(goal_local, dtype=float)
                if (goal_local.shape != (3,) or not np.isfinite(goal_local).all()
                        or np.linalg.norm(goal_local-local) > 1e-6):
                    raise RuntimeError("cuRobo goal differs from the target object centre")
                goal_world = base[:3, :3] @ goal_local + base[:3, 3]
                if np.linalg.norm(model.error_vec(goal, target)) > 0.001:
                    raise RuntimeError("cuRobo/Pilot grasp goal FK mismatch")
                self._progress("Validating path and grasp FK", generation)
                positions, dt = np.asarray(path.positions, dtype=float), float(path.dt_s)
                if (positions.ndim != 2 or positions.shape[1] != 4 or not 2 <= len(positions) <= 4000
                        or not np.isfinite(positions).all() or not np.isfinite(dt) or not 0.03 <= dt <= 0.1
                        or (len(positions)-1)*dt > 120):
                    raise RuntimeError("invalid or oversized timed trajectory")
                # Re-sample to <=25 ms so full Robot speed caps do not produce
                # one-step jumps larger than the measured-following window.
                subdivisions = max(1, int(np.ceil(dt / 0.025)))
                if subdivisions > 1:
                    dense = [positions[0]]
                    for left, right in zip(positions[:-1], positions[1:]):
                        dense.extend(left + (right-left) * (step/subdivisions)
                                     for step in range(1, subdivisions+1))
                    positions = np.asarray(dense, dtype=float)
                    dt /= subdivisions
                if len(positions) > 5000:
                    raise RuntimeError("resampled trajectory exceeds 5000 waypoints")
                # Recheck collision and limits at every interpolated sample.
                planner.validate_path(positions, target_sphere_center=local)
                # Repeat endpoint parity before making a live execution available.
                if not np.allclose(positions[0], start, atol=1e-4, rtol=0):
                    raise RuntimeError("trajectory start differs from measured start")
                for q in (start, positions[-1]):
                    gpu_world = base[:3, :3] @ np.asarray(planner.forward(q)["position"])+base[:3, 3]
                    if np.linalg.norm(gpu_world-model.grasp_position(q)) > 1e-4:
                        raise RuntimeError("cuRobo/Pilot FK calibration mismatch")
                if np.linalg.norm(model.error_vec(positions[-1], target)) > 0.001:
                    raise RuntimeError("trajectory endpoint does not place the target between the claws")
                result = (positions, dt, goal_world.tolist())
            except Exception as exc:
                result = str(exc)
            return result

        def process_plan():
            process, channel = self._ensure_curobo_process()
            channel.send({
                "type": "plan", "digest": digest,
                "expected_bounds": expected_bounds,
                "start": start.tolist(), "target": target.tolist(),
                "base": base.tolist(),
            })
            stage = "Starting CUDA worker"
            stage_started = time.monotonic()
            while generation == self._generation:
                if channel.poll(0.2):
                    message = channel.recv()
                    if message.get("type") == "stage":
                        stage = str(message["stage"])
                        stage_started = time.monotonic()
                        self._progress(stage, generation)
                        continue
                    if message.get("type") != "result":
                        raise RuntimeError("invalid response from isolated cuRobo worker")
                    if not message.get("ok"):
                        return str(message.get("error", "cuRobo planning failed"))
                    positions = np.asarray(message["positions"], dtype=float)
                    dt = float(message["dt"])
                    goal = np.asarray(message["goal"], dtype=float)
                    goal_local = np.asarray(message.get("goal_xyz"), dtype=float)
                    if (goal_local.shape != (3,) or not np.isfinite(goal_local).all()
                            or np.linalg.norm(base[:3, :3] @ goal_local + base[:3, 3] - target) > 1e-6):
                        raise RuntimeError("cuRobo worker omitted its grasp goal")
                    goal_world = base[:3, :3] @ goal_local + base[:3, 3]
                    if np.linalg.norm(model.error_vec(goal, target)) > 0.001:
                        raise RuntimeError("cuRobo/Pilot grasp goal FK mismatch")
                    gpu_fk = message.get("fk")
                    if not isinstance(gpu_fk, list) or len(gpu_fk) != 2:
                        raise RuntimeError("cuRobo worker omitted FK parity checks")
                    for q, fk_position in zip((start, positions[-1]), gpu_fk):
                        gpu_world = base[:3, :3] @ np.asarray(fk_position, dtype=float)+base[:3, 3]
                        if np.linalg.norm(gpu_world-model.grasp_position(q)) > 1e-4:
                            raise RuntimeError("cuRobo/Pilot FK calibration mismatch")
                    if np.linalg.norm(model.error_vec(positions[-1], target)) > 0.001:
                        raise RuntimeError("trajectory endpoint does not place the target between the claws")
                    self._empty_world = bool(message.get("empty_world"))
                    return positions, dt, goal_world.tolist()

                if not process.is_alive():
                    raise RuntimeError(
                        f"cuRobo worker exited during {stage} (exit code {process.exitcode})"
                    )
                timeout = _CUROBO_STAGE_TIMEOUTS_S.get(stage, 180.0)
                elapsed = time.monotonic()-stage_started
                if elapsed > timeout:
                    self._terminate_curobo_process(expected=process)
                    return (f"cuRobo timed out during {stage} after {timeout:.0f} s; "
                            "the CUDA worker was terminated. Plan again to start a fresh worker.")
            return "cuRobo plan cancelled"

        def worker():
            try:
                result = thread_plan() if self.planner_factory is not None else process_plan()
            except Exception as exc:
                result = str(exc)
            with self._lock:
                if generation == self._generation:
                    self._result = generation, result

        self._worker = threading.Thread(target=worker, name="curobo-plan", daemon=True)
        self._worker.start()
        return self.status()

    def _ensure_curobo_process(self):
        with self._cuda_process_lock:
            process, channel = self._cuda_process, self._cuda_connection
            if self._cuda_terminating:
                if process is not None and process.is_alive():
                    raise RuntimeError(
                        "CUDA worker has not exited after cancellation; restart Pilot before planning"
                    )
                self._close_curobo_process_locked()
                process, channel = None, None
            if process is not None and process.is_alive() and channel is not None:
                return process, channel
            self._close_curobo_process_locked()
            parent, child = multiprocessing.get_context("spawn").Pipe(duplex=True)
            process = multiprocessing.get_context("spawn").Process(
                target=_curobo_worker_process,
                args=(child, str(self.model_dir), str(self.world_path)),
                name="elesim-curobo-cuda",
                daemon=True,
            )
            try:
                process.start()
            except Exception:
                parent.close()
                child.close()
                raise
            child.close()
            self._cuda_process, self._cuda_connection = process, parent
            self._cuda_terminating = False
            return process, parent

    def _close_curobo_process_locked(self):
        channel, process = self._cuda_connection, self._cuda_process
        self._cuda_connection = self._cuda_process = None
        self._cuda_terminating = False
        if channel is not None:
            try:
                channel.close()
            except OSError:
                pass
        if process is not None:
            try:
                process.close()
            except (OSError, ValueError):
                pass

    def _refresh_cuda_process_state(self):
        with self._cuda_process_lock:
            if (self._cuda_terminating and self._cuda_process is not None
                    and not self._cuda_process.is_alive()):
                self._close_curobo_process_locked()

    def _terminate_curobo_process(self, *, expected=None):
        with self._cuda_process_lock:
            process = self._cuda_process
            channel = self._cuda_connection
            if process is None or (expected is not None and process is not expected):
                return
            if channel is not None:
                try:
                    channel.close()
                except OSError:
                    pass
            if process.is_alive():
                process.terminate()
                process.join(timeout=0.5)
                if process.is_alive():
                    process.kill()
                    process.join(timeout=0.5)
            if process.is_alive():
                self._cuda_connection = None
                self._cuda_terminating = True
            else:
                self._close_curobo_process_locked()

    def execute_arm_trajectory(self, plan_id):
        if self._phase != "ready" or not plan_id or plan_id != self._plan_id:
            raise RuntimeError("no matching unconsumed prepared trajectory")
        host = self._feedback(torque=True)
        if (self.route() != self._route or self._digest() != self._digest_at_plan
                or self.clock()-self._prepared_at > 120 or self._busy()):
            raise RuntimeError("plan expired, route/model/world changed or arm workflow busy; replan")
        require_stable_arm_base(
            self._base, base_world_transform_from_context(self._context(host)))
        if np.any(np.abs(q_values(host.q)-self._start) > [0.001, 0.015, 0.015, 0.015]):
            raise RuntimeError("measured start moved; replan from the current posture")
        current_target = [self.state.target_x, self.state.target_y, self.state.target_z]
        if not np.allclose(current_target, self._target, atol=1e-7, rtol=0):
            raise RuntimeError("Target XYZ changed; replan")
        self.connection.clear_pending_motion()
        self._phase, self._index, self._last = "executing", 0, self._start.copy()
        self._stage = "Streaming timed path"
        self._next_at = self.clock()
        self._execution_started_at = self._next_at
        self._wait_started_at = None
        self._settle_stable_samples = 0
        self._tracking = None
        return self.status()

    def cancel_arm_trajectory(self):
        return self._cancel("operator cancelled")

    def request_cancel(self):
        """Lifecycle thread requests; DDS thread owns lease release."""
        self._cancel_requested.set()
        self._terminate_curobo_process()

    def _cancel(self, reason, *, release=True):
        moving = self._phase in {"executing", "settling"}
        planning = self._phase == "planning"
        self._generation += 1
        self._phase, self._reason, self._plan_id = "cancelled", str(reason), ""
        self._planned_xyz = None
        self._stage, self._finished_at = "cancelled", self.clock()
        self.connection.clear_pending_motion()
        if planning:
            self._terminate_curobo_process()
            # The isolated worker has been killed and its channel closed, so
            # do not make the operator wait for the monitor thread's next poll.
            if self.planner_factory is None:
                self._worker = None
        if moving and release:
            # Releasing the exact lease invokes Robot's local measured safe hold.
            # Do not replace a stale/lost target with a newly selected target.
            self.connection.release_trajectory_route(self._route)
        return self.status()

    def guard_operator(self, operation, name):
        if self.active and operation != "view_snapshot":
            if operation == "service_call" and name in {
                "cancel_arm_trajectory", "torque_off", "clear_safety_fault",
                "reboot_faulted_motor",
            }:
                self._cancel(
                    name,
                    release=name not in {
                        "torque_off", "clear_safety_fault", "reboot_faulted_motor",
                    },
                )
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
                print(f"[curobo-plan] failed: {self._reason}", flush=True)
            else:
                if len(result[1]) == 3:
                    positions, dt, self._planned_xyz = result[1]
                    self._plan = positions, dt
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
            require_stable_arm_base(
                self._base, base_world_transform_from_context(self._context(host)))
            measured = q_values(host.q)
            error = np.abs(measured-self._last)
            self._tracking = {"measured_q":measured.tolist(), "commanded_q":self._last.tolist(),
                              "error":error.tolist(), "waiting":False}
            limits = np.array([0.008, 0.12, 0.12, 0.12])
            positions, dt = self._plan
            self._tracking["goal_error"] = np.abs(measured-positions[-1]).tolist()
            if np.any(error > limits):
                i = int(np.argmax(error/limits))
                axis = ("Linear", "Roll", "Bend1", "Bend2")[i]
                scale, unit = (1000., "mm") if i == 0 else (1., "rad")
                raise RuntimeError(f"measured tracking error: {axis} {error[i]*scale:.6f} {unit} "
                                   f"exceeds {limits[i]*scale:.6f}; measured={measured.tolist()}, "
                                   f"commanded={self._last.tolist()}")
            if now < self._next_at:
                return
            lateness = max(0.0, now-self._next_at)
            if lateness > _TRAJECTORY_MAX_TICK_STALL_S:
                raise RuntimeError(
                    f"Pilot control loop stalled for {lateness*1000:.1f} ms "
                    f"(limit {_TRAJECTORY_MAX_TICK_STALL_S*1000:.0f} ms); "
                    "path stopped without skipping waypoints"
                )
            if lateness > _TRAJECTORY_REBASE_AFTER_S:
                # Preserve the current waypoint and pause the trajectory clock.
                # Sending a burst of stale samples would create a catch-up jump.
                print(
                    f"[curobo-execute] scheduler delayed {lateness*1000:.1f} ms; "
                    "rebasing path clock without skipping a waypoint",
                    flush=True,
                )
                self._next_at = now
            duration = (len(positions)-1)*dt
            # Real feedback-paced hardware can take several times the nominal
            # cuRobo duration. Keep a firm whole-path bound with headroom for
            # measured-following pauses; each individual waypoint still has
            # its separate 2 s follow timeout above.
            if now-self._execution_started_at > max(20., duration*4+8):
                raise RuntimeError("execution exceeded bounded feedback-following timeout")
            advance = True
            if self._phase == "settling":
                if np.all(self._tracking["goal_error"] <= limits):
                    self._settle_stable_samples += 1
                    if self._settle_stable_samples >= _TRAJECTORY_GOAL_SETTLE_STABLE_SAMPLES:
                        self._phase, self._plan_id = "completed", ""
                        self._stage = "Measured goal stable within tracking tolerance"
                        self.state.set_q(*map(float, measured))
                        return
                    self._stage = (
                        "Confirming measured goal "
                        f"({self._settle_stable_samples}/"
                        f"{_TRAJECTORY_GOAL_SETTLE_STABLE_SAMPLES} stable samples)"
                    )
                else:
                    self._settle_stable_samples = 0
                    self._stage = "Settling measured goal within tracking tolerance"
                q = positions[-1]
            else:
                candidate = positions[self._index]
                # Keep headroom under the unchanged hard tracking bound. A
                # delayed servo/telemetry sample must not make the command run
                # farther ahead. Resend the existing goal to maintain deadman.
                if np.any(np.abs(candidate-measured) > limits*.90):
                    if self._wait_started_at is None:
                        self._wait_started_at = now
                    if now-self._wait_started_at > 3.:
                        raise RuntimeError(f"Robot did not follow waypoint {self._index} within 3 seconds; "
                                           f"measured={measured.tolist()}, commanded={self._last.tolist()}")
                    q, advance = self._last.copy(), False
                    self._tracking["waiting"] = True
                    self._stage = "Waiting for measured Robot feedback (path clock paused)"
                else:
                    self._wait_started_at = None
                    self._stage = "Streaming path with measured feedback"
                    q = candidate
                if advance:
                    self._index += 1
                    if self._index == len(positions):
                        self._phase, self._settling_at = "settling", now
            self.connection.send_trajectory_q(q.tolist(), self._route)
            self._last = q.copy()
            self.state.set_q(*map(float, q))
            self._next_at += dt
        except Exception as exc:
            print(f"[curobo-execute] waypoint={self._index} failed: {exc}; tracking={self._tracking}", flush=True)
            self._cancel(str(exc))
            self._phase = "failed"
            self._stage = "failed"
