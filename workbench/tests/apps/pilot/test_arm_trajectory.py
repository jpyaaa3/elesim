from types import SimpleNamespace as NS

import numpy as np
import pytest

from elesim_pilot.arm_trajectory import ArmTrajectory
from elesim_protocol import SimQ


@pytest.fixture
def rig(tmp_path):
    stamp = [10.0]
    sent, released = [], []
    state = NS(target_x=0., target_y=0., target_z=0., ik_running=False,
               set_q=lambda *q: None)
    host = NS(connected=True, q=SimQ(-.1, 0., 0., 0.), torque_enabled=True,
              safety_fault="", reply_ok=True, reply_reason="", claw_closed=False,
              go2_vel=(0., 0., 0.))
    connection = NS(active_target="robot", lease_id="lease", telemetry_received_at=10.,
                    trajectory_telemetry={"imu_model": {"id": "identity", "version": 1}},
                    endpoints=[dict(endpoint_id="robot", role="robot", instance_id="boot")],
                    state_sink=NS(get_state=lambda: host), clear_pending_motion=lambda: None,
                    send_trajectory_q=lambda q, route: sent.append((q, route)),
                    release_trajectory_route=lambda route: released.append(route))
    service = NS(_visual_busy=lambda: False, _pick_busy=lambda: False, _gaze_busy=lambda: False,
                 pick_e2e_running=lambda: False, wrap_grasp_running=lambda: False,
                 sim_reset_in_progress=False)
    (tmp_path/"world.yml").write_text("{}")
    for name in ("metadata.json", "arm.urdf", "robot.yml"):
        (tmp_path/name).write_text(name)
    runner = ArmTrajectory(connection, service, state, model_dir=tmp_path,
                           world_path=tmp_path/"world.yml", clock=lambda: stamp[0])
    runner._context = lambda h: {"base_world_T": np.eye(4)}
    runner._base = np.eye(4)
    runner._route = runner.route()
    runner._digest_at_plan = runner._digest()
    runner._prepared_at = 10.
    runner._target = [0., 0., 0.]
    runner._start = np.array([-.1, 0., 0., 0.])
    runner._plan = (np.array([[-.1, 0., 0., 0.], [-.0998, 0., 0., 0.],
                              [-.0996, 0., 0., 0.]]), .05)
    runner._phase, runner._plan_id = "ready", "plan"
    return NS(r=runner, c=connection, h=host, s=state, time=stamp, sent=sent, released=released,
              tmp=tmp_path)


def test_plan_is_not_motion_and_execute_consumes_once(rig):
    r = rig.r
    r.tick()
    assert not rig.sent
    r.execute_arm_trajectory("plan")
    r.tick()
    rig.time[0] += .025
    r.tick()
    assert len(rig.sent) == 1
    for _ in range(2):
        rig.time[0] = r._next_at
        rig.c.telemetry_received_at = rig.time[0]
        r.tick()
    assert r.status()["phase"] == "settling"
    rig.h.q = SimQ(-.0996, 0., 0., 0.)
    for _ in range(3):
        rig.time[0] = r._next_at
        rig.c.telemetry_received_at = rig.time[0]
        r.tick()
    assert r.status()["phase"] == "completed"
    assert len(rig.sent) == 5
    with pytest.raises(RuntimeError, match="unconsumed"):
        r.execute_arm_trajectory("plan")


def test_final_goal_accepts_stable_feedback_inside_path_tracking_limit(rig):
    r = rig.r
    r.execute_arm_trajectory("plan")
    for _ in range(3):
        rig.time[0] = r._next_at
        rig.c.telemetry_received_at = rig.time[0]
        r.tick()
    assert r._phase == "settling"

    # This residual is larger than the old 1 mm / 0.015 rad completion gate,
    # but remains within the 8 mm / 0.12 rad tracking bound used along the path.
    rig.h.q = SimQ(-.0936, .10, 0., 0.)
    for _ in range(3):
        rig.time[0] = r._next_at
        rig.c.telemetry_received_at = rig.time[0]
        r.tick()
    assert r._phase == "completed"
    np.testing.assert_allclose(r._tracking["goal_error"], [.006, .10, 0., 0.])


def test_execute_tolerates_measured_go2_pose_jitter(rig):
    # Jetson stationary samples show sub-mm / 0.1 degree base pose variation.
    current = np.eye(4)
    angle = np.deg2rad(0.1)
    current[:3, :3] = [[np.cos(angle), -np.sin(angle), 0.],
                       [np.sin(angle), np.cos(angle), 0.],
                       [0., 0., 1.]]
    current[:3, 3] = [0.001, 0., 0.]
    rig.r._context = lambda _host: {"base_world_T": current}

    assert rig.r.execute_arm_trajectory("plan")["phase"] == "executing"
    assert not rig.sent  # Execute acceptance itself does not send a waypoint.


@pytest.mark.parametrize("translation_m,angle_deg", [
    (0.003, 0.0),
    (0.0, 0.5),
])
def test_execute_still_rejects_meaningful_arm_base_motion(rig, translation_m, angle_deg):
    current = np.eye(4)
    angle = np.deg2rad(angle_deg)
    current[:3, :3] = [[np.cos(angle), -np.sin(angle), 0.],
                       [np.sin(angle), np.cos(angle), 0.],
                       [0., 0., 1.]]
    current[0, 3] = translation_m
    rig.r._context = lambda _host: {"base_world_T": current}

    with pytest.raises(RuntimeError, match=r"arm base moved; replan.*translation .*rotation"):
        rig.r.execute_arm_trajectory("plan")
    assert not rig.sent


def test_execution_cancels_if_arm_base_moves_beyond_jitter_window(rig):
    rig.r.execute_arm_trajectory("plan")
    current = np.eye(4)
    current[0, 3] = 0.003
    rig.r._context = lambda _host: {"base_world_T": current}

    rig.r.tick()

    assert rig.r.status()["phase"] == "failed"
    assert "arm base moved; replan" in rig.r.status()["reason"]
    assert "3.00 mm / 2.00 mm" in rig.r.status()["reason"]
    assert not rig.sent
    assert rig.released == [("robot", "boot", "lease")]


@pytest.mark.parametrize("change,reason", [
    (lambda x: setattr(x.h, "torque_enabled", False), "Torque"),
    (lambda x: setattr(x.h, "claw_closed", True), "open claws"),
    (lambda x: setattr(x.h, "q", SimQ(-.11, 0., 0., 0.)), "start moved"),
    (lambda x: setattr(x.c, "telemetry_received_at", 9.), "fresh measured"),
    (lambda x: setattr(x.c, "lease_id", "new-lease"), "route/model/world"),
    (lambda x: x.c.endpoints[0].update(instance_id="new-boot"), "route/model/world"),
    (lambda x: (x.tmp/"world.yml").write_text("cuboid: {}"), "route/model/world"),
    (lambda x: setattr(x.s, "target_x", .1), "XYZ changed"),
    (lambda x: setattr(x.h, "safety_fault", "overcurrent"), "overcurrent"),
])
def test_execution_rechecks_prepared_state_without_sending(rig, change, reason):
    change(rig)
    with pytest.raises(RuntimeError, match=reason):
        rig.r.execute_arm_trajectory("plan")
    assert not rig.sent


def test_go2_arm_planning_requires_measured_go2_pose_and_leg_state(rig):
    rig.r.service._go2_arm_mount = object()
    with pytest.raises(RuntimeError, match="body position and orientation"):
        rig.r._feedback()

    rig.h.go2_base_pos = (0.0, 0.0, 0.4)
    rig.h.go2_base_rpy = (0.0, 0.0, 0.0)
    with pytest.raises(RuntimeError, match="12-joint GO2 state"):
        rig.r._feedback()

    rig.h.go2_leg_q = (0.0,) * 12
    assert rig.r._feedback() is rig.h


def test_go2_stationary_gate_tolerates_imu_bias_but_rejects_motion(rig):
    rig.r.service._go2_arm_mount = object()
    rig.h.go2_base_pos = (0.0, 0.0, 0.4)
    rig.h.go2_base_rpy = (0.0, 0.0, 0.0)
    rig.h.go2_leg_q = (0.0,) * 12
    rig.h.go2_base_lin_vel_body = (1e-7, -2e-7, 1e-7)
    # This is the gyro noise measured while the real GO2 is standing still.
    rig.h.go2_base_ang_vel = (-0.018, 0.021, -0.014)
    assert rig.r._feedback() is rig.h

    rig.h.go2_base_ang_vel = (0.0, 0.0, 0.051)
    with pytest.raises(RuntimeError, match=r"angular speed 0\.0510 rad/s.*limit 0\.0500"):
        rig.r._feedback()

    rig.h.go2_base_ang_vel = (0.0, 0.0, 0.0)
    rig.h.go2_base_lin_vel_body = (0.011, 0.0, 0.0)
    with pytest.raises(RuntimeError, match=r"linear speed 0\.0110 m/s.*limit 0\.0100"):
        rig.r._feedback()


def test_status_reports_current_xyz_while_go2_is_moving(monkeypatch, rig):
    class ReachModel:
        def __init__(self, *, context, limit):
            assert context["base_world_T"] is not None

        def grasp_position(self, q):
            return np.array([0.12, -0.01, 0.50])

    monkeypatch.setattr("elesim_pilot.arm_trajectory._ReachModel", ReachModel)
    rig.r._context = lambda _host: {"base_world_T": np.eye(4), "limit": object()}
    rig.r.service._go2_arm_mount = object()
    rig.h.go2_base_pos = (0.0, 0.0, 0.4)
    rig.h.go2_base_rpy = (0.0, 0.0, 0.0)
    rig.h.go2_leg_q = (0.0,) * 12
    rig.h.go2_base_lin_vel_body = (0.02, 0.0, 0.0)
    rig.h.go2_base_ang_vel = (0.0, 0.0, 0.0)

    status = rig.r.status()

    assert "linear speed 0.0200 m/s" in status["blocked_reason"]
    assert status["can_plan"] is False
    assert status["current_xyz"] == [0.12, -0.01, 0.50]


@pytest.mark.parametrize("change,reason", [
    (lambda x: (x.time.__setitem__(0, 10.4), setattr(x.c, "telemetry_received_at", 10.4)),
     "control loop stalled"),
    (lambda x: setattr(x.h, "q", SimQ(-.11, 0., 0., 0.)), "tracking error"),
    (lambda x: setattr(x.c, "telemetry_received_at", 9.), "fresh measured"),
    (lambda x: setattr(x.h, "torque_enabled", False), "Torque"),
    (lambda x: setattr(x.c, "lease_id", "new"), "boot/lease"),
])
def test_execution_cancels_on_feedback_route_and_deadline_failure(rig, change, reason):
    rig.r.execute_arm_trajectory("plan")
    change(rig)
    rig.r.tick()
    assert rig.r.status()["phase"] == "failed"
    assert reason in rig.r.status()["reason"]
    assert not rig.sent
    assert rig.released == [("robot", "boot", "lease")]


def test_short_scheduler_delay_rebases_clock_without_skipping_waypoints(rig):
    rig.r.execute_arm_trajectory("plan")
    rig.time[0] += 0.15
    rig.c.telemetry_received_at = rig.time[0]

    rig.r.tick()

    assert rig.r.status()["phase"] == "executing"
    assert rig.r._index == 1
    assert rig.r._next_at == pytest.approx(rig.time[0] + 0.05)
    assert len(rig.sent) == 1
    assert not rig.released


def test_operator_interlock_preserves_stop_without_torque_on(rig):
    rig.r.execute_arm_trajectory("plan")
    for name in ("home_controls", "apply_partial_control_u", "torque_on", "select_endpoint"):
        with pytest.raises(RuntimeError, match="workflow active"):
            rig.r.guard_operator("service_call", name)
    rig.r.guard_operator("view_snapshot", "")
    assert not rig.r.allow_motion({"command": "target", "q": [0]*4})
    # Torque OFF must use the existing lease, not a lease revoked before delivery.
    rig.r.guard_operator("service_call", "torque_off")
    assert not rig.released
    assert rig.r.allow_motion({"command": "torque_off"})
    assert rig.r.status()["phase"] == "cancelled"


def test_motor_recovery_cancels_trajectory_without_releasing_robot_lease(rig):
    rig.r.execute_arm_trajectory("plan")

    rig.r.guard_operator("service_call", "reboot_faulted_motor")

    assert rig.r.status()["phase"] == "cancelled"
    assert rig.released == []


def test_cancel_discards_late_cuda_result(rig):
    rig.r._phase = "planning"
    generation = rig.r._generation
    rig.r.cancel_arm_trajectory()
    rig.r._result = generation, rig.r._plan
    rig.r.tick()
    assert rig.r.status()["phase"] == "cancelled"
    assert not rig.sent


def test_cancel_terminates_cuda_process_and_unblocks_next_plan(rig):
    class Process:
        exitcode = -15

        def __init__(self):
            self.alive = True
            self.terminated = False

        def is_alive(self):
            return self.alive

        def terminate(self):
            self.terminated = True
            self.alive = False

        def join(self, timeout=None):
            pass

        def close(self):
            pass

    class Channel:
        closed = False

        def close(self):
            self.closed = True

    process, channel = Process(), Channel()
    rig.r._phase = "planning"
    rig.r._cuda_process, rig.r._cuda_connection = process, channel

    status = rig.r.cancel_arm_trajectory()

    assert process.terminated
    assert channel.closed
    assert status["phase"] == "cancelled"
    assert status["can_plan"] is True


def test_cancel_keeps_planning_blocked_if_cuda_process_will_not_exit(rig):
    class StuckProcess:
        exitcode = None

        def is_alive(self):
            return True

        def terminate(self):
            pass

        def kill(self):
            pass

        def join(self, timeout=None):
            pass

    class Channel:
        def close(self):
            pass

    rig.r._phase = "planning"
    rig.r._cuda_process = StuckProcess()
    rig.r._cuda_connection = Channel()

    status = rig.r.cancel_arm_trajectory()

    assert status["can_plan"] is False
    assert "restart Pilot" in status["blocked_reason"]


def test_next_plan_replaces_cuda_process_after_late_termination(monkeypatch, rig):
    class Process:
        exitcode = None

        def __init__(self, alive):
            self.alive = alive

        def is_alive(self):
            return self.alive

        def start(self):
            self.alive = True

        def close(self):
            self.alive = False

    class Channel:
        def close(self):
            pass

    class Context:
        def __init__(self):
            self.created = Process(False)

        def Pipe(self, duplex):
            assert duplex
            return Channel(), Channel()

        def Process(self, **kwargs):
            assert kwargs["target"].__name__ == "_curobo_worker_process"
            return self.created

    old = Process(False)
    rig.r._cuda_process, rig.r._cuda_connection = old, Channel()
    rig.r._cuda_terminating = True
    context = Context()
    monkeypatch.setattr("elesim_pilot.arm_trajectory.multiprocessing.get_context",
                        lambda _name: context)

    process, channel = rig.r._ensure_curobo_process()

    assert process is context.created
    assert process.is_alive()
    assert channel is rig.r._cuda_connection
    assert not rig.r._cuda_terminating


def test_spawned_curobo_worker_reports_errors_and_accepts_shutdown(tmp_path):
    import multiprocessing
    from elesim_pilot.arm_trajectory import _curobo_worker_process

    world = tmp_path / "world.yml"
    world.write_text("{}")
    context = multiprocessing.get_context("spawn")
    parent, child = context.Pipe(duplex=True)
    process = context.Process(
        target=_curobo_worker_process,
        args=(child, str(tmp_path / "missing-model"), str(world)),
        daemon=True,
    )
    process.start()
    child.close()
    try:
        parent.send({
            "type": "plan", "digest": ["fixture"],
            "expected_bounds": [[-0.2, 0.0], [-1.5, 1.5], [-0.6, 0.6], [-0.6, 0.6]],
            "start": [-0.1, 0.0, 0.0, 0.0],
            "target": [0.0, 0.0, 0.0], "base": np.eye(4).tolist(),
        })
        assert parent.poll(10.)
        assert parent.recv() == {"type": "stage", "stage": "Loading CUDA and collision model"}
        assert parent.poll(10.)
        result = parent.recv()
        assert result["type"] == "result" and not result["ok"]
        parent.send({"type": "stop"})
        process.join(5.)
        assert not process.is_alive()
        assert process.exitcode == 0
    finally:
        parent.close()
        if process.is_alive():
            process.terminate()
            process.join(2.)
        process.close()


@pytest.mark.parametrize("failure", [None, "standoff", "endpoint"])
def test_cuda_worker_returns_object_center_goal_and_rejects_partial_approach(
    monkeypatch, tmp_path, failure,
):
    from elesim_pilot.arm_trajectory import _curobo_worker_process
    bounds = [[-1., 1.]] * 4
    base = np.eye(4)
    base[:3, :3] = [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]
    base[:3, 3] = [.1, .2, .3]
    target = base[:3, :3] @ [.2, 0., 0.] + base[:3, 3]
    checked = []

    class Planner:
        def __init__(self, *_args, **_kwargs):
            self.bounds = np.asarray(bounds)

        def plan_to_target(self, start, local, **_kwargs):
            np.testing.assert_allclose(local, [.2, 0., 0.], atol=1e-8)
            goal = [.2, 0., 0., 0.]
            endpoint = [.17, 0., 0., 0.] if failure == "endpoint" else goal
            reported = [.17, 0., 0.] if failure == "standoff" else local
            return goal, NS(positions=[start, endpoint], dt_s=.05), reported

        def validate_path(self, points, *, target_sphere_center):
            checked.append(len(points))
            np.testing.assert_allclose(target_sphere_center, [.2, 0., 0.], atol=1e-8)

        def forward(self, q):
            return {"position": [q[0], 0., 0.]}

    monkeypatch.setattr("elesim_pilot.robot.arm.curobo.CuRoboPlanner", Planner)
    requests = iter([dict(type="plan", digest=["test"], expected_bounds=bounds,
                          start=[0.]*4, target=target.tolist(), base=base.tolist()),
                     dict(type="stop")])
    sent = []
    world = tmp_path / "world.yml"
    world.write_text("{}")
    _curobo_worker_process(NS(recv=lambda: next(requests), send=sent.append), str(tmp_path), str(world))
    result = sent[-1]
    if failure is None:
        assert result["ok"]
        np.testing.assert_allclose(result["goal_xyz"], [.2, 0., 0.], atol=1e-8)
        assert checked == [3]  # the inserted execution sample is checked too
    else:
        assert not result["ok"]
        assert ("object centre" if failure == "standoff" else "between the claws") in result["error"]


def test_cancel_releases_original_route(rig):
    rig.r.execute_arm_trajectory("plan")
    rig.r.tick()
    rig.r.cancel_arm_trajectory()
    rig.time[0] += .05
    rig.r.tick()
    assert len(rig.sent) == 1
    assert rig.released == [("robot", "boot", "lease")]


def test_preflight_failure_is_published_instead_of_silent_idle(rig):
    rig.c.active_target = ""
    with pytest.raises(RuntimeError):
        rig.r.plan_arm_to_target()
    status = rig.r.status()
    assert status["phase"] == "failed"
    assert "fresh measured" in status["reason"] or "not connected" in status["reason"]
    assert "not connected" in status["blocked_reason"]
    assert not status["can_plan"]
    assert not rig.sent


def test_progress_elapsed_time_and_cancel_fence(rig):
    rig.r._phase = "planning"
    rig.r._started_at = rig.time[0]
    generation = rig.r._generation
    rig.r._progress("Solving position IK", generation)
    rig.time[0] += 2
    assert rig.r.status()["stage"] == "Solving position IK"
    assert rig.r.status()["elapsed_s"] == 2
    rig.r.cancel_arm_trajectory()
    rig.time[0] += 10
    rig.r._progress("Old worker completed", generation)
    assert rig.r.status()["stage"] == "cancelled"
    assert rig.r.status()["elapsed_s"] == 2


def test_measured_linear_endpoint_noise_does_not_reject_reachable_plan():
    from elesim_pilot.arm_trajectory import measured_planning_start
    bounds = np.array([[-.142,0],[-np.pi/2,np.pi/2],[-.6284,.6284],[-.6284,.6284]])
    measured = np.array([-.14212454212454212, .00076699, -.5936421, .59272149])
    start = measured_planning_start(measured, bounds)
    assert start[0] == -.142
    assert measured[0] == -.14212454212454212
    np.testing.assert_array_equal(start[1:], measured[1:])
    np.testing.assert_array_equal(measured_planning_start(start, bounds), start)


@pytest.mark.parametrize("axis,value,name", [(0,-.142201,"Linear"), (0,.000201,"Linear"),
                                            (1,np.pi/2+.00101,"Roll"),
                                            (2,-.62941,"Bend1"), (3,.62941,"Bend2")])
def test_real_start_limit_violations_still_fail_with_axis_and_excess(axis,value,name):
    from elesim_pilot.arm_trajectory import measured_planning_start
    bounds = [[-.142,0],[-np.pi/2,np.pi/2],[-.6284,.6284],[-.6284,.6284]]
    q = [-.1,0,0,0]; q[axis] = value
    with pytest.raises(RuntimeError, match=f"Measured start {name}=.*excess"):
        measured_planning_start(q,bounds)


def test_ghost_preview_is_ready_only_bounded_and_keeps_original_timing(rig):
    rig.r._plan = (np.linspace([-.1,0,0,0],[-.05,.2,.1,-.1],4000),.03)
    original = rig.r._plan[0].copy()
    preview = rig.r.preview()
    assert 2 <= len(preview['positions']) <= 96
    assert preview['positions'][0] == original[0].tolist()
    assert preview['positions'][-1] == original[-1].tolist()
    assert preview['times_s'][0] == 0
    assert preview['times_s'][-1] == 3999*.03
    np.testing.assert_array_equal(rig.r._plan[0],original)
    assert not rig.sent
    for phase in ('planning','executing','settling','cancelled','failed','completed'):
        rig.r._phase = phase
        assert rig.r.preview() is None
    rig.r._phase='ready';rig.time[0]=131
    assert rig.r.preview() is None


def test_slow_robot_pauses_path_instead_of_running_commands_ahead(rig):
    rig.r._plan=(np.array([[-.1,0,0,0],[-.098,0,0,0],[-.092,0,0,0]]),.05)
    rig.r.execute_arm_trajectory("plan")
    for _ in range(3):
        rig.time[0]=rig.r._next_at
        rig.c.telemetry_received_at=rig.time[0]
        rig.r.tick()
    assert rig.r._index==2
    assert rig.sent[-1][0][0]==-.098  # repeat last goal, not the distant next sample
    assert rig.r.status()['tracking']['waiting']
    rig.h.q=SimQ(-.093,0,0,0)
    rig.time[0]=rig.r._next_at;rig.c.telemetry_received_at=rig.time[0]
    rig.r.tick()
    assert rig.sent[-1][0][0]==-.092
    assert rig.r._phase=='settling'
    assert not rig.released


def test_stationary_robot_follow_wait_is_bounded_and_releases_lease(rig):
    rig.r._plan=(np.array([[-.1,0,0,0],[-.098,0,0,0],[-.092,0,0,0]]),.05)
    rig.r.execute_arm_trajectory("plan")
    for _ in range(70):
        rig.time[0]=rig.r._next_at;rig.c.telemetry_received_at=rig.time[0]
        rig.r.tick()
        if rig.r._phase=='failed':break
    assert rig.r._phase=='failed'
    assert 'within 3 seconds' in rig.r._reason
    assert rig.released
    assert max(q[0] for q,route in rig.sent)<=-.098


def test_tracking_failure_names_axis_and_values_without_relaxing_limit(rig):
    rig.r.execute_arm_trajectory("plan")
    rig.h.q=SimQ(-.109,0,0,0)
    rig.r.tick()
    assert 'Linear' in rig.r._reason
    assert 'measured=' in rig.r._reason and 'commanded=' in rig.r._reason
    assert rig.r._phase=='failed' and not rig.sent
