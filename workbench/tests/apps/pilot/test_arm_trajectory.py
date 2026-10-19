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
    rig.time[0] = r._next_at
    rig.h.q = SimQ(-.0996, 0., 0., 0.)
    r.tick()
    assert r.status()["phase"] == "completed"
    assert len(rig.sent) == 3
    with pytest.raises(RuntimeError, match="unconsumed"):
        r.execute_arm_trajectory("plan")


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


@pytest.mark.parametrize("change,reason", [
    (lambda x: x.time.__setitem__(0, 10.2), "scheduling overrun"),
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


def test_cancel_discards_late_cuda_result(rig):
    rig.r._phase = "planning"
    generation = rig.r._generation
    rig.r.cancel_arm_trajectory()
    rig.r._result = generation, rig.r._plan
    rig.r.tick()
    assert rig.r.status()["phase"] == "cancelled"
    assert not rig.sent


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
