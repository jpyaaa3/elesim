from __future__ import annotations

import numpy as np
import pytest
from types import SimpleNamespace

from elesim_sim.robot.go2.locomotion.kinematics import NOMINAL_FOOT_OFFSET_BODY
from elesim_sim.robot.go2.locomotion.command import Go2CommandShaper, trot_all_stance
from elesim_sim.robot.go2.locomotion.pose import JointPoseTransition, smoothstep_quintic
from elesim_sim.robot.go2.locomotion.types import ALL_LEGS, Go2Command
from elesim_sim.robot.go2.pympc_controller import (
    PyMpcGenesisController,
    ContactGatedGait,
    contact_schedule,
    planned_footholds,
    future_touchdown_times,
    swing_foot_reference,
    touchdown_offsets_body,
)
from elesim_sim.robot.go2.pympc_solver import PyMpcForceSolver, PyMpcInput


@pytest.mark.parametrize('sim_dt,backend,expected', [
    (.02, 'jax_mppi', 1), (.005, 'jax_mppi', 4),
    (.01, 'jax_mppi', 2), (.007, 'jax_mppi', 2),
    (.02, 'acados', 2), (.005, 'acados', 8),
])
def test_gpu_force_refresh_matches_prediction_stage_and_preserves_cpu_cadence(sim_dt, backend, expected):
    from elesim_sim.robot.go2.pympc_controller import solver_update_stride
    assert solver_update_stride(sim_dt, .02, backend=backend) == expected


@pytest.mark.parametrize('sim_dt,prediction_dt', [(0., .02), (.02, float('nan'))])
def test_force_refresh_rejects_invalid_intervals(sim_dt, prediction_dt):
    from elesim_sim.robot.go2.pympc_controller import solver_update_stride
    with pytest.raises(ValueError):
        solver_update_stride(sim_dt, prediction_dt, backend='jax_mppi')


@pytest.mark.parametrize('sim_dt', [.02, .005])
def test_gpu_torque_loop_requests_fresh_force_at_each_prediction_stage(sim_dt):
    from elesim_sim.robot.go2.pympc_controller import solver_update_stride
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_torque_controller(c)
    c._dt = sim_dt
    c._solve_stride = solver_update_stride(sim_dt, .02, backend='jax_mppi')
    ticks = []
    def solve(_sample):
        ticks.append(c._step_i)
        return np.tile([0., 0., 50.], (4, 1))
    c._solver = SimpleNamespace(solve=solve, backend='jax_mppi')
    state = PyMpcInput(**{**vars(sample()), 'contacts':np.ones((4, 3))})
    for c._step_i in range(round(.06/sim_dt)):
        c._torques(state, state.feet_world, np.zeros((4, 3)),
                   np.tile(np.eye(3), (4, 1, 1)), np.zeros(12))
    np.testing.assert_allclose(np.array(ticks)*sim_dt, [0., .02, .04])


def test_planner_holds_swing_touchdown_while_body_and_candidates_move():
    feet = np.arange(12, dtype=float).reshape(4, 3)
    candidates = feet + 10
    latched = feet + 20
    result = planned_footholds(feet, candidates, [1, 0, 0, 1], [0, 1, 0, 1], latched)
    np.testing.assert_array_equal(result[[0, 3]], feet[[0, 3]])
    np.testing.assert_array_equal(result[1], candidates[1])
    np.testing.assert_array_equal(result[2], latched[2])
    later = planned_footholds(feet, candidates + 100, [1, 0, 0, 1], [1, 0, 0, 1], result)
    np.testing.assert_array_equal(later[[1, 2]], result[[1, 2]])
    assert not np.shares_memory(result, feet)


def test_next_stance_touchdown_is_previewed_only_when_it_reenters_horizon():
    schedule = contact_schedule(0., gait_hz=2.5, duty=.6, dt=.02, horizon=12)
    np.testing.assert_allclose(
        future_touchdown_times(schedule, gait_time=0., gait_hz=2.5),
        [0., .2, .2, 0.], atol=1e-9,
    )
    held = np.tile(schedule[:, :1], (1, 12))
    np.testing.assert_array_equal(future_touchdown_times(held, gait_time=0., gait_hz=2.5), 0.)


def test_swing_velocity_matches_position_derivative():
    start, end = np.array([0., .1, .02]), np.array([.15, .2, .025])
    p, duration, epsilon = .4, .18, 1e-6
    target, velocity = swing_foot_reference(p, start, end, duration_s=duration)
    before, _ = swing_foot_reference(p - epsilon, start, end, duration_s=duration)
    after, _ = swing_foot_reference(p + epsilon, start, end, duration_s=duration)
    np.testing.assert_allclose(velocity, (after - before) / (2 * epsilon * duration), atol=1e-8)
    np.testing.assert_allclose(swing_foot_reference(0, start, end, duration_s=duration)[0], start)
    np.testing.assert_allclose(swing_foot_reference(1, start, end, duration_s=duration)[0], end)
    assert target[2] > max(start[2], end[2])


def test_terminal_swing_loads_the_measured_support_plane_without_moving_mpc_foothold():
    # d3b0762 GPU2: terminal FL/RL centers remained at 0.026--0.027 m,
    # normal force 0. Loaded feet settled near 0.023 m. A zero-velocity
    # reference at 0.025 m can leave the foot unloaded above that plane.
    start, end = np.array([0., .17, .023]), np.array([.1, .17, .025])
    position, velocity = swing_foot_reference(
        1., start, end, duration_s=.18, touchdown_depth_m=.008,
    )
    task_force = 1000.*(position-np.array([.1, .17, .023])) + 20.*velocity
    assert task_force[2] < -5.
    np.testing.assert_array_equal(end, [.1, .17, .025])
    np.testing.assert_array_equal(velocity, np.zeros(3))
    np.testing.assert_allclose(swing_foot_reference(
        0., start, end, duration_s=.18, touchdown_depth_m=.008)[0], start)
    p, epsilon = .8, 1e-6
    before, _ = swing_foot_reference(p-epsilon, start, end, duration_s=.18, touchdown_depth_m=.008)
    after, _ = swing_foot_reference(p+epsilon, start, end, duration_s=.18, touchdown_depth_m=.008)
    _, velocity = swing_foot_reference(p, start, end, duration_s=.18, touchdown_depth_m=.008)
    np.testing.assert_allclose(velocity, (after-before)/(2*epsilon*.18), atol=1e-8)


@pytest.mark.parametrize('command,depth', [
    (Go2Command(vx=.8), .008),
    (Go2Command(vx=-.8), .008),
    (Go2Command(vy=.35), 0.),
    (Go2Command(vy=-.35), 0.),
    (Go2Command(vx=.8, vy=.35), .008*.8**2/(.8**2+.35**2)),
])
def test_swing_landing_load_tracks_command_direction_and_is_latched_through_stop(command, depth):
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_torque_controller(c)
    c._dt = .02
    c._cmd = command
    c._contact_gait.swing_progress.fill(1.)
    c._last_contacts.fill(1.)
    state = PyMpcInput(**{**vars(sample()), 'contacts': np.zeros((4, 3))})
    state.footholds_world[:, 2] = .025
    feet = state.footholds_world.copy()
    c._swing_starts[:] = feet
    jacobians = np.tile(np.eye(3), (4, 1, 1))
    first = c._torques(state, feet, np.zeros((4, 3)), jacobians, np.zeros(12))
    np.testing.assert_allclose(first.reshape(4, 3)[:, 2], -1000.*depth)
    # A user stop during the current swing must not move the actuator's
    # terminal height upwards before the foot lands.
    c._cmd = Go2Command()
    second = c._torques(state, feet, np.zeros((4, 3)), jacobians, np.zeros(12))
    np.testing.assert_allclose(second, first)


class FakeSolver:
    def __init__(self, force=None, status=0):
        self.force = np.asarray(
            force if force is not None else [4, 3, 90] * 4, dtype=float
        )
        self.status = status
        self.received = None
        self.reset_calls = 0

    def reset(self):
        self.reset_calls += 1

    def compute_control(self, state, reference, contacts, **kwargs):
        self.received = (state, reference, contacts, kwargs)
        return self.force, np.zeros((4, 3)), np.zeros(24), self.status


def sample() -> PyMpcInput:
    feet = np.array([[0.2, 0.1, 0.02], [0.2, -0.1, 0.02],
                     [-0.2, 0.1, 0.02], [-0.2, -0.1, 0.02]])
    return PyMpcInput(
        com_position=np.array([1.0, 2.0, 0.3]),
        com_velocity=np.array([0.1, 0.2, 0.0]),
        rpy=np.array([0.01, 0.02, 0.03]),
        angular_velocity_body=np.array([0.0, 0.0, 0.1]),
        feet_world=feet,
        desired_com_position=np.array([1.1, 2.1, 0.3]),
        desired_com_velocity=np.array([0.2, 0.1, 0.0]),
        desired_rpy=np.array([0.0, 0.0, 0.04]),
        desired_angular_velocity_body=np.array([0.0, 0.0, 0.1]),
        footholds_world=feet.copy(),
        contacts=np.array([[1, 0, 1], [0, 1, 0], [0, 1, 0], [1, 0, 1]]),
        mass_kg=18.0,
        inertia_body=np.diag([0.2, 0.5, 0.5]),
    )


def _torque_config(**overrides):
    values = dict(
        gait_hz=2.5,
        gait_duty=0.6,
        force_filter_alpha=1.0,
        optimization_friction=0.55,
        fz_max_n=180.0,
        torque_warmup_s=0.0,
        torque_ramp_s=0.001,
        aux_kp=0.0,
        aux_kv=0.0,
        tau_filter_alpha=1.0,
        joint_damping=.1,
        leg_kv_damping=1.,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def _init_contact_state(controller, elapsed_s=0.0):
    controller._joint_gravity = np.zeros(12)
    controller._base_velocity_world = np.zeros(3)
    controller._contact_gait = ContactGatedGait()
    controller._contact_gait.prime([50.] * 4, dt=.04)
    controller._contact_gait.time = elapsed_s
    controller._contact_gait.phase_rate = 1.0
    controller._was_searching = np.zeros(4, dtype=bool)
    controller._search_targets = np.zeros((4, 3))
    controller._search_min_z = np.zeros(4)
    controller._touchdown_depths = np.full(4, .008)
    controller._normal_contact_forces = lambda: np.full(4, 50.)
    controller._reachable_target = lambda _leg, target, **_kwargs: target.copy()


def _init_torque_controller(controller, *, elapsed_s=1.0, config=None):
    _init_contact_state(controller, elapsed_s)
    controller._metrics = None
    controller._contact_diagnostics = None
    controller._step_i = 0
    controller._solve_stride = 2
    controller._solver = SimpleNamespace(
        solve=lambda _sample: np.tile([0.0, 0.0, 50.0], (4, 1)), backend='jax_mppi', dt=.02,
    )
    controller._timing_sink = None
    controller._settling = False
    controller._config = config or _torque_config()
    phase = (elapsed_s * controller._config.gait_hz + np.array([0., .5, .5, 0.])) % 1.
    controller._contact_gait.swing_progress = np.clip(
        (phase - controller._config.gait_duty) / (1. - controller._config.gait_duty), 0., 1.)
    controller._contact_gait._swinging[:] = True
    controller._sim_time = elapsed_s
    controller._walk_started_s = 0.0
    controller._last_contacts = np.ones(4)
    controller._force_requested = np.zeros((4, 3))
    controller._forces = np.zeros((4, 3))
    controller._tau_filt = np.zeros(12)
    controller._tau_limited = np.zeros(12)
    controller._tau_raw = np.zeros(12)
    controller._swing_starts = np.zeros((4, 3))
    controller._touchdowns = np.zeros((4, 3))
    controller._cmd = Go2Command(vx=.8)
    controller._tau_lim = np.full(12, 100.0)
    controller._swing_velocity_limits = np.full(12, 30.)
    controller._kin = SimpleNamespace(ready_q=np.ones(12))
    controller._leg_dof_idxs = list(range(12))
    controller._entity = SimpleNamespace(
        get_dofs_position=lambda **_kwargs: np.zeros(12),
        set_dofs_damping=lambda *_args, **_kwargs: None,
    )


def test_diagonal_contact_sequence() -> None:
    schedule = contact_schedule(0.0, gait_hz=2.5, duty=0.5, dt=0.1, horizon=4)
    assert schedule.shape == (4, 4)
    np.testing.assert_array_equal(schedule[0], schedule[3])
    np.testing.assert_array_equal(schedule[1], schedule[2])
    np.testing.assert_array_equal(schedule[0] + schedule[1], np.ones(4))


def test_touchdown_preserves_nominal_foot_width_and_scales_commanded_stride() -> None:
    nominal = np.asarray([NOMINAL_FOOT_OFFSET_BODY[leg] for leg in ALL_LEGS])
    half_stance_s = 0.12
    placement_scale = 1.35
    placements = touchdown_offsets_body(
        nominal,
        command_body=np.array([0.2, 0.0, 0.0]),
        yaw_rate=0.0,
        half_stance_s=half_stance_s,
        placement_scale=placement_scale,
    )

    np.testing.assert_allclose(placements[:, 1], nominal[:, 1])
    np.testing.assert_allclose(
        placements[:, 0], nominal[:, 0] + 0.2 * half_stance_s * placement_scale
    )
    assert placements[0, 1] > 0.13
    assert placements[1, 1] < -0.13


def test_solver_passes_world_state_and_dynamic_payload_then_masks_swing() -> None:
    fake = FakeSolver(force=[30, -30, 300] * 4)
    adapter = PyMpcForceSolver(
        horizon=3, friction=0.5, max_normal_force_n=100,
        solver_factory=lambda: fake,
    )
    force = adapter.solve(sample())
    state, reference, contacts, kwargs = fake.received
    np.testing.assert_array_equal(state["position"], [1.0, 2.0, 0.3])
    np.testing.assert_array_equal(state["foot_FL"], [0.2, 0.1, 0.02])
    np.testing.assert_array_equal(reference["ref_foot_FL"], [[0.2, 0.1, 0.02]])
    np.testing.assert_array_equal(kwargs["inertia"], np.diag([0.2, 0.5, 0.5]).reshape(9))
    assert kwargs["mass"] == 18.0
    np.testing.assert_array_equal(contacts, sample().contacts)
    np.testing.assert_array_equal(force[0], [30, -30, 100])
    np.testing.assert_array_equal(force[1], [0, 0, 0])
    np.testing.assert_array_equal(force[2], [0, 0, 0])


def test_solver_uses_bounded_finite_force_when_iteration_limit_is_reached(capsys) -> None:
    fake = FakeSolver(force=[30, -30, 300] * 4, status=2)
    adapter = PyMpcForceSolver(
        horizon=3, friction=0.5, max_normal_force_n=100,
        solver_factory=lambda: fake,
    )

    force = adapter.solve(sample())
    adapter.solve(sample())

    np.testing.assert_array_equal(force[0], [30, -30, 100])
    np.testing.assert_array_equal(force[1], [0, 0, 0])
    np.testing.assert_array_equal(force[2], [0, 0, 0])
    assert np.all(np.isfinite(force))
    assert capsys.readouterr().out.count("using the finite, bounded force iterate") == 1


def test_force_solver_reset_clears_backend_warm_start_and_warning_state() -> None:
    fake = FakeSolver()
    adapter = PyMpcForceSolver(horizon=3, solver_factory=lambda: fake)
    adapter._maxiter_warning_logged = True

    adapter.reset()

    assert fake.reset_calls == 1
    assert not adapter._maxiter_warning_logged


@pytest.mark.parametrize("force,status", [([1, 2, 3] * 4, 4), ([float("nan"), 0, 1] * 4, 0)])
def test_solver_failure_is_not_reused_as_valid_force(force, status) -> None:
    adapter = PyMpcForceSolver(horizon=3, solver_factory=lambda: FakeSolver(force, status))
    with pytest.raises((RuntimeError, ValueError)):
        adapter.solve(sample())


def test_invalid_inertia_and_contact_are_rejected_before_solver() -> None:
    adapter = PyMpcForceSolver(horizon=3, solver_factory=FakeSolver)
    valid = sample()
    with pytest.raises(ValueError, match="inertia"):
        adapter.solve(PyMpcInput(**{**vars(valid), "inertia_body": np.zeros((3, 3))}))
    with pytest.raises(ValueError, match="binary"):
        adapter.solve(PyMpcInput(**{**vars(valid), "contacts": np.full((4, 3), 0.5)}))


def test_stance_ground_reaction_is_opposed_by_joint_torque() -> None:
    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_contact_state(controller)
    _init_torque_controller(controller)
    controller._solver = SimpleNamespace(solve=lambda _sample: np.tile([0, 0, 50], (4, 1)), backend='jax_mppi')
    state = PyMpcInput(**{**vars(sample()), "contacts": np.ones((4, 3))})
    jacobians = np.tile(np.eye(3), (4, 1, 1))
    torques = controller._torques(
        state, state.feet_world, np.zeros((4, 3)), jacobians, np.zeros(12)
    )
    np.testing.assert_array_equal(torques.reshape(4, 3), np.tile([0, 0, -50], (4, 1)))


@pytest.mark.parametrize('mode', ['stance', 'swing', 'search'])
def test_diagnostic_joint_gravity_does_not_change_actuation_after_gait_regression(mode):
    state = sample()
    state.contacts[:] = float(mode == 'stance')
    def run(gravity):
        c = PyMpcGenesisController.__new__(PyMpcGenesisController)
        _init_torque_controller(c)
        c._dt = .02
        c._joint_gravity = gravity.copy()
        c._touchdowns[:] = state.footholds_world
        if mode == 'search':
            c._contact_gait.search.fill(True)
            c._last_contacts.fill(0.)
            c._contact_gait.swing_progress.fill(1.)
        tau = c._torques(state, state.feet_world, np.zeros((4, 3)),
                         np.tile(np.eye(3), (4, 1, 1)), np.zeros(12))
        return tau, c._forces
    plain, plain_grf = run(np.zeros(12))
    gravity = np.linspace(-1.2, 1.2, 12)
    compensated, compensated_grf = run(gravity)
    np.testing.assert_allclose(compensated-plain, np.zeros(12), atol=1e-10)
    np.testing.assert_array_equal(compensated_grf, plain_grf)


@pytest.mark.parametrize('case,compensated', [
    ('settled', True), ('walking', False), ('linear_motion', False),
    ('angular_motion', False), ('joint_motion', False),
    ('unlanded', False), ('loading', False), ('cpu_backend', False),
])
def test_gravity_feedforward_is_limited_to_quasistatic_four_foot_settling(case, compensated):
    state = PyMpcInput(**{**vars(sample()), 'contacts':np.ones((4, 3)),
                         'com_velocity':np.zeros(3), 'angular_velocity_body':np.zeros(3)})
    if case == 'linear_motion':
        state.com_velocity[0] = .2
    if case == 'angular_motion':
        state.angular_velocity_body[1] = .3
    if case == 'unlanded':
        state.contacts[2] = 0.
    joint_velocity = np.full(12, .6 if case == 'joint_motion' else 0.)
    def run(gravity):
        c = PyMpcGenesisController.__new__(PyMpcGenesisController)
        _init_torque_controller(c)
        c._dt = .01
        c._settling = case != 'walking'
        c._joint_gravity = gravity.copy()
        if case == 'cpu_backend':
            c._solver.backend = 'acados'
        if case == 'loading':
            c._contact_gait.load[2] = .5
        return c._torques(state, state.feet_world, np.zeros((4, 3)),
                         np.tile(np.eye(3), (4, 1, 1)), joint_velocity)
    gravity = np.linspace(-1.2, 1.2, 12)
    difference = run(gravity)-run(np.zeros(12))
    np.testing.assert_allclose(difference, gravity if compensated else 0., atol=1e-10)


def test_force_filter_reprojects_stance_and_zeros_swing_legs() -> None:
    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_contact_state(controller)
    _init_torque_controller(
        controller,
        config=_torque_config(
            force_filter_alpha=0.5,
            optimization_friction=0.4,
            fz_max_n=60.0,
        ),
    )
    controller._solver = SimpleNamespace(
        solve=lambda _sample: np.tile([60.0, -60.0, 100.0], (4, 1)), backend='jax_mppi',
    )
    controller._forces[:] = [0.0, 0.0, 20.0]
    state = PyMpcInput(
        **{
            **vars(sample()),
            "contacts": np.array(
                [[1, 1, 1], [0, 0, 0], [1, 1, 1], [0, 0, 0]], dtype=float
            ),
        }
    )

    controller._torques(
        state,
        state.feet_world,
        np.zeros((4, 3)),
        np.tile(np.eye(3), (4, 1, 1)),
        np.zeros(12),
    )

    np.testing.assert_allclose(controller._forces[[0, 2]], [[24, -24, 60]] * 2)
    np.testing.assert_array_equal(controller._forces[[1, 3]], np.zeros((2, 3)))


def test_swing_uses_velocity_reference_without_stance_damping_or_old_torque():
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_contact_state(c)
    _init_torque_controller(c, elapsed_s=.32, config=_torque_config(
        gait_hz=2.5, gait_duty=.6, aux_kv=100., tau_filter_alpha=.1,
    ))
    c._tau_filt[:] = 90.
    state = sample()
    state.contacts[:] = 0.
    c._last_contacts[:] = 0.
    c._swing_starts[:] = [0., 0., .02]
    c._touchdowns[:] = [.1, 0., .02]
    phase = (.32 * 2.5 + np.array([0., .5, .5, 0.])) % 1.
    feet, velocities = zip(*(swing_foot_reference(
        np.clip((p - .6) / .4, 0, 1), c._swing_starts[i], c._touchdowns[i], duration_s=.16,
        touchdown_depth_m=.008,
    ) for i, p in enumerate(phase)))
    # Perfect tracking offsets the remaining passive .1 joint damping. A
    # stale 90-Nm support filter or the 100-Nm stance damping must not leak in.
    tau = c._torques(state, np.array(feet), np.array(velocities),
                     np.tile(np.eye(3), (4, 1, 1)), np.ones(12))
    np.testing.assert_allclose(tau, .1)


def test_torque_startup_ramp_fades_ready_pose_assist() -> None:
    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_contact_state(controller)
    _init_torque_controller(
        controller,
        elapsed_s=0.11,
        config=_torque_config(
            torque_warmup_s=0.05,
            torque_ramp_s=0.12,
            aux_kp=15.0,
            aux_kv=6.0,
        ),
    )
    state = PyMpcInput(**{**vars(sample()), "contacts": np.ones((4, 3))})
    jacobians = np.tile(np.eye(3), (4, 1, 1))
    joint_velocity = np.ones(12)
    # With a stationary body and identity Jacobians this is actual foot
    # motion, so the damping must remain present throughout the torque ramp.
    foot_velocity = joint_velocity.reshape(4, 3)

    ramped = controller._torques(
        state, state.feet_world, foot_velocity, jacobians, joint_velocity
    )

    np.testing.assert_allclose(
        ramped.reshape(4, 3), [[1.1625, 1.1625, -32.5875]] * 4
    )
    controller._sim_time = 0.17
    controller._contact_gait.time = 0.17
    controller._step_i = 1  # reuse the filtered GRF; do not solve again
    completed = controller._torques(
        state, state.feet_world, foot_velocity, jacobians, joint_velocity
    )
    np.testing.assert_allclose(completed.reshape(4, 3), [[-6.5, -6.5, -56.5]] * 4)


def test_contact_wait_does_not_freeze_startup_torque_or_pose_assistance():
    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_torque_controller(controller, elapsed_s=1.,
                           config=_torque_config(torque_warmup_s=.05, torque_ramp_s=.12,
                                                 aux_kp=15., aux_kv=2.))
    controller._contact_gait.time = 0.
    controller._contact_gait.blocked = True
    assert controller._torque_scale() == 1.
    np.testing.assert_array_equal(controller._aux_pd_torque(np.zeros(12)), np.zeros(12))


@pytest.mark.parametrize('command,velocity,waiting,expected', [
    (Go2Command(vx=.8), [.7, 0., .1], True, [.7, 0., 0.]),
    (Go2Command(vx=-.8), [-.7, 0., -.1], True, [-.7, 0., 0.]),
    (Go2Command(vy=.35), [.1, .3, .1], True, [0., .3, 0.]),
    (Go2Command(vx=.2), [.7, .1, .1], True, [.2, 0., 0.]),
    (Go2Command(vx=.8), [-.2, .1, .1], True, [0., 0., 0.]),
    (Go2Command(), [.7, 0., .1], True, [0., 0., 0.]),
    (Go2Command(vx=.8), [.7, 0., .1], False, [.8, 0., 0.]),
])
@pytest.mark.parametrize('backend', ['jax_mppi', 'acados'])
@pytest.mark.parametrize('shared_rotation', [False, True])
def test_landing_wait_keeps_search_reach_separate_from_ground_support(
        command, velocity, waiting, expected, backend, shared_rotation, monkeypatch):
    from pathlib import Path
    import pinocchio as pin
    from elesim_sim.robot.go2.mpc.config import Go2MpcConfig
    root = next(p for p in Path(__file__).resolve().parents if (p/'payload').is_dir())
    model = pin.buildModelFromUrdf(
        str(root/'payload/data/models/assemblies/zed-mini/assets/go2/go2.urdf'),
        pin.JointModelFreeFlyer())
    q = pin.neutral(model)
    q[:3] = [0., 0., .32]
    q[7:] = np.tile([0., .9, -1.8], 4)
    dq = np.r_[velocity, np.zeros(model.nv-3)]
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    c._pin = pin
    c._model, c._data = model, model.createData()
    c._frames = [model.getFrameId(f'{leg.value}_foot') for leg in ALL_LEGS]
    c._bridge = SimpleNamespace(read_pin_q_dq=lambda **kw: (q, dq))
    if shared_rotation:
        from scipy.spatial.transform import Rotation
        import elesim_sim.robot.go2.pympc_controller as module
        c._bridge.base_rotation = Rotation.from_quat(q[3:7])
        def repeated_rotation(*args, **kwargs):
            raise AssertionError('sample reconstructed the already-read body rotation')
        monkeypatch.setattr(module, 'Rotation', SimpleNamespace(
            from_quat=repeated_rotation, from_euler=Rotation.from_euler))
    c._config, c._payload = Go2MpcConfig(), None
    c._solver = SimpleNamespace(backend=backend, horizon=12, dt=.02)
    c._command_scale = lambda: 1.
    c._normal_contact_forces = lambda: np.full(4, 40.)
    search = np.array([False, False, True, True]) if waiting else np.zeros(4, dtype=bool)
    contacts = np.tile((1.-search)[:, None], (1, 12))
    updates = []
    def update_contacts(*args, **kwargs):
        updates.append(kwargs)
        return contacts.copy()
    c._contact_gait = SimpleNamespace(
        update=update_contacts, blocked=waiting,
        stance_duration_s=lambda **kw: kw['duty']/kw['gait_hz'],
        load=1.-search, search=search, swing_remaining=np.zeros(4), time=0.)
    c._cmd, c._dt, c._sim_time = command, .02, 1.
    c._step_i, c._solve_stride = 0, 1
    c._command_shaper = Go2CommandShaper(current=command, target=command)
    c._walk_started_s, c._settling = 0., False
    c._last_contacts, c._touchdowns = contacts[:, 0].copy(), np.zeros((4, 3))
    c._was_searching = np.zeros(4, dtype=bool)
    c._nominal_foot_offsets_body = np.array([NOMINAL_FOOT_OFFSET_BODY[leg] for leg in ALL_LEGS])
    state, *_ = c._sample()
    # GPU handover retains bounded existing progress; the CPU policy is
    # unchanged. Neither branch promises support from the searching feet.
    expected = np.zeros(3) if waiting and backend == 'acados' else expected
    np.testing.assert_allclose(state.desired_com_velocity, expected, atol=1e-9)
    np.testing.assert_allclose(c._base_velocity_world, velocity)
    np.testing.assert_array_equal(state.landing_search, search)
    np.testing.assert_array_equal(state.contacts[search.astype(bool)], 0.)
    assert updates[-1]['support_handover'] is (backend == 'jax_mppi')


def test_payload_com_height_keeps_body_height_error_when_arm_raises_com() -> None:
    from elesim_sim.robot.go2.pympc_controller import payload_aware_com_height

    assert payload_aware_com_height(0.42, 0.30, payload_active=False,
                                   base_height=0.30) == pytest.approx(0.30)
    # The arm may raise COM without requiring the legs to extend farther.
    assert payload_aware_com_height(0.42, 0.30, payload_active=True,
                                   base_height=0.30) == pytest.approx(0.42)
    # Raising the whole robot by 8 cm must retain an 8 cm restoring error.
    assert payload_aware_com_height(0.50, 0.30, payload_active=True,
                                   base_height=0.38) == pytest.approx(0.42)
    assert payload_aware_com_height(0.36, 0.30, payload_active=True,
                                   base_height=0.24) == pytest.approx(0.42)


@pytest.mark.parametrize("moving", [False, True])
def test_idle_clears_previous_gait_force_and_rearms_torque_mode(moving) -> None:
    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_contact_state(controller)
    controller._solver = SimpleNamespace(backend='jax_mppi')
    controller._metrics = None
    controller._contact_diagnostics = None
    controller._dt = 0.02
    controller._sim_time = 1.0
    controller._walk_started_s = 0.0
    controller._faulted = False
    controller._cmd = Go2Command()
    controller._config = SimpleNamespace(
        command_idle_threshold=0.05, gait_hz=2.5, gait_duty=0.6
    )
    controller._command_shaper = Go2CommandShaper(stop_dwell_s=0.2)
    controller._command_shaper._zero_since_s = 0.0
    controller._bridge = SimpleNamespace(last_q=np.r_[np.zeros(3), [0, 0, 0, 1], np.zeros(12)], last_dq=np.zeros(18))
    controller._pose_transition = JointPoseTransition(0.35)
    controller._pose_transition.reset(np.ones(12))
    controller._pose_stage = "walk"
    controller._active = True
    controller._step_i = 9
    controller._force_requested = np.ones((4, 3))
    controller._forces = np.ones((4, 3))
    controller._tau_filt = np.ones(12)
    controller._tau_hold = np.ones(12)
    controller._tau_limited = np.ones(12)
    controller._tau_raw = np.ones(12)
    controller._swing_starts = np.ones((4, 3))
    controller._touchdowns = np.ones((4, 3))
    controller._last_contacts = np.zeros(4)
    controller._kin = SimpleNamespace(stand_q=np.zeros(12))
    controller._leg_dof_idxs = list(range(12))
    calls = []
    controlled_pose = []

    def control_position(pose, **_kwargs):
        controlled_pose.append(np.asarray(pose, dtype=float).copy())

    def record_controlled_pose(pose, **kwargs):
        calls.append("position")
        control_position(pose, **kwargs)

    controller._set_stand_actuation = lambda: calls.append("stand")
    controller._torque_mode_active = True
    controller._sample = lambda: (None, None, None, None, None)
    controller._torques = lambda *_args: np.zeros(12)
    controller._entity = SimpleNamespace(
        get_dofs_position=lambda **_kwargs: np.ones(12),
        control_dofs_position=record_controlled_pose,
        control_dofs_force=lambda *_args, **_kwargs: None,
    )
    if moving:
        controller._bridge.last_dq[1] = .2
        controller.step()
        assert controller._settling
        assert controller._pose_stage == "walk"
        assert calls == []
        return
    controller.step()
    assert calls == ["stand", "position"]
    assert controller._step_i == 0
    assert not controller._active
    assert not controller._forces.any()
    assert not controller._force_requested.any()
    assert not controller._swing_starts.any()
    assert not controller._touchdowns.any()
    assert not controller._tau_filt.any()
    assert not controller._tau_hold.any()
    assert controller._last_contacts.all()
    assert np.max(np.abs(controlled_pose[0] - np.ones(12))) < 0.02


def test_idle_startup_stays_in_stand_during_stop_dwell() -> None:
    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    controller._solver = SimpleNamespace(backend='jax_mppi')
    _init_contact_state(controller)
    controller._metrics = None
    controller._contact_diagnostics = None
    controller._dt = 0.02
    controller._sim_time = 0.0
    controller._faulted = False
    controller._cmd = Go2Command()
    controller._command_shaper = Go2CommandShaper(stop_dwell_s=0.2)
    controller._pose_transition = JointPoseTransition(0.35)
    controller._pose_transition.reset(np.zeros(12))
    controller._pose_stage = "stand"
    controller._config = SimpleNamespace(
        command_idle_threshold=0.05, gait_hz=2.5, gait_duty=0.6
    )
    controller._active = False
    controller._step_i = 0
    controller._forces = np.zeros((4, 3))
    controller._tau_hold = np.zeros(12)
    controller._last_contacts = np.ones(4)
    controller._kin = SimpleNamespace(stand_q=np.zeros(12), ready_q=np.ones(12))
    controller._leg_dof_idxs = list(range(12))
    calls = []
    controller._set_stand_actuation = lambda: calls.append("stand")
    controller._torque_mode_active = True
    controller._sample = lambda: (None, None, None, None, None)
    controller._torques = lambda *_args: np.zeros(12)
    controller._entity = SimpleNamespace(
        control_dofs_position=lambda pose, **_kwargs: calls.append(tuple(pose))
    )

    controller.step()

    assert calls == [tuple(controller._kin.stand_q)]
    assert not controller._active


def test_command_shaper_limits_linear_and_yaw_acceleration_through_reversal() -> None:
    shaper = Go2CommandShaper(linear_accel_mps2=1.2, yaw_accel_radps2=3.0)
    shaper.set_target(Go2Command(vx=0.4, vy=-0.3, yaw_rate=0.8))
    forward = shaper.update(0.1)
    assert np.hypot(forward.vx, forward.vy) == pytest.approx(0.12)
    assert forward.vx == pytest.approx(0.096)
    assert forward.vy == pytest.approx(-0.072)
    assert forward.yaw_rate == pytest.approx(0.3)

    shaper.set_target(Go2Command(vx=-0.4, vy=0.3, yaw_rate=-0.8))
    previous = forward
    crossed_zero = [False, False, False]
    for _ in range(8):
        current = shaper.update(0.1)
        assert np.hypot(current.vx - previous.vx, current.vy - previous.vy) <= 0.12 + 1e-12
        assert abs(current.yaw_rate - previous.yaw_rate) <= 0.3 + 1e-12
        crossed_zero[0] |= previous.vx > 0.0 >= current.vx
        crossed_zero[1] |= previous.vy < 0.0 <= current.vy
        crossed_zero[2] |= previous.yaw_rate > 0.0 >= current.yaw_rate
        previous = current
    assert all(crossed_zero)
    assert previous.vx < 0.0
    assert previous.vy > 0.0
    assert previous.yaw_rate < 0.0
    assert not shaper.stop_ready(
        time_s=1.0, threshold=0.05, gait_hz=2.2, gait_duty=0.6
    )


def test_brief_zero_input_does_not_arm_stand_transition() -> None:
    shaper = Go2CommandShaper(stop_dwell_s=0.2)
    shaper.current = Go2Command(vx=0.04)
    shaper.set_target(Go2Command())
    shaper.update(0.01)
    assert not shaper.stop_ready(
        time_s=1.01, threshold=0.05, gait_hz=2.2, gait_duty=0.6
    )

    shaper.set_target(Go2Command(vx=-0.4))
    shaper.update(0.01)
    assert not shaper.stop_ready(
        time_s=1.02, threshold=0.05, gait_hz=2.2, gait_duty=0.6
    )


def test_stopping_waits_for_dwell_and_four_foot_support() -> None:
    shaper = Go2CommandShaper(stop_dwell_s=0.2)
    assert not shaper.stop_ready(
        time_s=1.0, threshold=0.05, gait_hz=2.2, gait_duty=0.6
    )
    assert not shaper.stop_ready(
        time_s=1.21, threshold=0.05, gait_hz=2.2, gait_duty=0.6
    )
    support_time = 3.5 / 2.2
    assert trot_all_stance(support_time, gait_hz=2.2, duty=0.6)
    assert shaper.stop_ready(
        time_s=support_time, threshold=0.05, gait_hz=2.2, gait_duty=0.6
    )


def test_joint_pose_transition_starts_continuously_and_reaches_exact_target() -> None:
    transition = JointPoseTransition(0.4)
    start = np.zeros(12)
    target = np.ones(12)
    transition.reset(start)
    transition.begin(start, target)

    samples = [transition.update(0.1)[0] for _ in range(4)]

    assert np.all(samples[0] > start)
    assert np.all(samples[0] < 0.2)
    assert np.all(np.diff(np.stack(samples), axis=0) > 0.0)
    np.testing.assert_array_equal(samples[-1], target)
    assert not transition.active


def test_new_pose_transition_can_interrupt_from_current_joint_pose() -> None:
    transition = JointPoseTransition(0.4)
    transition.reset(np.zeros(12))
    transition.begin(np.zeros(12), np.ones(12))
    current, _ = transition.update(0.2)
    transition.begin(current, np.full(12, -1.0))

    restarted, done = transition.update(0.01)

    assert not done
    assert np.max(np.abs(restarted - current)) < 0.02


def test_idle_to_motion_interpolates_stand_pose_before_ready_hold() -> None:
    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_contact_state(controller)
    controller._metrics = None
    controller._contact_diagnostics = None
    controller._metrics = None
    controller._dt = 0.1
    controller._sim_time = 0.0
    controller._faulted = False
    controller._cmd = Go2Command()
    controller._command_shaper = Go2CommandShaper(stop_dwell_s=0.2)
    controller._config = SimpleNamespace(
        command_idle_threshold=0.05,
        gait_hz=2.5,
        gait_duty=0.6,
        pose_transition_s=0.4,
        ready_pose_s=0.12,
        command_ramp_s=0.4,
        ready_kp=120.0,
        ready_kv=6.0,
    )
    controller._active = False
    controller._ready_until = 0.0
    controller._walk_started_s = None
    controller._torque_mode_active = False
    controller._step_i = 0
    controller._forces = np.zeros((4, 3))
    controller._tau_hold = np.zeros(12)
    controller._last_contacts = np.ones(4)
    controller._kin = SimpleNamespace(stand_q=np.zeros(12), ready_q=np.ones(12))
    controller._pose_transition = JointPoseTransition(0.4)
    controller._pose_transition.reset(controller._kin.stand_q)
    controller._pose_stage = "stand"
    controller._leg_dof_idxs = list(range(12))
    entity_state = {"q": np.zeros(12)}
    commanded_poses = []

    def control_position(pose, **_kwargs):
        entity_state["q"] = np.asarray(pose, dtype=float).copy()
        commanded_poses.append(entity_state["q"])

    controller._entity = SimpleNamespace(
        get_dofs_position=lambda **_kwargs: entity_state["q"],
        control_dofs_position=control_position,
    )
    controller._set_stand_actuation = lambda: None
    controller._set_ready_actuation = lambda: None
    from unittest.mock import Mock
    controller._bridge = SimpleNamespace(reset=Mock())
    controller._solver = SimpleNamespace(reset=Mock(), backend='acados')
    controller._payload = SimpleNamespace(reset=Mock())
    controller._set_torque_actuation = lambda: None

    controller.step()
    controller.set_command(Go2Command(vx=0.45, vy=0.6, yaw_rate=0.8))
    for _ in range(4):
        controller.step()

    assert len(commanded_poses) == 5
    assert np.all(commanded_poses[1] > 0.0)
    assert np.all(commanded_poses[1] < 0.2)
    assert np.all(np.diff(np.stack(commanded_poses[1:]), axis=0) > 0.0)
    np.testing.assert_array_equal(commanded_poses[-1], controller._kin.ready_q)
    assert controller._pose_stage == "ready_hold"
    controller._bridge.reset.assert_called_once()
    controller._solver.reset.assert_called_once()
    controller._payload.reset.assert_called_once()

    controller.step()
    controller.step()
    assert controller._pose_stage == "walk"
    assert controller._command_scale() == 0.0

    observed_command_scale = []
    def sample_with_command_ramp():
        observed_command_scale.append(controller._command_scale())
        return (
            None,
            np.zeros((4, 3)),
            np.zeros((4, 3)),
            np.zeros((4, 3, 3)),
            np.zeros(12),
        )

    controller._sample = sample_with_command_ramp
    controller._torques = lambda *_args: np.zeros(12)
    controller._entity.control_dofs_force = lambda *_args, **_kwargs: None
    controller.step()
    assert observed_command_scale[0] == pytest.approx(smoothstep_quintic(0.25))


def test_pympc_uses_a_smooth_startup_command_ramp() -> None:
    pympc = PyMpcGenesisController.__new__(PyMpcGenesisController)
    pympc._solver = SimpleNamespace(backend='acados')
    pympc._config = SimpleNamespace(command_ramp_s=0.4)
    pympc._sim_time = 1.2
    pympc._walk_started_s = 1.0
    assert pympc._command_scale() == pytest.approx(0.5)

    assert smoothstep_quintic(-0.1) == 0.0
    assert smoothstep_quintic(0.5) == pytest.approx(0.5)
    assert smoothstep_quintic(1.1) == 1.0


def test_pympc_records_contact_forces_and_preclip_torque():
    from unittest.mock import Mock

    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_contact_state(controller)
    controller._metrics = None
    controller._contact_diagnostics = None
    controller._metrics = Mock()
    controller._contact_diagnostics = Mock(cadence_steps=5)
    controller._step_i = 5
    controller._dt = 0.02
    controller._sim_time = 1.2
    controller._config = SimpleNamespace(physical_friction=0.55)
    controller._force_requested = np.full((4, 3), 7.0)
    controller._forces = np.arange(12).reshape(4, 3)
    controller._tau_raw = np.full(12, 30.0)
    controller._tau_limited = np.full(12, 25.0)
    controller._tau_hold = np.full(12, 20.0)
    controller._record_contact_metrics(sample())
    request = controller._contact_diagnostics.sample.call_args.kwargs
    assert list(request["stance"].values()) == [True, False, False, True]
    assert request["elapsed_s"] == pytest.approx(0.1)
    recorded = controller._metrics.sample_contact.call_args.kwargs
    np.testing.assert_array_equal(recorded["raw_grf"], controller._force_requested)
    np.testing.assert_array_equal(recorded["tau_raw"], np.full(12, 30.0))
    np.testing.assert_array_equal(recorded["tau_limited"], np.full(12, 25.0))
    np.testing.assert_array_equal(recorded["tau_applied"], np.full(12, 20.0))
    controller._metrics.reset_mock()
    controller._contact_diagnostics.sample.return_value = None
    controller._record_contact_metrics(sample())
    controller._metrics.sample_contact.assert_not_called()


def test_pympc_emits_walking_rows_in_stand_and_torque_modes():
    from unittest.mock import Mock
    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_contact_state(controller)
    controller._step = Mock()
    controller._metrics = Mock()
    controller._entity = object()
    controller._cmd = Go2Command(vx=0.2)
    controller._command_source = "test"
    controller._arm_q = (0, 0, 0, 0)
    controller._tau_hold = np.ones(12)
    controller._sim_time = 1.0
    controller._rate_info = object()
    controller._bridge = SimpleNamespace(last_dq=np.arange(18.0))
    controller._faulted = False
    for active in (False, True):
        controller._torque_mode_active = active
        controller.step()
        row = controller._metrics.sample_go2.call_args.kwargs
        assert row["torque_update_flag"] == active
        assert row["go2_cmd"] == (0.2, 0.0, 0.0)
        assert (row["tau"] is not None) == active
    assert controller._metrics.sample_go2.call_count == 2


def test_contact_transition_forces_solve_between_regular_ticks():
    from unittest.mock import Mock
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_contact_state(c)
    _init_torque_controller(c, elapsed_s=1.0)
    c._step_i = 1
    c._solve_stride = 2
    c._solver = SimpleNamespace(solve=Mock(return_value=np.tile([0, 0, 50.], (4, 1))), backend='jax_mppi')
    c._last_contacts = np.zeros(4)
    state = PyMpcInput(**{**vars(sample()), "contacts": np.ones((4, 3))})
    tau = c._torques(state, state.feet_world, np.zeros((4, 3)),
                     np.tile(np.eye(3), (4, 1, 1)), np.zeros(12))
    c._solver.solve.assert_called_once()
    np.testing.assert_array_equal(tau.reshape(4, 3)[:, 2], np.full(4, -50.))


def test_gait_starts_at_same_support_phase_independent_of_uptime():
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_contact_state(c)
    for start in (0.0, 7.13, 103.29):
        c._walk_started_s = start
        c._sim_time = start
        mask = contact_schedule(c._gait_time(), gait_hz=2.5, duty=0.6, dt=0.02, horizon=12)
        np.testing.assert_array_equal(mask[:, 0], np.ones(4))
        c._sim_time = start + 10.0
        assert c._gait_time() == 0.0  # Wall/simulation uptime cannot advance the gated clock.
        c._contact_gait.time = 0.04
        assert c._gait_time() == pytest.approx(0.04)
        c._contact_gait.time = 0.0


def test_solver_failure_returns_to_stand_without_pose_step():
    from unittest.mock import Mock
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_contact_state(c)
    c._solver = SimpleNamespace(backend='jax_mppi')
    c._dt = 0.02
    c._sim_time = 10.0
    c._walk_started_s = 9.0
    c._faulted = False
    c._command_shaper = Go2CommandShaper()
    c._command_shaper.set_target(Go2Command(vx=0.35))
    c._config = SimpleNamespace(command_idle_threshold=0.01, gait_hz=2.5, gait_duty=0.6)
    c._pose_stage = "walk"
    c._torque_mode_active = True
    c._pose_transition = JointPoseTransition(0.4)
    c._kin = SimpleNamespace(stand_q=np.zeros(12))
    c._leg_dof_idxs = list(range(12))
    c._tau_hold = np.ones(12)
    c._force_requested = np.zeros((4, 3))
    c._forces = np.zeros((4, 3))
    c._tau_filt = np.zeros(12)
    c._tau_limited = np.zeros(12)
    c._tau_raw = np.zeros(12)
    c._entity = SimpleNamespace(get_dofs_position=lambda **kw: np.ones(12),
                                control_dofs_position=Mock())
    c._set_stand_actuation = Mock()
    c._sample = Mock(side_effect=RuntimeError("solver failure"))
    c._step()
    first = c._entity.control_dofs_position.call_args.args[0]
    assert np.all(first > 0.99)
    assert c._faulted
    assert c.fault_reason == 'solver failure'
    for _ in range(25):
        c._step()
    np.testing.assert_allclose(c._entity.control_dofs_position.call_args.args[0], np.zeros(12))
    c._sample.assert_called_once()


def test_controller_reset_preempts_command_and_resets_solver_and_bridge():
    from unittest.mock import Mock

    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_contact_state(controller)
    controller._command_shaper = SimpleNamespace(reset=Mock())
    controller._contact_diagnostics = None
    controller._pose_transition = SimpleNamespace(reset=Mock())
    controller._kin = SimpleNamespace(stand_q=np.zeros(12))
    controller._config = SimpleNamespace(stand_kp=20.0, stand_kv=1.0, joint_damping=.1)
    controller._entity = Mock()
    controller._leg_dof_idxs = list(range(12))
    controller._payload = None
    controller._bridge = SimpleNamespace(reset=Mock())
    controller._solver = SimpleNamespace(reset=Mock())
    controller._force_requested = np.ones((4, 3))
    controller._forces = np.ones((4, 3))
    controller._tau_filt = np.ones(12)
    controller._tau_hold = np.ones(12)
    controller._tau_limited = np.ones(12)
    controller._tau_raw = np.ones(12)
    controller._swing_starts = np.ones((4, 3))
    controller._touchdowns = np.ones((4, 3))
    controller._last_contacts = np.zeros(4)
    controller._fault_reason = 'old controller failure'

    controller.reset()

    assert controller._cmd == Go2Command()
    assert controller._command_shaper.reset.call_count == 1
    assert controller._bridge.reset.call_count == 1
    assert controller._solver.reset.call_count == 1
    assert not np.any(controller._force_requested)
    assert not np.any(controller._forces)
    assert not np.any(controller._tau_hold)
    assert not np.any(controller._swing_starts)
    assert not np.any(controller._touchdowns)
    assert not np.any(controller._touchdown_depths)
    assert controller._faulted is False
    assert controller.fault_reason == ''


def test_force_mode_damping_is_passive_and_restored_for_position_control():
    from unittest.mock import Mock
    from elesim_sim.robot.go2.mpc.config import Go2MpcConfig
    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    controller._entity = Mock()
    controller._leg_dof_idxs = list(range(12))
    controller._config = Go2MpcConfig(joint_damping=.1, leg_kv_damping=3.)
    # Genesis FORCE mode bypasses actuator KV entirely. Passive damping is
    # evaluated each physics substep and remains active in that mode.
    for transition, expected in [(controller._set_torque_actuation, 3.1),
                                 (controller._set_stand_actuation, .1),
                                 (controller._set_torque_actuation, 3.1),
                                 (controller._set_ready_actuation, .1)]:
        transition()
        assert controller._entity.set_dofs_damping.call_args is not None
        args, kwargs = controller._entity.set_dofs_damping.call_args
        np.testing.assert_allclose(args[0], np.full(12, expected))
        assert kwargs['dofs_idx_local'] == controller._leg_dof_idxs


def test_genesis_bridge_estimates_stale_zero_base_twist_from_pose_delta():
    from elesim_sim.robot.go2.mpc.genesis_pin_bridge import GenesisPinBridge

    pose = {"x": 0.0, "yaw": 0.0}

    def quaternion_wxyz():
        half_yaw = pose["yaw"] / 2.0
        return np.array([np.cos(half_yaw), 0.0, 0.0, np.sin(half_yaw)])

    base = type(
        "Base",
        (),
        {
            "get_pos": lambda _self: np.array([pose["x"], 0.0, 0.32]),
            "get_quat": lambda _self: quaternion_wxyz(),
            "get_vel": lambda _self: np.zeros(3),
            "get_ang": lambda _self: np.zeros(3),
        },
    )()
    entity = type(
        "Entity",
        (),
        {
            "get_link": lambda _self, _name: base,
            "get_dofs_position": lambda _self, *, dofs_idx_local: np.zeros(len(dofs_idx_local)),
            "get_dofs_velocity": lambda _self, *, dofs_idx_local: np.zeros(len(dofs_idx_local)),
        },
    )()
    bridge = GenesisPinBridge(entity, list(range(12)))

    _, initial_dq = bridge.read_pin_q_dq(dt=0.02)
    pose["x"] = 0.002
    pose["yaw"] = 0.01
    _, moving_dq = bridge.read_pin_q_dq(dt=0.02)

    np.testing.assert_array_equal(initial_dq[:6], np.zeros(6))
    np.testing.assert_allclose(
        moving_dq[:3], [np.cos(0.01) * 0.1, -np.sin(0.01) * 0.1, 0.0], atol=1e-10
    )
    np.testing.assert_allclose(moving_dq[3:6], [0.0, 0.0, 0.5], atol=1e-10)

    bridge.reset()
    pose["x"] = 0.5
    _, after_reset_dq = bridge.read_pin_q_dq(dt=0.02)
    np.testing.assert_array_equal(after_reset_dq[:6], np.zeros(6))


@pytest.mark.parametrize("twist,tilt,expected", [
    ([0, 0, 0, 0, 0, 0], 0., True),
    ([0, .12, 0, 0, 0, 0], 0., False),
    ([0, 0, 0, 0, 0, .5], 0., False),
    ([0, 0, 0, .5, 0, 0], 0., False),
    ([0, 0, 0, 0, 0, 0], .3, False),
    ([float("nan"), 0, 0, 0, 0, 0], 0., False),
])
def test_stand_transition_requires_actual_settled_body(twist, tilt, expected):
    from scipy.spatial.transform import Rotation
    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_contact_state(controller)
    controller._bridge = SimpleNamespace(
        last_q=np.r_[np.zeros(3), Rotation.from_euler("x", tilt).as_quat(), np.zeros(12)],
        last_dq=np.r_[twist, np.zeros(12)],
    )
    assert controller._body_ready_to_stand() is expected
    controller._bridge.last_q = None
    assert not controller._body_ready_to_stand()


def test_repeated_zero_commands_do_not_restart_stop_dwell():
    shaper = Go2CommandShaper(stop_dwell_s=0.2)
    ready = []
    for tick in range(51):
        shaper.set_target(Go2Command())
        shaper.update(.02)
        ready.append(shaper.stop_ready(time_s=tick * .02, threshold=.05,
                                      gait_hz=2.5, gait_duty=.6))
    assert not any(ready[:10])
    assert any(ready[10:])
    shaper.set_target(Go2Command(vx=.1))
    assert shaper._zero_since_s is None


@pytest.mark.parametrize("actual,desired,shift", [
    ([0., .25, 0.], [0., .25, 0.], [0., 0.]),
    ([.2, .05, 0.], [.1, .25, 0.], [.01, -.02]),
    ([2., -2., 0.], [0., 0., 0.], [.04, -.04]),
])
def test_touchdown_velocity_correction_is_world_aligned_and_bounded(actual, desired, shift):
    from scipy.spatial.transform import Rotation
    from elesim_sim.robot.go2.pympc_controller import touchdown_candidates_world
    offsets = np.array([[.2, .17, 0.], [.2, -.17, 0.], [-.2, .17, 0.], [-.2, -.17, 0.]])
    yaw = Rotation.from_euler("z", np.pi / 2).as_matrix()
    original = offsets.copy()
    result = touchdown_candidates_world(np.array([1., 2., .3]), offsets,
        yaw_rotation=yaw, velocity_world=actual, desired_velocity_world=desired)
    expected = np.array([1., 2., .3]) + offsets @ yaw.T
    expected[:, :2] += shift
    np.testing.assert_allclose(result, expected)
    np.testing.assert_array_equal(offsets, original)


def test_missed_landing_keeps_ground_target_instead_of_resetting_to_air():
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_torque_controller(c)
    c._dt = .02
    c._last_contacts.fill(0)
    c._contact_gait.search.fill(True)
    c._contact_gait.swing_progress.fill(1.)
    c._contact_gait.missing_age.fill(.02)
    c._touchdowns[:] = sample().footholds_world
    c._touchdowns[:, 2] = .025
    feet = c._touchdowns.copy()
    feet[:, 2] = .08
    state = PyMpcInput(**{**vars(sample()), 'contacts': np.zeros((4, 3))})
    c._torques(state, feet, np.zeros((4, 3)), np.tile(np.eye(3), (4, 1, 1)), np.zeros(12))
    np.testing.assert_allclose(c._search_targets[:, :2], c._touchdowns[:, :2])
    assert np.all(c._search_targets[:, 2] <= .025)


def test_missed_swing_continues_downward_from_loaded_endpoint_with_total_depth_cap():
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_torque_controller(c)
    c._dt = .02
    c._last_contacts.fill(0)
    c._contact_gait.search.fill(True)
    c._contact_gait.swing_progress.fill(1.)
    c._contact_gait.missing_age.fill(.02)
    c._touchdowns[:] = [.1, .17, .025]
    feet = np.tile([.1, .17, .017], (4, 1))
    state = PyMpcInput(**{**vars(sample()), 'contacts': np.zeros((4, 3))})
    c._torques(state, feet, np.zeros((4, 3)), np.tile(np.eye(3), (4, 1, 1)), np.zeros(12))
    assert np.all(c._search_targets[:, 2] < feet[:, 2])
    for step in range(1, 15):
        c._contact_gait.missing_age.fill((step+1)*c._dt)
        c._torques(state, feet, np.zeros((4, 3)), np.tile(np.eye(3), (4, 1, 1)), np.zeros(12))
    np.testing.assert_allclose(c._search_targets[:, 2], .025-.04, atol=1e-10)


def test_search_depth_cap_also_stops_velocity_feedforward():
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_torque_controller(c)
    c._dt = .02
    c._last_contacts.fill(0)
    c._was_searching.fill(True)
    c._contact_gait.search.fill(True)
    c._contact_gait.missing_age.fill(.30)
    feet = sample().feet_world.copy()
    c._search_targets[:] = feet
    c._search_min_z[:] = feet[:, 2]
    state = PyMpcInput(**{**vars(sample()), 'contacts': np.zeros((4, 3))})
    tau = c._torques(state, feet, np.zeros((4, 3)), np.tile(np.eye(3), (4, 1, 1)), np.zeros(12))
    np.testing.assert_allclose(tau, np.zeros(12), atol=1e-10)


def test_search_retarget_velocity_matches_lateral_reference_motion():
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_torque_controller(c)
    c._dt = .02
    c._last_contacts.fill(0)
    c._was_searching.fill(True)
    c._contact_gait.search.fill(True)
    c._contact_gait.missing_age.fill(.30)
    feet = sample().feet_world.copy()
    c._search_targets[:] = feet
    c._search_min_z[:] = feet[:, 2]
    c._reachable_target = lambda _leg, target, **kw: target + np.array([.01,0,0])
    state = PyMpcInput(**{**vars(sample()), 'contacts': np.zeros((4, 3))})
    tau = c._torques(state, feet, np.zeros((4, 3)), np.tile(np.eye(3),(4,1,1)), np.zeros(12))
    np.testing.assert_allclose(tau.reshape(4,3), np.tile([20.,0,0],(4,1)))
