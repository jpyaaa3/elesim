"""The GPU rollout must predict the force that startup actuation can apply."""
from dataclasses import replace

import numpy as np
import pytest

from elesim_sim.robot.go2.pympc_solver import PyMpcForceSolver, support_load_horizon
from workbench.tests.apps.sim.scenarios.go2.test_53_pympc_gpu import RecordingBackend, standing
from workbench.tests.apps.sim.scenarios.go2.test_58_stance_tracking import controller


class ReferenceBackend(RecordingBackend):
    def compute_control(self, state, reference, contacts, **parameters):
        self.reference = reference
        return super().compute_control(state, reference, contacts, **parameters)


def test_prediction_combines_independent_contact_and_actuator_ramps():
    scales = np.linspace(.35, 1., 12)
    state = replace(standing(), support_loads=np.array([.5, 1., 1., 0.]), actuation_scales=scales)
    fake = ReferenceBackend()
    PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: fake).solve(state)
    expected = support_load_horizon(state.contacts, state.support_loads, .02)*scales[:, None]
    np.testing.assert_allclose(fake.reference['support_loads'], expected)


@pytest.mark.parametrize('scales', [np.ones(11), np.full(12, np.nan),
                                    np.full(12, -.1), np.full(12, 1.1)])
def test_bad_actuation_scale_is_rejected_before_dispatch(scales):
    fake = ReferenceBackend()
    with pytest.raises(ValueError, match='actuation_scales'):
        PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: fake).solve(
            replace(standing(), actuation_scales=scales))
    assert not fake.received


def test_cpu_does_not_receive_the_gpu_startup_force_model():
    fake = ReferenceBackend()
    PyMpcForceSolver(solver_factory=lambda: fake).solve(
        replace(standing(), actuation_scales=np.full(12, .35)))
    assert 'support_loads' not in fake.reference


def startup_controller(elapsed=.05, backend='jax_mppi'):
    c, state = controller()
    c._solver.backend = backend
    c._solver.horizon = state.contacts.shape[1]
    c._sim_time, c._walk_started_s = elapsed, 0.
    c._config.torque_ramp_s = .17
    return c, state


def test_force_preview_follows_the_actual_gain_at_each_prediction_stage():
    c, _ = startup_controller()
    expected = np.minimum(1., .35+.65*(.05+np.arange(c._solver.horizon)*.02)/.17)
    np.testing.assert_allclose(c._actuation_scale_horizon(), expected)
    assert c._actuation_scale_horizon()[0] == c._torque_scale()
    c._solver.backend = 'acados'
    assert c._actuation_scale_horizon() is None


@pytest.mark.parametrize('elapsed,backend,settling,hold', [
    (0., 'jax_mppi', False, True), (.05, 'jax_mppi', False, True),
    (.17, 'jax_mppi', False, False), (.2, 'jax_mppi', False, False),
    (0., 'acados', False, False), (.2, 'jax_mppi', True, True),
])
def test_initial_support_is_retained_until_torque_actuation_is_ready(elapsed, backend, settling, hold):
    c, _ = startup_controller(elapsed, backend)
    c._settling = settling
    assert c._balance_only() is hold


def test_contact_schedule_cannot_start_a_swing_during_the_actuator_ramp():
    from elesim_sim.robot.go2.pympc_controller import ContactGatedGait
    c, _ = startup_controller(0.)
    c._contact_gait = ContactGatedGait()
    c._contact_gait.prime(np.full(4, 50.), dt=.04)
    for tick in range(17):
        c._sim_time = tick*.01
        contacts = c._contact_gait.update(np.full(4, 50.), foot_heights=np.full(4, .023),
            dt=.01, gait_hz=2.2, duty=.6, horizon_dt=.02, horizon=12,
            settling=c._balance_only(), support_handover=True)
        np.testing.assert_array_equal(contacts, np.ones((4, 12)))
        assert not np.any(c._contact_gait._swinging)
    for tick in range(17, 25):
        c._sim_time = tick*.01
        c._contact_gait.update(np.full(4, 50.), foot_heights=np.full(4, .023),
            dt=.01, gait_hz=2.2, duty=.6, horizon_dt=.02, horizon=12,
            settling=c._balance_only(), support_handover=True)
    assert np.any(c._contact_gait._swinging)


def test_stance_reference_scales_only_the_active_part_of_the_motor_damping():
    c, state = startup_controller()
    velocity = np.tile([-.3, 0., 0.], 4)
    torque = c._torques(state, state.feet_world, np.zeros((4, 3)),
                       np.tile(np.eye(3), (4, 1, 1)), velocity)
    net = torque.reshape(4, 3)-1.1*velocity.reshape(4, 3)
    expected = np.tile([0., 0., -50.*c._torque_scale()], (4, 1))
    np.testing.assert_allclose(net, expected, atol=1e-12)


def test_search_reference_keeps_passive_damping_unscaled_during_startup():
    from workbench.tests.apps.sim.scenarios.go2.test_59_touchdown_tracking import search_controller, JACOBIAN
    c, state = search_controller()
    c._sim_time = .05
    c._config.torque_ramp_s = .17
    velocity = np.linalg.solve(JACOBIAN, [.1, 0., -.03])
    torque = c._torques(state, state.feet_world, np.zeros((4, 3)),
                       np.tile(JACOBIAN, (4, 1, 1)), np.tile(velocity, 4))
    net = torque.reshape(4, 3)-1.1*velocity
    expected = JACOBIAN.T@np.array([0., 0., -4.5*c._torque_scale()])
    np.testing.assert_allclose(net, np.tile(expected, (4, 1)), atol=1e-12)


@pytest.mark.parametrize('scale', [.35, 1.])
def test_contact_diagnostics_report_the_actual_startup_force_multiplier(scale):
    from unittest.mock import Mock
    c, state = controller()
    state = replace(state, actuation_scales=np.full(state.contacts.shape[1], scale))
    c._metrics = Mock()
    c._contact_diagnostics = Mock(cadence_steps=5)
    c._config.physical_friction = .55
    c._tau_hold = np.zeros(12)
    c._forces[:] = [0., 0., 50.]
    c._record_contact_metrics(state)
    reported = c._contact_diagnostics.sample.call_args.kwargs['desired_grf_world']
    for force in reported.values():
        np.testing.assert_allclose(force, [0., 0., 50.*scale])


@pytest.mark.parametrize('first', [True, False])
def test_first_force_solution_has_no_zero_history_but_later_filtering_is_retained(first):
    c, state = startup_controller()
    c._base_velocity_world[:] = 0.
    c._config.force_filter_alpha = .35
    c._config.tau_filter_alpha = .25
    c._step_i = 0 if first else 2
    torque = c._torques(state, state.feet_world, np.zeros((4, 3)),
                       np.tile(np.eye(3), (4, 1, 1)), np.zeros(12))
    filter_gain = 1. if first else .35*.25
    expected = np.tile([0., 0., -50.*c._torque_scale()*filter_gain], (4, 1))
    np.testing.assert_allclose(torque.reshape(4, 3), expected, atol=1e-12)
