"""Score only support forces the bounded motor commands can produce."""
from dataclasses import replace
import numpy as np
import pytest

from elesim_sim.robot.go2.pympc_solver import PyMpcForceSolver
from workbench.tests.apps.sim.scenarios.go2.test_53_pympc_gpu import cpu_math, standing
from workbench.tests.apps.sim.scenarios.go2.test_58_stance_tracking import controller
from workbench.tests.apps.sim.scenarios.go2.test_59_touchdown_tracking import JACOBIAN


class Capture:
    def __init__(self):
        self.calls = 0

    def compute_control(self, state, reference, contacts, **parameters):
        self.calls += 1
        self.state, self.reference, self.parameters = state, reference, parameters
        return np.zeros(12), None, None, 0


def arguments():
    sample = standing()
    capture = Capture()
    PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: capture).solve(sample)
    return sample, capture


@pytest.mark.parametrize('gain,load', [(1., 1.), (1., .25), (.35, .25)])
@pytest.mark.parametrize('bias', [-1., 1.])
def test_rollout_respects_raw_and_final_motor_bounds_including_bias(cpu_math, gain, load, bias):
    sample, c = arguments()
    maps = np.tile(np.vstack((-.2*np.eye(3)/gain, -.2*np.eye(3))), (12, 4, 1, 1))
    offsets = np.zeros((12, 4, 6))
    offsets[..., 2] = bias
    offsets[..., 5] = -bias
    limits = np.full((12, 4, 6), 100.)
    limits[..., [2, 5]] = 5.
    c.reference.update(support_loads=np.full((12, 4), gain*load),
        support_torque_maps=maps, support_torque_offsets=offsets,
        support_torque_limits=limits)
    forces, *_ = cpu_math.compute_control(c.state, c.reference, sample.contacts, **c.parameters)
    applied = forces.reshape(4, 3)*gain*load
    actual = np.einsum('ijk,ik->ij', maps[0], applied)+offsets[0]
    assert np.all(np.abs(actual) <= limits[0]+2e-5), actual
    assert np.isfinite(cpu_math.last_cost)
    assert np.all(applied[:, 2] >= 0.)
    assert np.all(np.abs(applied[:, :2]) <= .55*applied[:, 2:3]+1e-5)


def test_infeasible_normal_floor_does_not_override_the_motor_limit(cpu_math):
    sample, c = arguments()
    maps = np.tile(np.vstack((-np.eye(3), -np.eye(3))), (12, 4, 1, 1))
    limits = np.full((12, 4, 6), 5.)
    c.reference.update(support_force_floor_n=10., support_torque_maps=maps,
        support_torque_offsets=np.zeros((12, 4, 6)), support_torque_limits=limits)
    forces, *_ = cpu_math.compute_control(c.state, c.reference, sample.contacts, **c.parameters)
    assert np.all(forces.reshape(4, 3)[:, 2] <= 5.+1e-5)
    assert np.isfinite(cpu_math.last_cost)


def test_absent_motor_constraints_keep_the_existing_optimizer_result(cpu_math):
    sample, c = arguments()
    baseline, *_ = cpu_math.compute_control(c.state, c.reference, sample.contacts, **c.parameters)
    cpu_math.reset()
    c.reference.update(support_torque_maps=np.zeros((12, 4, 6, 3)),
        support_torque_offsets=np.zeros((12, 4, 6)),
        support_torque_limits=np.ones((12, 4, 6)))
    actual, *_ = cpu_math.compute_control(c.state, c.reference, sample.contacts, **c.parameters)
    np.testing.assert_array_equal(actual, baseline)


@pytest.mark.parametrize('supported', [True, False])
def test_a_motor_bias_no_support_force_can_repair_is_not_scored_as_feasible(cpu_math, supported):
    sample, c = arguments()
    c.reference.update(support_torque_maps=np.zeros((12, 4, 6, 3)),
        support_torque_offsets=np.full((12, 4, 6), 2.),
        support_torque_limits=np.ones((12, 4, 6)))
    if supported:
        with pytest.raises(RuntimeError, match='no finite force trajectory'):
            cpu_math.compute_control(c.state, c.reference, sample.contacts, **c.parameters)
    else:
        forces, *_ = cpu_math.compute_control(c.state, c.reference,
            np.zeros_like(sample.contacts), **c.parameters)
        np.testing.assert_array_equal(forces, 0.)


@pytest.mark.parametrize('gain', [.35, .675, 1.])
def test_motor_maps_match_the_two_actual_clipping_stages(gain):
    from elesim_sim.robot.go2.pympc_controller import support_actuator_constraints
    raw_bias = np.arange(12)/10.
    auxiliary = np.linspace(-1., 1., 12)
    gains = np.array([gain, 1., 1.])
    maps, offsets, limits = support_actuator_constraints(np.tile(JACOBIAN, (4, 1, 1)),
        raw_bias, auxiliary, np.full(12, 20.), gains)
    requested = np.tile([5., -2., 40.], (4, 1))
    loads = np.array([1/6, 2/3, 1., 0.])
    raw = np.array([-JACOBIAN.T@force*load for force, load in zip(requested, loads)])+raw_bias.reshape(4, 3)
    for stage, actual_gain in enumerate(gains):
        actual = np.einsum('ijk,ik->ij', maps[stage], requested*loads[:, None]*actual_gain)+offsets[stage]
        expected = np.concatenate((raw, raw*actual_gain+auxiliary.reshape(4, 3)), axis=1)
        np.testing.assert_allclose(actual, expected, atol=1e-12)
    np.testing.assert_array_equal(limits, np.full((3, 4, 6), 20.))


@pytest.mark.parametrize('backend', ['jax_mppi', 'acados'])
def test_adapter_transfers_the_complete_constraint_bundle_only_to_gpu(backend):
    c = Capture()
    bundle = dict(support_torque_maps=np.zeros((12, 4, 6, 3)),
                  support_torque_offsets=np.zeros((12, 4, 6)),
                  support_torque_limits=np.ones((12, 4, 6)))
    PyMpcForceSolver(backend=backend, solver_factory=lambda: c).solve(replace(standing(), **bundle))
    for name, value in bundle.items():
        if backend == 'jax_mppi':
            np.testing.assert_array_equal(c.reference[name], value)
        else:
            assert name not in c.reference


@pytest.mark.parametrize('field,value', [('support_torque_maps', np.zeros((4, 3, 3))),
    ('support_torque_maps', np.full((12, 4, 6, 3), np.nan)),
    ('support_torque_offsets', None), ('support_torque_limits', np.zeros((12, 4, 6)))])
def test_invalid_motor_constraints_are_rejected_before_dispatch(field, value):
    c = Capture()
    bundle = dict(support_torque_maps=np.zeros((12, 4, 6, 3)),
                  support_torque_offsets=np.zeros((12, 4, 6)),
                  support_torque_limits=np.ones((12, 4, 6)))
    bundle[field] = value
    with pytest.raises(ValueError):
        PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: c).solve(replace(standing(), **bundle))
    assert c.calls == 0


@pytest.mark.parametrize('backend', ['jax_mppi', 'acados'])
def test_torque_controller_supplies_its_actual_bias_without_mutating_the_observation(backend):
    c, state = controller()
    c._solver.backend = backend
    captured = []
    c._solver.solve = lambda value: (captured.append(value) or np.tile([0., 0., 50.], (4, 1)))
    velocity = np.tile([.1, -.2, .3], 4)
    c._torques(state, state.feet_world, np.zeros((4, 3)),
        np.tile(JACOBIAN, (4, 1, 1)), velocity)
    assert state.support_torque_maps is None
    actual = captured[0]
    if backend == 'acados':
        assert actual.support_torque_maps is None
    else:
        force = c._forces*c._contact_gait.load[:, None]*c._torque_scale()
        torque = np.einsum('ijk,ik->ij', actual.support_torque_maps[0], force)+actual.support_torque_offsets[0]
        expected = np.concatenate((c._tau_raw.reshape(4, 3), c._tau_limited.reshape(4, 3)), axis=1)
        np.testing.assert_allclose(torque, expected, atol=1e-12)


def test_force_history_cannot_reintroduce_a_now_infeasible_motor_load():
    c, state = controller()
    c._base_velocity_world.fill(0.)
    c._step_i = 2
    c._config.force_filter_alpha = .35
    c._tau_lim[:] = 1.
    c._forces[:] = [0., 0., 40.]
    c._solver.solve = lambda value: np.zeros((4, 3))
    c._torques(state, state.feet_world, np.zeros((4, 3)),
        np.tile(np.eye(3), (4, 1, 1)), np.zeros(12))
    np.testing.assert_allclose(c._forces[:, 2], .65, atol=1e-12)
