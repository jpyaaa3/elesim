"""GPU adapter contracts and CPU-executed JAX math, not GPU acceptance."""
from dataclasses import replace

import numpy as np
import pytest

from elesim_sim.robot.go2.pympc_solver import PyMpcForceSolver, PyMpcInput
from elesim_sim.robot.go2.pympc_jax import JaxMppiSolver


def standing(mass=18., time=0.):
    feet = np.array([[.2, .12, 0.], [.2, -.12, 0.], [-.2, .12, 0.], [-.2, -.12, 0.]])
    return PyMpcInput(np.array([0., 0., .3]), np.zeros(3), np.zeros(3), np.zeros(3),
                      feet, np.array([0., 0., .3]), np.zeros(3), np.zeros(3), np.zeros(3),
                      feet.copy(), np.ones((4, 12)), mass, np.diag([.2, .5, .5]), time)


class RecordingBackend:
    def __init__(self):
        self.received = []
        self.resets = 0

    def reset(self):
        self.resets += 1

    def compute_control(self, state, reference, contacts, **parameters):
        self.received.append(parameters)
        return np.tile([0., 0., 50.], 4), None, None, 0


def test_backend_selection_preserves_cpu_default():
    fake = RecordingBackend()
    cpu = PyMpcForceSolver(solver_factory=lambda: fake)
    cpu.solve(standing())
    assert cpu.backend == 'acados'
    assert 'elapsed_s' not in fake.received[-1]
    with pytest.raises(ValueError, match='unknown'):
        PyMpcForceSolver(backend='automatic', solver_factory=lambda: fake)


def test_gpu_receives_payload_and_actual_elapsed_time_then_reset():
    fake = RecordingBackend()
    solver = PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: fake)
    solver.solve(standing(time=1.))
    solver.solve(standing(mass=23., time=1.02))  # support change before periodic solve
    assert fake.received[0]['elapsed_s'] == 0.
    assert fake.received[1]['elapsed_s'] == pytest.approx(.02)
    assert fake.received[1]['mass'] == 23.
    np.testing.assert_array_equal(fake.received[1]['inertia'], np.diag([.2, .5, .5]).reshape(9))
    with pytest.raises(ValueError, match='backwards'):
        solver.solve(standing(time=0.))
    solver.reset()
    solver.solve(standing(time=0.))
    assert fake.resets == 1 and fake.received[-1]['elapsed_s'] == 0.


@pytest.mark.parametrize('field,value', [('samples', 0), ('samples', 100000), ('samples', True),
                                         ('iterations', 0), ('iterations', 1.5), ('seed', -1)])
def test_sampling_budget_rejected_before_initializing_jax(field, value):
    args = dict(horizon=12, dt=.02, friction=.55, max_normal_force_n=180.,
                samples=64, iterations=1, seed=42)
    args[field] = value
    with pytest.raises(ValueError):
        JaxMppiSolver(**args)


def test_explicit_gpu_request_does_not_fall_back_to_cpu(monkeypatch):
    jax = pytest.importorskip('jax')
    def no_gpu(_):
        raise RuntimeError('no cuda')
    monkeypatch.setattr(jax, 'devices', no_gpu)
    with pytest.raises(RuntimeError, match='CPU fallback is disabled'):
        JaxMppiSolver(horizon=12, dt=.02, friction=.55, max_normal_force_n=180.,
                      samples=64, iterations=1, seed=42)


def test_multiple_visible_gpus_require_explicit_device_selection(monkeypatch):
    jax = pytest.importorskip('jax')
    monkeypatch.setattr(jax, 'devices', lambda _: [object(), object()])
    with pytest.raises(RuntimeError, match='exactly one'):
        JaxMppiSolver(horizon=12, dt=.02, friction=.55, max_normal_force_n=180.,
                      samples=64, iterations=1, seed=42)


@pytest.fixture
def cpu_math():
    # Deliberately bypass GPU selection ONLY for arithmetic unit coverage.
    jax = pytest.importorskip('jax')
    import jax.numpy as jnp
    kernel = JaxMppiSolver.__new__(JaxMppiSolver)
    kernel.horizon, kernel.dt = 12, .02
    kernel.samples, kernel.iterations, kernel.seed = 64, 1, 42
    kernel._jax, kernel._jnp = jax, jnp
    kernel.device = jax.devices('cpu')[0]
    kernel._optimize = jax.jit(kernel._make_optimizer(.55, 180.), device=kernel.device)
    kernel.reset()
    return kernel


def test_dynamic_mass_support_bounds_and_reset_determinism(cpu_math):
    adapter = PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: cpu_math)
    light = adapter.solve(standing())
    adapter.reset()
    repeated = adapter.solve(standing())
    np.testing.assert_array_equal(light, repeated)
    np.testing.assert_allclose(light.sum(axis=0), [0., 0., 18*9.81], atol=.1)
    adapter.reset()
    heavy = adapter.solve(standing(mass=28.))
    np.testing.assert_allclose(heavy.sum(axis=0), [0., 0., 28*9.81], atol=.1)
    diagonal = standing(time=.02)
    diagonal.contacts[[1, 2]] = 0.
    forces = adapter.solve(diagonal)
    np.testing.assert_array_equal(forces[[1, 2]], 0.)
    assert np.all(forces[:, 2] >= 0.) and np.all(forces[:, 2] <= 180.)
    assert np.all(np.abs(forces[:, :2]) <= .55*forces[:, 2:3] + 1e-6)
    assert np.isfinite(cpu_math.last_cost)


def test_dynamic_inertia_changes_solution_without_recompiling(cpu_math):
    adapter = PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: cpu_math)
    sample = replace(standing(), rpy=np.array([.1, .15, 0.]))
    first = adapter.solve(sample)
    cache_size = cpu_math._optimize._cache_size()
    adapter.reset()
    second = adapter.solve(replace(sample, inertia_body=sample.inertia_body*2.))
    assert not np.allclose(first, second)
    assert cpu_math._optimize._cache_size() == cache_size


def test_bad_inertia_is_rejected_before_gpu_dispatch():
    fake = RecordingBackend()
    adapter = PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: fake)
    inertia = np.diag([.2, .5, .5]); inertia[0, 1] = 1.
    with pytest.raises(ValueError, match='inertia'):
        adapter.solve(replace(standing(), inertia_body=inertia))
    assert not fake.received


def test_acados_iteration_status_is_not_a_gpu_success():
    class FailedGpu(RecordingBackend):
        def compute_control(self, *args, **kwargs):
            return np.zeros(12), None, None, 2
    adapter = PyMpcForceSolver(backend='jax_mppi', solver_factory=FailedGpu)
    with pytest.raises(RuntimeError, match='jax_mppi solve failed'):
        adapter.solve(standing())
