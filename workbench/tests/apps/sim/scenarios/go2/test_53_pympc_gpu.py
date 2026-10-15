"""GPU adapter contracts and CPU-executed JAX math, not GPU acceptance."""
from dataclasses import replace

import numpy as np
import pytest

from elesim_sim.robot.go2.pympc_solver import PyMpcForceSolver, PyMpcInput, support_load_horizon
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


def test_support_load_horizon_matches_contact_gate_ramp_and_liftoff():
    # Stage zero is the load that will multiply the current GRF in _torques.
    # A future landing starts the same 60 ms ramp only after its first stance.
    contacts = np.array([[1., 1., 1., 1., 0.],
                         [0., 0., 1., 1., 1.],
                         [1., 1., 0., 1., 1.],
                         [0., 0., 0., 0., 0.]])
    loads = support_load_horizon(contacts, np.array([1/3, 0., 1., 0.]), .02)
    np.testing.assert_allclose(loads[:, 0], [1/3, 2/3, 1., 1., 0.])
    np.testing.assert_allclose(loads[:, 1], [0., 0., 1/3, 2/3, 1.])
    np.testing.assert_allclose(loads[:, 2], [1., 1., 0., 1/3, 2/3])
    np.testing.assert_array_equal(loads[:, 3], 0.)


def test_gpu_passes_measured_support_load_without_changing_cpu_contract():
    class RecordingReferenceBackend(RecordingBackend):
        def compute_control(self, state, reference, contacts, **parameters):
            self.reference = reference
            return super().compute_control(state, reference, contacts, **parameters)

    sample = replace(standing(), support_loads=np.array([1/3, 1., 1., 0.]))
    gpu = RecordingReferenceBackend()
    PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: gpu).solve(sample)
    np.testing.assert_allclose(gpu.reference['support_loads'][0], sample.support_loads)
    np.testing.assert_allclose(gpu.reference['support_loads'][1, 0], 2/3)
    cpu = RecordingReferenceBackend()
    PyMpcForceSolver(solver_factory=lambda: cpu).solve(sample)
    assert 'support_loads' not in cpu.reference


def test_gpu_passes_support_force_floor_without_changing_cpu_contract():
    class ReferenceBackend(RecordingBackend):
        def compute_control(self, state, reference, contacts, **parameters):
            self.reference = reference
            return super().compute_control(state, reference, contacts, **parameters)
    sample = replace(standing(), support_force_floor_n=10.)
    gpu = ReferenceBackend()
    PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: gpu).solve(sample)
    assert gpu.reference['support_force_floor_n'] == 10.
    cpu = ReferenceBackend()
    PyMpcForceSolver(solver_factory=lambda: cpu).solve(sample)
    assert 'support_force_floor_n' not in cpu.reference


@pytest.mark.parametrize('floor', [-1., np.nan, 181.])
def test_bad_support_force_floor_is_rejected_before_gpu_dispatch(floor):
    fake = RecordingBackend()
    with pytest.raises(ValueError, match='support_force_floor_n'):
        PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: fake).solve(
            replace(standing(), support_force_floor_n=floor))
    assert not fake.received


def test_gpu_receives_search_reach_mask_without_promising_ground_force():
    class ReferenceBackend(RecordingBackend):
        def compute_control(self, state, reference, contacts, **parameters):
            self.reference = reference
            return super().compute_control(state, reference, contacts, **parameters)
    sample = replace(standing(), landing_search=np.array([0., 0., 1., 1.]))
    sample.contacts[2:] = 0.
    gpu = ReferenceBackend()
    force = PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: gpu).solve(sample)
    np.testing.assert_array_equal(gpu.reference['landing_search'], sample.landing_search)
    np.testing.assert_array_equal(force[2:], 0.)
    cpu = ReferenceBackend()
    PyMpcForceSolver(solver_factory=lambda: cpu).solve(sample)
    assert 'landing_search' not in cpu.reference


@pytest.mark.parametrize('search', [np.ones(3), np.array([0., 0., np.nan, 1.]),
                                   np.array([0., 0., -1., 0.])])
def test_invalid_landing_search_rejected_before_gpu_dispatch(search):
    fake = RecordingBackend()
    with pytest.raises(ValueError, match='landing_search'):
        PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: fake).solve(
            replace(standing(), landing_search=search))
    assert not fake.received


def test_unlanded_rear_feet_still_limit_body_extension_without_receiving_grf(cpu_math):
    from scipy.spatial.transform import Rotation
    from elesim_sim.robot.go2.locomotion.kinematics import HIP_OFFSET_BODY
    from elesim_sim.robot.go2.locomotion.types import ALL_LEGS
    # Rounded GPU2 failure geometry: the nose-down body lifts the rear hips
    # until a ground target exceeds the calf extension limit. Arithmetic only.
    position = np.array([1.47175, .02996, .30479])
    com = np.array([1.46515, .02835, .28155])
    rpy = np.array([.0414, .3895, .0489])
    feet = np.array([[1.42832, .23518, .02136], [1.60777, -.16167, .02251],
                     [1.25748, .16997, .02530], [1.14572, -.11029, .03005]])
    targets = feet.copy()
    targets[2:] = [[1.25588, .17304, .025], [1.12638, -.11198, .025]]
    hips = np.array([HIP_OFFSET_BODY[leg] for leg in ALL_LEGS])
    hips += (Rotation.from_euler('xyz', rpy).as_matrix().T @ (position-com))[None]
    sample = replace(standing(mass=15.881), com_position=com,
                     com_velocity=np.array([.3216, .0944, .0793]), rpy=rpy,
                     desired_com_position=np.array([1.46515, .02835, .27845]),
                     feet_world=feet, footholds_world=targets, hip_offsets_com_body=hips,
                     support_force_floor_n=10.)
    sample.contacts[2:] = 0.
    adapter = PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: cpu_math)
    ignored = adapter.solve(sample)
    adapter.reset()
    aware = adapter.solve(replace(sample, landing_search=np.array([0., 0., 1., 1.])))
    assert aware[:, 2].sum() < ignored[:, 2].sum()-1., (ignored, aware)
    np.testing.assert_array_equal(aware[2:], 0.)


@pytest.mark.parametrize('swing_leg', [None, 1])
def test_mppi_retains_ground_load_after_landing_without_loading_swing(cpu_math, swing_leg):
    from elesim_sim.robot.go2.foot_workspace import foot_position_body
    base = np.array([0., 0., .37])
    feet = base + np.array([foot_position_body(i, [0., .9, -.9]) for i in range(4)])
    sample = replace(standing(mass=15.6), com_position=base+[.02, 0., -.02],
                     desired_com_position=base+[.02, 0., -.02],
                     feet_world=feet, footholds_world=feet.copy(),
                     support_loads=np.array([1., 1., 1/3, 1.]))
    if swing_leg is not None:
        sample.contacts[swing_leg] = 0.
        sample.support_loads[swing_leg] = 0.
    adapter = PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: cpu_math)
    ignored = adapter.solve(sample)
    support = sample.contacts[:, 0].astype(bool)
    assert np.min(ignored[support, 2]*sample.support_loads[support]) < 5.
    adapter.reset()
    force = adapter.solve(replace(sample, support_force_floor_n=10.))
    # The torque path uses the ramped force, rather than the uncapped raw
    # request. Do not cancel a just-confirmed landing with a near-zero GRF.
    assert np.all(force[support, 2]*sample.support_loads[support] >= 10.-1e-4)
    if swing_leg is not None:
        np.testing.assert_array_equal(force[swing_leg], 0.)


@pytest.mark.parametrize('support_loads', [np.array([-0.1, 1., 1., 1.]),
                                                   np.array([1.1, 1., 1., 1.]),
                                                   np.array([np.nan, 1., 1., 1.])])
def test_invalid_support_load_rejected_before_gpu_dispatch(support_loads):
    fake = RecordingBackend()
    with pytest.raises(ValueError, match='support_loads'):
        PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: fake).solve(
            replace(standing(), support_loads=support_loads))
    assert not fake.received


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


@pytest.mark.parametrize("count", [1, 3])
def test_visible_gpus_select_first_without_restricting_visibility(monkeypatch, count):
    from types import SimpleNamespace
    jax = pytest.importorskip('jax')
    devices = [SimpleNamespace(platform="gpu", device_kind="test GPU", id=i)
               for i in range(count)]
    monkeypatch.setattr(jax, 'devices', lambda _: devices)
    selected = []
    monkeypatch.setattr(jax, 'jit', lambda function, *, device: selected.append(device))
    monkeypatch.setattr(jax, 'device_put', lambda value, device: value)
    monkeypatch.setattr(JaxMppiSolver, 'compute_control', lambda *args, **kwargs: None)
    solver = JaxMppiSolver(horizon=12, dt=.02, friction=.55, max_normal_force_n=180.,
                           samples=64, iterations=1, seed=42)
    assert solver.device is devices[0]
    assert selected == [devices[0]]


def test_empty_gpu_inventory_reports_no_gpu(monkeypatch):
    jax = pytest.importorskip('jax')
    monkeypatch.setattr(jax, 'devices', lambda _: [])
    with pytest.raises(RuntimeError, match='working CUDA GPU'):
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


def test_landing_load_is_used_when_allocating_vertical_support(cpu_math):
    adapter = PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: cpu_math)
    # A forward moving, pitched body with a newly landed front-left foot.
    # Scoring full requested GRF overestimates support from that foot.
    sample = replace(standing(), com_position=np.array([.1, 0., .3]),
                     com_velocity=np.array([.8, 0., 0.]),
                     rpy=np.array([0., .3, 0.]),
                     angular_velocity_body=np.array([0., .5, 0.]),
                     desired_com_position=np.array([.15, 0., .3]),
                     desired_com_velocity=np.array([.8, 0., 0.]))
    sample.contacts[3] = 0.
    loads = np.array([1/3, 1., 1., 0.])
    ignored = adapter.solve(sample)
    adapter.reset()
    aware = adapter.solve(replace(sample, support_loads=loads))
    old_applied = float(np.dot(ignored[:, 2], loads))
    new_applied = float(np.dot(aware[:, 2], loads))
    assert new_applied > old_applied + 20., (ignored, aware, old_applied, new_applied)
    assert new_applied > 150.


def test_supported_legs_near_extension_make_mppi_lower_the_predicted_body(cpu_math):
    from elesim_sim.robot.go2.foot_workspace import foot_position_body
    from elesim_sim.robot.go2.locomotion.kinematics import HIP_OFFSET_BODY
    from elesim_sim.robot.go2.locomotion.types import ALL_LEGS
    # The long GPU2 traces reach the calf's -0.83776 extension limit after
    # body roll/height makes support unreachable. Start 20 mm before that
    # limit with a height reference that alone would retain this posture.
    base = np.array([0., 0., .37])
    com_offset = np.array([.02, 0., -.02])
    feet = base + np.array([foot_position_body(leg, [0., .9, -.9]) for leg in range(4)])
    hips = np.array([HIP_OFFSET_BODY[leg] for leg in ALL_LEGS])-com_offset
    sample = replace(standing(mass=15.6), com_position=base+com_offset,
                     desired_com_position=base+com_offset,
                     feet_world=feet, footholds_world=feet.copy())
    adapter = PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: cpu_math)
    ignored = adapter.solve(sample)
    adapter.reset()
    aware = adapter.solve(replace(sample, hip_offsets_com_body=hips))
    # Confirm that the actual optimizer scoring path changes the force,
    # rather than merely exposing a disconnected geometry calculation.
    assert aware[:, 2].sum() < ignored[:, 2].sum()-15., (ignored, aware)
    z, vz = float(sample.com_position[2]), 0.
    for stage in np.asarray(cpu_math._mean):
        acceleration = stage[:, 2].sum()/sample.mass_kg-9.81
        z += vz*cpu_math.dt+.5*acceleration*cpu_math.dt**2
        vz += acceleration*cpu_math.dt
    assert z < sample.com_position[2]-.01


def test_support_extension_penalty_matches_urdf_limit_and_ignores_swing():
    from pathlib import Path
    import xml.etree.ElementTree as ET
    from elesim_sim.robot.go2.pympc_jax import _support_extension_cost
    from elesim_sim.robot.go2.foot_workspace import foot_position_body
    from elesim_sim.robot.go2.locomotion.kinematics import HIP_OFFSET_BODY
    from elesim_sim.robot.go2.locomotion.types import ALL_LEGS
    root = next(p for p in Path(__file__).resolve().parents if (p/'payload').is_dir())
    urdf = ET.parse(root/'payload/data/models/assemblies/zed-mini/assets/go2/go2.urdf')
    limit = float(urdf.find(".//joint[@name='FL_calf_joint']/limit").get('upper'))-.04
    hips = np.array([HIP_OFFSET_BODY[leg] for leg in ALL_LEGS])
    def vectors(calf, hip):
        return np.array([foot_position_body(i, [hip, -calf/2, calf]) for i in range(4)])-hips
    # Hip abduction changes direction, not the two-link extension radius.
    nominal = vectors(-1.8, .2)
    assert _support_extension_cost(np, nominal, np.ones(4)) == 0.
    straight = vectors(limit, .3)
    np.testing.assert_allclose(_support_extension_cost(np, straight, np.ones(4)), 800., rtol=1e-5)
    np.testing.assert_allclose(_support_extension_cost(np, vectors(limit, -.3), np.ones(4)), 800., rtol=1e-5)
    assert _support_extension_cost(np, straight, np.zeros(4)) == 0.
    np.testing.assert_allclose(_support_extension_cost(np, straight, [1., .5, 0., 0.]), 300., rtol=1e-5)


def test_gpu_receives_com_relative_hip_geometry_without_changing_cpu_contract():
    class GeometryBackend(RecordingBackend):
        def compute_control(self, state, reference, contacts, **parameters):
            self.reference = reference
            return super().compute_control(state, reference, contacts, **parameters)
    hips = np.arange(12, dtype=float).reshape(4, 3)/100.
    sample = replace(standing(), hip_offsets_com_body=hips)
    gpu = GeometryBackend()
    PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: gpu).solve(sample)
    np.testing.assert_array_equal(gpu.reference['hip_offsets_com_body'], hips)
    cpu = GeometryBackend()
    PyMpcForceSolver(solver_factory=lambda: cpu).solve(sample)
    assert 'hip_offsets_com_body' not in cpu.reference


@pytest.mark.parametrize('hips', [np.zeros((3, 3)), np.full((4, 3), np.nan)])
def test_bad_hip_geometry_is_rejected_before_gpu_dispatch(hips):
    fake = RecordingBackend()
    with pytest.raises(ValueError, match='hip_offsets_com_body'):
        PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: fake).solve(
            replace(standing(), hip_offsets_com_body=hips))
    assert not fake.received


def test_rising_body_recovers_height_despite_horizontal_command_and_warm_start(cpu_math):
    """An old propulsive solution must not keep launching an elevated body."""
    adapter = PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: cpu_math)
    sample = replace(standing(), com_position=np.array([0., 0., .36]),
                     com_velocity=np.array([0., 0., .25]),
                     desired_com_velocity=np.array([.8, 0., 0.]))
    cpu_math._mean = cpu_math._jax.device_put(
        np.tile([20., 0., 55.], (12, 4, 1)).astype(np.float32), cpu_math.device)
    forces = adapter.solve(sample)
    z, vz = sample.com_position[2], sample.com_velocity[2]
    heights = []
    for stage in np.asarray(cpu_math._mean):
        acceleration = stage[:, 2].sum() / sample.mass_kg - 9.81
        z += vz * cpu_math.dt + .5 * acceleration * cpu_math.dt**2
        vz += acceleration * cpu_math.dt
        heights.append(z)
    # At +0.25 m/s some further rise is unavoidable. Bound it to 2 cm and
    # require descent by the end of the 240 ms prediction horizon.
    assert max(heights) < .38 and vz < 0., (forces, heights, vz)


def test_future_stance_support_uses_preview_only_after_liftoff():
    from types import SimpleNamespace
    from elesim_sim.robot.go2.pympc_controller import contact_schedule
    from elesim_sim.robot.go2.pympc_jax import JaxMppiSolver
    adapter = JaxMppiSolver.__new__(JaxMppiSolver)
    adapter.horizon, adapter.dt = 12, .02
    adapter._jax = SimpleNamespace(device_put=lambda value, _device: value)
    adapter.device = object()
    adapter._key = np.array([0, 42], dtype=np.uint32)
    adapter._mean = None
    seen = {}

    def optimize(x, ref, feet, contacts, *rest):
        seen['feet'] = np.asarray(feet)
        seen['contacts'] = np.asarray(contacts)
        return np.zeros((12, 4, 3)), adapter._key, 0.

    adapter._optimize = optimize
    nominal = standing()
    schedule = contact_schedule(0., gait_hz=2.5, duty=.6, dt=.02, horizon=12)
    state = dict(position=nominal.com_position, linear_velocity=nominal.com_velocity,
                 orientation=nominal.rpy, angular_velocity=nominal.angular_velocity_body)
    ref = dict(ref_position=nominal.desired_com_position,
               ref_linear_velocity=nominal.desired_com_velocity,
               ref_orientation=nominal.desired_rpy,
               ref_angular_velocity=nominal.desired_angular_velocity_body)
    for leg, foot in zip(('FL','FR','RL','RR'), nominal.feet_world):
        state[f'foot_{leg}'] = foot
        ref[f'ref_foot_{leg}'] = foot[None]
        ref[f'preview_foot_{leg}'] = foot[None] + [0.2, 0., 0.]
    adapter.compute_control(state, ref, schedule, mass=18.,
                            inertia=np.diag([.2,.5,.5]).reshape(9), elapsed_s=0.)
    origin = nominal.com_position
    np.testing.assert_allclose(seen['feet'][0, 1], nominal.feet_world[1]-origin, atol=1e-6)
    np.testing.assert_allclose(seen['feet'][-1, 1], nominal.feet_world[1]+[.2,0,0]-origin, atol=1e-6)
    np.testing.assert_allclose(seen['feet'][:, 0],
                               np.tile(nominal.feet_world[0]-origin, (12,1)), atol=1e-6)


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


def test_large_world_translation_does_not_quantize_lever_arms(cpu_math):
    adapter = PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: cpu_math)
    sample = replace(standing(), rpy=np.array([.1, .05, 0.]),
                     desired_com_velocity=np.array([.2, 0., 0.]))
    force = adapter.solve(sample)
    adapter.reset()
    offset = np.array([100000., -100000., 1000.])
    translated = replace(sample, com_position=sample.com_position+offset,
                         desired_com_position=sample.desired_com_position+offset,
                         feet_world=sample.feet_world+offset,
                         footholds_world=sample.footholds_world+offset)
    np.testing.assert_allclose(adapter.solve(translated), force, atol=1e-4)
