"""Input transport changes must preserve MPPI arithmetic and device state."""
from types import SimpleNamespace

import numpy as np
import pytest

from elesim_sim.robot.go2.pympc_jax import _MppiInputLayout
from elesim_sim.robot.go2.pympc_solver import PyMpcForceSolver
from workbench.tests.apps.sim.scenarios.go2.test_53_pympc_gpu import cpu_math, standing


def test_each_solve_uploads_one_host_buffer_and_keeps_mean_and_key_on_device(cpu_math):
    adapter = PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: cpu_math)
    uploads = []
    put = cpu_math._jax.device_put

    def record(value, device):
        uploads.append(value)
        return put(value, device)

    cpu_math._jax = SimpleNamespace(device_put=record)
    for time in (0., .02, .04):
        uploads.clear()
        adapter.solve(standing(time=time))
        assert len(uploads) == 1
        packet = uploads[0]
        assert isinstance(packet, np.ndarray)
        assert packet.dtype == np.float32 and packet.ndim == 1
        assert packet.flags.c_contiguous and packet.nbytes == 6972
        assert cpu_math._mean.device == cpu_math.device
        assert cpu_math._key.device == cpu_math.device
        assert cpu_math._key.dtype == np.uint32


@pytest.mark.parametrize('horizon', [2, 12, 64])
@pytest.mark.parametrize('use_mean', [False, True])
def test_packet_round_trip_preserves_each_field_and_owns_its_storage(horizon, use_mean):
    shapes = ((12,), (12,), (horizon, 4, 3), (horizon, 4), (horizon, 4), (),
              (3, 3), (3, 3), (), (), (4, 3), (), (), (4,),
              (horizon, 4, 6, 3), (horizon, 4, 6), (horizon, 4, 6))
    values = [np.asfortranarray(np.arange(int(np.prod(shape)), dtype=np.float32)
                               .reshape(shape) + i*1000).reshape(shape)
              for i, shape in enumerate(shapes)]
    values[8] = np.bool_(use_mean)
    layout = _MppiInputLayout(horizon)
    packet = layout.pack(values)
    original = packet.copy()
    assert packet.ndim == 1 and packet.flags.c_contiguous
    assert packet.dtype == np.float32
    for expected, actual in zip(values, layout.unpack(packet)):
        np.testing.assert_array_equal(actual, np.asarray(expected, dtype=np.float32))
    values[0][0] = -999.
    layout.pack(values)
    np.testing.assert_array_equal(packet, original)


def test_packed_kernel_matches_original_inputs_through_warm_shift_and_fresh_start(cpu_math):
    """Reference uses the unchanged 19-argument kernel, with its own key/mean."""
    jax = cpu_math._jax
    original = jax.jit(cpu_math._make_optimizer(.55, 180.), device=cpu_math.device)
    raw_mean = jax.device_put(np.zeros((12, 4, 3), dtype=np.float32), cpu_math.device)
    packed_mean = raw_mean
    raw_key = jax.device_put(np.array([0, 2**31-1], dtype=np.uint32), cpu_math.device)
    packed_key = raw_key
    for step, (elapsed, use_mean) in enumerate(((0., False), (.01, True), (.06, True),
                                               (.25, False), (.02, True))):
        feet = np.tile(standing().feet_world-[0., 0., .3], (12, 1, 1)).astype(np.float32)
        contacts = np.ones((12, 4), dtype=np.float32)
        contacts[4:8, [1, 2]] = 0.
        feet[8:, [1, 2], 0] += .08
        loads = contacts.copy()
        loads[:3, 0] *= 1/3
        inertia = np.diag([.2, .5, .5]).astype(np.float32)*(1.+.1*step)
        values = (
            np.array([0., 0., 0., .15, -.03, .01, .03, -.05, .02, .04, .01, -.02], dtype=np.float32),
            np.array([.02, 0., 0., .3, 0., 0., 0., 0., 0., 0., 0., 0.], dtype=np.float32),
            feet, contacts, loads, np.float32(18.+step), inertia, np.linalg.inv(inertia),
            np.bool_(use_mean), np.float32(elapsed),
            np.array([[.2, .05, 0.], [.2, -.05, 0.], [-.2, .05, 0.], [-.2, -.05, 0.]], dtype=np.float32),
            np.float32(step % 2), np.float32(5.), np.array([0., 0., 1., 0.], dtype=np.float32),
            np.tile(np.vstack((-.05*np.eye(3), -.1*np.eye(3))), (12, 4, 1, 1)).astype(np.float32),
            np.full((12, 4, 6), .1*step, dtype=np.float32),
            np.full((12, 4, 6), 18., dtype=np.float32),
        )
        # Explicit original ordering: mean/key never enter the float packet.
        args = (*values[:8], raw_mean, values[8], values[9], raw_key, *values[10:])
        raw_mean, raw_key, raw_cost = original(*args)
        packet = jax.device_put(cpu_math._input_layout.pack(values), cpu_math.device)
        packed_mean, packed_key, packed_cost = cpu_math._optimize(packet, packed_mean, packed_key)
        np.testing.assert_allclose(packed_mean, raw_mean, rtol=2e-6, atol=2e-5)
        np.testing.assert_allclose(packed_cost, raw_cost, rtol=2e-6, atol=2e-5)
        np.testing.assert_array_equal(packed_key, raw_key)


def test_packet_rejects_missing_fields():
    with pytest.raises(ValueError, match='17 host fields'):
        _MppiInputLayout(12).pack(())
