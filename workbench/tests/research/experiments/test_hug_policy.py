"""NumPy-only policy format checks. Torch export parity runs during export."""
import json
import numpy as np
import pytest
from workbench.research.experiments.hug_interpolation.policy import NumpyPolicy, OBSERVATIONS
from elesim_sim.rl.trajectory import Trajectory


def policy_file(tmp_path, **changes):
    values = dict(schema=np.asarray('elesim-hug-policy'), schema_version=np.asarray(1),
                  observation_names=np.asarray(OBSERVATIONS), metadata_json=np.asarray(json.dumps({'case_id': 'test'})),
                  norm_mean=np.ones(7, dtype=np.float32), norm_std=np.ones(7, dtype=np.float32),
                  norm_eps=np.asarray(.01), n_layers=np.asarray(2),
                  w0=np.eye(7, dtype=np.float32)[:3], b0=np.zeros(3, dtype=np.float32),
                  w1=np.array([[1, 1, 0], [0, 1, 1]], dtype=np.float32), b1=np.array([.5, -.5], dtype=np.float32))
    values.update(changes)
    path = tmp_path/'policy.npz'; np.savez_compressed(path, **values)
    return path


def test_normalization_elu_and_output_dimensions(tmp_path):
    p = NumpyPolicy(policy_file(tmp_path))
    x = np.array([[0, 2, 1, 1, 1, 1, 1]], dtype=np.float32)
    expected = [[np.expm1(-1/1.01)+1/1.01+.5, 1/1.01-.5]]
    np.testing.assert_allclose(p(x), expected, rtol=1e-6)
    assert p.metadata['case_id'] == 'test'
    with pytest.raises(ValueError):
        p(np.zeros((1, 16)))  # old Wrap observation convention is not this task


@pytest.mark.parametrize('change', [dict(norm_std=-np.ones(7)), dict(w1=np.zeros((4, 3))),
                                    dict(schema=np.asarray('elesim-command-trajectory'))])
def test_reject_wrong_model(tmp_path, change):
    with pytest.raises(ValueError):
        NumpyPolicy(policy_file(tmp_path, **change))


def test_policy_and_trajectory_npz_are_not_interchangeable(tmp_path):
    with pytest.raises(ValueError):
        Trajectory.load(policy_file(tmp_path))
    path = tmp_path/'motion.npz'
    Trajectory([0, 1], [[0], [1]], ('q',)).save(path)
    with pytest.raises(ValueError):
        NumpyPolicy(path)


def test_torch_export_keeps_repeated_activation_positions(tmp_path):
    torch = pytest.importorskip('torch')
    from types import SimpleNamespace
    from workbench.research.experiments.hug_interpolation.policy import export_policy

    class Model(torch.nn.Module):
        def __init__(self):
            super().__init__()
            activation = torch.nn.ELU()
            self.mlp = torch.nn.Sequential(torch.nn.Linear(7, 8), activation,
                                           torch.nn.Linear(8, 8), activation, torch.nn.Linear(8, 2))
            self.obs_normalizer = SimpleNamespace(_mean=torch.zeros(7), _std=torch.ones(7), eps=1e-8)

        def forward(self, observations):
            return self.mlp(observations/(1+self.obs_normalizer.eps))

        def as_jit(self):
            return self

    error = export_policy(Model(), tmp_path/'policy.npz', {'fixture': True})
    assert error < 1e-5
    assert len(NumpyPolicy(tmp_path/'policy.npz').layers) == 3
