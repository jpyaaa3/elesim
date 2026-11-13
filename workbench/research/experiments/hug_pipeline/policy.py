"""Budget-aware 21-observation PPO weights; legacy 20-input policies stay separate."""
import json
from pathlib import Path
import numpy as np

from workbench.research.experiments.hug_c.configure import OBSERVATIONS as BASE_OBSERVATIONS, ACTIONS

OBSERVATIONS = (*BASE_OBSERVATIONS, "linear_remaining_fraction")
POLICY_VERSION = 1


class NumpyPolicy:
    def __init__(self, path):
        with np.load(path, allow_pickle=False) as data:
            if str(data['schema'].item()) != 'elesim-hug-pipeline-linear-policy' or int(data['schema_version'].item()) != POLICY_VERSION:
                raise ValueError('Not an exported Hug policy')
            if tuple(data['observation_names'].tolist()) != OBSERVATIONS:
                raise ValueError('Observation convention differs')
            if tuple(data['action_names'].tolist()) != ACTIONS:
                raise ValueError('Action convention differs')
            self.metadata = json.loads(str(data['metadata_json'].item()))
            self.mean, self.std = data['norm_mean'].copy(), data['norm_std'].copy()
            self.eps = float(data['norm_eps'])
            self.layers = [(data[f'w{i}'].copy(), data[f'b{i}'].copy()) for i in range(int(data['n_layers']))]
        if self.mean.shape != (len(OBSERVATIONS),) or self.std.shape != (len(OBSERVATIONS),) or not np.isfinite(self.eps) or self.eps <= 0:
            raise ValueError('Invalid normalization')
        if not np.isfinite(self.mean).all() or not np.isfinite(self.std).all() or np.any(self.std < 0):
            raise ValueError('Invalid normalization')
        if not isinstance(self.metadata, dict):
            raise ValueError('Policy metadata must be a mapping')
        width = len(OBSERVATIONS)
        for w, b in self.layers:
            if w.ndim != 2 or w.shape[1] != width or b.shape != (w.shape[0],):
                raise ValueError('Invalid layer dimensions')
            if not np.isfinite(w).all() or not np.isfinite(b).all():
                raise ValueError('Nonfinite weights')
            width = len(b)
        if not self.layers or width != len(ACTIONS):
            raise ValueError('Expected four C-hug actions')

    def __call__(self, observations):
        x = np.atleast_2d(np.asarray(observations, dtype=np.float32))
        if x.ndim != 2 or x.shape[1] != len(OBSERVATIONS) or not np.isfinite(x).all():
            raise ValueError('Expected finite C-hug actor observations')
        x = (x-self.mean)/(self.std+self.eps)
        for index, (w, b) in enumerate(self.layers):
            x = x@w.T+b
            if index+1 < len(self.layers):
                x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))
        if not np.isfinite(x).all():
            raise ValueError('Nonfinite policy output')
        return x


def export_policy(policy, path, metadata):
    """Export pinned rsl_rl ELU MLP; compare NumPy with Torch before publication."""
    import torch
    import tempfile
    path = Path(path)
    if path.suffix.lower() != '.npz':
        raise ValueError('Policy output must end in .npz')
    jit = policy.as_jit().to('cpu').eval()
    # Sequential may reuse one ELU instance in several positions. modules()
    # deduplicates those instances and loses the actual forward ordering.
    leaves = list(jit.mlp)
    if any(not isinstance(m, (torch.nn.Linear, torch.nn.ELU)) for m in leaves):
        raise ValueError('Only Linear/ELU MLP export is supported')
    layers = [m for m in leaves if isinstance(m, torch.nn.Linear)]
    expected = [torch.nn.Linear if i % 2 == 0 else torch.nn.ELU for i in range(2*len(layers)-1)]
    if (len(leaves) != len(expected) or any(not isinstance(m, kind) for m, kind in zip(leaves, expected))
            or any(isinstance(m, torch.nn.ELU) and m.alpha != 1 for m in leaves)):
        raise ValueError('Unsupported activation ordering/alpha')
    norm = jit.obs_normalizer
    arrays = dict(schema=np.asarray('elesim-hug-pipeline-linear-policy'), schema_version=np.asarray(POLICY_VERSION),
                  observation_names=np.asarray(OBSERVATIONS), action_names=np.asarray(ACTIONS), metadata_json=np.asarray(json.dumps(metadata, allow_nan=False)),
                  norm_mean=norm._mean.detach().cpu().numpy().reshape(-1),
                  norm_std=norm._std.detach().cpu().numpy().reshape(-1),
                  norm_eps=np.asarray(float(norm.eps)), n_layers=np.asarray(len(layers)))
    for i, layer in enumerate(layers):
        arrays[f'w{i}'] = layer.weight.detach().cpu().numpy()
        arrays[f'b{i}'] = layer.bias.detach().cpu().numpy()
    with tempfile.TemporaryDirectory() as temporary:
        probe_file = Path(temporary)/'policy.npz'
        np.savez_compressed(probe_file, **arrays)
        loaded = NumpyPolicy(probe_file)
        observations = np.random.default_rng(0).normal(size=(128, len(OBSERVATIONS))).astype(np.float32)
        with torch.inference_mode():
            expected_values = jit(torch.from_numpy(observations)).numpy()
        error = float(np.max(abs(loaded(observations)-expected_values)))
        if not np.isfinite(error) or error > 1e-4:
            raise ValueError(f'NumPy/Torch policy mismatch: {error}')
        arrays['export_max_abs_error'] = np.asarray(error)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('xb') as stream:
            np.savez_compressed(stream, **arrays)
    return error
