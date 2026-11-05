import ast
from pathlib import Path
import numpy as np
import pytest
from elesim_sim.rl.execution import MotionClock, snapshot
from elesim_sim.rl.trajectory import Trajectory
from elesim_sim.rl.replay import replay
from elesim_sim.rl.scene import PhysicsConfig


def test_ramp_preserves_legacy_macro_timing():
    commands = list(MotionClock(.01, 40, .6).targets(np.zeros(2), np.ones(2)))
    assert len(commands) == 40
    np.testing.assert_allclose(commands[0][1], [1/24, 1/24])
    np.testing.assert_allclose(commands[23][1], [1, 1])
    np.testing.assert_allclose(commands[-1][1], [1, 1])


def test_terminal_snapshot_survives_in_place_reset():
    source = {'state': np.array([1., 2.]), 'ids': [np.array([3])]}
    saved = snapshot(source)
    source['state'][:] = 0
    source['ids'][0][:] = 0
    assert saved['state'].tolist() == [1., 2.]
    assert saved['ids'][0].tolist() == [3]


def test_trajectory_roundtrip_refuses_overwrite(tmp_path):
    path = tmp_path/'trace.npz'
    trace = Trajectory([0, .2], [[0], [2]], ('joint_rad',), {'frame': 'joint'})
    trace.save(path)
    np.testing.assert_allclose(Trajectory.load(path).sample([0, .1, .2]), [[0], [1], [2]])
    with pytest.raises(FileExistsError):
        trace.save(path)


@pytest.mark.parametrize('times,commands', [([0, 0], [[0], [1]]), ([1, 2], [[0], [1]]),
                                           ([0, 1], [[0], [np.nan]]), ([0, 1], [[0, 1], [1, 2]])])
def test_invalid_trajectory(times, commands):
    with pytest.raises(ValueError):
        Trajectory(times, commands, ('joint',)).validate()


class Target:
    def reset(self, metadata):
        self.value = 0
        self.ticks = 0
    def command(self, channels, values):
        self.value = float(values[0])
    def step(self):
        self.ticks += 1
    def measure(self):
        return {'q': self.value}
    def evaluate(self, samples):
        return {'success': self.ticks == 2 and samples[-1]['state']['q'] == 2}


def test_replay_uses_command_time_not_policy_and_includes_endpoint():
    target = Target()
    result = replay(Trajectory([0, .2], [[0], [2]], ('q',)), target, dt=.1)
    assert result['evaluation']['success']
    assert [s['state']['q'] for s in result['samples']] == [0, 1, 2]
    with pytest.raises(ValueError):
        replay(Trajectory([0, .25], [[0], [2]], ('q',)), target, dt=.1)


def test_runtime_rl_does_not_import_external_tasks():
    import elesim_sim.rl
    root = Path(elesim_sim.rl.__file__).parent
    for path in root.rglob('*.py'):
        for node in ast.walk(ast.parse(path.read_text())):
            names = [n.name for n in node.names] if isinstance(node, ast.Import) else ([node.module or ''] if isinstance(node, ast.ImportFrom) else [])
            assert not any(name.startswith('workbench') for name in names), path
    assert not (root/'envs').exists()
    assert not (root/'configs').exists()


@pytest.mark.parametrize('kwargs', [dict(dt=0), dict(substeps=0), dict(dt=float('nan')), dict(max_contacts=0), dict(max_contacts=1.5)])
def test_invalid_physics(kwargs):
    with pytest.raises(ValueError):
        PhysicsConfig(**kwargs)


def test_runner_accepts_external_environment_without_discovery(monkeypatch, tmp_path):
    import sys
    from types import SimpleNamespace
    from elesim_sim.rl.runner import make_runner
    calls = []
    def construct(*args, **kwargs):
        calls.append((args, kwargs))
        return 'runner'
    monkeypatch.setitem(sys.modules, 'rsl_rl.runners', SimpleNamespace(OnPolicyRunner=construct))
    env = SimpleNamespace(device='cpu', num_envs=2, num_actions=7)
    assert make_runner(env, {'seed': 42}, tmp_path) == 'runner'
    assert calls[0][0] == (env, {'seed': 42})
    assert calls[0][1] == {'log_dir': str(tmp_path), 'device': 'cpu'}


def test_unserializable_metadata_leaves_no_artifact(tmp_path):
    trace = Trajectory([0, 1], [[0], [1]], ('q',), {'bad': float('nan')})
    path = tmp_path/'bad.npz'
    with pytest.raises(ValueError):
        trace.save(path)
    assert not path.exists()


def test_evaluation_runner_does_not_create_a_literal_none_log_directory(monkeypatch):
    import sys
    from types import SimpleNamespace
    from elesim_sim.rl.runner import make_runner
    calls = []
    def construct(*args, **kwargs):
        calls.append(kwargs)
    monkeypatch.setitem(sys.modules, 'rsl_rl.runners', SimpleNamespace(OnPolicyRunner=construct))
    make_runner(SimpleNamespace(device='cpu'), {}, None)
    assert calls[0]['log_dir'] is None


def test_npz_arrays_preserve_float32_without_pickle(tmp_path):
    trace = Trajectory([0, 1], np.array([[0, 1], [2, 3]], dtype=np.float32), ('a', 'b'),
                       {'case': 'example'}, {'replay_success': True})
    path = tmp_path/'trajectory.npz'
    trace.save(path)
    with np.load(path, allow_pickle=False) as data:
        assert data['commands'].dtype == np.float32
        assert data['schema_version'].item() == 2
        assert all(data[key].dtype.kind != 'O' for key in data.files)
    loaded = Trajectory.load(path)
    assert loaded.metadata == trace.metadata
    assert loaded.diagnostics == trace.diagnostics
    assert loaded.commands.dtype == np.float32


def test_legacy_json_can_be_migrated_but_new_json_writes_are_rejected(tmp_path):
    import json
    path = tmp_path/'legacy.json'
    path.write_text(json.dumps(dict(schema='elesim-command-trajectory', schema_version=1,
                                   times_s=[0, 1], commands=[[0], [1]], channels=['q'])))
    trace = Trajectory.load(path)
    trace.save(tmp_path/'converted.npz')
    with pytest.raises(ValueError, match='npz'):
        trace.save(tmp_path/'new.json')
