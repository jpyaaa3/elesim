import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from elesim_protocol.scene_artifact import validate_obj
from elesim_sim.simulation.scene_rebuild import (
    SceneRebuilder, SceneRollbackError, load_candidate, warmup_mesh_bytes,
)


class Entity:
    n_dofs = 1

    def __init__(self, name, **kwargs):
        self.name = name
        self.desc = NS(morphs=(kwargs.get("morph"),), material=kwargs.get("material"),
            surface=kwargs.get("surface"), visualize_contact=False)
        self.q, self.v = [0.], [0.]
        self.props = {key: [1.] for key in ("kp", "kv", "damping", "armature", "frictionloss")}

    def get_qpos(self): return self.q
    def get_dofs_velocity(self): return self.v
    def set_qpos(self, q): self.q = q.copy()
    def set_dofs_velocity(self, v): self.v = v.copy()
    def __getattr__(self, key):
        if key.startswith("get_dofs_"):
            return lambda: self.props[key[9:]]
        if key.startswith("set_dofs_"):
            return lambda value: self.props.__setitem__(key[9:], value.copy())
        raise AttributeError(key)


class Scene:
    is_built = True

    def __init__(self): self.entities = [Entity("robot", morph=object())]
    def get_entity(self, name): return next(e for e in self.entities if e.name == name)
    def step(self): self.editor.apply()


class Editor:
    def __init__(self, scene): self.scene = scene; scene.editor = self; self._rebuild_pending = False
    def rebuild(self, *, entities_kwargs): self.kwargs = entities_kwargs; self._rebuild_pending = True
    def apply(self):
        self.scene.entities = [Entity(name, **kwargs) for name, kwargs in self.kwargs.items()]
        self._rebuild_pending = False


def fixture():
    scene = Scene()
    old = scene.entities[0]
    old.q, old.v, old.props['kp'] = [1.2], [.3], [9.]
    gs = NS(morphs=NS(Mesh=lambda **kwargs: NS(**kwargs)))
    rebuilder = SceneRebuilder(scene, gs, editor_factory=Editor)
    artifact = NS(objects=[NS(object_id="apple", mesh_file=Path("apple.obj"))])
    return scene, old, rebuilder, artifact


def test_rebuild_changes_handles_and_restores_state_and_gains_after_rebind():
    scene, old, edit, artifact = fixture()
    seen = []
    def rebind():
        current = scene.get_entity('robot')
        seen.append(current)
        current.q, current.props['kp'] = [-9.], [0.]
    entities, elapsed = edit.replace(artifact, rebind=rebind, cameras=lambda a, e: None)
    assert scene.get_entity('robot') is not old and seen == [scene.get_entity('robot')]
    assert scene.get_entity('robot').q == [1.2]
    assert scene.get_entity('robot').v == [.3]
    assert scene.get_entity('robot').props['kp'] == [9.]
    assert list(entities) == ['apple'] and elapsed >= 0
    edit.replace(None, rebind=lambda: None, cameras=lambda a, e: None)
    assert [e.name for e in scene.entities] == ['robot']


def test_camera_failure_restores_old_revision_state_and_bindings():
    scene, old, edit, artifact = fixture()
    calls = []
    def cameras(value, entities):
        calls.append((value, entities))
        if value is artifact: raise RuntimeError('camera startup failed')
    with pytest.raises(RuntimeError, match='previous scene restored'):
        edit.replace(artifact, rebind=lambda: None, cameras=cameras)
    assert calls[-1] == (None, None)
    assert [e.name for e in scene.entities] == ['robot']
    assert scene.get_entity('robot').q == [1.2]
    assert set(edit.current_kwargs) == {'robot'}


def test_session_loss_during_build_rolls_back_and_never_activates_candidate():
    scene, old, edit, artifact = fixture()
    valid = [True]
    calls = []
    def rebind(): valid[0] = False
    with pytest.raises(RuntimeError, match='session changed'):
        edit.replace(artifact, rebind=rebind, cameras=lambda a, e: calls.append((a, e)),
            authorized=lambda: valid[0])
    assert calls == [(None, None)]
    assert scene.get_entity('robot').q == [1.2]


def test_failed_rollback_is_fatal_and_never_commits_metadata():
    scene, old, edit, artifact = fixture()
    def broken(*args): raise RuntimeError('broken renderer')
    with pytest.raises(SceneRollbackError):
        edit.replace(artifact, rebind=lambda: None, cameras=broken)
    assert set(edit.current_kwargs) == {'robot'}


def test_warmup_is_a_closed_oriented_concave_prism():
    data = warmup_mesh_bytes()
    assert validate_obj(data) == (12, 20)
    faces = [tuple(int(x)-1 for x in line.split()[1:]) for line in data.decode().splitlines() if line.startswith('f ')]
    edges = [(a, b) for face in faces for a, b in zip(face, (*face[1:], face[0]))]
    assert all(edges.count((b, a)) == 1 for a, b in edges)


def test_import_rejects_absolute_and_escaping_paths_before_reading(tmp_path):
    for reference in ('../scene.json', '/etc/passwd', 'sub/../scene.json'):
        with pytest.raises(ValueError):
            load_candidate(tmp_path, reference)


def test_runtime_rejects_static_arrays_selected_by_genesis_environment(monkeypatch):
    from elesim_sim.simulation.runtime_scene_edit import apply_scene_command

    monkeypatch.setitem(sys.modules, 'genesis', NS(use_ndarray=False))
    app = NS(sim_scene=NS(), cfg=NS(genesis_performance_mode=False,
        camera_execution='async_process'))
    with pytest.raises(ValueError, match='dynamic arrays'):
        apply_scene_command(NS(app=app), NS(command='clear_scene'))
