"""Main-thread Genesis scene edits with preserved robot state and rollback.

Mesh delivery is external. Only validated scene artifacts below the configured
Sim import root are accepted; the wire carries a relative manifest reference.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import os
from pathlib import Path
import time

from elesim_protocol import SimulationCommandRequest
from elesim_protocol.scene_artifact import SceneArtifact


WARMUP_NAME = "elesim_nonconvex_warmup"
_POLYGON = ((0, 0), (2, 0), (2, 1), (1, 1), (1, 2), (0, 2))


def warmup_mesh_bytes():
    """A closed, concave L prism. Its collision branch is shared by new OBJs."""
    vertices = [(x * .01, y * .01, z * .01) for z in (0, 1) for x, y in _POLYGON]
    cap = ((0, 1, 3), (1, 2, 3), (0, 3, 5), (3, 4, 5))
    faces = [(c, b, a) for a, b, c in cap] + [(a + 6, b + 6, c + 6) for a, b, c in cap]
    for a in range(6):
        b = (a + 1) % 6
        faces.extend(((a, b, b + 6), (a, b + 6, a + 6)))
    return ("".join("v %.5f %.5f %.5f\n" % v for v in vertices)
            + "".join("f %d %d %d\n" % (a + 1, b + 1, c + 1) for a, b, c in faces)).encode()


def add_collision_warmup(scene, genesis):
    from elesim_sim.cache import _prepare_cache_dir

    root = Path(os.environ.get("XDG_CACHE_HOME", str(Path.home() / ".cache"))) / "elesim"
    _prepare_cache_dir(root, private=True)
    data = warmup_mesh_bytes()
    path = root / ("nonconvex-" + hashlib.sha256(data).hexdigest()[:16] + ".obj")
    if path.is_symlink():
        raise ValueError("warmup mesh must not be a symlink")
    if not path.exists():
        with path.open("xb") as handle:
            handle.write(data)
    if path.read_bytes() != data:
        raise ValueError("warmup mesh differs from the owned geometry")
    return scene.add_entity(genesis.morphs.Mesh(file=str(path), pos=(0., 0., -100.),
        fixed=True, collision=True, visualization=False, convexify=False,
        decimate=False, watertighten=None, file_meshes_are_zup=True, align=False), name=WARMUP_NAME)


def import_root(configured=""):
    root = (Path(configured).expanduser().absolute() if configured else
            Path(os.environ.get("XDG_CACHE_HOME", str(Path.home() / ".cache"))) / "scene-imports")
    if any(p.is_symlink() for p in (root, *root.parents)):
        raise ValueError("scene import root must not contain symlinks")
    root.mkdir(mode=0o700, parents=True, exist_ok=True)
    return root


def load_candidate(root, reference, expected_revision=""):
    args = SimulationCommandRequest._arguments("load_scene", {
        "manifest": reference, "expected_revision": expected_revision})
    artifact = SceneArtifact.load(Path(root) / args["manifest"])
    if expected_revision and artifact.revision != expected_revision:
        raise ValueError("scene artifact revision differs from the requested revision")
    return artifact


def entity_kwargs(entity):
    desc = entity.desc
    return dict(morph=desc.morphs[0] if len(desc.morphs) == 1 else tuple(desc.morphs),
        material=desc.material, surface=desc.surface, visualize_contact=desc.visualize_contact)


def _copy(value):
    return value.clone() if hasattr(value, "clone") else value.copy()


@dataclass
class EntityState:
    qpos: object
    velocity: object
    properties: dict

    @classmethod
    def read(cls, entity):
        properties = {}
        if int(entity.n_dofs):
            for name in ("kp", "kv", "damping", "armature", "frictionloss"):
                properties[name] = _copy(getattr(entity, "get_dofs_" + name)())
        return cls(_copy(entity.get_qpos()), _copy(entity.get_dofs_velocity()), properties)

    def restore(self, entity):
        for name, value in self.properties.items():
            getattr(entity, "set_dofs_" + name)(value)
        if len(self.qpos):
            entity.set_qpos(self.qpos)
        if len(self.velocity):
            entity.set_dofs_velocity(self.velocity)


class SceneRollbackError(RuntimeError):
    """A failed rollback leaves no valid physics state to resume."""


class SceneRebuilder:
    def __init__(self, scene, genesis, imported_entities=None, *, editor_factory=None):
        if editor_factory is None:
            from genesis.engine.interactive_scene import InteractiveScene

            editor_factory = InteractiveScene

        self.scene, self.genesis = scene, genesis
        self.editor = editor_factory(scene)
        imported = {id(value) for value in (imported_entities or {}).values()}
        self.base_names = tuple(entity.name for entity in scene.entities if id(entity) not in imported)
        self.current_kwargs = {entity.name: entity_kwargs(entity) for entity in scene.entities}
        self.last_timings = {}

    def desired(self, artifact):
        desired = {name: self.current_kwargs[name] for name in self.base_names}
        names = {}
        for obj in artifact.objects if artifact else ():
            name = "scene_" + obj.object_id
            if name in desired:
                raise ValueError("scene object name conflicts with a runtime entity")
            names[obj.object_id] = name
            desired[name] = dict(morph=self.genesis.morphs.Mesh(file=str(obj.mesh_file),
                pos=(0., 0., 0.), scale=1., fixed=True, collision=True,
                file_meshes_are_zup=True, align=False, decimate=False,
                convexify=False, watertighten=None))
        return desired, names

    def replace(self, artifact, *, rebind, cameras, authorized=lambda: True, validate=lambda: None):
        desired, names = self.desired(artifact)
        old = self.current_kwargs
        state = {entity.name: EntityState.read(entity) for entity in self.scene.entities}
        if not authorized():
            raise PermissionError("simulation session changed before scene rebuild")

        def build(kwargs):
            self.editor.rebuild(entities_kwargs=kwargs)
            self.scene.step()
            if self.editor._rebuild_pending or not self.scene.is_built:
                raise RuntimeError("Genesis did not apply the pending scene rebuild")
            for entity in self.scene.entities:
                if entity.name in state:
                    state[entity.name].restore(entity)
            rebind()
            # Rebinding must not overwrite the saved qpos/velocity or gains.
            for entity in self.scene.entities:
                if entity.name in state:
                    state[entity.name].restore(entity)

        started = time.monotonic()
        self.last_timings = {}
        try:
            build(desired)
            self.last_timings['physics_s'] = time.monotonic() - started
            validate()
            if not authorized():
                raise PermissionError("simulation session changed during scene rebuild")
            entities = {key: self.scene.get_entity(name) for key, name in names.items()}
            camera_started = time.monotonic()
            cameras(artifact, entities)
            self.last_timings['cameras_s'] = time.monotonic() - camera_started
            if not authorized():
                raise PermissionError("simulation session changed before scene activation")
        except Exception as error:
            try:
                build(old)
                cameras(None, None)  # Restore the previously committed camera revision.
            except Exception as rollback_error:
                raise SceneRollbackError(f"scene rollback failed: {rollback_error}") from error
            raise RuntimeError(f"scene rebuild failed; previous scene restored: {error}") from error
        self.current_kwargs = desired
        return entities, time.monotonic() - started
