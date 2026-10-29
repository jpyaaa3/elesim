import hashlib
import json
from types import SimpleNamespace

import pytest

from elesim_sim.config import load_app_config
from elesim_sim.config.yaml_schema import bundle_to_yaml_data
from elesim_sim.simulation.scene_artifact import add_scene_artifact


def manifest(tmp_path):
    mesh = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
    (tmp_path / "apple-1.obj").write_bytes(mesh)
    path = tmp_path / "scene.json"
    path.write_text(json.dumps({"schema_version": 1, "scene_id": "orchard", "frame_id": "world",
        "units": "m", "provenance": {}, "objects": [{"object_id": "apple-1", "label": "apple",
            "role": "target", "mesh_file": "apple-1.obj", "sha256": hashlib.sha256(mesh).hexdigest(),
            "observed_points": 10, "predicted_points": 0}]}))
    return path


class FakeScene:
    def __init__(self):
        self.entities = []

    def add_entity(self, morph):
        self.entities.append(morph)
        return morph


@pytest.mark.parametrize("collision", [True, False])
def test_physics_and_render_replica_import_world_mesh_without_extra_transform(tmp_path, collision):
    path = manifest(tmp_path)
    scene = FakeScene()
    genesis = SimpleNamespace(morphs=SimpleNamespace(Mesh=lambda **kw: kw))
    artifact, entities = add_scene_artifact(scene, genesis, path, collision=collision)
    assert artifact.scene_id == "orchard"
    assert entities == {"apple-1": {"file": str(tmp_path / "apple-1.obj"),
        "pos": (0., 0., 0.), "scale": 1., "fixed": True, "collision": collision,
        "file_meshes_are_zup": True, "align": False, "decimate": False,
        "convexify": False, "watertighten": None}}
    assert scene.entities == list(entities.values())


def test_bundle_change_during_entity_construction_aborts_readiness(tmp_path):
    path = manifest(tmp_path)
    class ChangingScene(FakeScene):
        def add_entity(self, morph):
            (tmp_path / "apple-1.obj").write_text("changed")
            return super().add_entity(morph)
    genesis = SimpleNamespace(morphs=SimpleNamespace(Mesh=lambda **kw: kw))
    with pytest.raises(ValueError, match="digest mismatch"):
        add_scene_artifact(ChangingScene(), genesis, path)


def test_render_replica_rejects_a_different_valid_revision(tmp_path):
    path = manifest(tmp_path)
    genesis = SimpleNamespace(morphs=SimpleNamespace(Mesh=lambda **kw: kw))
    artifact, _ = add_scene_artifact(FakeScene(), genesis, path)
    raw = json.loads(path.read_text())
    raw["scene_id"] = "replaced-scene"
    path.write_text(json.dumps(raw))
    render = FakeScene()
    with pytest.raises(ValueError, match="revision differs"):
        add_scene_artifact(render, genesis, path, collision=False, expected_revision=artifact.revision)
    assert render.entities == []


def test_scene_manifest_path_resolves_and_roundtrips_in_sim_config(tmp_path):
    config = tmp_path / "sim.yaml"
    config.write_text("schema_version: 1\nsimulation:\n  runtime:\n    scene_manifest: captured/scene.json\n")
    bundle = load_app_config(config)
    assert bundle.sim_config.scene_manifest == str(tmp_path / "captured/scene.json")
    raw = bundle_to_yaml_data(bundle, config_dir=str(tmp_path))
    assert raw["simulation"]["runtime"]["scene_manifest"] == "captured/scene.json"
