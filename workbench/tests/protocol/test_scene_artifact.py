import hashlib
import json

import pytest

from elesim_protocol.scene_artifact import SceneArtifact, validate_obj


TRIANGLE = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"


def artifact(tmp_path):
    mesh = tmp_path / "apple-1.obj"
    mesh.write_bytes(TRIANGLE)
    raw = {"schema_version": 1, "scene_id": "orchard", "frame_id": "world", "units": "m",
        "objects": [{"object_id": "apple-1", "label": "apple", "role": "target",
            "mesh_file": mesh.name, "sha256": hashlib.sha256(TRIANGLE).hexdigest(),
            "observed_points": 30, "predicted_points": 40}], "provenance": {}}
    path = tmp_path / "scene.json"
    path.write_text(json.dumps(raw))
    return path, raw


def test_scene_artifact_verifies_revision_and_geometry(tmp_path):
    path, _ = artifact(tmp_path)
    scene = SceneArtifact.load(path)
    assert scene.revision == hashlib.sha256(path.read_bytes()).hexdigest()
    assert scene.objects[0].mesh_file == tmp_path / "apple-1.obj"
    (tmp_path / "apple-1.obj").write_bytes(TRIANGLE + b"# modified\n")
    with pytest.raises(ValueError, match="digest mismatch"):
        SceneArtifact.load(path)


@pytest.mark.parametrize("field,value", [("units", "mm"), ("frame_id", "camera"),
    ("schema_version", True), ("schema_version", 2)])
def test_incompatible_frame_units_or_schema_fail(tmp_path, field, value):
    path, raw = artifact(tmp_path)
    raw[field] = value
    path.write_text(json.dumps(raw))
    with pytest.raises(ValueError):
        SceneArtifact.load(path)


@pytest.mark.parametrize("field,value", [("mesh_file", "../apple-1.obj"),
    ("object_id", "../apple-1"), ("observed_points", 0), ("predicted_points", True),
    ("role", "fruit"), ("sha256", "wrong")])
def test_invalid_object_contract_fails(tmp_path, field, value):
    path, raw = artifact(tmp_path)
    raw["objects"][0][field] = value
    path.write_text(json.dumps(raw))
    with pytest.raises(ValueError):
        SceneArtifact.load(path)


def test_manifest_rejects_nonfinite_metadata(tmp_path):
    path, raw = artifact(tmp_path)
    raw["provenance"] = {"quality": float("nan")}
    path.write_text(json.dumps(raw))
    with pytest.raises(ValueError, match="non-finite"):
        SceneArtifact.load(path)


def test_manifest_and_mesh_reject_symlinks(tmp_path):
    path, _ = artifact(tmp_path)
    linked = tmp_path / "linked"
    linked.symlink_to(tmp_path, target_is_directory=True)
    with pytest.raises(ValueError, match="symlink"):
        SceneArtifact.load(linked / "scene.json")
    mesh = tmp_path / "apple-1.obj"
    mesh.unlink()
    other = tmp_path / "other.obj"
    other.write_bytes(TRIANGLE)
    mesh.symlink_to(other)
    with pytest.raises(ValueError, match="symlink"):
        SceneArtifact.load(path)


@pytest.mark.parametrize("data", [b"v nan 0 0\n", b"mtllib external.mtl\n",
    b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 4\n",
    b"v 0 0 0\nv 1 0 0\nv 2 0 0\nf 1 2 3\n",
    b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf -1 -2 -3\n"])
def test_obj_accepts_only_bounded_finite_triangular_geometry(data):
    with pytest.raises(ValueError):
        validate_obj(data)
