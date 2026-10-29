"""Import the same verified scene into physics and camera replicas at build."""
from __future__ import annotations

from elesim_protocol.scene_artifact import SceneArtifact


def verify_scene_revision(manifest, expected_revision=""):
    artifact = SceneArtifact.load(manifest, expected_frame="world")
    if expected_revision and artifact.revision != expected_revision:
        raise ValueError("scene artifact revision differs from the physics scene")
    return artifact


def add_scene_artifact(scene, genesis, manifest, *, collision=True, expected_revision=""):
    artifact = verify_scene_revision(manifest, expected_revision)
    entities = {}
    for obj in artifact.objects:
        # Schema 1 vertices already use world axes/metres; do not translate or
        # infer an object origin a second time. Imported geometry is static.
        entities[obj.object_id] = scene.add_entity(genesis.morphs.Mesh(
            file=str(obj.mesh_file), pos=(0., 0., 0.), scale=1., fixed=True,
            collision=bool(collision), file_meshes_are_zup=True, align=False,
            decimate=False, convexify=False, watertighten=None))
    # Genesis may read files while constructing entities. Fail before readiness
    # if a producer replaced a supposedly immutable bundle during that work.
    verify_scene_revision(manifest, artifact.revision)
    return artifact, entities
