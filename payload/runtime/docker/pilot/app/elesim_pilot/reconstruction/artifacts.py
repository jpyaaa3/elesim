"""Export complete immutable scene bundles, preserving measured points separately."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import tempfile

import numpy as np

from elesim_protocol.scene_artifact import MAX_MESH_BYTES, SceneArtifact, validate_obj


def export_result(result, destination):
    destination = Path(destination).expanduser().absolute()
    if destination.exists() or destination.is_symlink():
        raise FileExistsError("scene bundles are immutable; use a new output directory")
    if any(p.is_symlink() for p in destination.parents):
        raise ValueError("scene output contains a symlink ancestor")
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".scene-", dir=destination.parent) as temporary:
        root = Path(temporary)
        objects = []
        for item in result.objects:
            cloud, mesh = item.shape.cloud, item.mesh
            lines = ["v %.17g %.17g %.17g\n" % tuple(vertex) for vertex in mesh.vertices]
            lines.extend("f %d %d %d\n" % tuple(face + 1) for face in mesh.triangles)
            payload = "".join(lines).encode()
            if len(payload) > MAX_MESH_BYTES:
                raise ValueError("mesh exceeds scene artifact budget")
            validate_obj(payload)
            filename = cloud.object_id + ".obj"
            (root / filename).write_bytes(payload)
            # Measurement and model prediction never overwrite one another.
            np.savez_compressed(root / (cloud.object_id + ".npz"),
                observed=cloud.observed, predicted=item.shape.predicted)
            objects.append({"object_id": cloud.object_id, "label": cloud.label, "role": cloud.role,
                "mesh_file": filename, "sha256": hashlib.sha256(payload).hexdigest(),
                "observed_points": len(cloud.observed), "predicted_points": len(item.shape.predicted)})
        manifest = {"schema_version": 1, "scene_id": result.task.scene_id,
            "frame_id": result.task.frame_id, "units": "m", "objects": objects, "provenance": result.provenance}
        (root / "scene.json").write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n")
        validated = SceneArtifact.load(root / "scene.json", expected_frame=result.task.frame_id)
        if result.plan is not None:
            plan = result.plan
            (root / "plan.json").write_text(json.dumps({"schema_version": 1,
                "scene_revision": validated.revision, "target_id": plan.target_id, "frame_id": plan.frame_id,
                "times_s": list(plan.times_s), "q": plan.q.tolist(), "contact_ids": list(plan.contact_ids),
                "executed": False}, indent=2, allow_nan=False) + "\n")
        # The manifest is copied last: interrupted exports cannot look complete.
        destination.mkdir()
        try:
            for path in sorted(root.iterdir(), key=lambda p: p.name == "scene.json"):
                shutil.copyfile(path, destination / path.name)
        except BaseException:
            for path in destination.iterdir():
                path.unlink()
            destination.rmdir()
            raise
    return destination / "scene.json"
