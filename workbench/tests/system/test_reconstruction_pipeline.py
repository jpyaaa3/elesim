"""Replay CLI to an independent Sim artifact consumer; no ROS, GPU or motion."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[3]
PROTOCOL = ROOT / "payload/runtime/common/protocol"
PILOT = ROOT / "payload/runtime/docker/pilot/app"
SIM = ROOT / "payload/runtime/docker/sim/app"


def test_external_adapter_cli_exports_scene_for_independent_sim_process(tmp_path):
    model = lambda name: {"factory": "pilot.reconstruction_fixtures:" + name}
    models = {"segmenter": model("Segmenter"), "pose": model("CameraPose"),
        "completion": model("Completion"), "planner": model("Planner"),
        "mesher": {"factory": "elesim_pilot.reconstruction.providers:VoxelSurface"}}
    config = tmp_path / "pipeline.yaml"
    config.write_text(yaml.safe_dump({"schema_version": 1, "models": models}))
    frames = tmp_path / "frames"
    frames.mkdir()
    image = np.zeros((4, 6, 3), np.uint8)
    image[:, :2, 1], image[:, 4:, 2] = 255, 255
    np.savez_compressed(frames / "0001.npz", schema_version=1, color_bgr=image,
        depth_raw=np.full((4, 6), .8, np.float32), depth_scale=1.,
        intrinsics=[20., 20., 2.5, 1.5, 6, 4], seq=1, ts=101.,
        source_id="camera", source_boot_id="boot", calibration_id="calibration", encoder_q=np.zeros(4))
    bundle = tmp_path / "orchard"
    environment = {**os.environ, "PYTHONPATH": os.pathsep.join(map(str,
        (PROTOCOL, PILOT, ROOT / "workbench/tests/apps")))}
    result = subprocess.run([sys.executable, "-m", "elesim_pilot.reconstruction.cli", "--config", str(config),
        "--frames", str(frames), "--output", str(bundle), "--scene-id", "orchard",
        "--target", "apple-1", "--task", "pick"], env=environment, cwd=tmp_path,
        text=True, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stderr
    # No Pilot or workbench code is importable by this consumer.
    environment["PYTHONPATH"] = os.pathsep.join(map(str, (PROTOCOL, SIM)))
    dependency_paths = [p for p in sys.path if p and Path(p).name in {"site-packages", "dist-packages"}]
    script = '''
import importlib.util, json, sys
from types import SimpleNamespace
# Skip editable .pth hooks from the all-project developer environment.
sys.path.extend(json.loads(sys.argv[2]))
from elesim_sim.simulation.scene_artifact import add_scene_artifact
assert importlib.util.find_spec("elesim_pilot") is None
class Scene:
    def __init__(self): self.meshes = []
    def add_entity(self, morph): self.meshes.append(morph); return morph
genesis = SimpleNamespace(morphs=SimpleNamespace(Mesh=lambda **kw: kw))
physics, render = Scene(), Scene()
artifact, entities = add_scene_artifact(physics, genesis, sys.argv[1])
add_scene_artifact(render, genesis, sys.argv[1], collision=False, expected_revision=artifact.revision)
assert [m["file"] for m in physics.meshes] == [m["file"] for m in render.meshes]
assert all(m["collision"] for m in physics.meshes)
assert not any(m["collision"] for m in render.meshes)
print(json.dumps({"scene": artifact.scene_id, "revision": artifact.revision, "ids": list(entities)}))
'''
    consumer = subprocess.run([sys.executable, "-S", "-c", script, str(bundle / "scene.json"),
        json.dumps(dependency_paths)],
        env=environment, cwd=tmp_path, text=True, capture_output=True, timeout=30)
    assert consumer.returncode == 0, consumer.stderr
    result = json.loads(consumer.stdout)
    assert result["scene"] == "orchard" and result["ids"] == ["apple-1", "tree-1"]
    assert result["revision"] == hashlib.sha256((bundle / "scene.json").read_bytes()).hexdigest()
    assert json.loads((bundle / "plan.json").read_text())["executed"] is False
