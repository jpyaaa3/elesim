"""Independent object surfaces, rather than the fitted floor, assess correction."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np

from workbench.research.experiments.vision_scene import produce
from workbench.research.experiments.vision_evaluate import evaluate


def test_blind_replay_corrects_observable_error_and_retains_xy_yaw_error(tmp_path):
    repository = Path(__file__).resolve().parents[4]
    data = tmp_path / "producer"
    produce(data, seeds=3, width=320, height=240)
    blind = tmp_path / "consumer"
    blind.mkdir()
    shutil.move(data / "sensors", blind / "sensors")
    script = repository / "workbench/research/experiments/vision_reconstruct.py"
    shutil.copyfile(script, blind / "reconstruct.py")
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(str(repository / path) for path in (
        "payload/runtime/docker/pilot/app", "payload/runtime/common/protocol"))
    env["OPENBLAS_NUM_THREADS"] = "1"
    completed = subprocess.run(
        [sys.executable, str(blind / "reconstruct.py"), str(blind / "sensors"), str(blind / "result")],
        cwd=blind, env=env, text=True, capture_output=True, timeout=60, check=True,
    )
    assert json.loads(completed.stdout)["valid_frames"] == 36
    result = evaluate(data / "truth", blind / "result")
    assert result["accepted"]
    assert result["metrics"]["observable"]["after_p95_m"] < 0.003
    assert result["metrics"]["unobservable"]["after_p95_m"] > 0.06
    assert result["evaluation_alignment"].startswith("none")
    for path in (blind / "sensors").glob("*.npz"):
        with np.load(path, allow_pickle=False) as packet:
            assert set(packet.files) == {
                "schema_version", "color_bgr", "depth_raw", "depth_scale", "intrinsics",
                "seq", "ts", "source_id", "source_boot_id", "calibration_id", "encoder_q",
            }
    # This is a CPU regression fixture. Container mount isolation and actual
    # Genesis rasterization are recorded separately by the GPU acceptance run.
    assert not (blind / "truth").exists()
