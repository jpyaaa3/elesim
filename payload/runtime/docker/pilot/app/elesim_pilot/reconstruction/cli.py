"""Model preflight and replay execution; no DDS, robot commands or training."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from elesim_pilot.vision.partial_cloud import SensorObservation
from .artifacts import export_result
from .contracts import Measurement, Task
from .pipeline import Pipeline


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--app-config", help="Pilot app YAML providing the actual operating joint limits")
    parser.add_argument("--check", action="store_true", help="load/validate model factories and pinned assets")
    parser.add_argument("--frames", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--scene-id", default="reconstructed")
    parser.add_argument("--target", default="target")
    parser.add_argument("--task", choices=("inspect", "pick", "wrap"), default="inspect")
    parser.add_argument("--role", action="append", default=[], metavar="OBJECT=ROLE")
    args = parser.parse_args()
    mapping = None
    if args.app_config:
        from elesim_pilot.config import load_app_config
        mapping = load_app_config(args.app_config).mapping_config
    pipeline = Pipeline.from_config(args.config, mapping=mapping)
    if args.check:
        print(json.dumps({"api_version": 1, "loaded": {key: model.provenance for key, model in pipeline.models.items()},
            "pose_corrections": [model.provenance for model in pipeline.corrections]}, indent=2))
        return
    if args.frames is None or args.output is None:
        parser.error("execution needs --frames and --output")
    files = sorted(args.frames.glob("*.npz"))
    roles = dict(value.split("=", 1) for value in args.role)
    task = Task(args.scene_id, args.target, operation=args.task, roles=roles)
    frames = (Measurement(SensorObservation.load(path)) for path in files)
    result = pipeline.run(frames, task, progress=lambda event: print(json.dumps(event), flush=True))
    print(export_result(result, args.output))


if __name__ == "__main__":
    main()
