"""Model preflight and replay execution; no DDS, robot commands or training."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from elesim_pilot.vision.partial_cloud import SensorObservation
from .artifacts import export_result
from .contracts import Measurement, Task
from .pipeline import Pipeline


def load_measurements(root):
    metadata = {}
    if (root/'frames').is_dir():
        report = json.loads((root/'scan.json').read_text())
        metadata = {'frames/'+f"{row['view']:03d}.npz": row for row in report['views']}
        metadata.update({'calibration/'+row['file']: row for row in report.get('calibration_observations', [])})
        files = [*(root/'frames').glob('*.npz'), *(root/'calibration').glob('*.npz')]
        def source_order(path):
            with np.load(path, allow_pickle=False) as frame:
                return float(frame['ts']), int(frame['seq'])
        files.sort(key=source_order)
    else:
        files = sorted(root.glob('*.npz'))
    for path in files:
        sensor = SensorObservation.load(path)
        row = metadata.get(path.relative_to(root).as_posix())
        if row is not None and row['seq'] != sensor.seq:
            raise ValueError('replay base pose metadata does not match capture sequence')
        yield Measurement(sensor, base_world_transform=None if row is None else row.get('base_world_transform'))


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
    with Pipeline.from_config(args.config, mapping=mapping) as pipeline:
        if args.check:
            print(json.dumps({"api_version": 1, "loaded": {key: model.provenance for key, model in pipeline.models.items()},
                "pose_corrections": [model.provenance for model in pipeline.corrections]}, indent=2))
            return
        if args.frames is None or args.output is None:
            parser.error("execution needs --frames and --output")
        roles = dict(value.split("=", 1) for value in args.role)
        task = Task(args.scene_id, args.target, operation=args.task, roles=roles)
        frames = load_measurements(args.frames)
        result = pipeline.run(frames, task, progress=lambda event: print(json.dumps(event), flush=True))
        print(export_result(result, args.output))


if __name__ == "__main__":
    main()
