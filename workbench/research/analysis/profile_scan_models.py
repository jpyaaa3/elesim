"""Time the exact installed scan model/planner preparation without transport."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--single-thread',action='store_true')
    args = parser.parse_args()
    if args.single_thread:
        os.environ['OPENBLAS_NUM_THREADS'] = '1'
        os.environ['OMP_NUM_THREADS'] = '1'
    began = time.monotonic()
    import numpy as np
    from elesim_pilot.reconstruction import Pipeline
    from elesim_pilot.reconstruction.scan_collision import ScanCollision
    from elesim_pilot.robot.arm.self_collision import ArmSelfCollision
    from elesim_pilot.vision.partial_cloud import ArmCameraModel
    from elesim_pilot.vision.multiview import ViewPlanner

    result = dict(single_thread=args.single_thread,imports_s=time.monotonic()-began,
        threads=next(s for s in Path('/proc/self/status').read_text().splitlines() if s.startswith('Threads:')))
    base = np.eye(4)
    base[2,3] = .8
    data = Path('/opt/elesim/data')
    phases = [('models',lambda:Pipeline.from_config('/opt/elesim/config/scan.example.yaml')),
        ('camera',lambda:ArmCameraModel(data/'models/arm/default.json',
            data/'calibration/cameras/zed_mini.hand_eye.json',base_world_transform=base)),
        ('arm',lambda:ArmSelfCollision(data/'models/planning/zed-mini',joint_uncertainty=[.003,.04,.04,.04]))]
    loaded = {}
    for name,factory in phases:
        began = time.monotonic()
        loaded[name] = factory()
        result[name+'_s'] = time.monotonic()-began
    began = time.monotonic()
    collision = ScanCollision(loaded['arm'],base)
    ViewPlanner(loaded['camera'],collision_checker=collision)
    result['view_planner_s'] = time.monotonic()-began
    print(json.dumps(result),flush=True)


if __name__ == '__main__':
    main()
