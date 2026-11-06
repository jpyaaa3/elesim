"""Batched static Home/goal screening before selecting RL anchors.

All catalog entries still come from mapper functions. This checks the complete
3D robot at those object placements, not approach/lift feasibility or learning.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from .configure import configure, write_json


def screen(args):
    import torch
    from .environment import HugEnv

    catalog = json.loads(args.catalog.read_text())
    experiment = configure(args.pilot)
    rows = catalog['rows']
    if (catalog['source']['kernel_sha256'] != experiment['source']['kernel_sha256']
            or catalog['diameter_mm'] != experiment['cases'][0]['diameter_mm']):
        raise ValueError('Catalog source/diameter differs from pilot')
    if args.batch < 1 or args.ticks < 1:
        raise ValueError('batch/ticks must be positive')
    args.output.mkdir(parents=True, exist_ok=False)
    env = HugEnv(experiment, experiment['cases'][0], args.output/'assets',
                 n_envs=args.batch, backend='gpu', auto_reset=False)
    origin_case = experiment['cases'][0]
    origin = np.asarray(origin_case['object_xyz_m'])
    origin_xy = np.asarray(origin_case['xy_mm'])
    results = []
    for begin in range(0, len(rows), args.batch):
        subset = rows[begin:begin+args.batch]
        padded = subset + [subset[-1]]*(args.batch-len(subset))
        xy = np.array([r['xy_mm'] for r in padded])
        delta = (xy-origin_xy)/1000
        positions = np.tile(origin, (args.batch, 1))
        positions[:, 0] += delta[:, 1]
        positions[:, 1] -= delta[:, 0]
        positions = torch.as_tensor(positions, device=env.device, dtype=torch.float32)
        goals = np.array([[0., -math.pi/2, *np.deg2rad(np.array(r['q_deg'])/5)] for r in padded])
        goals = torch.as_tensor(goals, device=env.device, dtype=torch.float32)
        checks = {}
        for phase in ('home', 'goal'):
            env.reset()
            env.scene.object.set_pos(positions)
            command = env.start.expand(args.batch, 4) if phase == 'home' else goals
            env.mapper.waypoint[:] = command
            targets = env.mapper.joint_targets()
            env.scene.robot.set_dofs_position(targets, dofs_idx_local=env.dofs, zero_velocity=True)
            env.scene.robot.control_dofs_position(targets, dofs_idx_local=env.dofs)
            for _ in range(args.ticks):
                env.tick(command)
            checks[phase] = env.fault.detach().cpu().tolist()
        for index, row in enumerate(subset):
            results.append(dict(row, home_fault=checks['home'][index], goal_fault=checks['goal'][index],
                                static_ready=not (checks['home'][index] or checks['goal'][index])))
        progress = dict(visited=len(results), total=len(rows), static_ready=sum(r['static_ready'] for r in results))
        temp = args.output/'progress.json.tmp'
        temp.write_text(json.dumps(progress, indent=2))
        temp.replace(args.output/'progress.json')
        print(json.dumps(progress), flush=True)
    report = dict(schema='elesim-c-hug-catalog-preflight', schema_version=1,
                  source=catalog['source'], diameter_mm=catalog['diameter_mm'],
                  step_deg=catalog['step_deg'], mu_max=catalog['mu_max'],
                  simulation_context=experiment['simulation_context'],
                  ticks_per_pose=args.ticks, batch=args.batch,
                  criterion='Home and directly placed goal have no faults; approach/lift untested',
                  results=results)
    write_json(args.output/'report.json', report)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--catalog', type=Path, required=True)
    parser.add_argument('--pilot', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--batch', type=int, default=64)
    parser.add_argument('--ticks', type=int, default=10)
    screen(parser.parse_args())


if __name__ == '__main__':
    main()
