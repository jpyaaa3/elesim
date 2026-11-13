"""Replay a saved candidate with physical failure termination and report video."""
import argparse
import json
from pathlib import Path
import re
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    candidate = parser.add_mutually_exclusive_group(required=True)
    candidate.add_argument('--candidate', type=int)
    candidate.add_argument('--candidate-name', help='Saved candidate directory, including an optional RRT lane')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    from workbench.research.experiments.hug_reverse_ga.prepare import digest, save
    from workbench.research.experiments.hug_reverse_ga.physics import admit, evaluate, sample
    from workbench.research.experiments.hug_c.configure import asset_digest
    from workbench.research.experiments.hug_c.environment import HugEnv
    from workbench.research.experiments.hug_interpolation.video import record_video
    plan = json.loads((args.run/'provenance.json').read_text())['plan']
    if plan['source_sha256'] != digest() or plan['simulation_context']['model_sha256'] != asset_digest():
        raise ValueError('Source/model mismatch')
    name = args.candidate_name or f'candidate-{args.candidate:04d}'
    if not re.fullmatch(r'candidate-\d{4}(?:-lane-\d+)?', name):
        raise ValueError('Expected a saved candidate directory name')
    source = args.run/name/'commands.npz'
    with np.load(source, allow_pickle=False) as data:
        commands = data['commands'].copy()
    args.output.mkdir(parents=True, exist_ok=False)
    env = HugEnv(plan, plan['cases'][0], args.output/'assets', n_envs=1,
                 backend='gpu', auto_reset=False, video=True)
    env.scene.cameras['report'].set_pose(pos=(.85,-1.,1.25), lookat=(.35,.05,.55), up=(0.,0.,1.))
    if not admit(env, args.output/'admission'):
        raise RuntimeError('Initial grasp failed')
    state, baseline = env.scene.scene.get_state(), sample(env)
    with record_video(env, args.output/'candidate.mp4'):
        reports = evaluate(env, commands, args.output, state, baseline)
    save(args.output/'video-report.json', dict(source=str(source), result=reports[0],
        training=False, note=('Fresh replay passed the original physical criteria; independent verification is in the source run.'
                              if reports[0]['success'] else 'Fresh replay; stops at physical failure. Unsuccessful demonstration.')))


if __name__ == '__main__':
    main()
