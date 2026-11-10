"""Record a C-hug policy from the arm side without changing its frozen task.

This report-only entrypoint owns camera presentation. It uses the existing
environment, policy and physics-tick recorder; no optimizer or task override is
involved. Keeping it outside hug_c lets an active, source-pinned study continue.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def position_camera(env):
    """Look down from negative world Y, opposite the C-hug cylinder side."""
    view = dict(name='arm-side-oblique-v1',
                pos=(.85, -1.15, 1.60), lookat=(.35, .05, .40), up=(0., 0., 1.))
    env.scene.cameras['report'].set_pose(**{key: view[key] for key in ('pos', 'lookat', 'up')})
    return view


def collect(args):
    from elesim_sim.rl.trajectory import Trajectory
    from workbench.research.experiments.hug_c.configure import CHANNELS, load_experiment, sha
    from workbench.research.experiments.hug_c.experiment import case_by_id, create_env
    from workbench.research.experiments.hug_c.policy import NumpyPolicy
    from workbench.research.experiments.hug_interpolation.video import record_video
    import torch

    for path, suffix in ((args.output, '.npz'), (args.video, '.mp4')):
        if path.suffix.lower() != suffix or path.exists():
            raise ValueError(f'Choose a new {suffix} report output')
    experiment = load_experiment(args.experiment)
    policy = NumpyPolicy(args.policy)
    if policy.metadata['simulation_context'] != experiment['simulation_context']:
        raise ValueError('Policy and experiment context differ')
    case = case_by_id(experiment, policy.metadata['case_id'])
    env = create_env(experiment, case, args.output.parent, args)
    observation, _ = env.reset()
    view = position_camera(env)
    env.record = [env.start.detach().cpu().numpy().copy()]
    with record_video(env, args.video), torch.inference_mode():
        for _ in range(env.max_episode_length):
            actions = torch.as_tensor(policy(observation['policy'].cpu().numpy()), device=env.device)
            observation, _, done, _ = env.step(actions)
            if bool(done[0]):
                break
    result = env.measure()
    trajectory = Trajectory(
        np.arange(len(env.record))*experiment['settings']['dt'], np.asarray(env.record), CHANNELS,
        dict(env.metadata(), policy_sha256=sha(args.policy)),
        dict(collection=result, report_camera=view, replay_success=None, physics_validated=False))
    trajectory.save(args.output)
    print(json.dumps(dict(collection=result, report_camera=view), indent=2))
    return 0 if result['success'] else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--experiment', type=Path, required=True)
    parser.add_argument('--policy', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--video', type=Path, required=True)
    parser.add_argument('--backend', choices=('gpu', 'cpu'), default='gpu')
    return collect(parser.parse_args(argv))


if __name__ == '__main__':
    raise SystemExit(main())
