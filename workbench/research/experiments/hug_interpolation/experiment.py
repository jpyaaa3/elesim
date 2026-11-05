"""Explicit configure/check/train/collect/replay/interpolate/validate commands.

Importing this CLI never starts Genesis or training. GPU imports are lazy.
"""
import argparse
import importlib.util
import importlib.metadata
import time
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from elesim_sim.rl.trajectory import Trajectory
from elesim_sim.rl.replay import replay
from .configure import configure, load_experiment, write_json, sha, VERSIONS
from .interpolate import CHANNELS, interpolate


def case_by_id(experiment, case_id):
    return next(c for c in experiment['cases'] if c['case_id'] == case_id)


def runner(env, directory):
    from elesim_sim.rl.runner import make_runner
    from workbench.research.experiments.wrap_grasp.configs.loader import load_config, to_dict
    cfg = dict(to_dict(load_config().train.runner))
    cfg.update(num_steps_per_env=16, save_interval=100, experiment_name='hug-local-pilot',
               logger='tensorboard', empirical_normalization=False,
               seed=env.experiment['settings']['seed'])
    return make_runner(env, cfg, directory)


def check_versions():
    actual = {name: importlib.metadata.version(name) for name in VERSIONS}
    mismatches = {name: version for name, version in actual.items() if version.split('+')[0] != VERSIONS[name]}
    if mismatches:
        raise ValueError(f'Package versions differ from experiment contract: {mismatches}; expected {VERSIONS}')
    return actual


def create_env(experiment, case, directory, args, n_envs=1, auto_reset=False):
    check_versions()
    from .environment import HugEnv
    return HugEnv(experiment, case, Path(directory)/'assets', n_envs=n_envs,
                  backend=args.backend, viewer=getattr(args, 'viewer', False), auto_reset=auto_reset)


def train(args):
    exp = load_experiment(args.experiment)
    if args.all:
        if args.case:
            raise ValueError('--all and --case are mutually exclusive')
        for case in exp['cases']:
            if case['split'] != 'train':
                continue
            command = [sys.executable, '-m', __package__+'.experiment', 'train',
                       '--experiment', str(args.experiment), '--case', case['case_id'],
                       '--runs', str(args.runs), '--num-envs', str(args.num_envs),
                       '--iterations', str(args.iterations), '--backend', args.backend]
            if args.resume:
                command.append('--resume')
            subprocess.run(command, check=True)
        return 0
    if not args.case:
        raise ValueError('Choose --case ID or --all')
    case = case_by_id(exp, args.case)
    if case['split'] != 'train':
        raise ValueError('Validation/test cases cannot enter training')
    if args.num_envs < 1 or args.iterations < 1:
        raise ValueError('num-envs and iterations must be positive')
    run = args.runs/case['case_id']
    record = dict(experiment=exp, case_id=case['case_id'], num_envs=args.num_envs)
    resume = None
    if args.resume:
        if json.loads((run/'run.json').read_text()) != record:
            raise ValueError('Resume run provenance differs')
        checkpoints = list(run.glob('model_*.pt'))
        if not checkpoints:
            raise ValueError('No checkpoint to resume')
        resume = max(checkpoints, key=lambda p: p.stat().st_mtime_ns)
    else:
        run.mkdir(parents=True, exist_ok=False)
        write_json(run/'run.json', record)
    from workbench.research.experiments.wrap_grasp.train import set_seed
    set_seed(exp['settings']['seed'])
    env = create_env(exp, case, run, args, args.num_envs, True)
    policy_runner = runner(env, run)
    if resume:
        policy_runner.load(str(resume))
    started = time.monotonic()
    policy_runner.learn(num_learning_iterations=args.iterations, init_at_random_ep_len=False)
    policy_runner.save(str(run/'model_final.pt'))
    with (run/'training_attempts.jsonl').open('a') as stream:
        stream.write(json.dumps(dict(iterations=args.iterations, elapsed_s=time.monotonic()-started,
                                    resumed_from=str(resume) if resume else None, versions=check_versions()))+'\n')
    return 0


def collect(args):
    import torch
    record = json.loads((args.run/'run.json').read_text())
    # Reuse the normal integrity check, without modifying the saved manifest.
    exp = load_experiment(args.experiment)
    if record['experiment'] != exp:
        raise ValueError('Run and experiment differ')
    case = case_by_id(exp, record['case_id'])
    checkpoint = args.checkpoint or args.run/'model_final.pt'
    if not checkpoint.is_file():
        raise FileNotFoundError(checkpoint)
    env = create_env(exp, case, args.run, args)
    policy_runner = runner(env, None)
    policy_runner.load(str(checkpoint))
    policy = policy_runner.get_inference_policy(device=env.device)
    obs, _ = env.reset()
    env.record = [env.start.detach().cpu().numpy().copy()]
    with torch.inference_mode():
        for _ in range(env.max_episode_length):
            obs, _, _, _ = env.step(policy(obs))
    result = env.measure()
    trajectory = Trajectory(np.arange(len(env.record))*exp['settings']['dt'], np.asarray(env.record),
                            CHANNELS, dict(env.metadata(), checkpoint_sha256=sha(checkpoint)),
                            dict(collection=result, replay_success=None, physics_validated=False))
    trajectory.save(args.output)
    print(json.dumps(result, indent=2))
    return 0 if result['success'] else 1


def replay_one(args):
    from .environment import ReplayAdapter
    exp = load_experiment(args.experiment)
    trajectory = Trajectory.load(args.trajectory)
    case = case_by_id(exp, trajectory.metadata['case_id'])
    env = create_env(exp, case, args.output.parent, args)
    result = replay(trajectory, ReplayAdapter(env), dt=exp['settings']['dt'])
    result['trajectory_sha256'] = sha(args.trajectory)
    write_json(args.output, result)
    if args.verified_output:
        trajectory.diagnostics.update(replay_success=bool(result['evaluation']['success']),
                                      physics_validated=True, replay_evaluation=result['evaluation'])
        trajectory.save(args.verified_output)
    print(json.dumps(result['evaluation'], indent=2))
    return 0 if result['evaluation']['success'] else 1


def make_candidates(args):
    exp = load_experiment(args.experiment)
    by_id = {}
    for path in args.anchors.glob('*.json'):
        trace = Trajectory.load(path)
        cid = trace.metadata['case_id']
        if cid in by_id:
            raise ValueError(f'Duplicate anchor {cid}')
        if case_by_id(exp, cid)['split'] != 'train':
            raise ValueError('Anchor directory contains a held-out case')
        by_id[cid] = trace
    duration = (exp['settings']['approach_steps']+exp['settings']['hold_steps'])*exp['settings']['action_ticks']*exp['settings']['dt']
    expected = {cid for triangle in exp['triangles'] for cid in triangle['anchor_ids']}
    if set(by_id) != expected:
        raise ValueError('Provide exactly the six original verified training anchors')
    prepared = []
    for case in exp['cases']:
        if case['split'] != args.split:
            continue
        triangle = next(t for t in exp['triangles'] if t['branch'] == case['branch'])
        target = dict(case, simulation_context=exp['simulation_context'], duration_s=duration,
                      motion_end_s=exp['settings']['approach_steps']*exp['settings']['action_ticks']*exp['settings']['dt'])
        anchors = [by_id[cid] for cid in triangle['anchor_ids']]
        # Same fixed task phase boundary/duration for every training trajectory.
        # Preserve the complete hold window by using per-tick samples.
        prepared.append(interpolate(anchors, target, samples=round(duration/exp['settings']['dt'])+1))
    args.output.mkdir(parents=True, exist_ok=False)
    for trace in prepared:
        trace.save(args.output/(trace.metadata['case_id']+'.json'))
    print(f'Wrote {len(prepared)} {args.split} candidates; no extra RL performed.')
    return 0


def validate(args):
    from .environment import ReplayAdapter
    exp = load_experiment(args.experiment)
    paths = sorted(args.trajectories.glob('*.json'))
    if not paths:
        raise ValueError('No trajectories')
    results = []; env = None
    expected = {c['case_id'] for c in exp['cases'] if c['split'] == args.split}
    traces = [Trajectory.load(path) for path in paths]
    if {t.metadata['case_id'] for t in traces} != expected or len(traces) != len(expected):
        raise ValueError('Evaluation directory must contain exactly the selected split, without duplicates')
    for path, trajectory in zip(paths, traces):
        case = case_by_id(exp, trajectory.metadata['case_id'])
        if env is None:
            env = create_env(exp, case, args.output.parent, args)
        result = replay(trajectory, ReplayAdapter(env), dt=exp['settings']['dt'])
        results.append(dict(case_id=case['case_id'], band=case['band'], split=case['split'],
                            trajectory_sha256=sha(path), **result['evaluation']))
    count = sum(bool(r['success']) for r in results)
    write_json(args.output, dict(schema='elesim-hug-evaluation', schema_version=1,
               simulation_context=exp['simulation_context'], successes=count, total=len(results), results=results))
    print(f'{count}/{len(results)} successful; failures are results, not requests for automatic RL.')
    return 0 if count == len(results) else 1


def check(args):
    exp = load_experiment(args.experiment)
    missing = [name for name in ('torch', 'genesis', 'tensordict', 'rsl_rl') if importlib.util.find_spec(name) is None]
    report = dict(missing=missing, model_and_source_match=True, physics_tested=False)
    if missing:
        write_json(args.output, report)
        print(json.dumps(report, indent=2)); return 2
    report['versions'] = check_versions()
    import torch
    if args.backend == 'gpu' and not torch.cuda.is_available():
        report['error'] = 'CUDA unavailable'
        write_json(args.output, report)
        print(json.dumps(report, indent=2)); return 2
    if args.physics:
        samples = []
        env = None
        for case in (c for c in exp['cases'] if c['split'] == 'train'):
            if env is None:
                env = create_env(exp, case, args.output.parent, args)
            env.set_case(case); env.reset(); env.verify_fk()
            # Ten physics ticks, no policy and no training. Detect invalid reset.
            for _ in range(10):
                env.tick(env.start.expand(1, 4))
            samples.append(dict(case_id=case['case_id'], **env.measure()))
        env.reset()
        start_pos = env.scene.object.get_pos().clone()
        velocity = env.scene.object.get_dofs_velocity().clone()
        velocity[:, 1] += exp['settings']['kick_velocity_m_s']
        env.scene.object.set_dofs_velocity(velocity)
        for _ in range(exp['settings']['hold_steps']*exp['settings']['action_ticks']):
            env.tick(env.start.expand(1, 4))
        ungrasped = float((env.scene.object.get_pos()-start_pos).norm(dim=1)[0])
        probe = runner(env, None)
        with torch.inference_mode():
            actions = probe.get_inference_policy(device=env.device)(env.get_observations())
        if actions.shape != (1, 2) or not torch.isfinite(actions).all():
            raise RuntimeError('PPO/environment interface check failed')
        report.update(physics_tested=True, samples=samples, ungrasped_displacement_m=ungrasped,
                      runner_constructed=True)
        report['ready'] = all(not r['fault'] for r in samples) and ungrasped > exp['settings']['hold_displacement_m']
    else:
        report['ready'] = None
    if args.output:
        write_json(args.output, report)
    print(json.dumps(report, indent=2))
    return 1 if report.get('ready') is False else 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    commands = p.add_subparsers(dest='command', required=True)
    c = commands.add_parser('configure'); c.add_argument('--pilot', type=Path, required=True)
    c.add_argument('--settings', type=Path); c.add_argument('--output', type=Path, required=True)
    for name in ('check', 'train', 'collect', 'replay', 'interpolate', 'validate'):
        c = commands.add_parser(name); c.add_argument('--experiment', type=Path, required=True)
        c.add_argument('--backend', choices=('gpu', 'cpu'), default='gpu')
        if name in ('check', 'collect', 'replay', 'validate'):
            c.add_argument('--viewer', action='store_true')
        if name == 'check':
            c.add_argument('--physics', action='store_true'); c.add_argument('--output', type=Path, required=True)
        elif name == 'train':
            c.add_argument('--case'); c.add_argument('--all', action='store_true')
            c.add_argument('--runs', type=Path, required=True); c.add_argument('--num-envs', type=int, default=256)
            c.add_argument('--iterations', type=int, default=2000); c.add_argument('--resume', action='store_true')
        elif name == 'collect':
            c.add_argument('--run', type=Path, required=True); c.add_argument('--checkpoint', type=Path)
            c.add_argument('--output', type=Path, required=True)
        elif name == 'replay':
            c.add_argument('--trajectory', type=Path, required=True); c.add_argument('--output', type=Path, required=True)
            c.add_argument('--verified-output', type=Path)
        elif name == 'interpolate':
            c.add_argument('--anchors', type=Path, required=True); c.add_argument('--output', type=Path, required=True)
            c.add_argument('--split', choices=('validation', 'test'), default='validation')
        elif name == 'validate':
            c.add_argument('--trajectories', type=Path, required=True); c.add_argument('--output', type=Path, required=True)
            c.add_argument('--split', choices=('validation', 'test'), default='validation')
    args = p.parse_args(argv)
    if args.command == 'configure':
        overrides = json.loads(args.settings.read_text()) if args.settings else None
        write_json(args.output, configure(args.pilot, overrides)); return 0
    return {'check': check, 'train': train, 'collect': collect, 'replay': replay_one,
            'interpolate': make_candidates, 'validate': validate}[args.command](args)


if __name__ == '__main__':
    raise SystemExit(main())
