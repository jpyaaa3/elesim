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
from .configure import CHANNELS, episode_duration, CONTROL_MODE, DEFAULT_NUM_ENVS
from .interpolate import interpolate
from workbench.research.experiments.hug_interpolation.video import record_video


def case_by_id(experiment, case_id):
    return next(c for c in experiment['cases'] if c['case_id'] == case_id)


def runner(env, directory):
    from elesim_sim.rl.runner import make_runner
    from workbench.research.experiments.wrap_grasp.configs.loader import load_config, to_dict
    cfg = dict(to_dict(load_config().train.runner))
    cfg.update(num_steps_per_env=16, save_interval=100, experiment_name='c-hug-home-lift',
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
                  backend=args.backend, viewer=getattr(args, 'viewer', False), auto_reset=auto_reset,
                  video=bool(getattr(args, 'video', None)))


def train(args):
    exp = load_experiment(args.experiment)
    preflight=json.loads(args.preflight.read_text())
    if (preflight.get('ready') is not True or preflight.get('physics_tested') is not True
        or preflight.get('simulation_context')!=exp['simulation_context']
        or set(preflight.get('case_ids',[]))!={c['case_id'] for c in exp['cases'] if c['split']=='train'}):
        raise ValueError('A matching successful three-anchor physics preflight is required before training')
    if args.all:
        if args.case:
            raise ValueError('--all and --case are mutually exclusive')
        for case in exp['cases']:
            if case['split'] != 'train':
                continue
            command = [sys.executable, '-m', __package__+'.experiment', 'train',
                       '--experiment', str(args.experiment), '--case', case['case_id'],
                       '--runs', str(args.runs), '--num-envs', str(args.num_envs),
                       '--iterations', str(args.iterations), '--backend', args.backend,'--preflight',str(args.preflight)]
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
        # rsl_rl 5.4 saves the last completed index, while learn() starts at
        # current_learning_iteration. Continue at the next update index.
        policy_runner.current_learning_iteration += 1
    started = time.monotonic()
    policy_runner.learn(num_learning_iterations=args.iterations, init_at_random_ep_len=False)
    policy_runner.save(str(run/'model_final.pt'))
    with (run/'training_attempts.jsonl').open('a') as stream:
        stream.write(json.dumps(dict(iterations=args.iterations, elapsed_s=time.monotonic()-started,
                                    resumed_from=str(resume) if resume else None, versions=check_versions()))+'\n')
    return 0


def export(args):
    from .policy import export_policy
    exp = load_experiment(args.experiment)
    record = json.loads((args.run/'run.json').read_text())
    if record['experiment'] != exp:
        raise ValueError('Run and experiment differ')
    if args.output.suffix.lower() != '.npz' or args.output.exists():
        raise ValueError('Choose a new .npz output')
    checkpoint = args.checkpoint or args.run/'model_final.pt'
    if not checkpoint.is_file():
        raise FileNotFoundError(checkpoint)
    env = create_env(exp, case_by_id(exp, record['case_id']), args.run, args)
    policy_runner = runner(env, None)
    policy_runner.load(str(checkpoint))
    error = export_policy(policy_runner.alg.get_policy(), args.output,
                          dict(env.metadata(), checkpoint_sha256=sha(checkpoint)))
    print(f'Exported policy weights to {args.output}; NumPy/Torch max error {error:.3g}')
    return 0


def collect(args):
    import torch
    from .policy import NumpyPolicy
    exp = load_experiment(args.experiment)
    if args.output.suffix.lower() != '.npz' or args.output.exists():
        raise ValueError('Choose a new .npz trajectory output')
    if args.baseline:
        if args.policy or args.run or args.checkpoint or not args.case:
            raise ValueError('Use --baseline with --case and without a policy/checkpoint')
        case=case_by_id(exp,args.case)
        env=create_env(exp,case,args.output.parent,args)
        origin=dict(controller='zero-four-dof-actions-at-home',trained=False)
    elif args.policy:
        if args.checkpoint or args.run:
            raise ValueError('Use --policy alone, or --run with optional --checkpoint')
        policy = NumpyPolicy(args.policy)
        if policy.metadata['simulation_context'] != exp['simulation_context']:
            raise ValueError('Policy and experiment context differ')
        case = case_by_id(exp, policy.metadata['case_id'])
        origin = dict(policy_sha256=sha(args.policy))
        env = create_env(exp, case, args.output.parent, args)
    else:
        if args.run is None:
            raise ValueError('Provide --policy or --run')
        record = json.loads((args.run/'run.json').read_text())
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
        origin = dict(checkpoint_sha256=sha(checkpoint))
    obs, _ = env.reset()
    env.record = [env.start.detach().cpu().numpy().copy()]
    with record_video(env, getattr(args, 'video', None)), torch.inference_mode():
        for _ in range(env.max_episode_length):
            if args.baseline:
                actions=torch.zeros((1,4),device=env.device)
            elif args.policy:
                actions = torch.as_tensor(policy(obs['policy'].cpu().numpy()), device=env.device)
            else:
                actions = policy(obs)
            obs, _, done, _ = env.step(actions)
            if bool(done[0]):break
    result = env.measure()
    trajectory = Trajectory(np.arange(len(env.record))*exp['settings']['dt'], np.asarray(env.record),
                            CHANNELS, dict(env.metadata(), **origin),
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
    if args.repetitions<1:raise ValueError('repetitions must be positive')
    results=[]
    for i in range(args.repetitions):
        with record_video(env,args.video if i==0 else None):
            results.append(replay(trajectory,ReplayAdapter(env),dt=exp['settings']['dt']))
    passed=all(r['evaluation']['success'] for r in results)
    result=dict(schema='elesim-c-hug-repeated-replay',schema_version=1,
                trajectory_sha256=sha(args.trajectory),repetitions=args.repetitions,success=passed,
                evaluations=[r['evaluation'] for r in results],first_replay=results[0])
    write_json(args.output,result)
    if args.verified_output:
        trajectory.diagnostics.update(replay_success=bool(passed),physics_validated=True,
            replay_repetitions=args.repetitions,replay_evaluation=results[0]['evaluation'])
        trajectory.save(args.verified_output)
    print(json.dumps(result['evaluations'],indent=2))
    return 0 if passed else 1


def make_candidates(args):
    exp = load_experiment(args.experiment)
    by_id = {}
    for path in args.anchors.glob('*.npz'):
        trace = Trajectory.load(path)
        cid = trace.metadata['case_id']
        if cid in by_id:
            raise ValueError(f'Duplicate anchor {cid}')
        if case_by_id(exp, cid)['split'] != 'train':
            raise ValueError('Anchor directory contains a held-out case')
        by_id[cid] = trace
    duration=max(trace.times[-1] for trace in by_id.values()) if by_id else 0.
    expected = {cid for triangle in [exp['triangle']] for cid in triangle['anchor_ids']}
    if set(by_id) != expected:
        raise ValueError('Provide exactly the three original verified C training anchors')
    prepared = []
    for case in exp['cases']:
        if case['split'] != args.split:
            continue
        triangle = next(t for t in [exp['triangle']] if t['branch'] == case['branch'])
        target = dict(case, simulation_context=exp['simulation_context'], duration_s=duration,
                      control_mode=CONTROL_MODE,episode_limit_s=episode_duration(exp['settings']))
        anchors = [by_id[cid] for cid in triangle['anchor_ids']]
        # Normalize free command paths to the longest original duration.
        # Retain per-physics-tick samples for subsequent physical replay.
        prepared.append(interpolate(anchors, target, samples=round(duration/exp['settings']['dt'])+1))
    args.output.mkdir(parents=True, exist_ok=False)
    for trace in prepared:
        trace.save(args.output/(trace.metadata['case_id']+'.npz'))
    print(f'Wrote {len(prepared)} {args.split} candidates; no extra RL performed.')
    return 0


def validate(args):
    from .environment import ReplayAdapter
    exp = load_experiment(args.experiment)
    paths = sorted(args.trajectories.glob('*.npz'))
    if not paths:
        raise ValueError('No trajectories')
    from .metrics import position_offsets_mm
    offsets=position_offsets_mm(args.offsets_mm)
    if args.limit is not None and args.limit<1:raise ValueError('Evaluation limit must be positive')
    results=[];env=None
    expected={c['case_id'] for c in exp['cases'] if c['split']==args.split}
    traces=[Trajectory.load(path) for path in paths]
    if {t.metadata['case_id'] for t in traces}!=expected or len(traces)!=len(expected):
        raise ValueError('Evaluation directory must contain exactly the selected split without duplicates')
    selected=list(zip(paths,traces))[:args.limit]
    for path,trajectory in selected:
        case=case_by_id(exp,trajectory.metadata['case_id'])
        if env is None:env=create_env(exp,case,args.output.parent,args)
        for offset in offsets:
            result=replay(trajectory,ReplayAdapter(env,offset),dt=exp['settings']['dt'])
            results.append(dict(case_id=case['case_id'],band=case['band'],split=case['split'],
                trajectory_sha256=sha(path),offset_xy_mm=offset,**result['evaluation']))
    count=sum(bool(r['success']) for r in results)
    write_json(args.output,dict(schema='elesim-c-hug-evaluation',schema_version=1,
        simulation_context=exp['simulation_context'],geometric_coverage=exp['selection'].get('coverage'),
        successes=count,total=len(results),evaluated_cases=len(selected),offsets_xy_mm=offsets,results=results))
    print(f'{count}/{len(results)} successful; no automatic extra RL.')
    return 0 if count==len(results) else 1


def check(args):
    exp = load_experiment(args.experiment)
    missing = [name for name in ('torch', 'genesis', 'tensordict', 'rsl_rl') if importlib.util.find_spec(name) is None]
    report = dict(missing=missing, model_and_source_match=True, physics_tested=False,
                  simulation_context=exp['simulation_context'],case_ids=[c['case_id'] for c in exp['cases'] if c['split']=='train'])
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
        samples=[];env=None
        for case in (c for c in exp['cases'] if c['split']=='train'):
            if env is None:env=create_env(exp,case,args.output.parent,args)
            env.set_case(case);env.reset();env.verify_fk()
            for _ in range(10):env.tick(env.start.expand(1,4))
            home=env.measure();env.reset()
            # Direct goal placement separates nominal feasibility from approach learning.
            env.mapper.waypoint[:]=env.goal
            targets=env.mapper.joint_targets()
            env.scene.robot.set_dofs_position(targets,dofs_idx_local=env.dofs,zero_velocity=True)
            env.scene.robot.control_dofs_position(targets,dofs_idx_local=env.dofs)
            for _ in range(10):env.tick(env.goal.expand(1,4))
            goal=env.measure()
            samples.append(dict(case_id=case['case_id'],q_deg=case['q_deg'],home=home,goal=goal))
        env.reset();probe=runner(env,None)
        with torch.inference_mode():actions=probe.get_inference_policy(device=env.device)(env.get_observations())
        if actions.shape!=(1,4) or not torch.isfinite(actions).all():raise RuntimeError('Invalid PPO/environment interface')
        report.update(physics_tested=True,samples=samples,runner_constructed=True,
                      approach_and_lift_validated=False,
                      excluded_q_recommendation=[r['q_deg'] for r in samples if r['home']['fault'] or r['goal']['fault']])
        report['ready']=all(not r['home']['fault'] and not r['goal']['fault'] for r in samples)
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
    for name in ('check', 'train', 'export', 'collect', 'replay', 'interpolate', 'validate'):
        c = commands.add_parser(name); c.add_argument('--experiment', type=Path, required=True)
        c.add_argument('--backend', choices=('gpu', 'cpu'), default='gpu')
        if name in ('check', 'collect', 'replay', 'validate'):
            c.add_argument('--viewer', action='store_true')
        if name in ('collect', 'replay'):
            c.add_argument('--video', type=Path, help='Save a single-environment oblique-view MP4 at physics ticks')
        if name == 'check':
            c.add_argument('--physics', action='store_true'); c.add_argument('--output', type=Path, required=True)
        elif name == 'train':
            c.add_argument('--case'); c.add_argument('--all', action='store_true')
            c.add_argument('--preflight',type=Path,required=True)
            c.add_argument('--runs', type=Path, required=True); c.add_argument('--num-envs', type=int, default=DEFAULT_NUM_ENVS)
            c.add_argument('--iterations', type=int, default=2000); c.add_argument('--resume', action='store_true')
        elif name in ('export', 'collect'):
            if name == 'collect':
                c.add_argument('--policy', type=Path)
                c.add_argument('--baseline',action='store_true')
                c.add_argument('--case')
            c.add_argument('--run', type=Path, required=name == 'export'); c.add_argument('--checkpoint', type=Path)
            c.add_argument('--output', type=Path, required=True)
        elif name == 'replay':
            c.add_argument('--trajectory', type=Path, required=True); c.add_argument('--output', type=Path, required=True)
            c.add_argument('--verified-output', type=Path)
            c.add_argument('--repetitions',type=int,default=3)
        elif name == 'interpolate':
            c.add_argument('--anchors', type=Path, required=True); c.add_argument('--output', type=Path, required=True)
            c.add_argument('--split', choices=('validation', 'test'), default='validation')
        elif name == 'validate':
            c.add_argument('--offsets-mm',type=float,nargs='+',default=[0.])
            c.add_argument('--limit',type=int,help='Evaluate a deterministic subset of the split')
            c.add_argument('--trajectories', type=Path, required=True); c.add_argument('--output', type=Path, required=True)
            c.add_argument('--split', choices=('validation', 'test'), default='validation')
    args = p.parse_args(argv)
    if args.command == 'configure':
        overrides = json.loads(args.settings.read_text()) if args.settings else None
        write_json(args.output, configure(args.pilot, overrides)); return 0
    return {'check': check, 'train': train, 'export': export, 'collect': collect, 'replay': replay_one,
            'interpolate': make_candidates, 'validate': validate}[args.command](args)


if __name__ == '__main__':
    raise SystemExit(main())
