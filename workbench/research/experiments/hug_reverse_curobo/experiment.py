"""Run a frozen reverse-GA case through cuRobo and unchanged Genesis criteria."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np


def timed_commands(positions, source_dt, start, home, lower, upper, rate, dt=.01):
    """Slow the entire polyline to satisfy tick slew; never clip its geometry."""
    p = np.asarray(positions, dtype=float)
    if p.ndim != 2 or p.shape[1] != 4 or len(p) < 2 or not np.isfinite(p).all():
        raise ValueError('Expected finite Nx4 trajectory')
    if not np.isfinite(source_dt) or source_dt <= 0:
        raise ValueError('Invalid trajectory timing')
    if not np.allclose(p[0], start, atol=1e-4, rtol=0) or not np.allclose(p[-1], home, atol=1e-4, rtol=0):
        raise ValueError('Planner endpoint mismatch')
    p = p.copy()
    original = p.copy()
    p[0], p[-1] = start, home
    # Only float32-scale roll jitter may be projected onto this task's
    # positive initial rotation. Larger corrections are rejected, not hidden.
    normalized = np.maximum.accumulate(np.clip(p[:, 1], start[1], home[1]))
    if np.max(np.abs(normalized-original[:, 1])) > 5e-7:
        raise ValueError('Roll normalization would alter a substantive reversal')
    p[:, 1] = normalized
    p = np.vstack((start, p, home))
    if np.any(p < lower-1e-7) or np.any(p > upper+1e-7):
        raise ValueError('Planner exceeds command bounds')
    segment_ticks = max(1, int(np.ceil(source_dt/dt)),
                        int(np.ceil(np.max(np.abs(np.diff(p, axis=0))/rate))))
    total = (len(p)-1)*segment_ticks
    steps = round(20/dt)
    if total > steps-round(1/dt):
        raise ValueError('Retimed path leaves less than one second for Home verification')
    times = np.arange(len(p))*segment_ticks
    return np.stack([np.interp(np.arange(steps+1), times, p[:,j]) for j in range(4)], axis=-1)[None]


def release_targets(start, home, lower, upper, count, seed):
    """Independent axis progress explores release families without fixing order."""
    rng = np.random.default_rng(seed)
    for i in range(count):
        fraction = rng.uniform(.05, .95, 4)
        # Include distal-first, proximal-first, roll-first and retraction-first.
        if i < 4:
            fraction[:] = .1
            fraction[i] = .8
        yield np.clip(start + fraction*(home-start), lower, upper)


def run(args):
    from workbench.research.experiments.hug_reverse_ga.prepare import digest, save
    from workbench.research.experiments.hug_reverse_ga.physics import admit, evaluate, sample, arrays
    from workbench.research.experiments.hug_reverse_ga.motion_budget import require_feasible
    from workbench.research.experiments.hug_c.configure import asset_digest
    from workbench.research.experiments.hug_c.experiment import check_versions
    from workbench.research.experiments.hug_ga.parade import create_env
    from elesim_pilot.robot.arm.curobo import CuRoboPlanner, CuRoboError
    plan = json.loads(args.plan.read_text())
    if (plan.get('schema') != 'hug-reverse-ga-plan' or plan.get('version') != 1
        or plan['source_sha256'] != digest()
        or plan['simulation_context']['model_sha256'] != asset_digest()):
        raise ValueError('Reverse plan/source/model mismatch; prepare a fresh matching plan')
    check_versions()
    case = plan['cases'][0]
    start, home = np.asarray(case['goal_command']), np.asarray(plan['home'])
    require_feasible(start, home)
    args.output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    result = dict(status='execution_error', automatic_ppo_launch=False)
    try:
        save(args.output/'provenance.json', dict(plan=plan,
            runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            planning_model_metadata=json.loads((args.model/'metadata.json').read_text()),
            planner_scope='Measured cylinder as 16 conservative cuboids plus arm self collision; floor and Go2 checked in Genesis',
            command_convention='linear m, roll rad, bends per-node rad; no additional divide by five',
            attempts=args.attempts, strategy=args.strategy, seed=plan['seed'], gpu_selection='Inherited CUDA_VISIBLE_DEVICES'))
        env = create_env(plan, case, args.output/'assets', n_envs=1, backend='gpu')
        if not admit(env, args.output/'admission'):
            result['status'] = 'initial_grasp_invalid'
            return result
        state, baseline = env.scene.scene.get_state(), sample(env)
        lower, upper, rate = arrays(env)
        import torch
        torch.cuda.synchronize()
        t = time.monotonic()
        from .world import from_env
        world = from_env(env)
        save(args.output/'planning-world.json', world)
        planner = CuRoboPlanner(args.model, world=world, interpolation_dt=.01)
        torch.cuda.synchronize()
        result['planner_initialization_s'] = time.monotonic()-t
        result['attempts'] = []
        # A fixed invalid endpoint cannot be repaired by repeating planning.
        for label, q in ((('home', home),) if args.strategy == 'offset-release' else (('start', start), ('home', home))):
            try:
                planner.validate_configuration(q)
            except CuRoboError as exc:
                result.update(status='planning_endpoint_invalid', endpoint=label, error=str(exc))
                return result
        targets = list(release_targets(start, home, lower, upper, args.attempts, plan['seed']))
        if args.strategy == 'offset-release':
            rng = np.random.default_rng(plan['seed'])
            # Bounded offsets: <=20 mm linear, <=10 deg roll, <=10 deg
            # total bend (2 deg per node). Never alter the object or model.
            span = np.array([.02, np.deg2rad(10), np.deg2rad(2), np.deg2rad(2)])
            targets = [np.clip(start+rng.uniform(-1,1,4)*span, lower, upper)
                       for _ in range(args.attempts)]
        for attempt in range(args.attempts):
            folder = args.output/f'candidate-{attempt:04d}'
            folder.mkdir()
            t = time.monotonic()
            try:
                if args.strategy == 'offset-release':
                    middle = targets[attempt]
                    save(folder/'offset.json', dict(command=middle.tolist(), delta=(middle-start).tolist(),
                         prefix='Rate-limited contact-release proposal, accepted only by full Genesis replay'))
                    trajectory = planner.plan_joint(middle, home)
                    # Explicit contact prefix is not falsely certified by cuRobo.
                    count = max(2, int(np.ceil(np.max(np.abs(middle-start)/rate)))+1)
                    prefix = np.linspace(start, middle, count)
                    suffix = timed_commands(trajectory.positions, trajectory.dt_s, middle, home,
                                            lower, upper, rate, env.s['dt'])[0]
                    planner.validate_path(suffix)
                    paths = np.concatenate((prefix[:-1],suffix),axis=0)[:round(20/env.s['dt'])+1][None]
                elif args.strategy == 'via-release':
                    middle = targets[attempt]
                    save(folder/'waypoint.json', dict(command=middle.tolist()))
                    first = planner.plan_joint(start, middle)
                    second = planner.plan_joint(middle, home)
                    if not np.isclose(first.dt_s, second.dt_s):
                        raise ValueError('Segment time grids differ')
                    from elesim_pilot.robot.arm.curobo import JointTrajectory
                    trajectory = JointTrajectory(first.positions + second.positions[1:], first.dt_s)
                else:
                    trajectory = planner.plan_joint(start, home)
                torch.cuda.synchronize()
                planning_s = time.monotonic()-t
                if args.strategy != 'offset-release':
                    paths = timed_commands(trajectory.positions, trajectory.dt_s, start, home,
                                           lower, upper, rate, env.s['dt'])
                    planner.validate_path(paths[0])
            except (CuRoboError, ValueError) as exc:
                entry = dict(status='proposal_failed', error=str(exc), planning_s=time.monotonic()-t)
                save(folder/'proposal.json', entry)
                result['attempts'].append(entry)
                continue
            np.savez_compressed(folder/'commands.npz', commands=paths,
                                raw_positions=trajectory.positions, raw_dt=trajectory.dt_s)
            t = time.monotonic()
            reports = evaluate(env, paths, folder, state, baseline)
            entry = dict(planning_s=planning_s, physics_s=time.monotonic()-t, report=reports[0])
            result['attempts'].append(entry)
            save(folder/'timing.json', entry)
            if not reports[0]['success']:
                continue
            replays = []
            for i in range(3):
                replay = args.output/f'verification-{i+1}'
                replay.mkdir()
                if not admit(env, replay/'admission'):
                    replays.append(False)
                    continue
                replay_state, replay_baseline = env.scene.scene.get_state(), sample(env)
                replays.append(bool(evaluate(env, paths, replay, replay_state, replay_baseline)[0]['success']))
            result.update(status='reverse_path_verified' if all(replays) else 'verification_failed',
                          replays=replays, candidate=attempt, forward_success=False,
                          lift_success=False, reset_pool_validated=False)
            return result
        result['status'] = 'budget_exhausted'
        return result
    except BaseException as exc:
        result.update(status='execution_error', error=repr(exc))
        raise
    finally:
        result['elapsed_s'] = time.monotonic()-started
        save(args.output/'completion.json', result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True, help='Existing matching reverse-GA plan.json')
    parser.add_argument('--output', type=Path, required=True, help='New independent directory')
    parser.add_argument('--model', type=Path, default=Path('payload/data/models/planning/zed-mini'))
    parser.add_argument('--attempts', type=int, default=8)
    parser.add_argument('--strategy', choices=('direct', 'via-release', 'offset-release'), default='direct')
    args = parser.parse_args()
    if args.attempts < 1:
        parser.error('attempts must be positive')
    result = run(args)
    print(json.dumps(result, indent=2))
    return 0 if result['status'] == 'reverse_path_verified' else 1


if __name__ == '__main__':
    raise SystemExit(main())
