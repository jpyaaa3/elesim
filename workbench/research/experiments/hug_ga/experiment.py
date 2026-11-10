"""Prepare without GPU; run explicitly to search Home trajectories in Genesis."""
import argparse
import hashlib
import json
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
import time
import numpy as np
from .search import seed_population, decode, breed, quality
from .parade import due
from .constraints import motion_contract,read_motion_contract,monitor_motion


def save(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False))


def source_digest():
    h = hashlib.sha256()
    for path in sorted(Path(__file__).parent.glob('*.py')):
        h.update(path.name.encode()); h.update(path.read_bytes())
    return h.hexdigest()


def prepare(args):
    from workbench.research.experiments.hug_c.configure import load_experiment
    exp = load_experiment(args.experiment)
    case = next(c for c in exp['cases'] if c['case_id'] == args.case)
    if case['split'] != 'train': raise ValueError('Only train cases may enter GA')
    if args.population < 2 or args.knots < 2 or args.generations < 1:
        raise ValueError('Require population/knots >=2 and generations >=1')
    constraints=motion_contract(args.max_object_xy_mm)
    args.output.mkdir(parents=True, exist_ok=False)
    save(args.output/'plan.json', dict(schema='hug-ga-plan', version=2,
         experiment=exp, case_id=args.case, population=args.population,
         knots=args.knots, generations=args.generations, seed=args.seed,
         ga_source_sha256=source_digest(), start='home', physics_tested=False, parade_every=10,
         constraints=constraints))
    print('Prepared only; no Genesis or search started:', args.output)


def evaluate(env, paths, directory, *, constraints=None):
    """One candidate per physical lane; terminated lanes receive no more credit."""
    import torch
    from workbench.research.experiments.hug_c.control import action_rates
    constraints=read_motion_contract(dict(constraints=constraints))
    n = len(paths); dt = env.s['dt']*env.s['action_ticks']
    active = np.ones(n, bool); lengths = np.zeros(n, int)
    wrong = np.zeros(n, bool); faults = np.zeros(n, bool); success = np.zeros(n, bool)
    grasp = np.zeros(n); lift = np.zeros(n); hold = np.zeros(n); distance = np.zeros(n)
    raised=np.zeros(n,bool);roll_error=np.zeros(n)
    observations = []; actions_saved = []; masks = []; object_positions = []; contacts = []
    with monitor_motion(env,constraints) as movement, torch.inference_mode():
        obs, _ = env.reset()
        for step in range(paths.shape[1]-1):
            current = env.mapper.waypoint.cpu().numpy()
            actions = np.clip((paths[:, step+1]-current)/action_rates(env.s), -1, 1)
            actions[~active] = 0
            observations.append(obs['policy'].cpu().numpy().copy())
            actions_saved.append(actions.copy()); masks.append(active.copy())
            obs, _, done, _ = env.step(torch.as_tensor(actions, device=env.device, dtype=torch.float32))
            outcome = env.outcome()
            def array(value): return value.detach().cpu().numpy()
            motion_bad=array(movement.violated)
            lengths += active
            wrong |= active & (array(env.macro_wrong) > 0)
            faults |= active & array(env.fault)
            success |= active & ~motion_bad & array(outcome['success'])
            grasp = np.maximum(grasp, np.where(active, np.clip(array(env.grasp_ticks)*env.s['dt']/env.s['grasp_confirm_s'], 0, 1), 0))
            lift = np.maximum(lift, np.where(active, array(env.ledger.best_lift), 0))
            hold = np.maximum(hold, np.where(active, array(env.ledger.best_hold), 0))
            distance = np.where(active, array(env.relative_object().norm(dim=-1)), distance)
            raised=np.where(active,array(outcome['lift_posture_complete']),raised)
            roll_error=np.where(active,array(outcome['lift_roll_error_rad']),roll_error)
            object_positions.append(array(env.scene.object.get_pos()).copy())
            contacts.append(array(env.last_touch).copy())
            active &= ~(array(done)|motion_bad)
            movement.active.copy_(torch.as_tensor(active,device=env.device))
            if not active.any(): break
    peak=movement.peak_mm.cpu().numpy();ground_peak=movement.peak_ground_mm.cpu().numpy()
    motion_bad=movement.violated.cpu().numpy()
    first_motion=movement.first_tick.cpu().numpy()
    reports = []
    for i in range(n):
        passed = bool(success[i] and not faults[i] and not wrong[i] and not motion_bad[i])
        reason = ('object_motion_limit' if motion_bad[i] else
                  'forbidden_contact_or_limit' if faults[i] else 'wrong_face' if wrong[i]
                  else 'success' if passed else 'no_grasp' if grasp[i]<1 else
                  'no_lift' if lift[i]<=0 else 'lift_incomplete' if lift[i]<1 else
                  'lift_posture_incomplete' if not raised[i] else 'hold_incomplete')
        reports.append(dict(candidate=i, success=passed, fault=bool(faults[i]),
            wrong_face_any=bool(wrong[i]), grasp_fraction=float(grasp[i]),
            lift_fraction=float(lift[i]), hold_fraction=float(hold[i]),
            lift_posture_complete=bool(raised[i]),lift_roll_error_rad=float(roll_error[i]),
            object_motion_violation=bool(motion_bad[i]),max_object_xy_displacement_mm=float(peak[i]),
            max_ground_object_xy_displacement_mm=float(ground_peak[i]),
            first_object_motion_violation_tick=int(first_motion[i]),
            object_motion_limit_mm=constraints['max_object_xy_mm'],
            terminal_distance_m=float(distance[i]), duration_s=float(lengths[i]*dt),
            reason=reason, imitation_eligible=False, requires_independent_replay=passed))
    np.savez_compressed(directory/'rollouts.npz', schema=np.asarray('hug-ga-rollouts-v1'),
        observations=np.asarray(observations), actions=np.asarray(actions_saved),
        valid=np.asarray(masks), object_positions=np.asarray(object_positions),
        peak_ground_object_xy_displacement_mm=ground_peak,
        contacts=np.asarray(contacts), lengths=lengths,peak_object_xy_displacement_mm=peak,
        first_object_motion_violation_tick=first_motion,
        metadata_json=np.asarray(json.dumps(dict(case=env.case, simulation_context=env.experiment['simulation_context'],
            constraints=constraints,observation_timing='before action', object_contact_timing='after action',
            note='Search data, not independently verified imitation demonstrations'))))
    return reports


def run(args):
    from workbench.research.experiments.hug_c.configure import load_experiment
    from workbench.research.experiments.hug_c.experiment import check_versions
    from workbench.research.experiments.hug_c.control import command_bounds, action_rates
    plan = json.loads((args.plan/'plan.json').read_text())
    if plan['schema']!='hug-ga-plan' or plan['version']!=2 or plan['ga_source_sha256']!=source_digest():
        raise ValueError('Plan/source mismatch; prepare a new plan')
    constraints=read_motion_contract(plan)
    # Validate the original context without rewriting its source provenance.
    exp_path = args.plan/'experiment.json'
    if exp_path.exists(): raise FileExistsError('Use a fresh prepared directory')
    save(exp_path, plan['experiment']); exp = load_experiment(exp_path)
    check_versions()
    from .parade import create_env, record
    case = next(c for c in exp['cases'] if c['case_id']==plan['case_id'])
    started=time.monotonic()
    env = create_env(exp, case, args.plan/'assets', n_envs=plan['population'], backend=args.backend)
    import imageio.v2 as imageio
    imageio.imwrite(args.plan/'parade-preview.png',env.scene.cameras['parade'].render()[0])
    initialization_s=time.monotonic()-started
    lower, upper = command_bounds(exp['settings']); start = np.asarray(case['start_command'])
    rng = np.random.default_rng(plan['seed'])
    lift_goal=np.asarray(case['goal_command']).copy();lift_goal[1]=exp['settings']['lift_roll_target_rad']
    population = seed_population(plan['population'], plan['knots'], start,
                                 np.asarray(case['goal_command']), lower, upper, rng,lift_goal=lift_goal)
    durations=[]
    for generation in range(plan['generations']):
        generation_started=time.monotonic()
        directory = args.plan/f'generation-{generation:04d}'; directory.mkdir()
        paths = decode(population, start, lower, upper, action_rates(env.s), env.max_episode_length)
        np.savez_compressed(directory/'candidates.npz', genomes=population, commands=paths)
        if due(generation+1, plan.get('parade_every',10)):
            with record(env,directory/'parade.mp4'):
                reports = evaluate(env, paths, directory,constraints=constraints)
        else:
            reports = evaluate(env, paths, directory,constraints=constraints)
        best = max(range(len(reports)), key=lambda i: quality(reports[i]))
        elapsed=time.monotonic()-generation_started
        durations.append(elapsed)
        save(directory/'report.json', dict(reports=reports, best=best, generation_wall_s=elapsed,
             video_recorded=due(generation+1,plan.get('parade_every',10)),
             constraints=constraints,ranking='feasibility-success-lift90-motion-progress-v3'))
        state = dict(completed_generations=generation+1, candidate_evaluations=(generation+1)*len(population),
                     target_generations=plan['generations'],population=plan['population'],constraints=constraints,
                     updated_utc=datetime.now(timezone.utc).isoformat(),initialization_s=initialization_s,
                     generation_wall_s=elapsed,elapsed_s=time.monotonic()-started,
                     estimated_remaining_s=float(np.mean(durations[-10:]))*(plan['generations']-generation-1),
                     estimate_note='Recent generation mean; video overhead uncertain until first parade',
                     rejection_counts=dict(Counter(r['reason'] for r in reports)),
                     best=reports[best], phase='success-found' if any(r['success'] for r in reports) else 'searching')
        save(args.plan/'status.json', state); print(json.dumps(state), flush=True)
        if any(r['success'] for r in reports): return 0
        population = breed(population, reports, rng)
    state['phase']='budget-exhausted'; save(args.plan/'status.json', state)
    return 1


def smoke(args):
    """Short actual GPU batch/video check, not a search or success certificate."""
    from workbench.research.experiments.hug_c.configure import load_experiment
    from workbench.research.experiments.hug_c.experiment import check_versions
    from workbench.research.experiments.hug_c.control import command_bounds,action_rates
    from .parade import create_env,record
    exp=load_experiment(args.experiment);check_versions()
    case=next(c for c in exp['cases'] if c['case_id']==args.case)
    args.output.mkdir(parents=True,exist_ok=False)
    env=create_env(exp,case,args.output/'assets',n_envs=args.population,backend=args.backend)
    lower,upper=command_bounds(exp['settings'])
    start=np.asarray(case['start_command']);rng=np.random.default_rng(42)
    lift_goal=np.asarray(case['goal_command']).copy();lift_goal[1]=exp['settings']['lift_roll_target_rad']
    population=seed_population(args.population,6,start,np.asarray(case['goal_command']),lower,upper,rng,lift_goal=lift_goal)
    paths=decode(population,start,lower,upper,action_rates(env.s),env.max_episode_length)
    with record(env,args.output/'parade.mp4'):
        reports=evaluate(env,paths[:,:args.steps+1],args.output,constraints=motion_contract(args.max_object_xy_mm))
    save(args.output/'smoke.json',dict(population=args.population,control_steps=args.steps,
        video_created=(args.output/'parade.mp4').stat().st_size>0,
        ga_source_sha256=source_digest(),simulation_context=exp['simulation_context'],
        purpose='Batch and video execution only',reports=reports))
    return 0


def recorded_run(args):
    """Keep an explicit terminal record after detached execution, including faults."""
    started=datetime.now(timezone.utc).isoformat()
    try:
        result=run(args)
    except BaseException as error:
        save(args.plan/'completion.json',dict(started_utc=started,
             finished_utc=datetime.now(timezone.utc).isoformat(),status='error',
             error=repr(error),automatic_ppo_started=False))
        raise
    save(args.plan/'completion.json',dict(started_utc=started,
         finished_utc=datetime.now(timezone.utc).isoformat(),returncode=result,
         status='success-found' if result==0 else 'budget-exhausted',automatic_ppo_started=False))
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('prepare'); p.add_argument('--experiment', type=Path, required=True)
    p.add_argument('--case', required=True); p.add_argument('--output', type=Path, required=True)
    p.add_argument('--population', type=int, default=128); p.add_argument('--knots', type=int, default=6)
    p.add_argument('--generations', type=int, default=100); p.add_argument('--seed', type=int, default=42)
    p.add_argument('--max-object-xy-mm',type=float,default=None,help='Disqualify initial-relative horizontal center excursions above this many mm while touching the floor; airborne motion is exempt')
    p = sub.add_parser('run'); p.add_argument('--plan', type=Path, required=True)
    p.add_argument('--backend', choices=('cpu','gpu'), default='gpu')
    p = sub.add_parser('smoke'); p.add_argument('--experiment',type=Path,required=True)
    p.add_argument('--case',required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--population',type=int,default=128);p.add_argument('--steps',type=int,default=10)
    p.add_argument('--max-object-xy-mm',type=float,default=None)
    p.add_argument('--backend',choices=('cpu','gpu'),default='gpu')
    args=parser.parse_args(argv)
    if args.command=='smoke' and (args.population<2 or args.steps<1):parser.error('Positive steps and population >=2 required')
    return {'prepare':prepare,'run':recorded_run,'smoke':smoke}[args.command](args)


if __name__=='__main__': raise SystemExit(main())
