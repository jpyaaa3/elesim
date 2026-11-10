"""Explicit prepare/run CLI for reverse GA; prepare never initializes Genesis."""
import argparse
from contextlib import nullcontext
import json
from pathlib import Path
import time
import numpy as np
from .prepare import prepare,save,digest
from .search import seed_population,decode,breed,quality


def run(args):
    from workbench.research.experiments.hug_c.configure import asset_digest
    from workbench.research.experiments.hug_c.experiment import check_versions
    plan=json.loads((args.plan/'plan.json').read_text())
    if (plan.get('schema')!='hug-reverse-ga-plan' or plan.get('version')!=1
        or plan['source_sha256']!=digest() or plan['simulation_context']['model_sha256']!=asset_digest()):
        raise ValueError('Plan/source/model mismatch; prepare again')
    check_versions()
    output=args.plan/'run';output.mkdir(exist_ok=False)
    started=time.monotonic()
    try:
        result=search(plan,output,args.backend)
    except BaseException as exc:
        save(output/'completion.json',dict(status='execution_error',error=repr(exc),elapsed_s=time.monotonic()-started))
        raise
    result['elapsed_s']=time.monotonic()-started;result['automatic_ppo_launch']=False
    save(output/'completion.json',result)
    print(json.dumps(result),flush=True)


def search(plan,output,backend):
    from workbench.research.experiments.hug_ga.parade import create_env,record,due
    from .physics import admit,evaluate,arrays,sample
    from .criteria import REASONS
    case=plan['cases'][0];n=plan['population']
    save(output/'provenance.json',dict(plan=plan,violation_columns=REASONS))
    env=create_env(plan,case,output/'assets',n_envs=n,backend=backend)
    with record(env,output/'initial-grasp.mp4'):
        passed=admit(env,output/'admission')
    if not passed:return dict(status='initial_grasp_invalid',generations_evaluated=0)
    state=env.scene.scene.get_state();baseline=sample(env)
    lower,upper,rate=arrays(env);start=np.asarray(case['goal_command']);home=np.asarray(plan['home'])
    steps=round(20/env.s['dt']);rng=np.random.default_rng(plan['seed'])
    population=seed_population(n,plan['knots'],start,home,lower,upper,rng)
    for generation in range(plan['generations']):
        folder=output/f'generation-{generation:04d}';folder.mkdir()
        paths=decode(population,start,home,lower,upper,rate,steps)
        np.savez_compressed(folder/'candidates.npz',genomes=population,commands=paths)
        with record(env,folder/'parade.mp4') if due(generation+1,plan['parade_every']) else nullcontext():
            reports=evaluate(env,paths,folder,state,baseline)
        best=max(range(n),key=lambda i:quality(reports[i]))
        progress=dict(generation=generation+1,generations=plan['generations'],best=reports[best],
            successes=sum(r['success'] for r in reports),clean=sum(not r['rejected'] for r in reports),
            direction='grasp-to-home')
        save(output/'progress.json',progress);print(json.dumps(progress),flush=True)
        if reports[best]['success']:
            # A short successful search must still have a full-population parade.
            if not due(generation+1,plan['parade_every']):
                parade_folder=output/'final-population';parade_folder.mkdir()
                with record(env,parade_folder/'parade.mp4'):
                    evaluate(env,paths,parade_folder,state,baseline)
            # Three fresh preparations, not three copies of the search snapshot.
            replay_results=[]
            for repeat in range(3):
                folder=output/f'verification-{repeat+1}';folder.mkdir()
                if not admit(env,folder/'admission'):
                    replay_results.append(False);continue
                replay_state=env.scene.scene.get_state();replay_baseline=sample(env)
                replay_paths=np.repeat(paths[best:best+1],n,axis=0)
                with record(env,folder/'replay.mp4'):
                    replay=evaluate(env,replay_paths,folder,replay_state,replay_baseline)
                replay_results.append(all(r['success'] for r in replay))
            verified=all(replay_results)
            save(output/'curriculum.json',dict(status='reverse_path_verified' if verified else 'verification_failed',
                generation=generation,candidate=best,replays=replay_results,
                states=f'generation-{generation:04d}/states.npz',lane=best,
                reset='Replay admitted reverse prefix preserving object pose and velocities',
                forward_success=False,lift_success=False,reset_pool_validated=False))
            return dict(status='reverse_path_verified' if verified else 'verification_failed',
                        generations_evaluated=generation+1,replays=replay_results)
        population=breed(population,reports,rng)
    return dict(status='budget_exhausted',generations_evaluated=plan['generations'])


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('prepare')
    p.add_argument('--mapper',type=Path,required=True)
    p.add_argument('--x',type=float,required=True,help='Mapper X in mm (C-band: nonpositive)')
    p.add_argument('--y',type=float,required=True,help='Mapper Y in mm (nonnegative)')
    p.add_argument('--diameter',type=float,required=True,help='Cylinder diameter in mm')
    p.add_argument('--mu-max',type=float,default=.4);p.add_argument('--seed-step',type=float,default=3.)
    p.add_argument('--population',type=int,default=128);p.add_argument('--generations',type=int,default=100)
    p.add_argument('--knots',type=int,default=6);p.add_argument('--seed',type=int,default=42)
    p.add_argument('--output',type=Path,required=True)
    p=sub.add_parser('run');p.add_argument('--plan',type=Path,required=True)
    p.add_argument('--backend',choices=('gpu','cpu'),default='gpu')
    args=parser.parse_args()
    if args.command=='prepare':
        if not prepare(args):raise SystemExit(1)
        print('Prepared; no simulation started:',args.output)
    else:run(args)


if __name__=='__main__':main()
