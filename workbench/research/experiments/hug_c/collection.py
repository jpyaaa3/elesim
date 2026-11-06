"""Independent diameter studies and exact (x,y,D) selection; never mixed-D RL.

Only the explicit `run` subcommand starts learning. `prepare` appends new
diameter inputs without modifying existing inputs, checkpoints or NPZ files.
`resolve` returns anchor policies/paths and q-space weights, not motor commands.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np

from .configure import (ACTIONS, OBSERVATIONS, CHANNELS, CONTROL_MODE, DEFAULT_NUM_ENVS,
                        asset_digest, load_experiment, sha, write_json)
from .prepare import prepare
from .source import read_kernel, inverse
from .triangle import weights

DIAMETERS = (100.,125.,150.,175.,200.)


def load_collection(path):
    path=Path(path)
    plan=json.loads(path.read_text())
    if (plan.get('schema')!='elesim-c-hug-collection' or plan.get('schema_version')!=1
            or plan.get('control_mode')!=CONTROL_MODE
            or plan.get('contract',{}).get('action_names')!=list(ACTIONS)
            or plan.get('contract',{}).get('observation_names')!=list(OBSERVATIONS)):
        raise ValueError('Unsupported diameter collection/control mode')
    ds=[p['diameter_mm'] for p in plan['profiles']]
    if len(ds)!=len(set(ds)):
        raise ValueError('Duplicate diameters in collection')
    for p in plan['profiles']:
        for name in ('pilot','catalog'):
            file=(path.parent/p[name]).resolve()
            if not file.is_relative_to(path.parent.resolve()) or sha(file)!=p[name+'_sha256']:
                raise ValueError('Collection input changed or escaped its directory')
    return plan


def save_plan(path, plan):
    temporary=path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(plan,indent=2,allow_nan=False)+'\n')
    temporary.replace(path)


def prepare_collection(mapper, output, *, diameters=DIAMETERS, mu_max=.4, step=1., seed=42,
                       validation=30, test=100):
    ds=[float(d) for d in diameters]
    if not ds or len(ds)!=len(set(ds)) or any(not math.isfinite(d) or not 0<d<=1000 for d in ds):
        raise ValueError('Require unique positive diameters')
    _,source=read_kernel(mapper)
    output=Path(output);index=output/'collection.json'
    selection=dict(mu_max=mu_max,step_deg=step,seed=seed,validation=validation,test=test)
    contract=dict(kernel_sha256=source['kernel_sha256'],model_sha256=asset_digest(),
                  action_names=list(ACTIONS),observation_names=list(OBSERVATIONS))
    if index.exists():
        plan=load_collection(index)
        if plan['contract']!=contract or plan['selection']!=selection:
            raise ValueError('Collection model/function/settings differ; choose a new collection')
    else:
        output.mkdir(parents=True,exist_ok=False)
        plan=dict(schema='elesim-c-hug-collection',schema_version=1,control_mode=CONTROL_MODE,
                  contract=contract,selection=selection,profiles=[])
        save_plan(index,plan)
    for diameter in ds:
        if any(p['diameter_mm']==diameter for p in plan['profiles']):
            continue  # Adding 110 mm never rewrites an existing diameter's data.
        tag=f'd{diameter:g}';directory=output/tag
        if directory.exists():
            raise ValueError(f'Incomplete/foreign profile directory: {directory}')
        pilot=prepare(mapper,diameter=diameter,mu_max=mu_max,step=step,seed=seed,
                      validation=validation,test=test,preview=directory/'triangle.html',
                      catalog=directory/'catalog.json')
        write_json(directory/'pilot.json',pilot)
        profile=dict(diameter_mm=diameter,pilot=f'{tag}/pilot.json',catalog=f'{tag}/catalog.json',
                     study=f'{tag}/study',pilot_sha256=sha(directory/'pilot.json'),
                     catalog_sha256=sha(directory/'catalog.json'),
                     feasible_samples=pilot['selection']['feasible_samples'],
                     q_triangle=pilot['selection']['vertices_total_deg'],
                     geometric_coverage=pilot['selection']['coverage'],
                     splits=dict(train=3,validation=validation,test=test),
                     readiness='geometry-only; physics preflight and training not run')
        plan['profiles'].append(profile)
        save_plan(index,plan)
        print(f'D={diameter:g} mm: {profile["feasible_samples"]} feasible q cells; '
              f'{100*profile["geometric_coverage"]:.2f}% covered; no RL executed.',flush=True)
    return plan


def completed_study(index, profile):
    """Require successful original replays and all held-out gates before selection."""
    from elesim_sim.rl.trajectory import Trajectory
    from .policy import NumpyPolicy
    root=Path(index).parent/profile['study']
    if not (root/'status.json').is_file():
        raise ValueError(f'D={profile["diameter_mm"]:g} mm has no trained study')
    if json.loads((root/'status.json').read_text()).get('phase')!='finished':
        raise ValueError('Diameter study has not passed all evaluation gates')
    exp=load_experiment(root/'experiment.json')
    original=json.loads((Path(index).parent/profile['pilot']).read_text())
    if (exp['source']!=original['source'] or exp['selection']!=original['selection']
            or exp['triangle']!=original['triangle']):
        raise ValueError('Trained study and diameter profile differ')
    n_validation=sum(c['split']=='validation' for c in exp['cases'])
    n_test=sum(c['split']=='test' for c in exp['cases'])
    expected={'validation-report':n_validation,'position-robustness':min(5,n_validation)*16,'test-report':n_test}
    for name,total in expected.items():
        report=json.loads((root/(name+'.json')).read_text())
        if (report.get('schema')!='elesim-c-hug-evaluation'
                or report.get('simulation_context')!=exp['simulation_context']
                or not total or report.get('total')!=total or report.get('successes')!=total):
            raise ValueError('Diameter study has failed/mismatched evaluation evidence')
    anchors=[]
    for cid in exp['triangle']['anchor_ids']:
        trace=Trajectory.load(root/'anchors'/(cid+'.npz'))
        policy=NumpyPolicy(root/'policies'/(cid+'.npz'))
        case=next(c for c in exp['cases'] if c['case_id']==cid)
        if (trace.channels!=CHANNELS or trace.diagnostics.get('replay_success') is not True
                or trace.diagnostics.get('physics_validated') is not True
                or trace.diagnostics.get('replay_repetitions',0)<3):
            raise ValueError('Each selected anchor requires three successful original replays')
        for meta in (trace.metadata,policy.metadata):
            if meta.get('control_mode')!=CONTROL_MODE or meta.get('simulation_context')!=exp['simulation_context'] or any(
                    meta.get(k)!=case[k] for k in ('case_id','q_deg','diameter_mm','xy_mm','start_command','goal_command','branch','roll_deg','split')):
                raise ValueError('Selected policy/trajectory context or case differs')
        anchors.append(dict(case_id=cid,policy_npz=str((root/'policies'/(cid+'.npz')).resolve()),
                            trajectory_npz=str((root/'anchors'/(cid+'.npz')).resolve())))
    return exp,anchors


def resolve(index, mapper, xy, diameter, *, require_trained=True):
    plan=load_collection(index)
    if not math.isfinite(diameter) or len(xy)!=2 or not np.isfinite(xy).all():
        raise ValueError('Require finite x, y and diameter')
    profile=next((p for p in plan['profiles'] if p['diameter_mm']==diameter),None)
    if profile is None:
        raise ValueError(f'D={diameter:g} mm is not collected; no silent nearest-diameter selection')
    kernel,source=read_kernel(mapper)
    if source['kernel_sha256']!=plan['contract']['kernel_sha256'] or asset_digest()!=plan['contract']['model_sha256']:
        raise ValueError('Selector mapper/model differ from collection')
    pilot=json.loads((Path(index).parent/profile['pilot']).read_text())
    if require_trained:
        _,anchors=completed_study(index,profile)
    else:
        anchors=[dict(case_id=cid) for cid in pilot['triangle']['anchor_ids']]
    catalog=json.loads((Path(index).parent/profile['catalog']).read_text())
    rows=catalog['rows'];points=np.asarray([r['xy_mm'] for r in rows])
    nearest=np.argsort(np.sum((points-np.asarray(xy))**2,axis=1))[:32]
    candidates=inverse(kernel,diameter,xy,[rows[i] for i in nearest],mu_max=plan['selection']['mu_max'])
    feasible=[]
    for row in candidates:
        w=weights(profile['q_triangle'],row['q_deg'])
        if np.all(w>=-1e-9):
            w=np.maximum(w,0);w/=w.sum();feasible.append((row,w))
    if not feasible:
        raise ValueError('No valid C solution inside this trained q triangle found for exact XY')
    row,w=min(feasible,key=lambda pair:pair[0]['mu_min'])
    return dict(schema='elesim-c-hug-selection',schema_version=1,control_mode=CONTROL_MODE,
                diameter_mm=diameter,requested_xy_mm=list(xy),**row,weights=w.tolist(),anchors=anchors,
                readiness='validated-anchor-selection' if require_trained else 'geometry-only',
                candidate_requires_physics_validation=True,
                interpolation='blend time-normalized command paths in q space; never blend network weights',
                kernel_sha256=source['kernel_sha256'])


def run_collection(index, *, diameters=None, num_envs=DEFAULT_NUM_ENVS, iterations=2000, report_every=500):
    """Explicit opt-in execution: sequential diameters, three independent RL anchors each."""
    plan=load_collection(index)
    if plan['contract']['model_sha256']!=asset_digest():
        raise ValueError('Collection robot model changed')
    if min(num_envs,iterations,report_every)<1:
        raise ValueError('Environment count and iteration budgets must be positive')
    chosen=plan['profiles'] if diameters is None else [
        next((p for p in plan['profiles'] if p['diameter_mm']==d),None) for d in diameters]
    if not chosen or any(p is None for p in chosen):
        raise ValueError('Requested diameter is absent from collection')
    base=Path(index).resolve().parent
    for profile in chosen:
        study=base/profile['study']
        if study.exists():
            completed_study(index,profile)  # Keep completed diameters and NPZs unchanged.
            continue
        command=[sys.executable,'-u','-m',__package__+'.study','--pilot',str(base/profile['pilot']),
                 '--output',str(study),'--num-envs',str(num_envs),'--iterations',str(iterations),
                 '--report-every',str(report_every)]
        result=subprocess.run(command)
        if result.returncode:
            return result.returncode  # Do not train more anchors/diameters through a failed gate.
    return 0


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    p=commands.add_parser('prepare');p.add_argument('mapper',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--diameters-mm',type=float,nargs='+',default=DIAMETERS)
    p.add_argument('--mu-max',type=float,default=.4);p.add_argument('--step-deg',type=float,default=1.)
    p.add_argument('--seed',type=int,default=42)
    p=commands.add_parser('run');p.add_argument('--collection',type=Path,required=True)
    p.add_argument('--diameters-mm',type=float,nargs='+')
    p.add_argument('--num-envs',type=int,default=DEFAULT_NUM_ENVS)
    p.add_argument('--iterations',type=int,default=2000);p.add_argument('--report-every',type=int,default=500)
    p=commands.add_parser('resolve');p.add_argument('--collection',type=Path,required=True)
    p.add_argument('--mapper',type=Path,required=True)
    p.add_argument('--x-mm',type=float,required=True);p.add_argument('--y-mm',type=float,required=True)
    p.add_argument('--diameter-mm',type=float,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--geometry-only',action='store_true',help='Preview selection without policy artifacts; never executable')
    args=parser.parse_args(argv)
    if args.command=='prepare':
        prepare_collection(args.mapper,args.output,diameters=args.diameters_mm,mu_max=args.mu_max,
                           step=args.step_deg,seed=args.seed)
        return 0
    if args.command=='run':
        return run_collection(args.collection,diameters=args.diameters_mm,num_envs=args.num_envs,
                              iterations=args.iterations,report_every=args.report_every)
    write_json(args.output,resolve(args.collection,args.mapper,[args.x_mm,args.y_mm],args.diameter_mm,
                                  require_trained=not args.geometry_only))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
