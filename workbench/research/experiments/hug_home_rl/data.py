"""Portable, source-pinned forward-prefix preparation without GPU imports."""
from pathlib import Path
import hashlib
import json
import shutil
import numpy as np
from workbench.research.experiments.hug_reverse_rl.data import sha
from workbench.research.experiments.hug_reverse_ga.prepare import save
from .smoothness import DEFAULTS as SMOOTH,validate as validate_smooth
from .schedule import DEFAULTS as SCHEDULE,validate as validate_schedule

FIELDS=('actual','actual_vel','joint_q','joint_qd','object_pos','object_quat','object_vel',
        'object_ang','touch','floor','fault','wrong','contact_force','contact_penetration','command','time_s')


def digest():
    from workbench.research.experiments.hug_reverse_rl.data import digest as base
    h=hashlib.sha256(base().encode())
    for p in sorted(Path(__file__).parent.glob('*.py')):h.update(p.name.encode());h.update(p.read_bytes())
    return h.hexdigest()


def canonical(value):return hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()


def context(base,smooth):
    return dict(base['simulation_context'],code_sha256=digest(),control_mode='home-prefix-four-dof-v1',
        reward_contract='bounded-milestones-lift90-v5+bounded-command-variation-v1',
        reset_contract='nominal-home-forward-prefix-v1',smoothness=smooth,case_sha256=canonical(base['case']))


def validate_prefix(path,metadata,base):
    from workbench.research.experiments.hug_c.configure import load_mechanism
    from workbench.research.experiments.hug_c.control import command_bounds,action_rates
    from workbench.research.experiments.hug_reverse_ga.criteria import tilt
    if (metadata.get('phase')!='nominal-Home-to-handoff' or metadata.get('training_updates')!=0
        or metadata.get('task_history_reset_at_handoff') is not True
        or metadata.get('simulation_context')!=base['simulation_context']):raise ValueError('Prefix provenance mismatch')
    if set(path)!=set(FIELDS):raise ValueError('Unknown/missing prefix fields')
    n=len(path['time_s']);s=base['settings'];home=np.asarray(base['home'])
    if n<2 or not np.allclose(path['time_s'],np.arange(n)*s['dt'],atol=1e-8,rtol=0):raise ValueError('Invalid prefix clock')
    for k,v in path.items():
        if len(v)!=n or not np.isfinite(v).all():raise ValueError('Invalid prefix array '+k)
    shapes={'actual':(n,4),'actual_vel':(n,4),'command':(n,4),'object_pos':(n,3),
        'object_quat':(n,4),'object_vel':(n,3),'object_ang':(n,3),'touch':(n,2)}
    for k in ('floor','fault','wrong','contact_force','contact_penetration'):shapes[k]=(n,)
    if any(path[k].shape!=v for k,v in shapes.items()):raise ValueError('Wrong prefix shape')
    if path['joint_q'].ndim!=2 or path['joint_q'].shape!=path['joint_qd'].shape or path['joint_q'].shape[1]<12:
        raise ValueError('Full individual joint q/qd required')
    if not np.array_equal(home,np.asarray(load_mechanism().presets['home'])):raise ValueError('Home preset changed')
    if not np.allclose(path['command'][0],home,atol=1e-7) or not np.allclose(path['actual'][0],home,atol=1e-6):
        raise ValueError('Prefix must start at Home')
    origin=np.asarray(base['case']['object_xyz_m'])
    if (not np.allclose(path['object_pos'][0],origin,atol=1e-6)
        or not np.allclose(path['object_quat'][0],[1.,0,0,0],atol=1e-6)
        or max(np.max(abs(path[k][0])) for k in ('object_vel','object_ang','joint_qd','actual_vel'))>1e-6):
        raise ValueError('Original nominal object start required')
    lo,hi=command_bounds(s);cmd=path['command']
    if np.any(cmd<lo-1e-7) or np.any(cmd>hi+1e-7) or np.any(abs(np.diff(cmd,axis=0))>action_rates(s)/s['action_ticks']+1e-7):
        raise ValueError('Prefix command bounds/rate exceeded')
    if path['fault'].any() or path['wrong'].any() or np.max(path['contact_penetration'])>s['max_penetration_m']:
        raise ValueError('Unsafe prefix')
    if (np.max(np.linalg.norm(path['object_pos']-origin,axis=-1))>.002
        or np.max(tilt(path['object_quat']))>np.deg2rad(2)
        or not path['floor'][-1] or np.linalg.norm(path['object_vel'][-1])>.01):
        raise ValueError('Entry prefix must retain nominal floor-supported object')
    return n-1


def read_prefix(file):
    with np.load(file,allow_pickle=False) as d:
        if set(d.files)!=set(FIELDS)|{'metadata_json'}:raise ValueError('Prefix archive fields differ')
        meta=json.loads(str(d['metadata_json'].item()))
        if any(d[k].ndim<2 or d[k].shape[1]!=1 for k in FIELDS if k!='time_s'):raise ValueError('Single-lane verified prefix required')
        path={k:d[k].copy() if k=='time_s' else d[k][:,0].copy() for k in FIELDS}
    return path,meta


def validate_initial(policy,base,checkpoint_sha):
    if (policy.metadata.get('simulation_context')!=base['simulation_context']
        or policy.metadata.get('checkpoint_sha256')!=checkpoint_sha
        or policy.metadata.get('reverse_reference_sha256')!=base['reference_path_sha256']
        or any(policy.metadata.get(k)!=base['case'][k] for k in ('goal_command','object_xyz_m','diameter_mm'))):
        raise ValueError('Initial policy/task/checkpoint mismatch')


def prepare(source_run,verified_bridge,output,*,spacing_s=.05,seed=42,smooth_budget=1.):
    from workbench.research.experiments.hug_reverse_rl.data import load_plan as load_base
    from workbench.research.experiments.hug_c.policy import NumpyPolicy
    source_run=Path(source_run);verified_bridge=Path(verified_bridge);output=Path(output)
    if output.exists():raise FileExistsError(output)
    if not np.isfinite(spacing_s) or spacing_s<=0 or type(seed) is not int or seed<0:raise ValueError('Invalid spacing/seed')
    base,_=load_base(source_run/'plan');checkpoint=source_run/'training/model_final.pt'
    pairing=Path(str(checkpoint)+'.curriculum.json');side=json.loads(pairing.read_text())
    if (side.get('schema')!='hug-reverse-ppo-checkpoint' or side.get('version')!=1
        or side['source_sha256']!=base['source_sha256'] or side['reference_path_sha256']!=base['reference_path_sha256']
        or side['checkpoint_sha256']!=sha(checkpoint)):raise ValueError('Initial checkpoint pairing mismatch')
    policy_file=source_run/'reports/model_final/policy.npz'
    validate_initial(NumpyPolicy(policy_file),base,sha(checkpoint))
    verification=json.loads((verified_bridge/'completion.json').read_text())
    if verification.get('status')!='verified' or len(verification.get('results',[]))!=3:raise ValueError('Require three verified bridge replays')
    path=None;metadata=None;inputs={};tolerances=base['state_tolerances']
    for i,entry in enumerate(verification['results'],1):
        folder=verified_bridge/f'replay-{i}';report_file=folder/'model_final.json';prefix_file=folder/'model_final-prefix.npz'
        report=json.loads(report_file.read_text())
        if (entry.get('returncode')!=0 or entry.get('result')!=report or report.get('policy_sha256')!=sha(policy_file)
            or report.get('fresh_handoff') is not True or len(report.get('results',[]))!=1
            or report['results'][0].get('start_tick')!=-1 or report['results'][0].get('success') is not True
            or report['results'][0].get('fault') is not False or report['results'][0].get('wrong_face') is not False):
            raise ValueError('Unverified nominal Home handoff')
        errors=report.get('physical_preservation_errors',{})
        if set(errors)!=set(tolerances) or any(not np.isfinite(errors[k]) or not 0<=errors[k]<=t for k,t in tolerances.items()):
            raise ValueError('Handoff must preserve full physical state')
        candidate,meta=read_prefix(prefix_file);end=validate_prefix(candidate,meta,base)
        if path is None:path,metadata=candidate,meta
        elif (meta!=metadata or any(not np.allclose(candidate[k],path[k],atol=1e-7,rtol=0) for k in FIELDS)):
            raise ValueError('Independent prefix replays differ')
        inputs[str(report_file)]=sha(report_file);inputs[str(prefix_file)]=sha(prefix_file)
    stride=max(1,round(spacing_s/base['settings']['dt']))
    ticks=sorted(set([0,end,*range(0,end+1,stride)]),reverse=True)
    if len(ticks)>32:raise ValueError('Reset bank limited to32 snapshots; increase spacing')
    smooth=dict(SMOOTH,budget=smooth_budget);validate_smooth(smooth)
    schedule=dict(SCHEDULE,seed=seed);validate_schedule(schedule)
    plan=dict(schema='hug-home-ppo-plan',version=1,source_sha256=digest(),base_plan=base,
        settings=base['settings'],case=base['case'],home=base['home'],pool_ticks=ticks,
        curriculum=schedule,smoothness=smooth,prefix_metadata=metadata,state_tolerances=tolerances,
        simulation_context=context(base,smooth),initial_checkpoint_sha256=sha(checkpoint),
        initial_policy_sha256=sha(policy_file),source_files=dict(inputs,**{str(verified_bridge/'completion.json'):sha(verified_bridge/'completion.json')}))
    output.mkdir(parents=True);np.savez_compressed(output/'reference-path.npz',**path)
    shutil.copyfile(checkpoint,output/'initial-model.pt');shutil.copyfile(policy_file,output/'initial-policy.npz')
    save(output/'initial-model-source.json',side)
    plan['initial_pairing_sha256']=sha(output/'initial-model-source.json')
    plan['reference_path_sha256']=sha(output/'reference-path.npz');save(output/'plan.json',plan)
    return plan


def load_plan(directory):
    from workbench.research.experiments.hug_c.configure import asset_digest,validate_settings
    from workbench.research.experiments.hug_c.policy import NumpyPolicy
    from workbench.research.experiments.hug_reverse_rl.data import digest as base_digest
    directory=Path(directory);plan=json.loads((directory/'plan.json').read_text())
    if (plan.get('schema')!='hug-home-ppo-plan' or plan.get('version')!=1 or plan['source_sha256']!=digest()
        or plan['simulation_context']['model_sha256']!=asset_digest()
        or sha(directory/'reference-path.npz')!=plan['reference_path_sha256']
        or sha(directory/'initial-model.pt')!=plan['initial_checkpoint_sha256']
        or sha(directory/'initial-policy.npz')!=plan['initial_policy_sha256']
        or sha(directory/'initial-model-source.json')!=plan['initial_pairing_sha256']):raise ValueError('Home plan/source/model/artifact mismatch')
    validate_settings(plan['settings']);validate_smooth(plan['smoothness']);validate_schedule(plan['curriculum'])
    base=plan['base_plan']
    if (base['source_sha256']!=base_digest() or base['simulation_context'].get('code_sha256')!=base['source_sha256']
        or plan['settings']!=base['settings'] or plan['case']!=base['case'] or plan['home']!=base['home']
        or plan['state_tolerances']!=base['state_tolerances'] or plan['simulation_context']!=context(base,plan['smoothness'])):
        raise ValueError('Home plan task/context mismatch')
    validate_initial(NumpyPolicy(directory/'initial-policy.npz'),base,plan['initial_checkpoint_sha256'])
    with np.load(directory/'reference-path.npz',allow_pickle=False) as d:path={k:d[k].copy() for k in d.files}
    end=validate_prefix(path,plan['prefix_metadata'],base);ticks=plan['pool_ticks']
    if (not 2<=len(ticks)<=32 or any(type(t) is not int for t in ticks)
        or ticks!=sorted(set(ticks),reverse=True) or ticks[0]!=end or ticks[-1]!=0):raise ValueError('Invalid endpoint-to-Home reset ordering')
    return plan,path
