"""CPU-only preparation and validation; no Genesis, Torch or training imports."""
import hashlib
import json
from pathlib import Path
import numpy as np
from workbench.research.experiments.hug_reverse_ga.prepare import save
from workbench.research.experiments.hug_reverse_ga.motion_budget import increment
from workbench.research.experiments.hug_reverse_ga.criteria import REASONS

FIELDS=('actual','actual_vel','joint_q','joint_qd','object_pos','object_quat','object_vel',
        'object_ang','touch','floor','fault','wrong','contact_force','contact_penetration','violation_mask','command','time_s')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def digest():
    from workbench.research.experiments.hug_reverse_ga.prepare import digest as ga_digest
    h=hashlib.sha256(ga_digest().encode())
    for p in sorted(Path(__file__).parent.glob('*.py')):
        h.update(p.name.encode());h.update(p.read_bytes())
    return h.hexdigest()


def require_verified(completion,curriculum,report,replays):
    if (completion.get('status')!='reverse_path_verified' or completion.get('replays')!=[True]*3
        or curriculum.get('status')!='reverse_path_verified' or curriculum.get('replays')!=[True]*3
        or report.get('success') is not True or report.get('rejected') is not False
        or report.get('reasons') or len(replays)!=3
        or any(not rows or any(r.get('success') is not True or r.get('rejected') is not False for r in rows) for rows in replays)):
        raise ValueError('Require a successful reverse search and three successful independent replays')


def validate_path(path,plan):
    from workbench.research.experiments.hug_reverse_ga.criteria import violations,home_good,admission_good
    from workbench.research.experiments.hug_c.control import command_bounds,action_rates
    if set(path)!=set(FIELDS):raise ValueError('Missing or unknown path arrays')
    t=np.asarray(path['time_s']);length=len(t);dt=plan['settings']['dt']
    if length<2 or not np.allclose(t,np.arange(length)*dt,atol=1e-8,rtol=0):raise ValueError('Invalid physics-tick timestamps')
    for k,v in path.items():
        if len(v)!=length or not np.isfinite(v).all():raise ValueError('Invalid path array: '+k)
    shapes={'actual':(length,4),'actual_vel':(length,4),'object_pos':(length,3),'object_quat':(length,4),
            'object_vel':(length,3),'object_ang':(length,3),'touch':(length,2),'command':(length,4),'violation_mask':(length,len(REASONS))}
    for k in ('floor','fault','wrong','contact_force','contact_penetration'):shapes[k]=(length,)
    for k,shape in shapes.items():
        if path[k].shape!=shape:raise ValueError('Wrong shape: '+k)
    if path['joint_q'].ndim!=2 or path['joint_q'].shape!=path['joint_qd'].shape or path['joint_q'].shape[1]<12:
        raise ValueError('Require full individual joint q/qd')
    case=plan['cases'][0];lo,hi=command_bounds(plan['settings']);rate=action_rates(plan['settings'])/plan['settings']['action_ticks']
    cmd=path['command'];previous=np.vstack((cmd[0],cmd[:-1]))
    travel=np.cumsum(increment(cmd,previous),axis=0)
    mask,_,_=violations(path,cmd,previous,lo,hi,rate,case['object_xyz_m'],travel)
    if mask.any() or path['violation_mask'].any():raise ValueError('Rejected samples cannot enter a reset pool')
    if not np.allclose(cmd[0],case['goal_command'],atol=1e-7):raise ValueError('Path does not start at solver grasp')
    if not admission_good({k:v[:1] for k,v in path.items()},np.asarray(case['goal_command']))[0]:
        raise ValueError('Initial state is not an admitted grasp')
    good=home_good(path,cmd,np.asarray(plan['home']))
    hold=round(1/dt)
    if len(good)<hold or not good[-hold:].all():raise ValueError('Missing continuous terminal Home verification')
    # Exclude the redundant terminal dwell from curriculum distance.
    end=length-hold
    return end


def prepare(reverse_root,output,*,spacing_s=.25,seed=42):
    from workbench.research.experiments.hug_reverse_ga.prepare import digest as ga_digest
    from workbench.research.experiments.hug_c.configure import asset_digest,validate_settings,load_mechanism
    from .schedule import DEFAULTS,validate_options
    reverse_root=Path(reverse_root);output=Path(output)
    if output.exists():raise FileExistsError(output)
    if not np.isfinite(spacing_s) or spacing_s<=0 or type(seed) is not int or seed<0:raise ValueError('Invalid spacing or seed')
    read=lambda p:json.loads((reverse_root/p).read_text())
    ga=read('plan.json');curr=read('run/curriculum.json');completion=read('run/completion.json')
    if (ga.get('schema')!='hug-reverse-ga-plan' or ga.get('version')!=1 or ga['source_sha256']!=ga_digest()
        or ga['simulation_context']['model_sha256']!=asset_digest()):raise ValueError('Reverse source/model mismatch')
    validate_settings(ga['settings'])
    if ga['home']!=list(load_mechanism().presets['home']):raise ValueError('Home preset changed')
    gen=curr['generation'];lane=curr['candidate']
    if type(gen) is not int or gen<0 or type(lane) is not int or lane<0:raise ValueError('Invalid winner index')
    expected=f'generation-{gen:04d}/states.npz'
    if curr['states']!=expected or curr['lane']!=lane:raise ValueError('Winner path mismatch')
    report_path=f'run/generation-{gen:04d}/report.json';reports=read(report_path)['candidates']
    report=reports[lane]
    if report['candidate']!=lane:raise ValueError('Winner report mismatch')
    replay_paths=[f'run/verification-{i}/report.json' for i in (1,2,3)]
    require_verified(completion,curr,report,[read(p)['candidates'] for p in replay_paths])
    states=reverse_root/'run'/expected
    with np.load(states,allow_pickle=False) as raw:
        meta=json.loads(str(raw['metadata_json'].item()))
        if meta['case']!=ga['cases'][0] or meta['simulation_context']!=ga['simulation_context'] or meta.get('direction')!='grasp-to-home':
            raise ValueError('Reverse states provenance mismatch')
        active=raw['active'][:,lane].astype(bool)
        n=round(report['duration_s']/ga['settings']['dt'])+1
        if n>len(active) or not active[:n].all() or active[n:].any():raise ValueError('Invalid winner validity mask')
        path={k:raw[k][:n].copy() if k=='time_s' else raw[k][:n,lane].copy() for k in FIELDS}
    # Search snapshots deliberately have zeroed task contact counters at t0.
    # Admission is saved separately and supplies only these observation flags.
    with np.load(reverse_root/'run/admission/states.npz',allow_pickle=False) as initial:
        for k in ('touch','floor','fault','wrong'):path[k][0]=initial[k][-1,lane]
    end=validate_path(path,ga);stride=max(1,round(spacing_s/ga['settings']['dt']))
    indices=sorted(set([0,end,*range(0,end+1,stride)]))
    options=dict(DEFAULTS,seed=seed);validate_options(options)
    plan=dict(schema='hug-reverse-ppo-plan',version=1,source_sha256=digest(),reverse_plan=ga,
        settings=ga['settings'],case=ga['cases'][0],home=ga['home'],pool_ticks=indices,
        curriculum=options,restore='validated full Genesis snapshots from admitted reverse prefix replay',
        state_tolerances=dict(joint_q=1e-4,joint_qd=.003,object_pos=1e-4,object_quat=1e-4,object_vel=.003,object_ang=.003),
        source_files={p:sha(reverse_root/p) for p in ['plan.json','run/curriculum.json','run/completion.json',report_path,'run/'+expected,'run/admission/states.npz',*replay_paths]},
        simulation_context=dict(ga['simulation_context'],code_sha256=digest(),
            control_mode='reverse-curriculum-four-dof-v1',reward_contract='bounded-milestones-lift90-v5',
            source_reverse_sha256=ga['source_sha256'],reset_contract='full-state-prefix-bank-v1'),
        physical_reset_pool_validated=False)
    output.mkdir(parents=True)
    np.savez_compressed(output/'reference-path.npz',**path)
    plan['reference_path_sha256']=sha(output/'reference-path.npz');save(output/'plan.json',plan)
    return plan


def load_plan(directory):
    from workbench.research.experiments.hug_c.configure import asset_digest,validate_settings
    from .schedule import validate_options
    directory=Path(directory);plan=json.loads((directory/'plan.json').read_text())
    if (plan.get('schema')!='hug-reverse-ppo-plan' or plan.get('version')!=1 or plan['source_sha256']!=digest()
        or plan['simulation_context']['model_sha256']!=asset_digest()
        or sha(directory/'reference-path.npz')!=plan['reference_path_sha256']):raise ValueError('PPO plan/source/model/path mismatch')
    validate_settings(plan['settings']);validate_options(plan['curriculum'])
    ga=plan['reverse_plan']
    expected=dict(ga['simulation_context'],code_sha256=digest(),
        control_mode='reverse-curriculum-four-dof-v1',reward_contract='bounded-milestones-lift90-v5',
        source_reverse_sha256=ga['source_sha256'],reset_contract='full-state-prefix-bank-v1')
    if (plan['settings']!=ga['settings'] or plan['case']!=ga['cases'][0] or plan['home']!=ga['home']
        or plan['simulation_context']!=expected):raise ValueError('PPO physics/case/context mismatch')
    with np.load(directory/'reference-path.npz',allow_pickle=False) as raw:path={k:raw[k].copy() for k in raw.files}
    end=validate_path(path,plan['reverse_plan'])
    ticks=plan['pool_ticks']
    if not ticks or ticks!=sorted(set(ticks)) or ticks[0]!=0 or ticks[-1]!=end or any(type(i) is not int for i in ticks):
        raise ValueError('Invalid reset pool indices')
    return plan,path
