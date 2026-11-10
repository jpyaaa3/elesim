"""Batched Home checks plus the same sustained Wrap admission used by reverse GA."""
import json
import numpy as np
from .contracts import save,make_case


def screen(root,study,output,batch=64):
    import torch
    from workbench.research.experiments.hug_c.environment import HugEnv
    from workbench.research.experiments.hug_reverse_ga.physics import sample
    from workbench.research.experiments.hug_reverse_ga.criteria import admission_good,tilt
    catalog=json.loads((root/'catalog.json').read_text());rows=catalog['rows']
    if not rows:raise ValueError('No geometric candidates')
    output.mkdir(parents=True,exist_ok=False)
    first=make_case(rows[0],study['diameter_mm'],study['home'])
    exp=dict(settings=study['settings'],simulation_context=study['shared_context'],cases=[first])
    env=HugEnv(exp,first,output/'assets',n_envs=batch,backend=study['backend'],auto_reset=False)
    results=[];dt=study['settings']['dt']
    for begin in range(0,len(rows),batch):
        subset=rows[begin:begin+batch];padded=subset+[subset[-1]]*(batch-len(subset))
        cases=[make_case(r,study['diameter_mm'],study['home']) for r in padded]
        positions=np.asarray([c['object_xyz_m'] for c in cases]);goals=np.asarray([c['goal_command'] for c in cases])
        facts={}
        for phase in ('home','goal'):
            env.reset();env.scene.object.set_pos(torch.as_tensor(positions,device=env.device,dtype=torch.float32))
            command=np.tile(study['home'],(batch,1)) if phase=='home' else goals
            cmd=torch.as_tensor(command,device=env.device,dtype=torch.float32);env.mapper.waypoint[:]=cmd
            targets=env.mapper.joint_targets();env.scene.robot.set_dofs_position(targets,dofs_idx_local=env.dofs,zero_velocity=True)
            env.scene.robot.control_dofs_position(targets,dofs_idx_local=env.dofs)
            bad=np.zeros(batch,bool);count=np.zeros(batch,int);peak=np.zeros(batch)
            for _ in range(round((.1 if phase=='home' else 1.)/dt)):
                env.tick(cmd);s=sample(env);drift=np.linalg.norm(s['object_pos'][:,:2]-positions[:,:2],axis=1)
                bad|=s['fault']|s['wrong']|(drift>(.001 if phase=='home' else .05))|(tilt(s['object_quat'])>np.deg2rad(10))
                peak=np.maximum(peak,drift)
                if phase=='goal':count=np.where(admission_good(s,goals)&~bad,count+1,0)
            facts[phase]=dict(passed=~bad if phase=='home' else ~bad&(count>=round(.2/dt)),peak=peak,count=count)
        for i,r in enumerate(subset):
            results.append(dict(r,static_ready=bool(facts['home']['passed'][i] and facts['goal']['passed'][i]),
                home_passed=bool(facts['home']['passed'][i]),wrap_admitted=bool(facts['goal']['passed'][i]),
                wrap_good_ticks=int(facts['goal']['count'][i]),home_drift_mm=float(facts['home']['peak'][i]*1000)))
        save(output/'progress.json',dict(visited=len(results),total=len(rows),admitted=sum(r['static_ready'] for r in results)))
    save(output/'report.json',dict(schema='elesim-c-hug-catalog-preflight',schema_version=4,source=catalog['source'],
        diameter_mm=study['diameter_mm'],step_deg=study['step_deg'],mu_max=study['mu_max'],
        simulation_context=study['shared_context'],results=results,
        criterion='Home safe with <=1mm drift; 1s goal settling and final 0.2s proper two-segment Wrap admission'))
