"""Separate-scene audit of contact-only parking; never modifies training.

Compare deterministic exported policies with a zero-action Home baseline under
identical task/reward settings. Record proper-contact roles, loaded link names,
reward deltas, and time-cost exemptions, rather than inferring intent from video.
"""
import argparse
import json
from pathlib import Path
import numpy as np


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--experiment',type=Path,required=True)
    parser.add_argument('--policy',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv)
    from workbench.research.experiments.hug_c.configure import load_experiment,write_json,sha
    from workbench.research.experiments.hug_c.environment import HugEnv
    from workbench.research.experiments.hug_c.policy import NumpyPolicy
    from workbench.research.experiments.hug_c.population import quality
    import torch
    policies=[NumpyPolicy(path) for path in args.policy]
    exp=load_experiment(args.experiment)
    ids={policy.metadata['case_id'] for policy in policies}
    if len(ids)!=1 or any(policy.metadata['simulation_context']!=exp['simulation_context'] for policy in policies):
        raise ValueError('Policies must share this experiment and case')
    case=next(c for c in exp['cases'] if c['case_id']==next(iter(ids)))
    args.output.mkdir(parents=True,exist_ok=False)
    env=HugEnv(exp,case,args.output/'assets',n_envs=1,backend='gpu',auto_reset=False)
    names=env.scene.links.name_by_index
    objects=set(env.scene.links.object_)
    reports=[]
    for label,policy,path in [('idle-home',None,None)]+[(f'policy-{i}',p,path) for i,(p,path) in enumerate(zip(policies,args.policy))]:
        rows=[];counts=dict(proximal_proper_ticks=0,distal_proper_ticks=0,both_proper_ticks=0,
                           gripper_only_loaded_ticks=0,any_loaded_contact_ticks=0,wrong_face_ticks=0)
        link_counts={};observed_grasp=False;waived=0;first_contact=None
        with torch.inference_mode():obs,_=env.reset()
        def monitor(task):
            nonlocal first_contact
            proper=task.last_touch[0].cpu().numpy()>0
            counts['proximal_proper_ticks']+=int(proper[0]);counts['distal_proper_ticks']+=int(proper[1])
            counts['both_proper_ticks']+=int(proper.all());counts['wrong_face_ticks']+=int(task.last_wrong_face[0])
            raw=task.scene.robot.get_contacts(exclude_self_contact=False,is_padded=True)
            a=raw['link_a'][0].cpu().numpy();b=raw['link_b'][0].cpu().numpy()
            valid=raw['valid_mask'][0].cpu().numpy()
            force=np.linalg.norm(raw['force_a'][0].cpu().numpy(),axis=-1)
            links=set()
            for i in np.flatnonzero(valid&(force>task.s['face_force_min_n'])):
                robot=int(b[i]) if int(a[i]) in objects else int(a[i]) if int(b[i]) in objects else None
                name=names.get(robot,'')
                if name.startswith('node') or name.startswith('gripper'):links.add(name)
            if links:
                if first_contact is None:first_contact=int(task.ticks[0])*task.s['dt']
                counts['any_loaded_contact_ticks']+=1
                counts['gripper_only_loaded_ticks']+=int(all(name.startswith('gripper') for name in links))
                for name in links:link_counts[name]=link_counts.get(name,0)+1
        env.tick_monitor=monitor
        discounted={name:0. for name in env.ledger.totals}
        with torch.inference_mode():
            initial_distance=float(env.relative_object()[0].norm())
            closest_distance=initial_distance
            for step in range(env.max_episode_length):
                actions=torch.zeros((1,4),device=env.device) if policy is None else torch.as_tensor(policy(obs['policy'].cpu().numpy()),device=env.device)
                obs,reward,done,extras=env.step(actions)
                current={name:float(v[0]) for name,v in env.ledger.totals.items()}
                previous=rows[-1]['reward_components'] if rows else dict.fromkeys(current,0.)
                delta={name:current[name]-previous[name] for name in current}
                for name in discounted:discounted[name]+=env.s['reward_gamma']**step*delta[name]
                waived+=int(abs(delta['time'])<1e-7)
                grasp=bool(env.grasp_ticks[0]>=round(env.s['grasp_confirm_s']/env.s['dt']))
                observed_grasp|=grasp
                closest_distance=min(closest_distance,float(env.relative_object()[0].norm()))
                rows.append(dict(time_s=(step+1)*env.s['dt']*env.s['action_ticks'],
                    actual=env.actual()[0].cpu().tolist(),proper_roles=env.last_touch[0].cpu().tolist(),
                    fault=bool(env.fault[0]),floor_contact=bool(env.last_floor[0]),confirmed_grasp=grasp,
                    hold_ticks=int(env.hold.count[0]),clearance_m=float(env.object_clearance()[0]),
                    object_xyz_m=env.scene.object.get_pos()[0].cpu().tolist(),
                    reward_components=current,reward_delta=delta))
                if bool(done[0]):break
        final=env.measure();dt=env.s['dt']
        duration=(step+1)*dt*env.s['action_ticks']
        safe=not final['fault'] and not counts['wrong_face_ticks']
        selection=dict(success=float(final['success'] and safe),hold=final['highest_hold_fraction'] if safe else 0.,lift=final['highest_lift_fraction'] if safe else 0.,
                       grasp=float(observed_grasp and safe),contact=float(safe and (counts['proximal_proper_ticks']+counts['distal_proper_ticks']>0)),
                       approach=(initial_distance-closest_distance)/max(initial_distance,1e-6) if safe else 0.,
                       efficiency=sum(final['reward_components'][k] for k in ('time','workspace','action')),safe=float(safe))
        summary=dict(label=label,source_policy=str(path) if path else None,source_sha256=sha(path) if path else None,
                     duration_s=duration,first_loaded_contact_s=first_contact,
                     contact_seconds={name:value*dt for name,value in counts.items()},
                     loaded_link_contact_seconds={name:value*dt for name,value in link_counts.items()},
                     time_cost_waived_s=waived*dt*env.s['action_ticks'],
                     final=final,discounted_rewards=discounted,discounted_return=sum(discounted.values()),
                     selection_report=selection,selection_quality=list(quality(selection)))
        write_json(args.output/(label+'.json'),dict(summary=summary,rows=rows))
        reports.append(summary);print(json.dumps(summary),flush=True)
    write_json(args.output/'report.json',dict(schema='elesim-contact-parking-audit',schema_version=1,
        optimizer_used=False,training_modified=False,case=case,simulation_context=exp['simulation_context'],reports=reports,
        note='Single deterministic replay per policy; not a stochastic training or multi-lane success-rate estimate'))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
