"""Read-only policy replay to distinguish command saturation from obstruction.

Uses a separate one-environment scene; never loads an optimizer or edits the
training source. Output captures actual policy observations, unbounded means,
applied command increments, contacts and tracking at each policy step.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study',type=Path,required=True)
    parser.add_argument('--case',required=True)
    parser.add_argument('--updates',type=int,nargs='+',default=[500,1000,1500,2000])
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv)
    from workbench.research.experiments.hug_c.configure import load_experiment,write_json
    from workbench.research.experiments.hug_c.environment import HugEnv
    from workbench.research.experiments.hug_c.policy import NumpyPolicy
    from workbench.research.experiments.hug_c.control import action_rates,command_bounds
    import torch

    args.output.mkdir(parents=True,exist_ok=False)
    experiment=load_experiment(args.study/'experiment.json')
    case=next(c for c in experiment['cases'] if c['case_id']==args.case)
    env=HugEnv(experiment,case,args.output/'assets',n_envs=1,backend='gpu',auto_reset=False)
    rates=action_rates(env.s);lower,upper=command_bounds(env.s);reports=[]
    for update in args.updates:
        directory=args.study/'reports'/args.case/f'updates-{update:05d}'
        policy=NumpyPolicy(directory/'policy.npz')
        with torch.inference_mode():obs,_=env.reset()
        rows=[]
        contacts=dict(proximal_ticks=0,distal_ticks=0,both_ticks=0,wrong_face_ticks=0)
        def monitor(task):
            touch=task.last_touch[0].detach().cpu().numpy()>0
            contacts['proximal_ticks']+=int(touch[0]);contacts['distal_ticks']+=int(touch[1])
            contacts['both_ticks']+=int(touch.all());contacts['wrong_face_ticks']+=int(task.last_wrong_face[0])
        env.tick_monitor=monitor
        with torch.inference_mode():
            for i in range(env.max_episode_length):
                observation=obs['policy'][0].detach().cpu().numpy().copy()
                raw=policy(observation)[0];before=env.mapper.waypoint[0].detach().cpu().numpy().copy()
                obs,reward,done,_=env.step(torch.tensor(raw,device=env.device).reshape(1,4))
                after=env.mapper.waypoint[0].detach().cpu().numpy().copy()
                actual=env.actual()[0].detach().cpu().numpy().copy()
                blocked=((after>=upper-1e-6)&(raw>0))|((after<=lower+1e-6)&(raw<0))
                rows.append(dict(time_s=(i+1)*env.s['action_ticks']*env.s['dt'],
                                 observation=observation.tolist(),raw_policy_mean=raw.tolist(),
                                 clipped_action=np.clip(raw,-1,1).tolist(),command_before=before.tolist(),
                                 command_after=after.tolist(),actual=actual.tolist(),
                                 applied_action=((after-before)/rates).tolist(),outward_at_limit=blocked.tolist(),
                                 reward=float(reward[0]),state=env.measure()))
                if bool(done[0]):break
        final=env.measure();stop=None
        commands=np.array([row['command_after'] for row in rows])
        for i in range(len(rows)):
            if len(rows)-i>=10 and np.max(np.ptp(commands[i:],axis=0)/rates)<.01:
                stop=rows[i]['time_s'];break
        discounted={key:float(sum(env.s['reward_gamma']**i*(row['state']['reward_components'][key]-(rows[i-1]['state']['reward_components'][key] if i else 0)) for i,row in enumerate(rows))) for key in final['reward_components']}
        summary=dict(updates=update,stationary_after_s=stop,steps=len(rows),contacts=contacts,
                     final=final,last_policy_mean=rows[-1]['raw_policy_mean'],
                     last_applied_action=rows[-1]['applied_action'],
                     last_outward_at_limit=rows[-1]['outward_at_limit'],discounted_rewards=discounted,
                     minimum_q_error=min(row['state']['goal_error_normalized'] for row in rows),
                     total_tracking_error_max=np.max(np.abs(commands-np.array([r['actual'] for r in rows])),axis=0).tolist())
        write_json(args.output/f'updates-{update:05d}.json',dict(summary=summary,rows=rows))
        reports.append(summary);print(json.dumps(summary),flush=True)
    write_json(args.output/'report.json',dict(schema='elesim-c-hug-policy-diagnosis',schema_version=1,
                optimizer_used=False,training_modified=False,case=case,simulation_context=experiment['simulation_context'],reports=reports))


if __name__=='__main__':main()
