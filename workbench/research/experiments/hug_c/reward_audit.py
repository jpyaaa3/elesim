"""Score declared scenarios and archived q-only failures without training.

The scenarios supply synthetic contact/hold flags to exercise reward ordering;
they are not physics rollouts. Archived failed trajectories lack contact-face
records, so their score omits that penalty and is an optimistic bound only.
"""
import argparse
import json
from pathlib import Path
import numpy as np

from .configure import DEFAULTS,CONTROL_MODE,write_json
from .control import action_rates
from .rewards import RewardLedger,q_potential


def scenario_scores(settings=None):
    s=settings or DEFAULTS
    home=np.array([-.12,0.,np.deg2rad(-34),np.deg2rad(34)])
    goal=np.array([0.,-np.pi/2,np.deg2rad(-.1),np.deg2rad(-35.7)])
    results=[]
    for name in ('idle','empty_q_target','wrong_face_push','proper_grasp_only','lift_then_drop',
                 'repeat_lift_drop','success','slow_success','body_collision'):
        r=RewardLedger(1,s);r.reset(np.array([0]),home[None],goal)
        discounted=0.;previous=home.copy()
        for i in range(s['episode_steps']):
            delay=40 if name=='slow_success' else 0
            q=home if name=='idle' else home+(goal-home)*np.clip((i+1-delay)/70,0,1)
            both=name in ('proper_grasp_only','lift_then_drop','repeat_lift_drop','success','slow_success') and i>=50+delay
            lift=name in ('lift_then_drop','repeat_lift_drop','success','slow_success') and i>=60+delay
            hold=np.clip((i-59-delay)/50,0,1) if lift else 0.
            drop=False
            if name=='lift_then_drop' and i>=80:
                both=False;lift=False;hold=0.;drop=i==80
            if name=='repeat_lift_drop' and i>=60:
                age=(i-60)%30;both=age<20;lift=both;hold=(age+1)/50 if lift else 0.;drop=age==20
            if lift:
                q=q.copy();q[1]=s['lift_roll_target_rad']
            passed=name in ('success','slow_success') and hold>=1
            arguments=dict(confirmed_grasp=np.array([both]),lift_fraction=np.array([float(lift)]),
                hold_fraction=np.array([hold]),wrong_fraction=np.array([float(name=='wrong_face_push' and 20<=i<80)]),
                fault=np.array([name=='body_collision' and i==20]),success=np.array([passed]),
                timeout=np.array([i==s['episode_steps']-1]),drops=np.array([int(drop)]))
            action=((q-previous)/action_rates(s)).clip(-1,1)[None];previous=q
            reward,done,_=r.step(q[None],goal,action,**arguments)
            discounted+=s['reward_gamma']**i*float(reward[0])
            if done[0]:break
        results.append(dict(scenario=name,duration_s=(i+1)*s['dt']*s['action_ticks'],
            total=float(sum(v[0] for v in r.totals.values())),discounted_total=discounted,
            components={k:float(v[0]) for k,v in r.totals.items()},synthetic=True))
    scores={v['scenario']:v['discounted_total'] for v in results}
    if not (scores['success']>scores['slow_success']>scores['lift_then_drop']>
            scores['proper_grasp_only']>scores['idle']>scores['wrong_face_push']):
        raise AssertionError(scores)
    if not scores['empty_q_target']<=scores['idle']+1e-5:
        raise AssertionError('Empty q target outscored idle')
    return results


def rescore_failures(study,settings=None):
    from elesim_sim.rl.trajectory import Trajectory
    s=settings or DEFAULTS;results=[]
    for path in sorted(Path(study).glob('reports/*/*/report.json')):
        report=json.loads(path.read_text());trace=Trajectory.load(path.parent/'trajectory.npz')
        data=json.loads((path.parent/'replay.json').read_text());samples=data['first_replay']['samples']
        if report['collection']['fault'] or trace.metadata.get('grasp_time_s') is not None:
            raise ValueError('This conservative q-only audit requires no faults or recorded C grasp')
        goal=np.asarray(trace.metadata['goal_command']);start=np.asarray(samples[0]['state']['actual_command'])
        r=RewardLedger(1,s);r.reset(np.array([0]),start[None],goal)
        stride=s['action_ticks'];discounted=0.
        for j,tick in enumerate(range(stride,len(samples),stride)):
            q=np.array(samples[tick]['state']['actual_command'])
            commanded=trace.sample(samples[tick]['time_s']);previous=trace.sample(samples[tick-stride]['time_s'])
            actions=((commanded-previous)/action_rates(s)).clip(-1,1)[None]
            reward,_,_=r.step(q[None],goal,actions,confirmed_grasp=np.array([False]),
                lift_fraction=np.array([0.]),hold_fraction=np.array([0.]),wrong_fraction=np.array([0.]),
                fault=np.array([False]),success=np.array([False]),timeout=np.array([tick==len(samples)-1]),drops=np.array([0.]))
            discounted+=s['reward_gamma']**j*float(reward[0])
        results.append(dict(checkpoint=path.parent.name,total=float(sum(v[0] for v in r.totals.values())),
            discounted_total=discounted,components={k:float(v[0]) for k,v in r.totals.items()},
            interpretation='optimistic bound: missing face/workspace costs omitted; applied command effort is only a lower bound',
            physics_replayed=False,original_video_goal_error=report['collection']['goal_error_normalized']))
    return results


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--study',type=Path,help='Optional archived study with trajectory/replay/report files')
    p.add_argument('--output',type=Path,required=True);args=p.parse_args(argv)
    report=dict(schema='elesim-c-hug-reward-audit',schema_version=1,control_mode=CONTROL_MODE,
                settings=DEFAULTS,scenarios=scenario_scores(),physics_tested=False,training_started=False,
                archived_failures=rescore_failures(args.study) if args.study else [])
    write_json(args.output,report)
    print(json.dumps([dict(scenario=r['scenario'],discounted_total=round(r['discounted_total'],3))
                      for r in report['scenarios']],indent=2))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
