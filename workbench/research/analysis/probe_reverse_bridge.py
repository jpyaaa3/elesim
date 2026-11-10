"""Test bounded command bridges into a frozen policy's successful reset region.

All initial states are validated reverse-prefix snapshots. Bridges run real
physics with existing action limits. No optimizer, teleport or velocity edit.
"""
import argparse
from copy import deepcopy
from contextlib import nullcontext
from pathlib import Path
import json
import numpy as np


def fresh_history(env):
    """Match a curriculum reset at handoff, preserving complete physical state."""
    from workbench.research.experiments.hug_reverse_ga.physics import sample,restore_leg_hold
    from workbench.research.experiments.hug_reverse_rl.bank import compare
    import torch
    before=sample(env);state=env.scene.scene.get_state();command=env.mapper.waypoint.clone()
    env.core.reset();env.scene.scene.reset(state);restore_leg_hold(env)
    env.mapper.waypoint[:]=command
    env.scene.robot.control_dofs_position(env.mapper.joint_targets(),dofs_idx_local=env.dofs)
    ids=torch.arange(env.num_envs,device=env.device)
    env.ledger.reset(ids,env.actual(),env.goal);env.episode_wrong.zero_()
    for attr,key in (('last_touch','touch'),('last_floor','floor'),('last_wrong_face','wrong')):
        target=getattr(env.core,attr);target[:]=torch.as_tensor(before[key],device=env.device,dtype=target.dtype)
    return compare(sample(env),before,env.plan['state_tolerances'])


def bridge_actions(waypoint, policy_actions, path_commands, reference_ticks,
                   starts, methods, elapsed, rates, target_tick=350, already_handed=None,target_offsets=None):
    """Bounded command-space proposals; the task applies its normal clamps."""
    output=np.array(policy_actions,copy=True)
    handed=np.zeros(len(starts),bool)
    for lane,(start,method) in enumerate(zip(starts,methods)):
        if method=='baseline' or already_handed is not None and lane in already_handed:
            handed[lane]=True;continue
        if method=='direct':
            target=path_commands[target_tick].copy()
            if target_offsets is not None:target+=target_offsets[lane]
            reached=np.max(abs(waypoint[lane]-target)/rates)<1e-4
            handed[lane]=reached
        else:
            reference=max(target_tick,reference_ticks[lane]-elapsed*10)
            target=path_commands[reference].copy()
            if target_offsets is not None:target+=target_offsets[lane]
            handed[lane]=elapsed>=(reference_ticks[lane]-target_tick+9)//10+1
        if not handed[lane]:output[lane]=np.clip((target-waypoint[lane])/rates,-1,1)
    return output,handed


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--starts',type=int,nargs='+',default=[376,377,380,390,400,425,450,475,490,-1])
    p.add_argument('--methods',choices=('baseline','direct','backtrack'),nargs='+',default=['baseline','direct','backtrack'])
    p.add_argument('--policies',nargs='+',default=['model_500','model_final']);p.add_argument('--video',action='store_true')
    p.add_argument('--target-linear-offsets',type=float,nargs='+',default=[0.])
    p.add_argument('--target-roll-offsets',type=float,nargs='+',default=[0.])
    p.add_argument('--capture-prefix',action='store_true')
    p.add_argument('--fresh-handoff',action='store_true',help='Clear task history at handoff with full physical state preserved')
    p.add_argument('--repeats',type=int,default=1)
    p.add_argument('--noise-scales',type=float,nargs='+',default=[0.])
    a=p.parse_args(argv)
    if a.video and any(len(v)!=1 for v in (a.starts,a.methods,a.policies,a.target_linear_offsets,a.target_roll_offsets)):p.error('Video requires a single start, method, policy and target')
    if not 1<=a.repeats<=64 or not all(np.isfinite(v) and 0<=v<=1 for v in a.noise_scales):p.error('Invalid repetition count or noise scale')
    if a.video and (a.repeats!=1 or len(a.noise_scales)!=1):p.error('Video requires a single lane')
    if a.fresh_handoff and any(len(v)!=1 for v in (a.starts,a.methods,a.target_linear_offsets,a.target_roll_offsets)):p.error('Fresh handoff requires a single lane')
    if not all(np.isfinite(v) and abs(v)<=.05 for v in a.target_linear_offsets) or not all(np.isfinite(v) and abs(v)<=.3 for v in a.target_roll_offsets):p.error('Diagnostic offsets exceed bounded search range')
    from workbench.research.experiments.hug_reverse_rl.data import load_plan,sha
    from workbench.research.experiments.hug_reverse_rl.environment import CurriculumEnv
    from workbench.research.experiments.hug_reverse_ga.prepare import save
    from workbench.research.experiments.hug_reverse_ga.physics import sample
    from workbench.research.experiments.hug_c.policy import NumpyPolicy
    from workbench.research.experiments.hug_c.control import action_rates
    from workbench.research.experiments.hug_interpolation.video import record_video
    import torch
    original,path=load_plan(a.run/'plan');plan=deepcopy(original)
    if any(t<350 and t!=-1 or t>490 for t in a.starts):raise ValueError('Starts must be recorded ticks350..490 or nominal Home(-1)')
    ticks=sorted(set([0,350,490,*[t for t in a.starts if t>=0]]));plan['pool_ticks']=ticks
    layout=[(s,m,dl,dr,ns) for s in a.starts for m in a.methods for dl in a.target_linear_offsets for dr in a.target_roll_offsets for ns in a.noise_scales for _ in range(a.repeats)]
    starts=[v[0] for v in layout];methods=[v[1] for v in layout]
    target_offsets=np.asarray([[v[2],v[3],0.,0.] for v in layout])
    noise_scales=np.asarray([v[4] for v in layout])
    reference_ticks=[490 if s<0 else s for s in starts]
    a.output.mkdir(parents=True,exist_ok=False)
    save(a.output/'protocol.json',dict(source=original['source_sha256'],layout=layout,target_tick=350,training_updates=0,
        criterion='Original wrap/lift/5s hold, original faults, full physical resets',
        caveat='A prescribed bridge is a feasibility diagnostic, not a learned Home policy'))
    env=CurriculumEnv(plan,path,a.output,num_envs=len(layout),backend='gpu',video=a.video);env.auto_reset=False
    if a.video:env.scene.cameras['report'].set_pose(pos=(.85,-1.,1.25),lookat=(.35,.05,.55),up=(0.,0.,1.))
    summaries=[]
    for label in a.policies:
        file=a.run/'reports'/label/'policy.npz';policy=NumpyPolicy(file)
        if policy.metadata['simulation_context']!=original['simulation_context']:raise ValueError('Policy context mismatch')
        std=np.zeros(4);rng=np.random.default_rng(42)
        if np.any(noise_scales):
            checkpoint=a.run/'training'/f'{label}.pt'
            weights=torch.load(checkpoint,map_location='cpu',weights_only=True)
            std=weights['actor_state_dict']['distribution.std_param'].numpy()
            if not np.isfinite(std).all() or np.any(std<=0):raise ValueError('Invalid scalar Gaussian std')
            save(a.output/(label+'-exploration.json'),dict(std=std.tolist(),seed=42,checkpoint_sha256=sha(checkpoint)))
        for lane,s in enumerate(starts):
            env.set_mode('nominal-home' if s<0 else 'fixed',0 if s<0 else ticks.index(s))
            env.reset(torch.tensor([lane],device=env.device))
        env.set_mode('fixed',0)
        active=torch.ones(len(layout),device=env.device,dtype=torch.bool)
        wrong=torch.zeros_like(active);passed=torch.zeros_like(active);faulted=torch.zeros_like(active)
        ended=np.zeros(len(layout),int);handoff={};frames=[];prefix=[sample(env)]
        prefix_commands=[env.mapper.waypoint.cpu().numpy().copy()];prefix_done=False;reset_errors=None
        def capture(core):
            if a.capture_prefix and not prefix_done:
                prefix.append(sample(env));prefix_commands.append(env.mapper.waypoint.cpu().numpy().copy())
        env.core.tick_monitor=capture
        context=record_video(env.core,a.output/f'{label}.mp4') if a.video else nullcontext()
        with torch.inference_mode(),context:
            for step in range(env.max_episode_length):
                obs=env.get_observations()['policy'].cpu().numpy()
                noise=rng.normal(size=(len(layout),4))*std*noise_scales[:,None]
                actions,handed=bridge_actions(env.mapper.waypoint.cpu().numpy(),policy(obs)+noise,path['command'],reference_ticks,starts,methods,step,action_rates(env.s),already_handed=handoff,target_offsets=target_offsets)
                for lane in np.flatnonzero(handed):
                    if lane not in handoff and bool(active[lane]):
                        state=sample(env);handoff[int(lane)]=dict(step=step,actual=state['actual'][lane].tolist(),object_pos=state['object_pos'][lane].tolist(),object_vel=state['object_vel'][lane].tolist())
                if len(handoff)==len(layout) and not prefix_done:
                    prefix_done=True
                    if a.fresh_handoff:
                        reset_errors=fresh_history(env)
                        actions=policy(env.get_observations()['policy'].cpu().numpy())+noise
                act=torch.as_tensor(actions,device=env.device,dtype=torch.float32);act[~active]=0
                _,_,done,_=env.step(act)
                wrong|=active&(env.macro_wrong>0);faulted|=active&env.fault;passed|=active&env.success&~faulted&~wrong
                end=active&done;ended[end.cpu().numpy()]=step+1
                frames.append(dict(q=env.actual().cpu().numpy().copy(),object=env.scene.object.get_pos().cpu().numpy().copy(),active=active.cpu().numpy().copy()))
                active&=~done
                if not bool(active.any()):break
        rows=[dict(start_tick=s,method=m,target_linear_offset_m=dl,target_roll_offset_rad=dr,noise_scale=ns,success=bool(passed[i]),fault=bool(faulted[i]),wrong_face=bool(wrong[i]),steps=int(ended[i]),handoff=handoff.get(i)) for i,(s,m,dl,dr,ns) in enumerate(layout)]
        save(a.output/(label+'.json'),dict(policy_sha256=sha(file),results=rows,fresh_handoff=a.fresh_handoff,physical_preservation_errors=reset_errors))
        if a.capture_prefix:
            np.savez_compressed(a.output/(label+'-prefix.npz'),**{k:np.asarray([f[k] for f in prefix]) for k in prefix[0]},command=np.asarray(prefix_commands),time_s=np.arange(len(prefix))*env.s['dt'],metadata_json=np.asarray(json.dumps(dict(simulation_context=original['simulation_context'],phase='nominal-Home-to-handoff',task_history_reset_at_handoff=a.fresh_handoff,training_updates=0))))
        np.savez_compressed(a.output/(label+'.npz'),**{k:np.asarray([f[k] for f in frames]) for k in frames[0]})
        summary=dict(policy=label,results=[{k:r[k] for k in ('start_tick','method','target_linear_offset_m','target_roll_offset_rad','noise_scale','success','fault','steps')} for r in rows]);summaries.append(summary)
        print(json.dumps(summary),flush=True)
    save(a.output/'completion.json',dict(status='evaluated',results=summaries,training_updates=0))


if __name__=='__main__':main()
