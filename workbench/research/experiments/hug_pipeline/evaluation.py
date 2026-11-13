"""Fresh Home policy/command evaluations, never prepared-prefix execution."""
from pathlib import Path
from contextlib import nullcontext
import json
import numpy as np
from .contracts import save,canonical


def check_commands(commands,study):
    from workbench.research.experiments.hug_c.control import command_bounds,action_rates
    c=np.asarray(commands,float);s=study['settings'];lo,hi=command_bounds(s)
    if c.ndim!=2 or c.shape[1]!=4 or len(c)<2 or not np.isfinite(c).all():raise ValueError('Invalid commands')
    if not np.allclose(c[0],study['home'],atol=1e-7):raise ValueError('Trajectory must start at Home')
    if np.any(c<lo-1e-7) or np.any(c>hi+1e-7) or np.any(abs(np.diff(c,axis=0))>action_rates(s)/s['action_ticks']+1e-6):raise ValueError('Command limits/slew exceeded')
    if (len(c)-1)>s['episode_steps']*s['action_ticks']:raise ValueError('Episode length exceeded')
    from .linear import check_trajectory
    check_trajectory(c)
    return c


def evaluate(study,case,output,*,policy_path=None,commands=None,repetitions=3,video=True):
    import torch
    from .task import LinearHugEnv
    from .policy import NumpyPolicy
    from workbench.research.experiments.hug_interpolation.video import record_video
    from workbench.research.experiments.hug_reverse_ga.physics import sample
    if (policy_path is None)==(commands is None):raise ValueError('Choose policy or command replay')
    if commands is not None:commands=check_commands(commands,study)
    ctx=dict(study['shared_context'],case_sha256=canonical(case))
    if policy_path:
        policy=NumpyPolicy(policy_path)
        if policy.metadata.get('simulation_context')!=ctx:raise ValueError('Policy case/shared context mismatch')
    exp=dict(settings=study['settings'],simulation_context=ctx,cases=[case])
    env=LinearHugEnv(exp,case,output/'assets',n_envs=1,backend=study['backend'],auto_reset=False,video=video)
    if video:env.scene.cameras['report'].set_pose(pos=(.85,-1.,1.25),lookat=(.35,.05,.55),up=(0.,0.,1.))
    summaries=[]
    for rep in range(repetitions):
        obs,_=env.reset();trace=[np.asarray(study['home'])];frames=[sample(env)]
        flags=[(False,False,0,False)];wrong=False;fault=False;passed=False
        def capture(core):
            nonlocal wrong,fault
            wrong|=bool(core.last_wrong_face[0]);fault|=bool(core.fault[0])
            trace.append(core.mapper.waypoint[0].detach().cpu().numpy().copy())
            frames.append(sample(core));flags.append((bool(core.grasp_tick[0]>=0),bool(core.object_clearance()[0]>=.02 and not core.last_floor[0]),int(core.hold.count[0]),bool(core.success[0])))
        env.tick_monitor=capture
        with torch.no_grad(), record_video(env,output/f'replay-{rep+1}.mp4') if video and rep==0 else nullcontext():
            if policy_path:
                for _ in range(env.max_episode_length):
                    action=torch.as_tensor(policy(obs['policy'].cpu().numpy()),device=env.device,dtype=torch.float32)
                    obs,_,done,_=env.step(action)
                    if bool(done[0]):break
            else:
                from workbench.research.experiments.hug_reverse_ga.physics import tick
                for cmd in commands[1:]:
                    tick(env,np.asarray(cmd)[None])
                    if fault or wrong:break
        passed=bool(env.success[0]) and not wrong and not fault
        f=np.asarray(flags);events=None
        if passed:
            # Final continuous hold onset, not an earlier interrupted hold.
            success=len(trace)-1;hold=success-int(f[-1,2])+1
            grasps=np.flatnonzero(f[:,0]);lifts=np.flatnonzero(f[:,1])
            if len(grasps) and len(lifts):events=[0,int(grasps[0]),int(lifts[0]),max(hold,int(lifts[0])),success]
        summary=dict(accepted=passed,fault=fault,wrong_face=wrong,ticks=len(trace)-1,events=events,
            case_id=case['case_id'],policy_execution=policy_path is not None,scripted_prefix=False,
            linear=env.linear_report())
        summaries.append(summary)
        state={k:np.asarray([s[k][0] for s in frames]) for k in frames[0]}
        np.savez_compressed(output/f'rollout-{rep+1}.npz',commands=np.asarray(trace),**state,
            events=np.asarray(events if events else [],int),metadata_json=np.asarray(json.dumps(dict(summary,shared_context=study['shared_context'],case=case))))
    result=dict(accepted=all(r['accepted'] for r in summaries),repetitions=repetitions,results=summaries,
        case_sha256=canonical(case),nominal_home=True)
    save(output/'evaluation.json',result);return result


def align(paths,study):
    """Stretch each physical phase only. Preserve final hold; replays still needed."""
    if len(paths)!=3:raise ValueError('Need three verified anchor paths')
    durations=[]
    for p in paths:
        c=check_commands(p['commands'],study);e=np.asarray(p['events'],int)
        if e.shape!=(5,) or e[0]!=0 or e[-1]!=len(c)-1 or np.any(np.diff(e)<=0):raise ValueError('Incompatible event ordering')
        durations.append(np.diff(e))
    common=np.max(durations,axis=0);result=[]
    for p in paths:
        chunks=[]
        for i,length in enumerate(common):
            segment=p['commands'][p['events'][i]:p['events'][i+1]+1]
            t=np.linspace(0,len(segment)-1,int(length)+1)
            chunk=np.column_stack([np.interp(t,np.arange(len(segment)),segment[:,j]) for j in range(4)])
            chunks.append(chunk if i==0 else chunk[1:])
        result.append(check_commands(np.concatenate(chunks),study))
    return result,np.r_[0,np.cumsum(common)].tolist()


def blend(aligned,vertices,target,study):
    from workbench.research.experiments.hug_c.triangle import weights
    w=weights(np.asarray(vertices),np.asarray(target))
    if np.any(w< -1e-9):raise ValueError('Extrapolation forbidden')
    if len({a.shape for a in aligned})!=1:raise ValueError('Align phases first')
    w=np.maximum(w,0);w/=w.sum()
    return check_commands(np.tensordot(w,np.asarray(aligned),axes=1),study),w
