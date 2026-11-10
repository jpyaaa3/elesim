"""Reverse commands are proposals; forward contacts must pass fresh physics."""
from pathlib import Path
import json
from contextlib import nullcontext
import numpy as np
from .contracts import save,sha,canonical


def proposal(reverse,packed):
    from workbench.research.experiments.hug_reverse_rl.data import prepare
    # This validates three reverse replays and valid winner frames, not a policy.
    if not packed.exists():prepare(reverse,packed)
    with np.load(packed/'reference-path.npz',allow_pickle=False) as d:cmd=d['command'].copy()
    p=json.loads((packed/'plan.json').read_text());end=p['pool_ticks'][-1]
    return cmd[:end+1][::-1].copy()


def seeds(proposed,count,knots,lo,hi,rng):
    """Seed interior knots from the reverse solution, with fixed true endpoints."""
    times=np.linspace(0,len(proposed)-1,knots+2)[1:-1]
    points=np.array([[np.interp(t,np.arange(len(proposed)),proposed[:,j]) for j in range(4)] for t in times])
    g=np.full((count,knots+1,5),.5)
    g[:,:-1,:4]=(points-lo)/(hi-lo)
    g[:,-1,4]=min(.9,max(.05,(len(proposed)-1)/1800))
    g[1:,:,:4]+=rng.normal(0,.08,g[1:,:,:4].shape)
    g[1:,:,4]+=rng.normal(0,.1,g[1:,:,4].shape)
    return np.clip(g,0,1)


def rollout(env,paths,*,record_states=False):
    from workbench.research.experiments.hug_reverse_ga.physics import sample,tick,arrays
    from workbench.research.experiments.hug_reverse_ga.criteria import violations,admission_good,error
    env.reset();lo,hi,rate=arrays(env);n=env.num_envs
    bad=np.zeros(n,bool);success=np.zeros(n,bool);window=np.zeros(n,int);end=np.zeros(n,int)
    best_window=np.zeros(n,int);best_tick=np.zeros(n,int);best_contacts=np.zeros(n);final_xy=np.zeros(n)
    distance=np.zeros(n);peak=np.zeros(n);reasons=np.zeros((n,6),bool)
    previous=paths[:,0];frames=[sample(env)] if record_states else []
    commands=[previous.copy()] if record_states else []
    for t in range(1,paths.shape[1]):
        active=~(bad|success);cmd=np.where(active[:,None],paths[:,t],previous)
        if record_states:s=tick(env,cmd)
        else:
            import torch
            with torch.no_grad():env.tick(torch.as_tensor(cmd,device=env.device,dtype=torch.float32))
            def array(value):return value.detach().cpu().numpy().copy()
            obj=env.scene.object
            s=dict(actual=array(env.actual()),object_pos=array(obj.get_pos()),object_quat=array(obj.get_quat()),
                object_vel=array(obj.get_vel()),object_ang=array(obj.get_ang()),touch=array(env.last_touch),
                floor=array(env.last_floor),fault=array(env.fault),wrong=array(env.last_wrong_face))
        mask,xy,_=violations(s,cmd,previous,lo,hi,rate,env.case['object_xyz_m'])
        reasons|=mask&active[:,None];bad|=mask.any(-1)&active
        good=admission_good(s,np.asarray(env.case['goal_command']))&~bad
        window=np.where(good,window+1,0);new=(window>=round(.3/env.s['dt']))&active
        success|=new;end[new]=t
        distance[active]=error(s['actual'],np.asarray(env.case['goal_command']))[active]
        best_tick=np.where(window>best_window,t,best_tick)
        best_window=np.maximum(best_window,window)
        near_goal=(distance<=1)&active&~bad
        best_contacts=np.maximum(best_contacts,np.where(near_goal,np.sum(s['touch']>0,axis=-1),0))
        final_xy[active]=xy[active]
        peak=np.maximum(peak,np.where(active,xy,0))
        if record_states:frames.append(s);commands.append(cmd.copy())
        previous=cmd
        if (bad|success).all():break
    reports=[dict(success=bool(success[i]),rejected=bool(bad[i]),end_tick=int(end[i]),
        pose_error=float(distance[i]),wrap_ticks=int(window[i]),best_wrap_ticks=int(best_window[i]),best_wrap_tick=int(best_tick[i]),
        near_goal_contacts=float(best_contacts[i]),final_xy_m=float(final_xy[i]),peak_xy_m=float(peak[i]),
        violation_mask=reasons[i].tolist()) for i in range(n)]
    return reports,frames,commands


def rank(r):
    # Once pose is within the declared tolerance, sub-tolerance q noise must
    # not outrank an actual second contact or a less displaced cylinder.
    return (2 if r['success'] else 0 if r['rejected'] else 1,
        r['best_wrap_ticks'],r['near_goal_contacts'],-max(0.,r['pose_error']-1.),
        -r['final_xy_m'],-r['peak_xy_m'])


def pause_proposal(path,tick,count):
    if not 0<tick<len(path) or count<1:raise ValueError('Invalid dwell insertion')
    return np.concatenate((path[:tick],np.repeat(path[tick:tick+1],count,axis=0),path[tick:]),axis=0)[:len(path)]


def slew_proposal(proposed,home,goal,rate,steps):
    seq=np.vstack((home,proposed,np.repeat([goal],100,axis=0)))
    path=np.empty((steps+1,4));path[0]=home
    for t in range(1,steps+1):path[t]=path[t-1]+np.clip(seq[min(t,len(seq)-1)]-path[t-1],-rate,rate)
    return path


def search(plan,case,reverse,output,study_root=None):
    from workbench.research.experiments.hug_ga.parade import create_env,record
    from workbench.research.experiments.hug_reverse_ga.physics import arrays
    from workbench.research.experiments.hug_reverse_ga.search import decode
    proposed=proposal(reverse,output/'reverse-export');n=plan['budgets']['population'];rng=np.random.default_rng(plan['seed'])
    exp=dict(settings=plan['settings'],simulation_context=plan['shared_context'],cases=[case])
    env=create_env(exp,case,output/'assets',n_envs=n,backend=plan['backend'])
    lo,hi,rate=arrays(env);steps=round(20/env.s['dt'])
    population=seeds(proposed,n,plan['knots'],lo,hi,rng);overrides=[]
    for entry in plan.get('forward_seeds',[]):
        file=study_root/entry['file']
        if sha(file)!=entry['sha256']:raise ValueError('Forward seed changed')
        with np.load(file,allow_pickle=False) as raw:
            key=next((k for k in ('command','commands','best_commands') if k in raw.files),None)
            if key is None:raise ValueError('Forward seed has no commands')
            candidate=raw[key].copy()
        if candidate.ndim!=2 or candidate.shape[1]!=4 or not np.isfinite(candidate).all():raise ValueError('Invalid forward seed')
        overrides.append(slew_proposal(candidate,plan['home'],case['goal_command'],rate,steps))
    for generation in range(plan['budgets']['repair_generations']):
        paths=decode(population,np.asarray(plan['home']),np.asarray(case['goal_command']),lo,hi,rate,steps)
        # Exact reverse proposal gets a chance before any mutation; force a rate-
        # limited bridge from true Home, never a state teleport to its first q.
        if generation==0:paths[0]=slew_proposal(proposed,plan['home'],case['goal_command'],rate,steps)
        for lane,candidate in enumerate(overrides[:max(0,n-1)],1):paths[lane]=candidate
        with record(env,output/f'generation-{generation+1:04d}.mp4') if (generation+1)%10==0 else nullcontext():
            reports,_,_=rollout(env,paths)
        order=sorted(range(n),key=lambda i:rank(reports[i]),reverse=True);best=order[0]
        np.savez_compressed(output/f'generation-{generation+1:04d}.npz',genomes=population,best_commands=paths[best])
        save(output/f'generation-{generation+1:04d}.json',dict(candidates=reports))
        save(output/'progress.json',dict(generation=generation+1,best=reports[best],successful=sum(r['success'] for r in reports)))
        if reports[best]['success']:
            winner=paths[best:best+1,:reports[best]['end_tick']+1];verified=[]
            for repetition in range(3):
                with record(env,output/f'verification-{repetition+1}.mp4') if repetition==0 else nullcontext():
                    rr,frames,commands=rollout(env,np.repeat(winner,n,axis=0),record_states=True)
                ok=all(r['success'] and not r['rejected'] for r in rr);verified.append(ok)
                save(output/f'verification-{repetition+1}.json',dict(accepted=ok,results=rr))
                if ok:
                    end=rr[0]['end_tick']+1
                    path={k:np.asarray([s[k][0] for s in frames[:end]]) for k in frames[0]}
                    path.update(command=np.asarray(commands[:end])[:,0],time_s=np.arange(end)*env.s['dt'])
                    np.savez_compressed(output/f'reference-{repetition+1}.npz',**path)
            result=dict(status='forward_reference_verified' if all(verified) else 'forward_verification_failed',
                replays=verified,selected_case_sha256=canonical(case),reverse_plan_sha256=sha(reverse/'plan.json'),
                policy_used=False,generations=generation+1)
            save(output/'completion.json',result);return result
        overrides=[paths[best].copy()]
        if reports[best]['best_wrap_ticks']>0:
            end=reports[best]['best_wrap_tick'];start=end-reports[best]['best_wrap_ticks']+1
            for t in sorted(set([start,(start+end)//2,end])):
                overrides.append(pause_proposal(paths[best],t,round(.5/env.s['dt'])))
        elite=population[order[:max(2,n//4)]];new=[population[order[0]].copy()]
        while len(new)<n:
            a,b=elite[rng.integers(len(elite),size=2)]
            child=np.where(rng.random(a.shape)<.5,a,b)
            child+=(rng.random(a.shape)<.3)*rng.normal(0,.12,a.shape)
            if rng.random()<.1:child=rng.random(a.shape)
            new.append(np.clip(child,0,1))
        population=np.asarray(new)
    result=dict(status='forward_reference_failed',policy_used=False,generations=plan['budgets']['repair_generations'])
    save(output/'completion.json',result);return result


def validate(path,plan):
    from workbench.research.experiments.hug_home_rl.data import FIELDS
    from workbench.research.experiments.hug_reverse_ga.criteria import admission_good,violations
    from workbench.research.experiments.hug_c.control import action_rates,command_bounds
    if set(path)!=set(FIELDS):raise ValueError('Full forward state fields required')
    n=len(path['time_s']);s=plan['settings'];case=plan['case']
    if n<2 or not np.allclose(path['time_s'],np.arange(n)*s['dt'],atol=1e-8):raise ValueError('Reference clock')
    if any(len(v)!=n or not np.isfinite(v).all() for v in path.values()):raise ValueError('Reference values')
    widths={'actual':4,'actual_vel':4,'command':4,'object_pos':3,'object_quat':4,'object_vel':3,'object_ang':3,'touch':2}
    if any(path[k].shape!=(n,w) for k,w in widths.items()):raise ValueError('Reference vector shapes')
    if any(path[k].shape!=(n,) for k in ('floor','fault','wrong','contact_force','contact_penetration')):raise ValueError('Reference scalar shapes')
    if path['joint_q'].ndim!=2 or path['joint_q'].shape!=path['joint_qd'].shape or path['joint_q'].shape[1]<12:raise ValueError('Full joint state required')
    if np.max(path['contact_penetration'])>s['max_penetration_m']:raise ValueError('Contact penetration')
    if not np.allclose(path['actual'][0],plan['home'],atol=1e-6) or not np.allclose(path['command'][0],plan['home'],atol=1e-7):raise ValueError('True Home start required')
    if (not np.allclose(path['object_pos'][0],case['object_xyz_m'],atol=1e-6)
        or not np.allclose(path['object_quat'][0],[1,0,0,0],atol=1e-6)
        or any(np.max(abs(path[k][0]))>1e-6 for k in ('joint_qd','actual_vel','object_vel','object_ang'))):raise ValueError('Nominal fresh reset required')
    cmd=path['command'];lo,hi=command_bounds(s)
    bad,_,_=violations(path,cmd,np.vstack((cmd[0],cmd[:-1])),lo,hi,action_rates(s)/s['action_ticks'],case['object_xyz_m'])
    if bad.any():raise ValueError('Unsafe forward reference')
    good=admission_good(path,np.asarray(case['goal_command']));hold=round(.3/s['dt'])
    if n<hold or not good[-hold:].all():raise ValueError('Stable selected Wrap endpoint required')
    return n-1
