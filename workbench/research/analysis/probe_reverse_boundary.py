"""Frozen-policy sweep of real reverse-prefix states; no optimizer updates."""
import argparse
import json
from copy import deepcopy
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--noise',action='store_true',help='Compare frozen Gaussian exploration scales with 32 replicas per state')
    a=p.parse_args()
    from workbench.research.experiments.hug_reverse_rl.data import load_plan,sha
    from workbench.research.experiments.hug_reverse_rl.environment import CurriculumEnv
    from workbench.research.experiments.hug_c.policy import NumpyPolicy
    from workbench.research.experiments.hug_reverse_ga.prepare import save
    import torch
    original,path=load_plan(a.run/'plan');plan=deepcopy(original)
    ticks=sorted(set([0,350,*range(375,401),425,450,475,490]))
    plan['pool_ticks']=ticks
    starts=list(range(len(ticks)))+[-1]
    noise_scales=np.zeros(len(starts))
    if a.noise:
        starts=[];noise_scales=[]
        for tick_ in (375,376,377,400):
            for scale in (0.,.25,1.):
                starts.extend([ticks.index(tick_)]*32);noise_scales.extend([scale]*32)
        noise_scales=np.asarray(noise_scales)
    a.output.mkdir(parents=True,exist_ok=False)
    save(a.output/'protocol.json',dict(original_source=original['source_sha256'],ticks=ticks,starts=starts,noise_scales=noise_scales.tolist(),nominal_home=not a.noise,training_updates=0,reset='Recorded full physical snapshots, no interpolation or velocity changes'))
    env=CurriculumEnv(plan,path,a.output,num_envs=len(starts),backend='gpu');env.auto_reset=False
    results=[]
    for label in (('model_final',) if a.noise else ('model_500','model_1000','model_final')):
        file=a.run/'reports'/label/'policy.npz'
        policy=NumpyPolicy(file)
        if policy.metadata['simulation_context']!=original['simulation_context']:raise ValueError('Policy context mismatch')
        rng=np.random.default_rng(42)
        std=np.zeros(4)
        if a.noise:
            ckpt=torch.load(a.run/'training'/f'{label}.pt',map_location='cpu',weights_only=True)
            std=ckpt['actor_state_dict']['distribution.std_param'].numpy()
            save(a.output/'exploration.json',dict(std=std.tolist(),seed=42,checkpoint_sha256=sha(a.run/'training'/f'{label}.pt')))
        for lane,b in enumerate(starts):
            env.set_mode('nominal-home' if b<0 else 'fixed',max(0,b))
            env.reset(torch.tensor([lane],device=env.device))
        env.set_mode('fixed',0)
        active=torch.ones(len(starts),dtype=torch.bool,device=env.device)
        wrong=torch.zeros_like(active);passed=torch.zeros_like(active)
        ended=np.zeros(len(starts),int);fault_details={};frames=[]
        def monitor(core):
            fresh=(active&core.fault).nonzero().flatten().cpu().tolist()
            fresh=[i for i in fresh if i not in fault_details]
            if not fresh:return
            raw=core.scene.robot.get_contacts(exclude_self_contact=False,is_padded=True)
            for lane in fresh:
                pairs=[]
                for j in raw['valid_mask'][lane].nonzero().flatten().cpu().tolist():
                    force=float(raw['force_a'][lane,j].norm());depth=float(raw['penetration'][lane,j])
                    if force<1e-4 and depth<.005:continue
                    ia=int(raw['link_a'][lane,j]);ib=int(raw['link_b'][lane,j])
                    names=core.scene.links.name_by_index
                    pairs.append(dict(a=names.get(ia,str(ia)),b=names.get(ib,str(ib)),force_n=force,penetration_m=depth))
                fault_details[lane]=dict(tick=int(core.ticks[lane]),contacts=pairs,object_pos=core.scene.object.get_pos()[lane].cpu().tolist(),joint_q=core.scene.robot.get_dofs_position(dofs_idx_local=core.dofs)[lane].cpu().tolist())
        env.core.tick_monitor=monitor
        with torch.inference_mode():
            for step in range(env.max_episode_length):
                obs=env.get_observations()['policy'].cpu().numpy()
                values=policy(obs)+rng.normal(size=(len(starts),4))*std*noise_scales[:,None]
                actions=torch.as_tensor(values,device=env.device,dtype=torch.float32);actions[~active]=0
                _,_,done,_=env.step(actions)
                wrong|=active&(env.macro_wrong>0);passed|=active&env.success&~env.fault&~wrong
                end=active&done;ended[end.cpu().numpy()]=step+1
                frames.append(dict(q=env.actual().cpu().numpy().copy(),object=env.scene.object.get_pos().cpu().numpy().copy(),active=active.cpu().numpy().copy()))
                active&=~done
                if not bool(active.any()):break
        rows=[]
        for i,b in enumerate(starts):
            t=None if b<0 else ticks[b]
            rows.append(dict(lane=i,start_tick=t,noise_scale=float(noise_scales[i]),success=bool(passed[i]),wrong_face=bool(wrong[i]),fault=bool(env.fault[i]) if ended[i]==0 else i in fault_details,steps=int(ended[i]),first_fault=fault_details.get(i),initial_object_pos=None if t is None else path['object_pos'][t].tolist(),initial_object_vel=None if t is None else path['object_vel'][t].tolist()))
        save(a.output/(label+'.json'),dict(policy_sha256=sha(file),results=rows))
        np.savez_compressed(a.output/(label+'.npz'),**{k:np.asarray([f[k] for f in frames]) for k in frames[0]})
        results.append(dict(policy=label,success_ticks=[r['start_tick'] for r in rows if r['success']],failed_ticks=[r['start_tick'] for r in rows if not r['success']]))
        print(json.dumps(results[-1]),flush=True)
    save(a.output/'completion.json',dict(status='evaluated',results=results,training_updates=0))


if __name__=='__main__':main()
