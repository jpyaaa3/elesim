"""Explicit GA demonstration verification -> BC actor -> ordinary PPO.

No command runs implicitly. Failed search rollouts cannot enter BC.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path, value): path.write_text(json.dumps(value, indent=2, allow_nan=False))


def load_demo(path):
    with np.load(path, allow_pickle=False) as data:
        if str(data['schema'].item()) != 'hug-ga-verified-demo-v1':
            raise ValueError('Require independently verified demonstration')
        meta=json.loads(str(data['metadata_json'].item()))
        x=data['observations'].copy(); y=data['actions'].copy()
    if meta.get('verified') is not True or meta.get('repetitions',0)<3:
        raise ValueError('Require three successful Home replays')
    if x.ndim!=2 or x.shape[1]!=20 or y.shape!=(len(x),4) or len(x)<1:
        raise ValueError('Invalid observation/action dimensions')
    if not np.isfinite(x).all() or not np.isfinite(y).all() or np.any(abs(y)>1.000001):
        raise ValueError('Invalid demonstration values')
    return x,y,meta


def rollout(env, action_source, *, constraints=None):
    import torch
    from .constraints import read_motion_contract,monitor_motion
    xs=[];ys=[];wrong=False
    contract=read_motion_contract(dict(constraints=constraints))
    with monitor_motion(env,contract) as movement, torch.inference_mode():
        obs,_=env.reset()
        for step in range(env.max_episode_length):
            x=obs['policy'].cpu().numpy().copy()
            action=np.asarray(action_source(step,x),dtype=np.float32).reshape(1,4)
            xs.append(x[0]);ys.append(action[0].copy())
            obs,_,done,_=env.step(torch.as_tensor(action,device=env.device))
            wrong |= bool((env.macro_wrong>0).any())
            if bool(done[0]) or bool(movement.violated[0]): break
    result=env.measure();result['wrong_face_any_episode']=wrong
    result['object_motion_violation']=bool(movement.violated[0])
    result['max_object_xy_displacement_mm']=float(movement.peak_mm[0])
    result['max_ground_object_xy_displacement_mm']=float(movement.peak_ground_mm[0])
    result['accepted']=bool(result['success'] and not result['fault'] and not wrong and not result['object_motion_violation'])
    return np.asarray(xs),np.asarray(ys),result


def verify(args):
    from workbench.research.experiments.hug_c.configure import load_experiment
    from workbench.research.experiments.hug_c.experiment import create_env
    from .constraints import read_motion_contract
    exp=load_experiment(args.experiment)
    with np.load(args.rollouts,allow_pickle=False) as d:
        if str(d['schema'].item())!='hug-ga-rollouts-v1':raise ValueError('Wrong rollout schema')
        meta=json.loads(str(d['metadata_json'].item()))
        actions=d['actions'].copy();valid=d['valid'].copy()
    if meta['simulation_context']!=exp['simulation_context']:raise ValueError('Context mismatch')
    constraints=read_motion_contract(meta)
    case=next(c for c in exp['cases'] if c['case_id']==meta['case']['case_id'])
    if case!=meta['case'] or case['split']!='train':raise ValueError('Case mismatch or held-out case')
    if actions.ndim!=3 or actions.shape[-1]!=4 or valid.shape!=actions.shape[:2]:raise ValueError('Invalid rollout shape')
    if not 0<=args.candidate<actions.shape[1]:raise ValueError('Candidate out of range')
    mask=valid[:,args.candidate].astype(bool);length=int(mask.sum())
    if length<1 or not mask[:length].all() or mask[length:].any():raise ValueError('Noncontiguous validity')
    sequence=actions[:length,args.candidate]
    if not np.isfinite(sequence).all() or np.any(abs(sequence)>1.000001):raise ValueError('Invalid actions')
    args.output.mkdir(parents=True,exist_ok=False)
    env=create_env(exp,case,args.output,args)
    results=[];xs=[];ys=[]
    for _ in range(3):
        def action(step,obs):
            if step>=len(sequence):raise ValueError('Source actions ended before success/termination')
            return sequence[step]
        x,y,result=rollout(env,action,constraints=constraints);results.append(result);xs.append(x);ys.append(y)
    approved=all(r['accepted'] for r in results)
    save(args.output/'verification.json',dict(verified=approved,results=results,constraints=constraints,source_sha256=sha(args.rollouts)))
    if not approved:return 1
    metadata=dict(verified=True,repetitions=3,case=case,simulation_context=exp['simulation_context'],constraints=constraints,
                  source_sha256=sha(args.rollouts),candidate=args.candidate,perturbations_tested=False)
    np.savez_compressed(args.output/'demonstration.npz',schema=np.asarray('hug-ga-verified-demo-v1'),
        observations=np.concatenate(xs),actions=np.concatenate(ys),metadata_json=np.asarray(json.dumps(metadata)))
    return 0


def fit_actor(x,y,epochs,batch_size,seed):
    import torch
    from tensordict import TensorDict
    from rsl_rl.models import MLPModel
    torch.manual_seed(seed)
    observation=TensorDict({'policy':torch.as_tensor(x,dtype=torch.float32)},batch_size=[len(x)])
    actor=MLPModel(observation,{'actor':['policy']},'actor',4,hidden_dims=[256,128,64],
                   activation='elu',obs_normalization=True,
                   distribution_cfg=dict(class_name='rsl_rl.modules.GaussianDistribution',init_std=.5,std_type='scalar',learn_std=True))
    actor.train();actor.update_normalization(observation);actor.eval()
    target=torch.as_tensor(y,dtype=torch.float32)
    optimizer=torch.optim.Adam(actor.mlp.parameters(),lr=3e-4)
    losses=[]
    for _ in range(epochs):
        total=0.
        for ids in torch.randperm(len(x)).split(batch_size):
            loss=(actor(observation[ids])-target[ids]).square().mean()
            if not torch.isfinite(loss):raise RuntimeError('Nonfinite BC loss')
            optimizer.zero_grad();loss.backward();optimizer.step();total+=float(loss.detach())*len(ids)
        losses.append(total/len(x))
    return actor,losses


def bc(args):
    import torch
    from workbench.research.experiments.hug_c.configure import load_experiment
    from workbench.research.experiments.hug_c.policy import export_policy
    from .constraints import read_motion_contract
    exp=load_experiment(args.experiment)
    demos=[load_demo(p) for p in args.demonstrations];reference=demos[0][2]
    if any(m['simulation_context']!=exp['simulation_context'] or m['case']!=reference['case'] for _,_,m in demos):
        raise ValueError('All demonstrations must share one case/context')
    constraints=read_motion_contract(reference)
    if any(read_motion_contract(m)!=constraints for _,_,m in demos):
        raise ValueError('Demonstrations have different GA movement constraints')
    case=reference['case']
    if case not in exp['cases'] or case['split']!='train':raise ValueError('Unknown or held-out case')
    args.output.mkdir(parents=True,exist_ok=False)
    actor,losses=fit_actor(np.concatenate([x for x,_,_ in demos]),np.concatenate([y for _,y,_ in demos]),args.epochs,args.batch_size,args.seed)
    torch.save(actor.state_dict(),args.output/'actor.pt')
    export_policy(actor,args.output/'policy.npz',dict(case,simulation_context=exp['simulation_context']))
    save(args.output/'bc.json',dict(schema='hug-ga-bc-v1',case=case,simulation_context=exp['simulation_context'],constraints=constraints,
        actor_sha256=sha(args.output/'actor.pt'),policy_sha256=sha(args.output/'policy.npz'),
        demonstrations=[dict(path=str(p),sha256=sha(p)) for p in args.demonstrations],
        epochs=args.epochs,seed=args.seed,training_losses=losses,physics_validated=False))
    return 0


def ppo(args):
    import torch
    from workbench.research.experiments.hug_c.configure import load_experiment
    from workbench.research.experiments.hug_c.experiment import create_env,runner
    from workbench.research.experiments.hug_c.policy import NumpyPolicy,export_policy
    exp=load_experiment(args.experiment);record=json.loads((args.bc/'bc.json').read_text())
    if record['schema']!='hug-ga-bc-v1' or record['simulation_context']!=exp['simulation_context']:
        raise ValueError('BC context mismatch')
    for name,key in [('actor.pt','actor_sha256'),('policy.npz','policy_sha256')]:
        if sha(args.bc/name)!=record[key]:raise ValueError('BC artifact hash mismatch')
    case=record['case']
    if case not in exp['cases'] or case['split']!='train':raise ValueError('Invalid BC case')
    args.output.mkdir(parents=True,exist_ok=False)
    policy=NumpyPolicy(args.bc/'policy.npz')
    env=create_env(exp,case,args.output,args,args.num_envs,True)
    learner=runner(env,args.output);actor=learner.alg.get_policy()
    actor.load_state_dict(torch.load(args.bc/'actor.pt',map_location=env.device,weights_only=True),strict=True)
    # Check normalization and deterministic actor transfer before any update.
    obs=env.get_observations()
    with torch.inference_mode():actual=actor(obs).cpu().numpy()
    error=float(np.max(abs(actual-policy(obs['policy'].cpu().numpy()))))
    if error>1e-4:raise ValueError('BC/PPO actor mismatch')
    # Full-batch deterministic gate avoids a second simultaneous Genesis scene.
    env.auto_reset=False;results=[]
    with torch.inference_mode():
        for _ in range(3):
            obs,_=env.reset();bad=torch.zeros(args.num_envs,dtype=torch.bool,device=env.device)
            active=torch.ones_like(bad);passed=torch.zeros_like(bad)
            for step in range(env.max_episode_length):
                actions=actor(obs);actions[~active]=0
                obs,_,done,_=env.step(actions)
                bad|=active&(env.fault|(env.macro_wrong>0))
                passed|=active&env.success&~bad
                active&=~done
                if not bool(active.any()):break
            results.append(bool((passed&~bad).all()))
    save(args.output/'bc-replay.json',dict(success=all(results),repetitions=results,actor_transfer_max_error=error,
        nominal_replicas_only=True,bc_policy_sha256=record['policy_sha256']))
    if not all(results):return 1
    env.auto_reset=True;env.reset()
    learner.save(str(args.output/'model_initial.pt'))
    save(args.output/'run.json',dict(experiment=exp,case_id=case['case_id'],num_envs=args.num_envs,
        strategy='single',initial_actor=record,critic='fresh',optimizer='fresh',reward_contract=exp['simulation_context']['reward_contract']))
    learner.learn(num_learning_iterations=args.iterations,init_at_random_ep_len=False)
    learner.save(str(args.output/'model_final.pt'))
    export_policy(actor,args.output/'policy.npz',dict(env.metadata(),checkpoint_sha256=sha(args.output/'model_final.pt')))
    return 0


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    for name in ('verify-demo','bc','ppo'):
        p=sub.add_parser(name);p.add_argument('--experiment',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
        if name=='verify-demo':
            p.add_argument('--rollouts',type=Path,required=True);p.add_argument('--candidate',type=int,required=True)
        elif name=='bc':
            p.add_argument('--demonstrations',type=Path,nargs='+',required=True)
            p.add_argument('--epochs',type=int,default=100);p.add_argument('--batch-size',type=int,default=256);p.add_argument('--seed',type=int,default=42)
        else:
            p.add_argument('--bc',type=Path,required=True);p.add_argument('--num-envs',type=int,default=512);p.add_argument('--iterations',type=int,default=2000)
        p.add_argument('--backend',choices=('gpu','cpu'),default='gpu')
    args=parser.parse_args(argv)
    for key in ('epochs','batch_size','num_envs','iterations'):
        if hasattr(args,key) and getattr(args,key)<1:parser.error(key+' must be positive')
    return {'verify-demo':verify,'bc':bc,'ppo':ppo}[args.command](args)


if __name__=='__main__':raise SystemExit(main())
