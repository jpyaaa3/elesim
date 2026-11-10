"""Explicit prepare/check/train/evaluate for Home-prefix PPO refinement."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from .data import prepare,load_plan,sha,validate_initial
from workbench.research.experiments.hug_reverse_ga.prepare import save


def create_env(args,plan,path,output):
    from workbench.research.experiments.hug_c.experiment import check_versions
    from .environment import HomeEnv
    check_versions()
    return HomeEnv(plan,path,output,num_envs=args.num_envs,backend=args.backend,video=getattr(args,'video',False))


def warm_start(learner,plan,directory,output,observations):
    import torch
    from workbench.research.experiments.hug_c.policy import export_policy,NumpyPolicy
    file=Path(directory)/'initial-model.pt'
    if sha(file)!=plan['initial_checkpoint_sha256']:raise ValueError('Initial model hash mismatch')
    state=torch.load(file,map_location=learner.device,weights_only=True)
    learner.alg.load(state,dict(actor=True,critic=True,optimizer=False,iteration=False,rnd=False),strict=True)
    learner.current_learning_iteration=0
    probe=Path(output)/'warm-start-policy.npz'
    export_policy(learner.alg.get_policy(),probe,dict(simulation_context=plan['simulation_context']))
    old=NumpyPolicy(Path(directory)/'initial-policy.npz');new=NumpyPolicy(probe)
    samples=np.concatenate((observations,np.random.default_rng(42).normal(size=(128,20)).astype(np.float32)))
    error=float(np.max(abs(old(samples)-new(samples))))
    if error>1e-4:raise RuntimeError('Warm-start actor/normalization differs')
    record=dict(source_checkpoint_sha256=sha(file),optimizer_restored=False,curriculum_restored=False,
        actor_normalization_restored=True,max_actor_error=error)
    save(Path(output)/'warm-start.json',record)
    return record


def bind_checkpoint(learner,env,plan):
    original=learner.save
    def paired(path,*a,**kw):
        result=original(path,*a,**kw)
        save(Path(str(path)+'.curriculum.json'),dict(schema='hug-home-ppo-checkpoint',version=1,
            source_sha256=plan['source_sha256'],reference_path_sha256=plan['reference_path_sha256'],
            initial_checkpoint_sha256=plan['initial_checkpoint_sha256'],checkpoint_sha256=sha(path),
            curriculum=env.schedule.state_dict()))
        return result
    learner.save=paired


def resume(learner,env,plan,checkpoint):
    import torch
    side=json.loads(Path(str(checkpoint)+'.curriculum.json').read_text())
    if (side.get('schema')!='hug-home-ppo-checkpoint' or side.get('version')!=1
        or any(side[k]!=plan[k] for k in ('source_sha256','reference_path_sha256','initial_checkpoint_sha256'))
        or side['checkpoint_sha256']!=sha(checkpoint)):raise ValueError('Home checkpoint pairing mismatch')
    state=torch.load(checkpoint,map_location=learner.device,weights_only=True)
    env.schedule.load_state_dict(side['curriculum'])
    learner.alg.load(state,None,strict=True);learner.current_learning_iteration=int(state['iter'])+1
    env.reset()


def train(args):
    from workbench.research.experiments.hug_c.experiment import runner
    from workbench.research.experiments.hug_c.policy import export_policy
    from workbench.research.experiments.wrap_grasp.train import set_seed
    plan,path=load_plan(args.plan);args.output.mkdir(parents=True,exist_ok=False)
    save(args.output/'run.json',dict(schema='hug-home-ppo-run',version=1,plan=plan,
        num_envs=args.num_envs,iterations=args.iterations,resumed_from=str(args.resume) if args.resume else None))
    started=time.monotonic()
    try:
        set_seed(plan['curriculum']['seed']);env=create_env(args,plan,path,args.output)
        learner=runner(env,args.output);bind_checkpoint(learner,env,plan)
        if args.resume:resume(learner,env,plan,args.resume)
        else:warm_start(learner,plan,args.plan,args.output,env.get_observations()['policy'].cpu().numpy())
        learner.save(str(args.output/'model_initial.pt'))
        learner.learn(num_learning_iterations=args.iterations,init_at_random_ep_len=False)
        learner.save(str(args.output/'model_final.pt'))
        export_policy(learner.alg.get_policy(),args.output/'policy.npz',dict(env.metadata(),checkpoint_sha256=sha(args.output/'model_final.pt')))
        result=dict(status='training_budget_finished',elapsed_s=time.monotonic()-started,
            curriculum=env.schedule.state_dict(),nominal_home_policy_verified=False)
    except BaseException as exc:
        save(args.output/'completion.json',dict(status='execution_error',error=repr(exc),elapsed_s=time.monotonic()-started));raise
    save(args.output/'completion.json',result);print(json.dumps(result),flush=True)
    return 0


def evaluate(args):
    import torch
    from workbench.research.experiments.hug_c.policy import NumpyPolicy
    from workbench.research.experiments.hug_interpolation.video import record_video
    plan,path=load_plan(args.plan)
    policy=NumpyPolicy(args.plan/'initial-policy.npz' if args.initial else args.policy)
    if args.initial:validate_initial(policy,plan['base_plan'],plan['initial_checkpoint_sha256'])
    elif (policy.metadata.get('simulation_context')!=plan['simulation_context']
        or policy.metadata.get('forward_reference_sha256')!=plan['reference_path_sha256']):raise ValueError('Home policy context mismatch')
    args.output.mkdir(parents=True,exist_ok=False);env=create_env(args,plan,path,args.output)
    env.auto_reset=False;env.set_mode(args.start,args.bin);summaries=[]
    with torch.inference_mode():
        for repetition in range(args.repetitions):
            obs,_=env.reset();active=torch.ones(args.num_envs,dtype=torch.bool,device=env.device)
            passed=torch.zeros_like(active);wrong=torch.zeros_like(active);fault=torch.zeros_like(active)
            returns=torch.zeros(args.num_envs,device=env.device);spent=torch.zeros_like(returns)
            env.core.record=[];env.core.record_env_indices=torch.arange(args.num_envs,device=env.device)
            observations=[];actions=[];masks=[];costs=[]
            video=args.output/f'replay-{repetition+1}.mp4' if args.video else None
            with record_video(env.core,video):
                for _ in range(env.max_episode_length):
                    action=torch.as_tensor(policy(obs['policy'].cpu().numpy()),device=env.device,dtype=torch.float32);action[~active]=0
                    observations.append(obs['policy'].cpu().numpy().copy());actions.append(action.cpu().numpy().copy());masks.append(active.cpu().numpy().copy())
                    before=env.smooth.spent.clone();obs,reward,done,_=env.step(action)
                    costs.append((env.smooth.spent-before).cpu().numpy().copy());returns+=reward*active
                    spent[:]=env.smooth.spent
                    wrong|=active&(env.macro_wrong>0);fault|=active&env.fault
                    passed|=active&env.success&~fault&~wrong;active&=~done
                    if not bool(active.any()):break
            row=dict(repetition=repetition+1,start=args.start,bin=args.bin,initial_policy=args.initial,
                successes=int(passed.sum()),total=args.num_envs,fault_episodes=int(fault.sum()),
                wrong_face_episodes=int(wrong.sum()),smoothness_spent=spent.cpu().tolist(),returns=returns.cpu().tolist())
            summaries.append(row)
            np.savez_compressed(args.output/f'rollout-{repetition+1}.npz',observations=np.asarray(observations),
                actions=np.asarray(actions),valid=np.asarray(masks),smoothness_cost=np.asarray(costs),commands=np.asarray(env.core.record),
                metadata_json=np.asarray(json.dumps(dict(row,simulation_context=plan['simulation_context']))))
            env.core.record=None;env.core.record_env_indices=None
    accepted=all(r['successes']==r['total'] for r in summaries)
    result=dict(start=args.start,results=summaries,accepted=accepted,nominal_home=args.start=='nominal-home',training_updates=0)
    save(args.output/'evaluation.json',result);print(json.dumps(result),flush=True)
    return 0 if accepted else 1


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    q=sub.add_parser('prepare');q.add_argument('--source-run',type=Path,required=True)
    q.add_argument('--verified-bridge',type=Path,required=True);q.add_argument('--output',type=Path,required=True)
    q.add_argument('--spacing-s',type=float,default=.05);q.add_argument('--seed',type=int,default=42)
    q.add_argument('--smooth-budget',type=float,default=1.)
    for name in ('check','train','evaluate'):
        q=sub.add_parser(name);q.add_argument('--plan',type=Path,required=True);q.add_argument('--output',type=Path,required=True)
        q.add_argument('--backend',choices=('cpu','gpu'),default='gpu');q.add_argument('--num-envs',type=int,default=512 if name=='train' else 1 if name=='evaluate' else 2)
        if name=='train':q.add_argument('--iterations',type=int,default=2000);q.add_argument('--resume',type=Path)
        if name=='evaluate':
            group=q.add_mutually_exclusive_group(required=True);group.add_argument('--policy',type=Path);group.add_argument('--initial',action='store_true')
            q.add_argument('--start',choices=('nominal-home','fixed'),default='nominal-home');q.add_argument('--bin',type=int,default=0)
            q.add_argument('--repetitions',type=int,default=3);q.add_argument('--video',action='store_true')
    args=p.parse_args(argv)
    for k in ('num_envs','iterations','repetitions'):
        if hasattr(args,k) and getattr(args,k)<1:p.error(k+' must be positive')
    if args.command=='prepare':
        prepare(args.source_run,args.verified_bridge,args.output,spacing_s=args.spacing_s,seed=args.seed,smooth_budget=args.smooth_budget)
        print('Prepared only:',args.output);return 0
    if args.command=='check':
        plan,path=load_plan(args.plan);args.output.mkdir(parents=True,exist_ok=False)
        env=create_env(args,plan,path,args.output);save(args.output/'check.json',dict(env.bank.report,training_started=False));return 0
    return train(args) if args.command=='train' else evaluate(args)


if __name__=='__main__':raise SystemExit(main())
