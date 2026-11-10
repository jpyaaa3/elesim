"""Explicit prepare/check/train/evaluate for PPO reverse curriculum."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from .data import prepare,load_plan,sha
from workbench.research.experiments.hug_reverse_ga.prepare import save


def create_env(args,plan,path,output):
    from workbench.research.experiments.hug_c.experiment import check_versions
    from .environment import CurriculumEnv
    check_versions()
    return CurriculumEnv(plan,path,output,num_envs=args.num_envs,backend=args.backend,video=getattr(args,'video',False))


def bind_checkpoint(learner,env,plan):
    original=learner.save
    def save_checkpoint(path,*a,**kw):
        result=original(path,*a,**kw)
        save(Path(str(path)+'.curriculum.json'),dict(schema='hug-reverse-ppo-checkpoint',version=1,
            reference_path_sha256=plan['reference_path_sha256'],source_sha256=plan['source_sha256'],
            checkpoint_sha256=sha(path),curriculum=env.schedule.state_dict()))
        return result
    learner.save=save_checkpoint


def resume(learner,env,plan,checkpoint):
    record=json.loads(Path(str(checkpoint)+'.curriculum.json').read_text())
    if (record.get('schema')!='hug-reverse-ppo-checkpoint' or record.get('version')!=1
        or record['source_sha256']!=plan['source_sha256'] or record['reference_path_sha256']!=plan['reference_path_sha256']
        or record['checkpoint_sha256']!=sha(checkpoint)):raise ValueError('Checkpoint/curriculum provenance mismatch')
    env.schedule.load_state_dict(record['curriculum']);learner.load(str(checkpoint))
    learner.current_learning_iteration+=1
    env.reset()


def train(args):
    from workbench.research.experiments.hug_c.experiment import runner
    from workbench.research.experiments.hug_c.policy import export_policy
    from workbench.research.experiments.wrap_grasp.train import set_seed
    plan,path=load_plan(args.plan);args.output.mkdir(parents=True,exist_ok=False)
    save(args.output/'run.json',dict(schema='hug-reverse-ppo-run',version=1,plan=plan,num_envs=args.num_envs,
        iterations=args.iterations,resumed_from=str(args.resume) if args.resume else None))
    started=time.monotonic()
    try:
        set_seed(plan['curriculum']['seed']);env=create_env(args,plan,path,args.output)
        learner=runner(env,args.output);bind_checkpoint(learner,env,plan)
        if args.resume:resume(learner,env,plan,args.resume)
        learner.save(str(args.output/'model_initial.pt'))
        learner.learn(num_learning_iterations=args.iterations,init_at_random_ep_len=False)
        learner.save(str(args.output/'model_final.pt'))
        export_policy(learner.alg.get_policy(),args.output/'policy.npz',dict(env.metadata(),checkpoint_sha256=sha(args.output/'model_final.pt')))
        result=dict(status='training_budget_finished',curriculum=env.schedule.state_dict(),
            nominal_home_policy_verified=False,elapsed_s=time.monotonic()-started)
    except BaseException as exc:
        save(args.output/'completion.json',dict(status='execution_error',error=repr(exc),elapsed_s=time.monotonic()-started));raise
    save(args.output/'completion.json',result)
    print(json.dumps(result),flush=True)


def evaluate(args):
    import torch
    from workbench.research.experiments.hug_c.policy import NumpyPolicy
    from workbench.research.experiments.hug_interpolation.video import record_video
    plan,path=load_plan(args.plan);policy=NumpyPolicy(args.policy)
    if (policy.metadata['simulation_context']!=plan['simulation_context']
        or policy.metadata.get('reverse_reference_sha256')!=plan['reference_path_sha256']):
        raise ValueError('Policy/plan mismatch')
    args.output.mkdir(parents=True,exist_ok=False);env=create_env(args,plan,path,args.output)
    env.auto_reset=False;env.set_mode(args.start,args.bin)
    summaries=[]
    with torch.inference_mode():
        for repetition in range(args.repetitions):
            obs,_=env.reset();active=torch.ones(args.num_envs,dtype=torch.bool,device=env.device)
            passed=torch.zeros_like(active);wrong=torch.zeros_like(active);fault=torch.zeros_like(active)
            env.core.record=[];observations=[];actions=[];masks=[]
            video=args.output/f'replay-{repetition+1}.mp4' if args.video else None
            with record_video(env.core,video):
                for _ in range(env.max_episode_length):
                    action=torch.as_tensor(policy(obs['policy'].cpu().numpy()),device=env.device,dtype=torch.float32)
                    action[~active]=0;observations.append(obs['policy'].cpu().numpy().copy());actions.append(action.cpu().numpy().copy());masks.append(active.cpu().numpy().copy())
                    obs,_,done,_=env.step(action)
                    wrong|=active&(env.macro_wrong>0);fault|=active&env.fault
                    passed|=active&env.success&~fault&~wrong;active&=~done
                    if not bool(active.any()):break
            record=dict(repetition=repetition+1,start=args.start,bin=args.bin,successes=int(passed.sum()),
                total=args.num_envs,wrong_face_episodes=int(wrong.sum()),fault_episodes=int(fault.sum()))
            summaries.append(record)
            np.savez_compressed(args.output/f'rollout-{repetition+1}.npz',observations=np.asarray(observations),actions=np.asarray(actions),valid=np.asarray(masks),commands=np.asarray(env.core.record),
                metadata_json=np.asarray(json.dumps(dict(record,simulation_context=plan['simulation_context']))))
            env.core.record=None
    save(args.output/'evaluation.json',dict(start=args.start,results=summaries,
        accepted=all(r['successes']==r['total'] for r in summaries),nominal_home=args.start=='nominal-home'))
    return 0 if all(r['successes']==r['total'] for r in summaries) else 1


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    q=sub.add_parser('prepare');q.add_argument('--reverse-run',type=Path,required=True)
    q.add_argument('--spacing-s',type=float,default=.25);q.add_argument('--seed',type=int,default=42)
    q.add_argument('--output',type=Path,required=True)
    for name in ('check','train','evaluate'):
        q=sub.add_parser(name);q.add_argument('--plan',type=Path,required=True);q.add_argument('--output',type=Path,required=True)
        q.add_argument('--backend',choices=('cpu','gpu'),default='gpu');q.add_argument('--num-envs',type=int,default=512 if name=='train' else 1 if name=='evaluate' else 2)
        if name=='train':
            q.add_argument('--iterations',type=int,default=2000);q.add_argument('--resume',type=Path)
        if name=='evaluate':
            q.add_argument('--policy',type=Path,required=True);q.add_argument('--start',choices=('nominal-home','fixed'),default='nominal-home')
            q.add_argument('--bin',type=int,default=0);q.add_argument('--repetitions',type=int,default=3);q.add_argument('--video',action='store_true')
    args=p.parse_args(argv)
    for key in ('num_envs','iterations','repetitions'):
        if hasattr(args,key) and getattr(args,key)<1:p.error(key+' must be positive')
    if args.command=='prepare':
        prepare(args.reverse_run,args.output,spacing_s=args.spacing_s,seed=args.seed);print('Prepared only:',args.output);return 0
    if args.command=='check':
        plan,path=load_plan(args.plan);args.output.mkdir(parents=True,exist_ok=False)
        env=create_env(args,plan,path,args.output)
        # No learner, optimizer or PPO update is constructed in check.
        save(args.output/'check.json',dict(env.bank.report,training_started=False))
        return 0
    return train(args) if args.command=='train' else evaluate(args)


if __name__=='__main__':raise SystemExit(main())
