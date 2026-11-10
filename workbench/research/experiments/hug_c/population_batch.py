"""Separate PPO learners sharing one synchronously stepped physics batch."""
import copy
import hashlib
import json
from pathlib import Path
import shutil
import time


def partitions(num_envs, members):
    if members < 1 or num_envs < members or num_envs % members:
        raise ValueError('Environment batch must divide evenly between policies')
    width=num_envs//members
    return tuple(slice(i*width,(i+1)*width) for i in range(members))


class PolicyEnvironments:
    """Runner construction view; physics can only be advanced by the joint loop."""
    def __init__(self,env,indices):
        self.env,self.indices=env,indices
        self.num_envs=indices.stop-indices.start
        self.device,self.num_actions=env.device,env.num_actions
        self.cfg=env.cfg
        self.max_episode_length=env.max_episode_length
        self.experiment=env.experiment

    def get_observations(self):
        return self.env.get_observations()[self.indices]

    @property
    def episode_length_buf(self):
        return self.env.episode_length_buf[self.indices]

    def step(self,actions):
        raise RuntimeError('Use the population joint rollout; a policy cannot step physics alone')


def learn_population(env,learners,updates,progress=None):
    """All policies act, one env.step, then each PPO records only its own lanes.

    Mirrors pinned rsl_rl 5.4's on-policy rollout/update order. Optimizer updates
    use each learner's private buffer; they are issued serially on the same GPU.
    current_learning_iteration is kept at the NEXT update index on return.
    """
    import torch
    groups=partitions(env.num_envs,len(learners))
    lengths={learner.cfg['num_steps_per_env'] for learner in learners}
    if len(lengths)!=1 or updates<1:
        raise ValueError('Population requires a common positive rollout length')
    rollout_steps=lengths.pop()
    if any(learner.env.num_envs!=group.stop-group.start for learner,group in zip(learners,groups)):
        raise ValueError('Learner allocation differs from the shared batch')
    for learner in learners:learner.alg.train_mode()
    started=time.monotonic()
    with torch.inference_mode():
        observations,_=env.reset()
    for update in range(updates):
        with torch.inference_mode():
            for _ in range(rollout_steps):
                actions=torch.cat([learner.alg.act(observations[group])
                                   for learner,group in zip(learners,groups)],dim=0)
                observations,rewards,dones,extras=env.step(actions)
                if not torch.isfinite(rewards).all():
                    raise RuntimeError('Nonfinite population rewards')
                for learner,group in zip(learners,groups):
                    local_extras={'time_outs':extras['time_outs'][group]} if 'time_outs' in extras else {}
                    learner.alg.process_env_step(observations[group],rewards[group],dones[group],local_extras)
            for learner,group in zip(learners,groups):
                learner.alg.compute_returns(observations[group])
        for learner in learners:
            loss=learner.alg.update()
            if any(not torch.isfinite(torch.as_tensor(value)).all() for value in loss.values()):
                raise RuntimeError('Nonfinite population PPO loss')
            learner.current_learning_iteration+=1
        if progress and ((update+1)%10==0 or update+1==updates):
            progress(update+1,time.monotonic()-started)
    return time.monotonic()-started


def restore_learner(learner,path):
    learner.load(str(path))
    learner.current_learning_iteration+=1
    # PPO's adaptive learning-rate scalar is not part of its checkpoint dict;
    # optimizer param groups do contain the actual last rate.
    rates={group['lr'] for group in learner.alg.optimizer.param_groups}
    if len(rates)!=1:
        raise ValueError('Unsupported mixed optimizer learning rates')
    learner.alg.learning_rate=rates.pop()


def transfer_state(source,destination,experiment,config):
    """Explicit compatible-task migration. Never rewrites the original run.

    Copy only last-complete-generation continuations, archive parents, and the
    historical champion. Preserve optimizer state, generations and search history.
    New source hashes are recorded as a transition, never disguised as old data.
    """
    source=Path(source).resolve();destination=Path(destination)
    old=json.loads((source/'population.json').read_text())
    if old.get('schema')!='c-hug-population' or old.get('version')!=3 or old.get('schedule')!='sequential-shared-physics':
        raise ValueError('Only an explicit v3 sequential population can be transferred')
    for key in ('case_id','num_envs','members','generation_updates','patience','reward_contract'):
        if old.get(key)!=config.get(key):
            raise ValueError(f'Transfer population differs: {key}')
    def task(value):
        value=copy.deepcopy(value)
        value['simulation_context'].pop('code_sha256',None)
        return value
    if task(old['experiment'])!=task(experiment):
        raise ValueError('Transfer changes the task, model, reward or target cases')
    state=json.loads((source/'population-state.json').read_text())
    if len(state['checkpoints'])!=config['members'] or state['updates']<1:
        raise ValueError('Require a complete population generation')
    manifest={}
    def copy_checkpoint(relative):
        path=(source/relative).resolve()
        if not path.is_relative_to(source) or path.suffix!='.pt' or not path.is_file():
            raise ValueError('Transfer checkpoint escapes the source run or is missing')
        result=Path('transferred')/path.relative_to(source)
        output=destination/result
        if str(result) not in manifest:
            output.parent.mkdir(parents=True,exist_ok=True)
            if output.exists():raise FileExistsError(output)
            shutil.copyfile(path,output)
            manifest[str(result)]=hashlib.sha256(output.read_bytes()).hexdigest()
        return str(result)
    previous=[str(Path(p).with_name('evaluated.pt')) for p in state['checkpoints']]
    state['checkpoints']=[copy_checkpoint(p) for p in state['checkpoints']]
    state['previous_evaluated']=[copy_checkpoint(p) for p in previous]
    for entry in state['archive'].values():entry['checkpoint']=copy_checkpoint(entry['checkpoint'])
    state['champion_checkpoint']=copy_checkpoint(state['champion_checkpoint'])
    shutil.copyfile(destination/state['champion_checkpoint'],destination/'model_final.pt')
    state['transfer']=dict(source_run=str(source),source_code_sha256=old['experiment']['simulation_context'].get('code_sha256'),
                           destination_code_sha256=experiment['simulation_context'].get('code_sha256'),
                           inherited_updates=state['updates'],inherited_generations=state['generation'],
                           checkpoint_sha256=manifest,physics_schedule_changed=True)
    (destination/'transfer.json').write_text(json.dumps(state['transfer'],indent=2))
    return state
