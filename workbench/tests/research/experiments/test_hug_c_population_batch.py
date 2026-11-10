"""One physical step per population step; no policy experience mixing."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from workbench.research.experiments.hug_c.population_batch import partitions,transfer_state


def test_partition_is_complete_disjoint_and_defaults_to_four_128_lane_groups():
    groups=partitions(512,4)
    assert [(s.start,s.stop) for s in groups]==[(0,128),(128,256),(256,384),(384,512)]
    assert [i for s in groups for i in range(s.start,s.stop)]==list(range(512))
    with pytest.raises(ValueError):partitions(513,4)


def test_transfer_preserves_search_and_checkpoint_bytes_and_rejects_task_changes(tmp_path):
    source=tmp_path/'old';source.mkdir();destination=tmp_path/'new';destination.mkdir()
    experiment={'settings':{'time':.3},'simulation_context':{'code_sha256':'old','model_sha256':'same'}}
    old=dict(schema='c-hug-population',version=3,schedule='sequential-shared-physics',experiment=experiment,
             case_id='case',num_envs=512,members=4,generation_updates=100,patience=3,reward_contract='v4')
    (source/'population.json').write_text(json.dumps(old))
    for i in range(4):
        p=source/f'generation-0003/member-{i:02d}/continuation.pt';p.parent.mkdir(parents=True);p.write_bytes(bytes([i])*8)
        p.with_name('evaluated.pt').write_bytes(bytes([i])*8)
    state=dict(updates=400,generation=4,variation_attempts=1,stale=[0,1,2,0],best=[[1]]*4,
               checkpoints=[f'generation-0003/member-{i:02d}/continuation.pt' for i in range(4)],
               archive={'cell':{'checkpoint':'generation-0003/member-00/continuation.pt'}},
               champion_checkpoint='generation-0003/member-00/continuation.pt')
    (source/'population-state.json').write_text(json.dumps(state))
    new_exp=copy.deepcopy(experiment);new_exp['simulation_context']['code_sha256']='new'
    config=dict(old,version=4,schedule='simultaneous-shared-physics')
    result=transfer_state(source,destination,new_exp,config)
    assert result['updates']==400 and result['variation_attempts']==1 and result['stale']==state['stale']
    assert len(result['previous_evaluated'])==4
    for i,p in enumerate(result['checkpoints']):assert (destination/p).read_bytes()==bytes([i])*8
    assert (destination/'model_final.pt').read_bytes()==bytes(8)
    bad=copy.deepcopy(new_exp);bad['settings']['time']=.9
    with pytest.raises(ValueError,match='task'):transfer_state(source,tmp_path/'bad',bad,config)
    assert json.loads((source/'population-state.json').read_text())==state


def test_joint_rollout_routes_actions_rewards_dones_and_timeouts_to_their_policy():
    torch=pytest.importorskip('torch')
    from tensordict import TensorDict
    from workbench.research.experiments.hug_c.population_batch import PolicyEnvironments,learn_population
    class Env:
        num_envs=8;num_actions=4;device='cpu';max_episode_length=20;cfg={};experiment={}
        episode_length_buf=torch.zeros(8,dtype=torch.long)
        steps=0
        def get_observations(self):
            return TensorDict({'policy':torch.arange(8).float()[:,None]},batch_size=[8])
        def reset(self):return self.get_observations(),{}
        def step(self,actions):
            self.steps+=1
            assert torch.equal(actions[:,0],torch.arange(4).float().repeat_interleave(2))
            values=torch.arange(8).float()
            return self.get_observations(),values,values%2==1,{'time_outs':values%3==0}
    class Algorithm:
        def __init__(self,i):self.i=i;self.seen=0;self.updated=0
        def train_mode(self):pass
        def act(self,obs):
            assert torch.equal(obs['policy'][:,0],torch.arange(self.i*2,self.i*2+2).float())
            return torch.full((2,4),float(self.i))
        def process_env_step(self,obs,reward,done,extras):
            expected=torch.arange(self.i*2,self.i*2+2).float()
            assert torch.equal(reward,expected)
            assert torch.equal(done,expected%2==1)
            assert torch.equal(extras['time_outs'],expected%3==0)
            self.seen+=1
        def compute_returns(self,obs):pass
        def update(self):self.updated+=1;return {'loss':0.}
    env=Env();views=[PolicyEnvironments(env,g) for g in partitions(8,4)]
    learners=[SimpleNamespace(env=view,cfg={'num_steps_per_env':2},alg=Algorithm(i),current_learning_iteration=0) for i,view in enumerate(views)]
    learn_population(env,learners,3)
    assert env.steps==6  # NOT 24 separate physical steps.
    assert all(l.alg.seen==6 and l.alg.updated==3 and l.current_learning_iteration==3 for l in learners)
    with pytest.raises(RuntimeError,match='joint rollout'):views[0].step(torch.zeros(2,4))


def test_joint_evaluation_keeps_each_path_length_and_ignores_post_done_contacts(tmp_path):
    torch=pytest.importorskip('torch')
    import numpy as np
    from tensordict import TensorDict
    from elesim_sim.rl.trajectory import Trajectory
    from workbench.research.experiments.hug_c.population import evaluate
    from workbench.research.experiments.hug_c.configure import DEFAULTS
    class Actor(torch.nn.Module):
        def forward(self,obs):return torch.zeros(len(obs),4)
    class Env:
        num_envs=4;device='cpu';auto_reset=True;max_episode_length=3;s=DEFAULTS
        start=torch.zeros(4)
        fault=torch.zeros(4,dtype=torch.bool);macro_wrong=torch.zeros(4)
        last_touch=torch.zeros(4,2)
        mapper=SimpleNamespace(waypoint=torch.zeros(4,4))
        ledger=SimpleNamespace(best_lift=torch.zeros(4),best_hold=torch.zeros(4))
        steps=0
        def get_observations(self):return TensorDict({'policy':torch.zeros(4,20)},batch_size=[4])
        def reset(self):return self.get_observations(),{}
        def relative_object(self):return torch.ones(4,3)
        def actual(self):return self.mapper.waypoint
        def metadata(self):return {'case_id':'synthetic'}
        def outcome(self):return {'grasp_confirmed':self.fault,'success':self.fault}
        def step(self,actions):
            self.steps+=1
            if self.steps>1:self.last_touch[0]=1  # Must not count after lane0 done.
            self.record.append(np.zeros((len(self.record_env_indices),4)))
            done=torch.full((4,),self.steps==3);done[0]=True
            return self.get_observations(),torch.zeros(4),done,{}
    env=Env();directories=[tmp_path/str(i) for i in range(4)]
    for directory in directories:directory.mkdir()
    reports,_=evaluate(env,[Actor() for _ in range(4)],directories)
    assert env.steps==3 and env.auto_reset
    assert reports[0]['contact']==0
    assert [len(Trajectory.load(d/'evaluation.npz').times) for d in directories]==[2,4,4,4]
