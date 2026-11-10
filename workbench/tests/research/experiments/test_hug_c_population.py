"""Selection regressions and optional pinned Torch mutation checks (no physics)."""
import copy
import numpy as np
import pytest
from workbench.research.experiments.hug_c.population import (
    quality, eligible, cell, select_replacement, mutate, set_exploration)


def report(**kwargs):
    return dict(dict(success=0., hold=0., lift=0., grasp=0., contact=0., safe=1.,
                     descriptor=[0.]*8), **kwargs)


def test_physical_progress_beats_safe_idle_without_reward_tuning():
    idle = report()
    grasp = report(contact=1., grasp=1.)
    assert quality(grasp) > quality(idle)
    assert not eligible(idle)
    assert eligible(grasp)


def test_protects_champion_and_distinct_contact_strategy():
    reports = [report(grasp=1.,contact=1.), report(contact=1.,descriptor=[.5]*8),
               report(), report(safe=.5)]
    assert select_replacement(reports, [9]*4, 3) == (0, 3)
    assert cell(reports[0]) != cell(reports[1])


def test_no_replacement_before_patience_and_only_one_victim():
    reports = [report() for _ in range(4)]
    assert select_replacement(reports, [0,1,2,2], 3) == (0,None)
    assert select_replacement(reports, [9]*4, 3) == (0,1)


def test_faulty_contact_is_not_an_archived_strategy():
    assert not eligible(report(safe=0.,contact=1.))


def test_mutation_is_bounded_and_restore_on_rejection():
    torch = pytest.importorskip('torch')
    class Actor(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.mlp = torch.nn.Sequential(torch.nn.Linear(20,16),torch.nn.ELU(),torch.nn.Linear(16,4))
        def forward(self, obs):
            return self.mlp(obs['policy'])
    torch.manual_seed(12)
    actor=Actor(); bank={'policy':torch.randn(32,20)}
    before=actor(bank).detach().clone()
    result=mutate(actor,bank,8)
    assert result['accepted']
    assert 0 < float((actor(bank)-before).detach().abs().max()) <= .12
    original=copy.deepcopy(actor.state_dict())
    rejected=mutate(actor,bank,8,maximum=0.)
    assert not rejected['accepted']
    for k,v in actor.state_dict().items():
        assert torch.equal(v,original[k])


def test_gaussian_exploration_floor_survives_low_parameter():
    torch=pytest.importorskip('torch')
    pytest.importorskip('rsl_rl')
    from rsl_rl.modules import GaussianDistribution
    from types import SimpleNamespace
    for kind in ('scalar','log'):
        dist=GaussianDistribution(4,std_type=kind,init_std=.001)
        set_exploration(SimpleNamespace(distribution=dist),.3)
        dist.update(torch.zeros(2,4))
        assert torch.allclose(dist.std,torch.full((4,),.3),atol=1e-6)


def test_novel_approach_can_survive_lower_fitness_than_parent():
    from workbench.research.experiments.hug_c.population import admit_child, archive_key
    parent=report(grasp=1.,contact=1.,descriptor=[0.]*4)
    child=report(contact=1.,descriptor=[.7]*4)
    assert quality(child)<quality(parent)
    assert admit_child(child,parent,{})
    occupied={archive_key(child):{'quality':quality(parent)}}
    assert not admit_child(child,parent,occupied)
    assert not admit_child(report(contact=1.,safe=.5),parent,{})


def test_precontact_stepping_stone_and_bounded_niches():
    assert eligible(report(approach=.1,descriptor=[.5]*4))
    assert not eligible(report(approach=.001))
    assert cell(report(descriptor=[-.01,0.,1.,1.01])) == (0,0,4,4)


def test_pinned_ppo_population_resume_and_branching():
    """Real PPO/NPZ APIs on a synthetic environment; no robot or Genesis."""
    torch=pytest.importorskip('torch')
    pytest.importorskip('rsl_rl')
    from tensordict import TensorDict
    from tempfile import TemporaryDirectory
    from pathlib import Path
    from types import SimpleNamespace
    from unittest.mock import patch
    import json
    from workbench.research.experiments.hug_c import population, experiment
    from workbench.research.experiments.hug_c.configure import DEFAULTS

    class Env:
        device='cpu'
        num_actions=4
        max_episode_length=2
        def __init__(self,n,exp):
            self.num_envs=n;self.experiment=exp;self.s=exp['settings'];self.cfg={}
            self.start=torch.zeros(4);self.auto_reset=True;self.record=None;self.record_env_indices=None
            self.mapper=SimpleNamespace(waypoint=torch.zeros(n,4))
            self.episode_length_buf=torch.zeros(n,dtype=torch.long)
            self.fault=torch.zeros(n,dtype=torch.bool);self.macro_wrong=torch.zeros(n)
            self.last_touch=torch.zeros(n,2)
            self.ledger=SimpleNamespace(best_lift=torch.zeros(n),best_hold=torch.zeros(n))
        def reset(self):
            self.episode_length_buf.zero_()
            return self.get_observations(),{}
        def get_observations(self):
            return TensorDict({'policy':torch.zeros(self.num_envs,20),
                               'privileged':torch.zeros(self.num_envs,8)},batch_size=[self.num_envs])
        def actual(self):return torch.zeros(self.num_envs,4)
        def relative_object(self):return torch.ones(self.num_envs,3)
        def metadata(self):return {'synthetic_test_only':True,'duration_s':.02}
        def outcome(self):
            return {'grasp_confirmed':self.fault,'success':self.fault}
        def step(self,actions):
            self.episode_length_buf+=1
            done=self.episode_length_buf>=2
            if self.record is not None:
                shape=(len(self.record_env_indices),4) if self.record_env_indices is not None else (4,)
                self.record.append(np.zeros(shape))
            if self.auto_reset:self.episode_length_buf[done]=0
            return self.get_observations(),-(actions**2).sum(-1),done,{'time_outs':torch.zeros_like(done)}

    real_runner=experiment.runner
    def small_runner(env,directory):
        r=real_runner(env,directory)
        r.alg.num_learning_epochs=1
        return r
    with TemporaryDirectory() as temporary:
        exp={'settings':dict(DEFAULTS),'simulation_context':{'reward_contract':'synthetic'}};case={'case_id':'test'}
        args=SimpleNamespace(population_size=3,num_envs=12,iterations=4,generation_updates=1,
                             stagnation_generations=1,runs=Path(temporary),resume=False)
        with patch.object(experiment,'create_env',lambda exp,case,directory,args,n,reset:Env(n,exp)), \
             patch.object(experiment,'runner',small_runner):
            assert population.run(args,exp,case)==0
            root=Path(temporary)/'test'
            state=json.loads((root/'population-state.json').read_text())
            assert state['updates']==4
            assert state['training_env_steps']==4*12*16
            assert json.loads((root/'population.json').read_text())['schedule']=='simultaneous-shared-physics'
            assert any(e.get('accepted') for e in state['events'])
            assert (root/'model_final.pt').is_file()
            before=(root/'generation-0000/member-00/evaluated.pt').read_bytes()
            args.resume=True;args.iterations=1
            assert population.run(args,exp,case)==0
            state=json.loads((root/'population-state.json').read_text())
            assert state['updates']==5
            assert before==(root/'generation-0000/member-00/evaluated.pt').read_bytes()
            # Replay a real optimizer/checkpoint transfer through the compatible
            # sequential schema; original checkpoints remain byte-identical.
            old=json.loads((root/'population.json').read_text())
            old.update(version=3,schedule='sequential-shared-physics')
            (root/'population.json').write_text(json.dumps(old))
            args.resume=False;args.population_from=root;args.runs=Path(temporary)/'migrated'
            args.iterations=2
            assert population.run(args,exp,case)==0
            transferred=args.runs/'test'
            migrated=json.loads((transferred/'population-state.json').read_text())
            assert migrated['updates']==7 and migrated['transfer']['inherited_updates']==5
            assert any(e.get('mode')=='rewind' for e in migrated['events'] if e['generation']>=5)
            assert before==(root/'generation-0000/member-00/evaluated.pt').read_bytes()
            assert (transferred/migrated['champion_checkpoint']).with_name('policy.npz').is_file()


def test_train_cli_defaults_to_population_and_preserves_single_baseline(monkeypatch):
    from workbench.research.experiments.hug_c import experiment
    captured=[]
    monkeypatch.setattr(experiment,'train',lambda args:captured.append(args) or 0)
    argv=['train','--experiment','exp.json','--preflight','check.json','--case','anchor','--runs','runs']
    assert experiment.main(argv)==0
    assert captured[-1].strategy=='population'
    assert captured[-1].population_size==4 and captured[-1].num_envs==512
    assert experiment.main(argv+['--strategy','single'])==0
    assert captured[-1].strategy=='single'


def test_variation_modes_do_not_lock_to_three_generation_stagnation():
    from workbench.research.experiments.hug_c.population import next_variation
    state={};events=[]
    for generation in range(13):
        if generation in (3,6,9,12):
            events.append(next_variation(state))
    assert events==['exploration','weights','rewind','exploration']
    restored=copy.deepcopy(state)
    assert next_variation(restored)=='weights'
