"""CPU integration of the real PPO budget adapter; no Genesis scene required."""
import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np


@unittest.skipUnless(importlib.util.find_spec('torch') and importlib.util.find_spec('tensordict'),
                     'Torch/tensordict unavailable')
class LinearAdapterTests(unittest.TestCase):
    def env(self):
        import torch
        from workbench.research.experiments.hug_pipeline.task import LinearHugEnv
        from workbench.research.experiments.hug_pipeline.linear import LinearBudget
        env=object.__new__(LinearHugEnv)
        env.device='cpu';env.num_envs=4;env.linear_enforced=True
        env.linear_budget=LinearBudget(4,torch,'cpu')
        env.mapper=SimpleNamespace(waypoint=torch.zeros((4,4)))
        env.mapper.waypoint[:,0]=-.12
        env.fault=torch.zeros(4,dtype=torch.bool)
        env.reset_linear_history()
        return env

    def test_numpy_torch_tracking_agree_with_subset_reset(self):
        import torch
        from workbench.research.experiments.hug_pipeline.linear import LinearBudget
        a=LinearBudget(4);b=LinearBudget(4,torch,'cpu')
        a.reset(np.arange(4),np.full(4,-.12));b.reset(torch.arange(4),torch.full((4,),-.12,dtype=torch.float64))
        rng=np.random.default_rng(45)
        for i in range(20):
            q=rng.uniform(-.12,0,4)
            a.advance(q);b.advance(torch.tensor(q))
            if i==10:
                a.reset([1,3],q[[1,3]]);b.reset(torch.tensor([1,3]),torch.tensor(q[[1,3]]))
        np.testing.assert_allclose(a.spent,b.spent.numpy(),atol=1e-8)
        np.testing.assert_array_equal(a.exceeded,b.exceeded.numpy())

    def test_fault_is_latched_before_physics_can_award_hold_or_success(self):
        import torch
        from workbench.research.experiments.hug_c.environment import HugEnv
        env=self.env();env.mapper.waypoint[:,0]=0;env.reset_linear_history()
        observed=[]
        def tick(core,command):observed.append(core.fault.clone());core.mapper.waypoint=command.clone()
        with patch.object(HugEnv,'tick',tick):
            q=env.mapper.waypoint.clone();q[1,0]=-21/1500
            env.tick(q)
            q[1,0]=0;env.tick(q)
        self.assertEqual(observed[0].tolist(),[False,True,False,False])
        self.assertEqual(observed[1].tolist(),[False,True,False,False])

    def test_bank_restore_debits_restored_command_without_changing_velocity(self):
        import torch
        from workbench.research.experiments.hug_pipeline.environment import LinearResetBank
        from workbench.research.experiments.hug_reverse_rl.bank import ResetBank
        env=self.env();bank=object.__new__(LinearResetBank);bank.env=env
        env.velocity=torch.full((4,4),.75)
        def restore(bank,bins,ids):
            bank.env.mapper.waypoint[ids,0]=torch.tensor([0.,-.06])
        with patch.object(ResetBank,'restore',restore):bank.restore([0,1],torch.tensor([1,3]))
        np.testing.assert_allclose(env.linear_budget.spent.numpy(),[0,180,0,90],atol=1e-4)
        self.assertTrue(torch.all(env.velocity==.75))

    def test_observation_contains_post_reset_remaining_budget(self):
        import torch
        from tensordict import TensorDict
        from workbench.research.experiments.hug_c.environment import HugEnv
        env=self.env();env.mapper.waypoint[:,0]=torch.tensor([-.12,0.,-.06,-190/1500])
        env.reset_linear_history()
        def observations(core):return TensorDict({'policy':torch.zeros((4,20))},batch_size=[4])
        with patch.object(HugEnv,'get_observations',observations):obs=env.get_observations()['policy']
        self.assertEqual(tuple(obs.shape),(4,21))
        np.testing.assert_allclose(obs[:,-1].numpy(),[1,.1,.55,.95],atol=1e-6)

    def test_teacher_replay_disables_budget_only_during_bank_construction(self):
        import torch
        from workbench.research.experiments.hug_pipeline.environment import LinearResetBank
        from workbench.research.experiments.hug_reverse_rl.bank import ResetBank
        env=self.env();states=[]
        def build(bank,core,plan,path,directory):
            states.append(core.linear_enforced)
            bank.env=core;bank.directory=Path(directory);bank.report={}
            core.mapper.waypoint[:,0]=0
        with TemporaryDirectory() as d,patch.object(ResetBank,'__init__',build):
            LinearResetBank(env,{'pool_ticks':[0,1]}, {'command':np.array([[0.,0,0,0],[-.12,0,0,0]])},d)
        self.assertEqual(states,[False]);self.assertTrue(env.linear_enforced)
        self.assertTrue(torch.allclose(env.linear_budget.spent,torch.full((4,),180.,dtype=torch.float64)))


if __name__=='__main__':unittest.main()
