"""Real CPU Torch/RSL adapter and safe checkpoint transfer tests."""
import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
import numpy as np


@unittest.skipUnless(importlib.util.find_spec('torch') and importlib.util.find_spec('tensordict'),'Torch/tensordict unavailable')
class HomeTests(unittest.TestCase):
    def fake_env(self):
        import torch
        from workbench.tests.research.experiments.test_hug_reverse_rl_torch import AdapterTests
        from workbench.research.experiments.hug_home_rl.environment import HomeEnv
        from workbench.research.experiments.hug_home_rl.schedule import WindowCurriculum
        from workbench.research.experiments.hug_home_rl.smoothness import Smoothness
        old=AdapterTests().environment();env=object.__new__(HomeEnv);env.__dict__.update(old.__dict__)
        env.plan={'pool_ticks':[110,50,0]};env.schedule=WindowCurriculum(3,dict(window=4,cooldown_steps=1))
        env.smooth=Smoothness(4,np.ones(4),.1,xp=torch,device='cpu')
        env.lift_seen=torch.zeros(4,dtype=torch.bool);env.core.ledger.finished=torch.zeros(4,dtype=torch.bool)
        return env

    def test_torch_numpy_smoothness_match_with_partial_resets(self):
        import torch
        from workbench.research.experiments.hug_home_rl.smoothness import Smoothness
        a=Smoothness(4,np.ones(4),.1);b=Smoothness(4,np.ones(4),.1,xp=torch,device='cpu')
        a.reset(np.arange(4),np.zeros((4,4)));b.reset(torch.arange(4),torch.zeros((4,4)))
        rng=np.random.default_rng(42)
        for i in range(100):
            q=rng.normal(size=(4,4)).astype(np.float32);seen=np.ones(4,bool);active=np.array([True,True,False,True])
            expected=a.step(q,seen,active);actual=b.step(torch.tensor(q),torch.tensor(seen),torch.tensor(active))
            np.testing.assert_allclose(actual.numpy(),expected,atol=1e-7)
            if i==50:
                a.reset([1,3],q[[1,3]]);b.reset(torch.tensor([1,3]),torch.tensor(q[[1,3]]))
        np.testing.assert_allclose(a.spent,b.spent.numpy(),atol=1e-6)

    def test_adapter_reset_preserves_velocities_and_other_cost_budgets(self):
        import torch
        env=self.fake_env();env.smooth.spent[:]=.4;env.set_mode('fixed',2)
        env.reset(torch.tensor([1,3]))
        self.assertTrue(torch.all(env.core.velocity[[1,3]]==2.75))
        self.assertTrue(torch.all(env.smooth.spent[[1,3]]==0))
        self.assertTrue(torch.all(env.smooth.spent[[0,2]]==.4))

    def test_adapter_records_success_before_auto_reset(self):
        import torch
        env=self.fake_env();env.reset();env.step(torch.zeros((4,4)))
        self.assertEqual(env.schedule.frontier,1)
        self.assertFalse(bool(env.core.success.any()))

    def test_safe_warm_start_preserves_actor_and_normalization_not_optimizer_or_iteration(self):
        if importlib.util.find_spec('rsl_rl') is None:self.skipTest('rsl_rl unavailable')
        import torch
        from workbench.research.experiments.hug_c.experiment import runner
        from workbench.research.experiments.hug_c.policy import export_policy
        from workbench.research.experiments.hug_home_rl.experiment import warm_start,bind_checkpoint,resume
        from workbench.research.experiments.hug_home_rl.data import sha
        from workbench.research.experiments.hug_home_rl.schedule import WindowCurriculum
        env=self.fake_env();env.reset();source=runner(env,None)
        # Populate a real Adam state, so an accidental optimizer restore is detectable.
        source.alg.optimizer.zero_grad()
        loss=sum((p*p).sum() for p in source.alg.get_policy().parameters());loss.backward();source.alg.optimizer.step()
        source.current_learning_iteration=123
        with TemporaryDirectory() as temp:
            folder=Path(temp);checkpoint=folder/'initial-model.pt';source.save(str(checkpoint))
            export_policy(source.alg.get_policy(),folder/'initial-policy.npz',{})
            destination=runner(env,None)
            self.assertEqual(len(destination.alg.optimizer.state),0)
            plan=dict(initial_checkpoint_sha256=sha(checkpoint),simulation_context={'new-task':True},
                      source_sha256='new-source',reference_path_sha256='forward-reference')
            record=warm_start(destination,plan,folder,folder,env.get_observations()['policy'].numpy())
            self.assertLessEqual(record['max_actor_error'],1e-4)
            self.assertEqual(destination.current_learning_iteration,0)
            self.assertEqual(len(destination.alg.optimizer.state),0)
            for k,v in source.alg.get_policy().state_dict().items():
                self.assertTrue(torch.equal(v,destination.alg.get_policy().state_dict()[k]),k)
            env.schedule.frontier=1;destination.current_learning_iteration=7
            bind_checkpoint(destination,env,plan);paired=folder/'new.pt';destination.save(str(paired))
            env.schedule=WindowCurriculum(3,dict(window=4,cooldown_steps=1))
            resume(destination,env,plan,paired)
            self.assertEqual(env.schedule.frontier,1);self.assertEqual(destination.current_learning_iteration,8)
            paired.write_bytes(b'changed')
            with self.assertRaises(ValueError):resume(destination,env,plan,paired)


if __name__=='__main__':unittest.main()
