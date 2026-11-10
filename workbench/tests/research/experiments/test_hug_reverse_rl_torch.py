"""CPU Torch integration checks runnable with unittest; no physics or PPO updates."""
import importlib.util
from types import SimpleNamespace
import unittest
import numpy as np


@unittest.skipUnless(importlib.util.find_spec('torch') and importlib.util.find_spec('tensordict'),'Torch/tensordict unavailable')
class AdapterTests(unittest.TestCase):
    def environment(self):
        import torch
        from tensordict import TensorDict
        from workbench.research.experiments.hug_reverse_rl.environment import CurriculumEnv
        from workbench.research.experiments.hug_reverse_rl.schedule import Curriculum
        from workbench.research.experiments.hug_c.configure import DEFAULTS
        n=4
        class Core:
            num_envs=n;device='cpu';num_actions=4;cfg={};s=dict(DEFAULTS)
            experiment={'settings':s};max_episode_length=200
            def __init__(self):
                self.pose=torch.zeros((n,4));self.velocity=torch.zeros((n,4));self.object_pos=torch.zeros((n,3))
                self.goal=torch.zeros(4);self.dofs=list(range(4));self.mapper=SimpleNamespace(waypoint=torch.zeros((n,4)),joint_targets=lambda:self.pose)
                self.success=torch.zeros(n,dtype=torch.bool);self.fault=torch.zeros_like(self.success);self.macro_wrong=torch.zeros(n)
                self.last_touch=torch.zeros((n,2));self.last_floor=torch.zeros_like(self.success);self.last_wrong_face=torch.zeros_like(self.success)
                self.episode_length_buf=torch.zeros(n,dtype=torch.long)
                self.scene=SimpleNamespace(robot=SimpleNamespace(control_dofs_position=lambda *a,**k:None))
                self.ledger=SimpleNamespace(reset=lambda ids,actual,goal:setattr(self,'ledger_pose',actual.clone()))
            def actual(self):return self.pose
            def reset(self,ids):
                self.pose[ids]=0;self.velocity[ids]=0;self.object_pos[ids]=0
                self.success[ids]=False;self.fault[ids]=False;self.macro_wrong[ids]=0
            def get_observations(self):
                return TensorDict({'policy':torch.zeros((n,20)),'privileged':torch.zeros((n,8))},batch_size=[n])
            def step(self,actions):
                self.success[:]=True
                return self.get_observations(),torch.ones(n),torch.ones(n,dtype=torch.bool),{'log':{}}
        core=Core();env=object.__new__(CurriculumEnv);env.core=core
        env.auto_reset=True;env.mode='curriculum';env.fixed_bin=0
        env.plan={'pool_ticks':[0,25,50]};env.schedule=Curriculum(3,dict(window=4,cooldown_steps=1))
        env.bins=np.zeros(n,int);env.epochs=np.zeros(n,int)
        env.episode_wrong=torch.zeros(n,dtype=torch.bool)
        class Bank:
            entries={k:{'sample':dict(touch=np.ones((n,2)),floor=np.ones(n,bool),wrong=np.zeros(n,bool))} for k in (-1,0,1,2)}
            def restore(self,bins,ids):
                for b,i in zip(bins,ids):
                    core.pose[i]=float(b)+.25;core.velocity[i]=float(b)+.75;core.object_pos[i]=float(b)+.5
        env.bank=Bank();return env

    def test_reset_preserves_snapshot_velocity_object_and_measured_reward_baseline(self):
        import torch
        env=self.environment();env.set_mode('fixed',2);env.reset(torch.tensor([1,3]))
        self.assertTrue(torch.all(env.core.pose[[1,3]]==2.25))
        self.assertTrue(torch.all(env.core.velocity[[1,3]]==2.75))
        self.assertTrue(torch.all(env.core.object_pos[[1,3]]==2.5))
        self.assertTrue(torch.all(env.core.pose[[0,2]]==0))
        self.assertTrue(torch.all(env.core.ledger_pose==2.25))

    def test_success_is_counted_before_reset_and_evaluation_cannot_advance(self):
        import torch
        env=self.environment();env.reset();env.step(torch.zeros((4,4)))
        self.assertEqual(env.schedule.frontier,1)
        self.assertFalse(env.core.success.any())
        env.set_mode('nominal-home');env.step(torch.zeros((4,4)))
        self.assertEqual(env.schedule.frontier,1)
        self.assertTrue(np.all(env.bins==-1))

    def test_real_runner_constructs_actor_and_value_network_without_learning(self):
        if importlib.util.find_spec('rsl_rl') is None:self.skipTest('rsl_rl unavailable')
        from workbench.research.experiments.hug_c.experiment import runner
        env=self.environment();env.reset();learner=runner(env,None)
        actor=learner.alg.get_policy();before=[p.detach().clone() for p in actor.parameters()]
        action=actor(env.get_observations())
        self.assertEqual(tuple(action.shape),(4,4))
        for a,b in zip(before,actor.parameters()):self.assertTrue(a.equal(b))

    def test_historical_wrong_face_prevents_curriculum_promotion(self):
        import torch
        env=self.environment();env.reset();original=env.core.step
        def touch_wrong(actions):
            env.core.macro_wrong[:]=1
            return env.get_observations(),torch.zeros(4),torch.zeros(4,dtype=torch.bool),{'log':{}}
        env.core.step=touch_wrong;env.step(torch.zeros((4,4)))
        env.core.macro_wrong.zero_();env.core.step=original
        env.step(torch.zeros((4,4)));self.assertEqual(env.schedule.frontier,0)

    def test_real_checkpoint_pairs_curriculum_without_optimizer_updates(self):
        if importlib.util.find_spec('rsl_rl') is None:self.skipTest('rsl_rl unavailable')
        from pathlib import Path
        from tempfile import TemporaryDirectory
        from workbench.research.experiments.hug_c.experiment import runner
        from workbench.research.experiments.hug_reverse_rl.experiment import bind_checkpoint,resume
        env=self.environment();env.reset();learner=runner(env,None)
        plan=dict(source_sha256='test-source',reference_path_sha256='test-path')
        before=[p.detach().clone() for p in learner.alg.get_policy().parameters()]
        with TemporaryDirectory() as directory:
            path=Path(directory)/'model.pt';bind_checkpoint(learner,env,plan);learner.save(str(path))
            self.assertTrue(Path(str(path)+'.curriculum.json').is_file())
            resume(learner,env,plan,path)
        for a,b in zip(before,learner.alg.get_policy().parameters()):self.assertTrue(a.equal(b))


if __name__=='__main__':unittest.main()
