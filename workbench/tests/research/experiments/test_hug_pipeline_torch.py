"""Real CPU optimizer/export/resume smoke with a fake VecEnv, not physics evidence."""
import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch


@unittest.skipUnless(importlib.util.find_spec('torch') and importlib.util.find_spec('tensordict') and importlib.util.find_spec('rsl_rl'),'Scientific stack unavailable')
class PipelineLearning(unittest.TestCase):
    def test_cold_optimizer_export_and_resume_without_initial_policy(self):
        from workbench.tests.research.experiments.test_hug_home_rl_torch import HomeTests
        from workbench.research.experiments.hug_pipeline import learning
        from workbench.research.experiments.hug_pipeline.policy import NumpyPolicy
        context={'contract':'test-only'}
        plan=dict(source_sha256='test-source',reference_path_sha256='test-reference',initial_checkpoint_sha256=None,
            curriculum={'seed':42},shared_context=context,case={'case_id':'fake'})
        def factory(*args,**kwargs):
            env=HomeTests().fake_env();env.reset()
            import torch
            original=env.core.get_observations
            def observations():
                obs=original();obs['policy']=torch.cat((obs['policy'],torch.ones((4,1))),dim=1)
                return obs
            env.core.get_observations=observations
            env.metadata=lambda:dict(simulation_context=context,forward_reference_sha256='test-reference')
            return env
        with TemporaryDirectory() as directory,patch.object(learning,'load',return_value=(plan,{})),patch.object(learning,'create_env',side_effect=factory):
            root=Path(directory)
            learning.train(root/'absent-input-policy',root/'cold',num_envs=4,iterations=1,backend='cpu')
            run=json.loads((root/'cold/run.json').read_text())
            self.assertEqual(run['initialization'],'cold');self.assertFalse(run['scripted_episode_actions'])
            NumpyPolicy(root/'cold/policy.npz')
            checkpoint=root/'cold/model_final.pt'
            first=json.loads(Path(str(checkpoint)+'.curriculum.json').read_text())
            self.assertIsNone(first['initial_checkpoint_sha256'])
            learning.train(root/'absent-input-policy',root/'resumed',num_envs=4,iterations=1,backend='cpu',resume=checkpoint)
            second=json.loads((root/'resumed/model_final.pt.curriculum.json').read_text())
            self.assertGreater(second['curriculum']['steps'],first['curriculum']['steps'])
            self.assertFalse(json.loads((root/'resumed/completion.json').read_text())['home_verified'])

    def test_three_evaluations_can_reset_tensors_created_by_previous_episode(self):
        import torch
        import numpy as np
        from types import SimpleNamespace
        from workbench.research.experiments.hug_pipeline import evaluation,contracts
        from workbench.research.experiments.hug_c.configure import DEFAULTS
        case={'case_id':'test'};shared={'test':True};context=dict(shared,case_sha256=contracts.canonical(case))
        class Policy:
            def __init__(self,path):self.metadata={'simulation_context':context}
            def __call__(self,obs):return np.zeros((1,4),np.float32)
        class Env:
            def __init__(self,*args,**kwargs):
                self.device='cpu';self.num_envs=1;self.max_episode_length=1
                self.last_touch=torch.zeros((1,2));self.fault=torch.zeros(1,dtype=torch.bool)
                self.last_wrong_face=torch.zeros_like(self.fault);self.last_floor=torch.zeros_like(self.fault)
                self.grasp_tick=torch.zeros(1,dtype=torch.long);self.hold=SimpleNamespace(
                    count=torch.ones(1,dtype=torch.long),fraction=lambda:torch.ones(1),
                    maximum_slip=torch.zeros(1))
                self.success=torch.ones(1,dtype=torch.bool);self.mapper=SimpleNamespace(waypoint=torch.zeros((1,4)))
            def reset(self):
                self.last_touch.zero_()
                return {'policy':torch.zeros((1,20))},{}
            def linear_report(self):return dict(budget_exceeded=False,at_target=True)
            def object_clearance(self):return torch.ones(1)*.03
            def step(self,action):
                self.last_touch=torch.ones((1,2));self.tick_monitor(self)
                return {'policy':torch.zeros((1,20))},torch.ones(1),torch.ones(1,dtype=torch.bool),{}
        study=dict(shared_context=shared,settings=dict(DEFAULTS),home=[0.,0.,0.,0.],backend='cpu')
        with TemporaryDirectory() as directory,patch('workbench.research.experiments.hug_pipeline.task.LinearHugEnv',Env),patch('workbench.research.experiments.hug_pipeline.policy.NumpyPolicy',Policy),patch('workbench.research.experiments.hug_reverse_ga.physics.sample',return_value={'actual':np.zeros((1,4))}):
            result=evaluation.evaluate(study,case,Path(directory),policy_path=Path(directory)/'mock',repetitions=3,video=False)
            self.assertTrue(result['accepted']);self.assertEqual(len(result['results']),3)

if __name__=='__main__':unittest.main()
