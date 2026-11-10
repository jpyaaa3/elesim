"""PPO owns every learned episode; prefix commands only construct reset states."""
from copy import deepcopy
from pathlib import Path
import numpy as np
import torch
from workbench.research.experiments.hug_c.environment import HugEnv
from workbench.research.experiments.hug_c.control import action_rates
from workbench.research.experiments.hug_reverse_rl.environment import CurriculumEnv
from .bank import HomeResetBank
from .schedule import WindowCurriculum
from .smoothness import Smoothness


class HomeEnv(CurriculumEnv):
    def __init__(self,plan,path,directory,*,num_envs=512,backend='gpu',video=False):
        self.plan=plan;self.auto_reset=True;self.mode='curriculum';self.fixed_bin=0
        case=deepcopy(plan['case']);case['start_command']=plan['home']
        experiment=dict(settings=plan['settings'],simulation_context=plan['simulation_context'],cases=[case])
        self.core=HugEnv(experiment,case,Path(directory)/'assets',n_envs=num_envs,
                         backend=backend,auto_reset=False,video=video)
        self.schedule=WindowCurriculum(len(plan['pool_ticks']),plan['curriculum'])
        self.bins=np.zeros(num_envs,int);self.epochs=np.zeros(num_envs,int)
        self.episode_wrong=torch.zeros(num_envs,dtype=torch.bool,device=self.device)
        self.lift_seen=torch.zeros_like(self.episode_wrong)
        self.smooth=Smoothness(num_envs,action_rates(self.s),self.s['dt']*self.s['action_ticks'],
                               plan['smoothness'],torch,self.device)
        self.bank=HomeResetBank(self.core,plan,path,Path(directory)/'reset-bank')
        self.core.tick_monitor=self._monitor;self.reset()
        if video:self.scene.cameras['report'].set_pose(pos=(.85,-1.,1.25),lookat=(.35,.05,.55),up=(0.,0.,1.))

    def _monitor(self,core):
        qualified=((core.grasp_tick>=0)&(core.last_touch>0).all(-1)&~core.last_floor
            &(core.object_clearance()>=self.plan['smoothness']['activation_clearance_m'])
            &~core.fault&~core.last_wrong_face)
        self.lift_seen|=qualified

    @torch.inference_mode()
    def reset(self,ids=None):
        selected=torch.arange(self.num_envs,device=self.device) if ids is None else ids
        result=super().reset(selected)
        self.smooth.reset(selected,self.mapper.waypoint[selected]);self.lift_seen[selected]=False
        return result

    def step(self,actions):
        active=~self.ledger.finished.clone();self.lift_seen.zero_()
        obs,reward,done,extras=self.core.step(actions)
        penalty=self.smooth.step(self.mapper.waypoint,self.lift_seen,active);reward=reward+penalty
        self.episode_wrong|=self.core.macro_wrong>0
        ids=done.nonzero(as_tuple=False).flatten();finished=ids.cpu().numpy()
        if self.mode=='curriculum':
            clean=(self.core.success&~self.core.fault&~self.episode_wrong).detach().cpu().numpy()
            self.schedule.observe(self.bins[finished],self.epochs[finished],clean[finished])
        extras['log'].update({
            'reward/smoothness':penalty.mean(),
            'smoothness/spent':self.smooth.spent.mean(),
            'smoothness/armed_fraction':self.smooth.armed.float().mean(),
            'curriculum/frontier':float(self.schedule.frontier),
            'curriculum/frontier_prefix_time_s':self.plan['pool_ticks'][self.schedule.frontier]*self.s['dt'],
            'curriculum/nominal_ready':float(self.schedule.nominal_ready),
            'curriculum/nominal_reset_fraction':float(np.mean(self.bins==-1))})
        if self.auto_reset and len(finished):obs,_=self.reset(ids)
        return obs,reward,done,extras

    def metadata(self):
        return dict(self.core.metadata(),control_mode=self.plan['simulation_context']['control_mode'],
            curriculum=self.schedule.state_dict(),reset_mode=self.mode,
            forward_reference_sha256=self.plan['reference_path_sha256'],smoothness=self.plan['smoothness'],
            physical_reset_pool_validated=self.bank.report['physical_reset_pool_validated'])
