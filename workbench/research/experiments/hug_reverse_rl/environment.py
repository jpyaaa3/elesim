"""Task adapter: ordinary PPO steps, independently sampled physical reset bins."""
from copy import deepcopy
import numpy as np
import torch
from workbench.research.experiments.hug_c.environment import HugEnv
from .schedule import Curriculum
from .bank import ResetBank


class CurriculumEnv:
    def __init__(self,plan,path,directory,*,num_envs=512,backend='gpu',video=False):
        from pathlib import Path
        self.plan=plan;self.auto_reset=True;self.mode='curriculum';self.fixed_bin=0
        case=deepcopy(plan['case']);case['start_command']=plan['home']
        experiment=dict(settings=plan['settings'],simulation_context=plan['simulation_context'],cases=[case])
        self.core=HugEnv(experiment,case,Path(directory)/'assets',n_envs=num_envs,
                         backend=backend,auto_reset=False,video=video)
        self.schedule=Curriculum(len(plan['pool_ticks']),plan['curriculum'])
        self.bins=np.zeros(num_envs,int);self.epochs=np.zeros(num_envs,int)
        self.episode_wrong=torch.zeros(num_envs,dtype=torch.bool,device=self.core.device)
        self.bank=ResetBank(self.core,plan,path,Path(directory)/'reset-bank')
        self.reset()

    def __getattr__(self,name):return getattr(self.core,name)

    def set_mode(self,mode,bin_index=0):
        if mode not in ('curriculum','nominal-home','fixed'):raise ValueError('Unknown reset mode')
        if mode=='fixed' and not 0<=bin_index<len(self.plan['pool_ticks']):raise ValueError('Unknown reset bin')
        self.mode,self.fixed_bin=mode,bin_index

    @torch.inference_mode()
    def reset(self,ids=None):
        ids=torch.arange(self.num_envs,device=self.device) if ids is None else ids
        indices=ids.cpu().numpy();n=len(indices)
        if self.mode=='curriculum':bins,epochs=self.schedule.sample(n)
        else:
            bins=np.full(n,-1 if self.mode=='nominal-home' else self.fixed_bin,int)
            epochs=np.full(n,self.schedule.epoch,int)
        self.core.reset(ids);self.bank.restore(bins,ids)
        self.bins[indices]=bins;self.epochs[indices]=epochs
        self.episode_wrong[ids]=False
        # Reset task history at the measured restored pose. Keep original object
        # reference, goal and velocities; preparation time earns no reward.
        self.ledger.reset(ids,self.actual()[ids],self.goal)
        for key in np.unique(bins):
            chosen=ids[torch.as_tensor(bins==key,device=ids.device)]
            state=self.bank.entries[int(key)]['sample']
            saved=chosen.cpu().numpy()
            for attr,name in (('last_touch','touch'),('last_floor','floor'),('last_wrong_face','wrong')):
                target=getattr(self.core,attr)
                target[chosen]=torch.as_tensor(state[name][saved],device=self.device,dtype=target.dtype)
        self.core.scene.robot.control_dofs_position(self.mapper.joint_targets(),dofs_idx_local=self.dofs)
        return self.get_observations(),{}

    def step(self,actions):
        obs,reward,done,extras=self.core.step(actions)
        self.episode_wrong|=self.core.macro_wrong>0
        indices=done.nonzero(as_tuple=False).flatten();finished=indices.cpu().numpy()
        # Observe terminal outcomes BEFORE auto-reset clears success/fault.
        if self.mode=='curriculum':
            ok=(self.core.success&~self.core.fault&~self.episode_wrong).detach().cpu().numpy()
            self.schedule.observe(self.bins[finished],self.epochs[finished],ok[finished])
        extras['log'].update({
            'curriculum/frontier':float(self.schedule.frontier),
            'curriculum/frontier_reverse_time_s':self.plan['pool_ticks'][self.schedule.frontier]*self.s['dt'],
            'curriculum/nominal_ready':float(self.schedule.nominal_ready),
            'curriculum/nominal_reset_fraction':float(np.mean(self.bins==-1))})
        if self.auto_reset and len(finished):obs,_=self.reset(indices)
        return obs,reward,done,extras

    def metadata(self):
        return dict(self.core.metadata(),curriculum=self.schedule.state_dict(),reset_mode=self.mode,
                    reverse_reference_sha256=self.plan['reference_path_sha256'],
                    physical_reset_pool_validated=self.bank.report['physical_reset_pool_validated'])
