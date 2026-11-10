"""Rebuild a full-state reset bank by physically replaying the reverse prefix."""
from copy import deepcopy
from pathlib import Path
import numpy as np
from workbench.research.experiments.hug_reverse_ga.physics import sample,tick,admit,arrays,restore_leg_hold
from workbench.research.experiments.hug_reverse_ga.criteria import violations,admission_good
from workbench.research.experiments.hug_reverse_ga.prepare import save


def compare(actual,expected,tolerances,ids=None):
    errors={}
    for key,tol in tolerances.items():
        value=actual[key] if ids is None else actual[key][ids]
        target=np.asarray(expected[key])
        if ids is not None and target.shape==actual[key].shape:target=target[ids]
        error=float(np.max(abs(value-target)))
        errors[key]=error
        if error>tol:raise RuntimeError(f'Physical state mismatch {key}: {error} > {tol}')
    return errors


class ResetBank:
    def __init__(self,env,plan,path,directory):
        self.env,self.plan=env,plan;self.entries={};self.directory=Path(directory)
        self.directory.mkdir(parents=True,exist_ok=False)
        try:self._build(path)
        except Exception as exc:
            save(self.directory/'validation.json',dict(physical_reset_pool_validated=False,error=repr(exc)))
            raise

    def _capture(self,key,command):
        self.entries[key]=dict(state=self.env.scene.scene.get_state(),command=np.asarray(command).copy(),
            sample=sample(self.env))

    def _restore(self,key,ids=None):
        import torch
        env=self.env;entry=self.entries[key]
        env.scene.scene.reset(entry['state'],envs_idx=ids)
        if ids is None:env.mapper.waypoint[:]=torch.as_tensor(entry['command'],device=env.device,dtype=torch.float32)
        else:env.mapper.waypoint[ids]=torch.as_tensor(entry['command'],device=env.device,dtype=torch.float32)
        env.scene.robot.control_dofs_position(env.mapper.joint_targets(),dofs_idx_local=env.dofs)
        restore_leg_hold(env,ids)

    def _safe(self,state,command,previous):
        lo,hi,rate=arrays(self.env)
        mask,_,_=violations(state,command,previous,lo,hi,rate,self.env.case['object_xyz_m'])
        if mask.any():raise RuntimeError('Unsafe reset prefix/contact probe: '+str(np.argwhere(mask)[:5].tolist()))

    def _build(self,path):
        env=self.env;plan=self.plan;home_case=deepcopy(env.case);tol=plan['state_tolerances'];checks=[]
        env.reset();self._capture(-1,plan['home'])
        grasp_case=dict(home_case,start_command=home_case['goal_command'])
        env.set_case(grasp_case)
        if not admit(env,self.directory/'admission'):raise RuntimeError('Initial solver grasp failed admission')
        admitted=env.scene.scene.get_state();baseline=sample(env)
        # Match the corrected GA snapshot restore, including standing leg PD.
        env.reset();env.scene.scene.reset(admitted);restore_leg_hold(env);env.mapper.waypoint[:]=env.start
        compare(sample(env),baseline,tol)
        bins={tick:i for i,tick in enumerate(plan['pool_ticks'])}
        previous=np.tile(path['command'][0],(env.num_envs,1))
        for t in range(plan['pool_ticks'][-1]+1):
            if t:
                command=np.tile(path['command'][t],(env.num_envs,1));state=tick(env,command)
                self._safe(state,command,previous);previous=command
            if t in bins:
                state=sample(env);errors=compare(state,{k:v[t] for k,v in path.items()},tol)
                self._capture(bins[t],path['command'][t]);checks.append(dict(bin=bins[t],tick=t,replay_errors=errors))
        # Validate state restoration and one forward physics tick from every
        # reset, preserving the recorded velocities rather than inventing rest.
        for key,entry in self.entries.items():
            self._restore(key);state=sample(env);compare(state,entry['sample'],tol)
            cmd=np.tile(entry['command'],(env.num_envs,1));state=tick(env,cmd)
            self._safe(state,cmd,cmd)
            if key==0 and not admission_good(state,np.asarray(home_case['goal_command'])).all():
                raise RuntimeError('Restored grasp does not retain proper loaded contact')
        # Restore a mixed subset and prove that all other lanes stay unchanged.
        if env.num_envs>1:
            import torch
            self._restore(0);before=sample(env)
            ids=torch.arange(0,env.num_envs,2,device=env.device);self._restore(-1,ids)
            after=sample(env);selected=ids.cpu().numpy();others=np.setdiff1d(np.arange(env.num_envs),selected)
            compare(after,self.entries[-1]['sample'],tol,selected)
            compare(after,{k:v[others] for k,v in before.items()},tol,others)
        env.set_case(home_case);env.reset()
        self.report=dict(physical_reset_pool_validated=True,source_sha256=plan['source_sha256'],
            reference_path_sha256=plan['reference_path_sha256'],num_envs=env.num_envs,
            bins=len(plan['pool_ticks']),nominal_home_checked=True,subset_restore_checked=env.num_envs>1,
            prefix_checks=checks,forward_grasp_or_lift_verified=False,
            note='One-tick reset safety probes; successful PPO lift remains required')
        save(self.directory/'validation.json',self.report)

    def restore(self,bins,ids):
        import torch
        for key in np.unique(bins):
            mask=torch.as_tensor(bins==key,device=ids.device)
            self._restore(int(key),ids[mask])
