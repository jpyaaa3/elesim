"""Snapshot bank from actual forward motion at nominal object placement."""
import numpy as np
from workbench.research.experiments.hug_reverse_rl.bank import ResetBank,compare
from workbench.research.experiments.hug_reverse_ga.physics import sample,tick
from workbench.research.experiments.hug_reverse_ga.prepare import save


class HomeResetBank(ResetBank):
    def _build(self,path):
        env=self.env;plan=self.plan;tol=plan['state_tolerances'];checks=[]
        env.reset();self._capture(-1,plan['home'])
        bins={t:i for i,t in enumerate(plan['pool_ticks'])}
        previous=np.tile(path['command'][0],(env.num_envs,1))
        for t in range(plan['pool_ticks'][0]+1):
            if t:
                command=np.tile(path['command'][t],(env.num_envs,1));state=tick(env,command)
                self._safe(state,command,previous);previous=command
            if t in bins:
                state=sample(env);errors=compare(state,{k:v[t] for k,v in path.items()},tol)
                self._capture(bins[t],path['command'][t]);checks.append(dict(bin=bins[t],tick=t,replay_errors=errors))
        for key,entry in self.entries.items():
            self._restore(key);compare(sample(env),entry['sample'],tol)
            cmd=np.tile(entry['command'],(env.num_envs,1));self._safe(tick(env,cmd),cmd,cmd)
        subset=False
        if env.num_envs>1:
            import torch
            # Compare one forward tick of untouched lanes, including controller
            # behavior, rather than only their instantaneous reset positions.
            self._restore(0);cmd=np.tile(self.entries[0]['command'],(env.num_envs,1))
            expected=tick(env,cmd)
            self._restore(0)
            ids=torch.arange(0,env.num_envs,2,device=env.device);selected=ids.cpu().numpy()
            others=np.setdiff1d(np.arange(env.num_envs),selected)
            self._restore(-1,ids);after=sample(env)
            compare(after,self.entries[-1]['sample'],tol,selected)
            compare(after,self.entries[0]['sample'],tol,others)
            mixed=env.mapper.waypoint.detach().cpu().numpy().copy();after=tick(env,mixed)
            compare(after,expected,tol,others);self._safe(after,mixed,mixed);subset=True
        env.reset()
        self.report=dict(physical_reset_pool_validated=True,source_sha256=plan['source_sha256'],
            reference_path_sha256=plan['reference_path_sha256'],num_envs=env.num_envs,
            bins=len(plan['pool_ticks']),nominal_home_checked=True,subset_restore_checked=subset,
            untouched_lane_forward_tick_checked=subset,prefix_checks=checks,
            forward_grasp_or_lift_verified=False,reset_direction='nominal Home to handoff',
            note='Reset safety verification; PPO must independently learn Home success')
        save(self.directory/'validation.json',self.report)
