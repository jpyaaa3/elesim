"""Linear-constrained PPO and evaluation; reverse teachers retain their physics."""
import torch
from workbench.research.experiments.hug_c.environment import HugEnv
from .linear import LinearBudget, LinearHoldTracker, LIMIT_DEG, at_target, contract


class LinearHugEnv(HugEnv):
    def __init__(self,*args,**kwargs):
        self.linear_budget=None
        self.linear_enforced=True
        super().__init__(*args,**kwargs)
        self.linear_budget=LinearBudget(self.num_envs,torch,self.device)
        self.hold=LinearHoldTracker(self,torch,self.device)
        self.reset()

    def reset_linear_history(self,ids=None):
        if self.linear_budget is None:return
        if ids is None:ids=torch.arange(self.num_envs,device=self.device)
        self.linear_budget.reset(ids,self.mapper.waypoint[ids,0])

    def reset(self,ids=None):
        super().reset(ids)
        self.reset_linear_history(ids)
        return self.get_observations(),{}

    def get_observations(self):
        obs=super().get_observations()
        remaining=(torch.zeros(self.num_envs,device=self.device) if self.linear_budget is None
                   else self.linear_budget.remaining()/LIMIT_DEG)
        obs['policy']=torch.cat((obs['policy'],remaining.to(obs['policy'].dtype)[:,None]),dim=1)
        return obs

    def tick(self,command):
        if self.linear_enforced:
            self.linear_budget.advance(command[:,0])
            # The base ledger blocks all positive rewards and charges exactly
            # one terminal fault penalty, even if success coincides with breach.
            self.fault|=self.linear_budget.exceeded
        super().tick(command)

    def step(self,actions):
        obs,reward,done,extras=super().step(actions)
        extras['log'].update({
            'linear/spent_motor_deg':self.linear_budget.spent.mean(),
            'linear/remaining_motor_deg':self.linear_budget.remaining().mean(),
            'linear/over_budget_fraction':self.linear_budget.exceeded.float().mean(),
            'linear/at_target_fraction':at_target(self.actual()[:,0],self.mapper.waypoint[:,0]).float().mean()})
        return obs,reward,done,extras

    def linear_report(self):
        from .linear import motor_position
        return dict(spent_motor_deg=float(self.linear_budget.spent[0]),
                    remaining_motor_deg=float(self.linear_budget.remaining()[0]),
                    budget_exceeded=bool(self.linear_budget.exceeded[0]),
                    actual_motor_deg=float(motor_position(self.actual()[0,0])),
                    command_motor_deg=float(motor_position(self.mapper.waypoint[0,0])),
                    at_target=bool(at_target(self.actual()[0,0],self.mapper.waypoint[0,0])))

    def metadata(self):
        return dict(super().metadata(),linear_motion_contract=contract())
