"""Bounded, auditable C-hug rewards, shared by NumPy tests and Torch MDP."""
import numpy as np
from .metrics import lift_posture_complete, lift_posture_fraction

COMPONENTS=('q_hint','grasp','lift','hold','success','wrong_face','drop','fault','timeout','action','time','workspace')


def q_potential(actual,goal,settings,xp=np):
    # Only q3/q4: linear/roll never contribute to this hint.
    delta=(actual[...,2:]-goal[...,2:])/(2*settings['bend_per_node_rad'])
    distance=xp.sqrt((delta*delta).sum(-1)/2)
    return settings['reward_q_hint']*(1-xp.clip(distance,0,1))


class RewardLedger:
    """Each physical milestone pays once or only for a new episode record.

    Bad-face steps cannot earn positive rewards or advance paid records. Q
    shaping uses gamma*Phi(next)-Phi(previous), with Phi(terminal)=0. It cannot
    pay a standalone discounted return for matching an empty target posture.
    """
    def __init__(self,n,settings,xp=np,device=None):
        self.s,self.xp=settings,xp;kw={'device':device} if device is not None else {}
        self.grasp_paid=xp.zeros(n,dtype=xp.bool,**kw) if xp is not np else np.zeros(n,dtype=bool)
        self.best_lift=xp.zeros(n,dtype=xp.float32,**kw)
        self.best_hold=xp.zeros_like(self.best_lift);self.previous_q=xp.zeros_like(self.best_lift)
        self.finished=xp.zeros_like(self.grasp_paid)
        self.totals={name:xp.zeros_like(self.best_lift) for name in COMPONENTS}

    def reset(self,ids,actual,goal):
        self.grasp_paid[ids]=False;self.best_lift[ids]=0;self.best_hold[ids]=0;self.finished[ids]=False
        self.previous_q[ids]=q_potential(actual,goal,self.s,self.xp)
        for value in self.totals.values():value[ids]=0

    def step(self,actual,goal,actions,*,confirmed_grasp,lift_fraction,hold_fraction,
             wrong_fraction,fault,success,timeout,drops,workspace_fraction=None):
        xp=self.xp;s=self.s
        active=~self.finished;blocked=(wrong_fraction>0)|fault
        allowed=active&~blocked
        raised=lift_posture_complete(actual,s,xp)
        passed=success&allowed&raised;done=active&(fault|passed|timeout)
        zero=xp.zeros_like(self.best_lift)
        phi=xp.where(allowed&~done,q_potential(actual,goal,s,xp),zero)
        q=xp.where(active,s['reward_gamma']*phi-self.previous_q,zero)
        self.previous_q[:]=phi
        first=allowed&confirmed_grasp&~self.grasp_paid
        self.grasp_paid|=first
        lift=xp.where(allowed&confirmed_grasp,
                      xp.minimum(xp.clip(lift_fraction,0,1),lift_posture_fraction(actual,s,xp)),zero)
        hold=xp.where(allowed&raised,xp.clip(hold_fraction,0,1),zero)
        dl=xp.maximum(lift-self.best_lift,zero);dh=xp.maximum(hold-self.best_hold,zero)
        self.best_lift[:]=xp.maximum(self.best_lift,lift);self.best_hold[:]=xp.maximum(self.best_hold,hold)
        duration=s['action_ticks']*s['dt']
        # Only a currently qualified loaded hold exempts verification time.
        # Touching/rubbing, a historical grasp, or an empty q target cannot.
        verifying=allowed&raised&confirmed_grasp&(hold_fraction>0)
        exposure=zero if workspace_fraction is None else xp.clip(workspace_fraction,0,1)
        terms=dict(q_hint=q,grasp=first*s['reward_grasp'],lift=dl*s['reward_lift'],hold=dh*s['reward_hold'],
            success=passed*s['reward_success'],wrong_face=-s['penalty_wrong_face_per_s']*active*wrong_fraction*duration,
            drop=-s['penalty_drop']*active*drops,fault=-s['penalty_fault']*(active&fault),
            timeout=-s['penalty_timeout']*(active&timeout&~fault&~passed),
            action=-s['penalty_action']*active*(xp.clip(actions,-1,1)**2).sum(-1),
            time=-s['penalty_time_per_s']*active*~verifying*duration,
            workspace=-s['penalty_workspace_per_s']*active*exposure*duration)
        for name,value in terms.items():self.totals[name]+=value
        self.finished|=done
        return sum(terms.values()),done,terms
