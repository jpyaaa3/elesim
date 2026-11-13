"""PPO-only linear travel accounting, in canonical motor-position degrees."""
import numpy as np
from elesim_protocol.messages import SimMappingConfig
from workbench.research.experiments.hug_c.metrics import HoldTracker

LIMIT_DEG = 200.
HOME_DEG = 180.
TARGET_DEG = 0.
TOLERANCE_DEG = 3.
EPS_DEG = 1e-4
_mapping = SimMappingConfig()
_scale = ((_mapping.linear_u_max-_mapping.linear_u_min)
          / (_mapping.linear_q_max_m-_mapping.linear_q_min_m))


def motor_position(linear_m):
    # Do not clamp measured feedback: an overshoot must remain visible.
    return _mapping.linear_u_min+(_mapping.linear_q_max_m-linear_m)*_scale


def contract():
    return dict(version=1,limit_motor_deg=LIMIT_DEG,home_motor_deg=HOME_DEG,
                target_motor_deg=TARGET_DEG,tolerance_motor_deg=TOLERANCE_DEG,
                numerical_epsilon_motor_deg=EPS_DEG,
                accounting='sum(abs(delta applied linear command)) at every physics tick',
                reset_debit='abs(reset command motor position - Home motor position)',
                hold='command and actual linear position within target tolerance for entire hold',
                over_budget='latched fault; terminal fault penalty; no positive reward',
                observation='linear_remaining_fraction')


def validate_resets(linear_m):
    u=motor_position(np.asarray(linear_m,float))
    required=abs(u-HOME_DEG)+abs(u-TARGET_DEG)
    if not np.isfinite(u).all() or np.any(required>LIMIT_DEG+EPS_DEG):
        raise ValueError('Reset cannot reach linear 0 within Home-debited 200-degree budget')


def at_target(actual_m,command_m):
    return ((abs(motor_position(actual_m)-TARGET_DEG)<=TOLERANCE_DEG+EPS_DEG)
            & (abs(motor_position(command_m)-TARGET_DEG)<=TOLERANCE_DEG+EPS_DEG))


def check_trajectory(commands):
    u=motor_position(np.asarray(commands)[:,0])
    spent=abs(u[0]-HOME_DEG)+np.abs(np.diff(u)).sum()
    if spent>LIMIT_DEG+EPS_DEG:
        raise ValueError('Linear cumulative travel exceeds 200 motor degrees')
    if abs(u[-1]-TARGET_DEG)>TOLERANCE_DEG+EPS_DEG:
        raise ValueError('Final linear position is not within 3 motor degrees of 0')
    return float(spent)


class LinearHoldTracker(HoldTracker):
    def __init__(self,owner,xp=np,device=None):
        self.owner=owner
        super().__init__(owner.num_envs,owner.s,xp,device)

    def update(self,clearance,floor,touch,actual,fault,relative,invalid_contact=None):
        if self.owner.linear_enforced:
            invalid=~at_target(actual[:,0],self.owner.mapper.waypoint[:,0])
            invalid_contact=invalid if invalid_contact is None else invalid_contact|invalid
        return super().update(clearance,floor,touch,actual,fault,relative,invalid_contact)


class LinearBudget:
    def __init__(self,n,xp=np,device=None):
        self.xp=xp
        kw={} if xp is np else dict(device=device)
        self.previous=xp.zeros(n,dtype=xp.float64,**kw)
        self.spent=xp.zeros_like(self.previous)
        self.exceeded=(np.zeros(n,bool) if xp is np else xp.zeros(n,dtype=xp.bool,**kw))

    def _position(self,command_m):
        q=(np.asarray(command_m,dtype=np.float64) if self.xp is np
           else command_m.to(dtype=self.previous.dtype))
        return motor_position(q)

    def reset(self,ids,command_m):
        u=self._position(command_m)
        self.previous[ids]=u
        self.spent[ids]=abs(u-HOME_DEG)
        self.exceeded[ids]=self.spent[ids]>LIMIT_DEG+EPS_DEG

    def advance(self,command_m):
        u=self._position(command_m)
        self.spent+=abs(u-self.previous)
        self.previous[:]=u
        self.exceeded|=self.spent>LIMIT_DEG+EPS_DEG

    def remaining(self):
        return self.xp.clip(LIMIT_DEG-self.spent,0,LIMIT_DEG)
