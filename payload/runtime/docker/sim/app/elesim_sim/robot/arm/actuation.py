"""Genesis position-actuator adapter shared by runtime and research."""
from dataclasses import dataclass, asdict
import math
import numpy as np


@dataclass(frozen=True)
class PositionGains:
    kp: float = 200.
    kv: float = 20.
    linear_kp: float = 2000.
    linear_kv: float = 200.
    force_limit: float = 100.

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError('position actuator gains must be finite and positive')


def configure_position_gains(entity, dofs, linear_dof, gains=PositionGains()):
    dofs=list(dofs)
    linear_dofs=[linear_dof] if isinstance(linear_dof,int) else list(linear_dof)
    if len(set(dofs)) != len(dofs) or not linear_dofs or not set(linear_dofs)<=set(dofs):
        raise ValueError('invalid arm actuator DOF layout')
    kp=np.full(len(dofs),gains.kp);kv=np.full(len(dofs),gains.kv)
    for index in linear_dofs:
        kp[dofs.index(index)]=gains.linear_kp
        kv[dofs.index(index)]=gains.linear_kv
    entity.set_dofs_kp(kp,dofs_idx_local=dofs)
    entity.set_dofs_kv(kv,dofs_idx_local=dofs)
    entity.set_dofs_force_range(np.full(len(dofs),-gains.force_limit),
                                np.full(len(dofs),gains.force_limit),dofs_idx_local=dofs)


class PositionActuator:
    def __init__(self,entity,dofs,linear_dof,gains=PositionGains()):
        self.entity=entity;self.dofs=list(dofs);self.linear_dof=linear_dof;self.gains=gains
        configure_position_gains(entity,self.dofs,linear_dof,gains)

    def command(self,target):
        q=np.asarray(target,float)
        if q.shape!=(len(self.dofs),) or not np.isfinite(q).all():
            raise ValueError('invalid arm position target')
        self.entity.control_dofs_position(q,dofs_idx_local=self.dofs)

    def disable(self):
        self.entity.control_dofs_force(np.zeros(len(self.dofs)),dofs_idx_local=self.dofs)


