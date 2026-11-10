"""Episode-bounded applied-command variation cost, NumPy/Torch compatible."""
import math
import numpy as np

DEFAULTS=dict(budget=1.,deadband=.1,reference_duration_s=5.,activation_clearance_m=.005)


def validate(options):
    if set(options)!=set(DEFAULTS):raise ValueError('Unknown/missing smoothness option')
    if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in options.values()):
        raise ValueError('Invalid smoothness value')
    if not 0<=options['budget']<=1 or not 0<=options['deadband']<2 or min(options['reference_duration_s'],options['activation_clearance_m'])<=0:
        raise ValueError('Invalid smoothness budget/deadband/duration')


class Smoothness:
    def __init__(self,n,rates,control_dt,options=None,xp=np,device=None):
        self.xp=xp;self.o=dict(DEFAULTS,**(options or {}));validate(self.o);self.dt=control_dt
        kw={'device':device} if xp is not np else {}
        self.previous=xp.zeros((n,4),**kw);self.velocity=xp.zeros((n,4),**kw)
        self.spent=xp.zeros(n,**kw)
        self.armed=xp.zeros(n,dtype=bool) if xp is np else xp.zeros(n,dtype=xp.bool,**kw)
        self.rates=np.asarray(rates) if xp is np else xp.as_tensor(rates,device=device,dtype=xp.float32)

    def reset(self,ids,command):
        self.previous[ids]=command;self.velocity[ids]=0;self.spent[ids]=0;self.armed[ids]=False

    def step(self,command,lift_seen,active):
        xp=self.xp;o=self.o
        velocity=xp.clip((command-self.previous)/self.rates,-1,1)
        delta=xp.clip((abs(velocity-self.velocity)-o['deadband'])/(2-o['deadband']),0,1)
        intensity=(delta*delta).mean(-1)
        # The first qualified lift step is free. Once armed, losing contact or
        # leaving the roll/hold window cannot switch this cost off.
        charge=xp.minimum(xp.clip(o['budget']-self.spent,0,o['budget']),
                          o['budget']*self.dt/o['reference_duration_s']*intensity)
        charge=charge*(self.armed&active)
        self.spent+=charge;self.armed|=lift_seen&active
        self.previous[:]=command;self.velocity[:]=velocity
        return -charge
