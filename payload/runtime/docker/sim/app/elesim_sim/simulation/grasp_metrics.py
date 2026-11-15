"""Array-based lift measurements, shared by host checks and Torch simulation."""
import numpy as np
import math


def lift_roll_error(actual, settings, xp=np):
    """Measured roll error from the raised pose (canonical roll0, not wrap -90)."""
    return xp.abs(actual[...,1]-settings['lift_roll_target_rad'])


def lift_posture_fraction(actual, settings, xp=np):
    """Geometric progress through the 90-degree lift; no scheduled commands."""
    tolerance=settings['lift_roll_tolerance_rad']
    error=lift_roll_error(actual,settings,xp)
    return xp.clip(1-xp.maximum(error-tolerance,xp.zeros_like(error))/(math.pi/2-tolerance),0,1)


def lift_posture_complete(actual, settings, xp=np):
    return lift_roll_error(actual,settings,xp)<=settings['lift_roll_tolerance_rad']


def local_link_index(robot, name):
    """Pose arrays use entity-local IDs; contact pairs use scene-global IDs."""
    matches = [link for link in robot.links if link.name == name]
    if len(matches) != 1:
        raise ValueError(f'Expected exactly one robot link {name!r}')
    return int(matches[0].idx_local)


def position_offsets_mm(magnitudes):
    """Deterministic XY placement errors; zero is included only if requested."""
    offsets = []
    for radius in magnitudes:
        if not np.isfinite(radius) or radius < 0:
            raise ValueError('Position error magnitudes must be finite and nonnegative')
        for x, y in ([(0., 0.)] if radius == 0 else
                     [(radius*x, radius*y) for x, y in
                      ((-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1))]):
            if [x, y] not in offsets:
                offsets.append([x, y])
    if not offsets:
        raise ValueError('Specify at least one position error magnitude')
    return offsets


def cylinder_clearance(position, quaternion, radius, height, xp=np):
    z=xp.clip(1-2*(quaternion[...,1]**2+quaternion[...,2]**2),-1,1)
    return position[...,2]-height/2*abs(z)-radius*xp.sqrt(xp.clip(1-z*z,0,1))


class HoldTracker:
    """A continuous, event-started lift window; no wall-time phase schedule.

    Uses the same array operations with NumPy in tests and Torch in the MDP.
    A drop, non-C posture, incomplete 90-degree lift, forbidden fault or excessive hand-relative slip
    resets the window. Contact coverage is measured across that window.
    """
    def __init__(self,n,settings,xp=np,device=None):
        self.s,self.xp=settings,xp
        kwargs={'device':device} if device is not None else {}
        self.count=xp.zeros(n,dtype=xp.int64,**kwargs)
        self.hits=xp.zeros(n,dtype=xp.float32,**kwargs)
        self.reference=xp.zeros((n,3),dtype=xp.float32,**kwargs)
        self.minimum_clearance=xp.full((n,),float('inf'),dtype=xp.float32,**kwargs)
        self.maximum_slip=xp.zeros(n,dtype=xp.float32,**kwargs)
        self.required_ticks=round(settings['hold_duration_s']/settings['dt'])
        self.interrupted=xp.zeros(n,dtype=xp.bool,**kwargs) if xp is not np else np.zeros(n,dtype=bool)

    def reset(self,ids):
        self.count[ids]=0;self.hits[ids]=0;self.reference[ids]=0
        self.minimum_clearance[ids]=float('inf');self.maximum_slip[ids]=0
        self.interrupted[ids]=False

    def fraction(self):
        return self.hits/self.xp.maximum(self.count,self.xp.ones_like(self.count))

    def update(self,clearance,floor,touch,actual,fault,relative,invalid_contact=None):
        xp=self.xp
        eligible=(clearance>=self.s['lift_clearance_m'])&~floor&~fault&(actual[:,2]<0)&(actual[:,3]<0)
        eligible&=lift_posture_complete(actual,self.s,xp)
        if invalid_contact is not None:eligible&=~invalid_contact
        delta=relative-self.reference;slip=xp.sqrt((delta*delta).sum(-1))
        restarting=eligible&((self.count==0)|(slip>self.s['hold_slip_m']))
        reset=(~eligible)|restarting
        self.interrupted[:]=(self.count>0)&reset
        self.count[reset]=0;self.hits[reset]=0
        self.minimum_clearance[reset]=float('inf');self.maximum_slip[reset]=0
        self.reference[restarting]=relative[restarting]
        slip=xp.where(restarting,xp.zeros_like(slip),slip)
        self.count+=eligible
        both=(touch[:,0]>0)&(touch[:,1]>0)
        self.hits+=eligible&both
        self.minimum_clearance[:]=xp.where(eligible,xp.minimum(self.minimum_clearance,clearance),self.minimum_clearance)
        self.maximum_slip[:]=xp.where(eligible,xp.maximum(self.maximum_slip,slip),self.maximum_slip)
        return (self.count>=self.required_ticks)&(self.fraction()>=self.s['hold_contact_fraction'])&eligible
