"""GA-only object-motion feasibility, sampled at every physics tick."""
from contextlib import contextmanager
import math
import numpy as np


def motion_contract(max_object_xy_mm=None):
    if max_object_xy_mm is not None:
        if (isinstance(max_object_xy_mm,bool) or not isinstance(max_object_xy_mm,(int,float))
            or not math.isfinite(max_object_xy_mm) or max_object_xy_mm<=0):
            raise ValueError('Object XY limit must be a finite positive number of mm or None')
        max_object_xy_mm=float(max_object_xy_mm)
    return dict(schema='hug-ga-object-motion-v2',metric='world-xy-center-displacement',
                reference='initial-object-center',max_object_xy_mm=max_object_xy_mm,
                sampling='physics-tick',comparison='strictly-greater-than',
                scope='ground-contact-only',
                airborne_rule='no-floor-contact-and-positive-bottom-clearance')


def read_motion_contract(metadata):
    contract=metadata.get('constraints')
    if contract is None:return motion_contract()  # Older unconstrained GA evidence.
    expected=motion_contract(contract.get('max_object_xy_mm'))
    if contract.get('schema')=='hug-ga-object-motion-v1':
        # Historical whole-episode constraints retain their original meaning.
        expected={k:v for k,v in expected.items() if k not in ('scope','airborne_rule')}
        expected['schema']='hug-ga-object-motion-v1'
    if contract!=expected:raise ValueError('Unsupported GA object motion contract')
    return expected


class MotionTracker:
    def __init__(self,origin,n,contract,xp=np,device=None):
        self.contract=read_motion_contract(dict(constraints=contract))
        self.limit=self.contract['max_object_xy_mm']
        self.ground_only=self.contract.get('scope')=='ground-contact-only'
        self.xp=xp;kw={} if device is None else dict(device=device)
        self.origin=xp.asarray(origin,dtype=xp.float32,**kw)
        self.peak_mm=xp.zeros(n,dtype=xp.float32,**kw)
        self.peak_ground_mm=xp.zeros_like(self.peak_mm)
        self.violated=xp.zeros(n,dtype=xp.bool if xp is not np else bool,**kw)
        self.active=xp.ones_like(self.violated)
        self.first_tick=xp.full((n,),-1,dtype=xp.int64,**kw)

    def observe(self,positions,ticks,*,floor_contact=None,clearance=None):
        xp=self.xp
        if floor_contact is None or clearance is None:
            if self.ground_only:
                raise ValueError('Ground-only constraint requires floor contact and bottom clearance')
            grounded=xp.ones_like(self.active)
        else:
            # Contact evidence wins over a small positive geometric gap. A
            # missing contact sample cannot exempt a cylinder touching the floor.
            grounded=floor_contact|(clearance<=0)
        restricted=self.active&grounded if self.ground_only else self.active
        delta=positions[...,:2]-self.origin[...,:2]
        distance=xp.sqrt((delta*delta).sum(-1))*1000
        self.peak_mm[:]=xp.where(self.active,xp.maximum(self.peak_mm,distance),self.peak_mm)
        self.peak_ground_mm[:]=xp.where(self.active&grounded,
                                       xp.maximum(self.peak_ground_mm,distance),self.peak_ground_mm)
        if self.limit is not None:
            hit=restricted&(distance>self.limit)
            self.first_tick[:]=xp.where(hit&~self.violated,ticks,self.first_tick)
            self.violated|=hit


@contextmanager
def monitor_motion(env,contract):
    import torch
    tracker=MotionTracker(env.origin,env.num_envs,contract,torch,env.device)
    previous=env.tick_monitor
    def capture(current):
        # HugEnv updates last_floor at this same physics tick before its monitor.
        tracker.observe(current.scene.object.get_pos(),current.ticks,
                        floor_contact=current.last_floor,clearance=current.object_clearance())
        if previous is not None:previous(current)
    env.tick_monitor=capture
    try:yield tracker
    finally:env.tick_monitor=previous
