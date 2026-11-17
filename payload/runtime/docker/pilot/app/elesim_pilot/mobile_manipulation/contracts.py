"""Measured state and proposals; coordinates are metres/radians in world axes."""
from dataclasses import dataclass, field
import numpy as np


def vector(value, n):
    a=np.asarray(value,float)
    if a.shape!=(n,) or not np.isfinite(a).all():raise ValueError('invalid finite vector')
    return a.copy()


def angle(value):
    return (value+np.pi)%(2*np.pi)-np.pi


def planar(pose):
    x,y,yaw=vector(pose,3);c,s=np.cos(yaw),np.sin(yaw)
    t=np.eye(4);t[:2,:2]=[[c,-s],[s,c]];t[:2,3]=[x,y]
    return t


@dataclass(frozen=True)
class TargetEstimate:
    track_id: str
    origin: np.ndarray
    axis: np.ndarray
    diameter: float
    error_m: float
    stamp: float
    revision: int
    source: tuple
    observed: np.ndarray=field(repr=False)
    def __post_init__(self):
        p=vector(self.origin,3);a=vector(self.axis,3)
        if np.linalg.norm(a)<.99 or not 0<self.diameter<2 or not 0<=self.error_m<1:
            raise ValueError('invalid target estimate')
        if not np.isfinite([self.diameter,self.error_m,self.stamp]).all():raise ValueError('invalid target time/size')
        points=np.asarray(self.observed,float)
        if points.ndim!=2 or points.shape[1]!=3 or len(points)>100000 or not np.isfinite(points).all():
            raise ValueError('invalid observed target points')
        object.__setattr__(self,'origin',p);object.__setattr__(self,'axis',a/np.linalg.norm(a))
        object.__setattr__(self,'observed',points.copy())
    def at_height(self,z):
        if abs(self.axis[2])<.9:raise ValueError('target is not upright')
        return self.origin+self.axis*((z-self.origin[2])/self.axis[2])


@dataclass(frozen=True)
class BaseGoal:
    pose: np.ndarray
    path: np.ndarray
    arm_q: np.ndarray
    report: dict
    def __post_init__(self):
        object.__setattr__(self,'pose',vector(self.pose,3))
        object.__setattr__(self,'arm_q',vector(self.arm_q,4))
        p=np.asarray(self.path,float)
        if p.ndim!=2 or p.shape[1]!=3 or not 1<=len(p)<=2048 or not np.isfinite(p).all():
            raise ValueError('invalid base path')
        object.__setattr__(self,'path',p.copy())


@dataclass(frozen=True)
class Settings:
    max_reobservations: int=3
    max_corrections: int=3
    search_limit: int=8
    max_views: int=12
    total_timeout_s: float=900.
    feedback_timeout_s: float=.5
    base_speed_mps: float=.08
    yaw_speed_radps: float=.15
    base_position_tolerance_m: float=.01
    base_yaw_tolerance_rad: float=.025
    settle_s: float=.5
    standing_tilt_tolerance_rad: float=.07
    base_timeout_s: float=90.
    target_age_s: float=45.
    drift_per_m: float=.02
    drift_per_rad: float=.005
    arrival_error_m: float=.003
    min_position_error_m: float=.003
    scan_roi_height_m: float=.18
    hug_drift_error_m: float=.004
    hug_body_position_tolerance_m: float=.003
    hug_body_rotation_tolerance_rad: float=.008726646259971648
    def __post_init__(self):
        for k,v in vars(self).items():
            if isinstance(v,bool) or not isinstance(v,(int,float)) or not np.isfinite(v) or v<=0:
                raise ValueError('invalid mobile manipulation setting '+k)
        if any(type(getattr(self,k)) is not int for k in ('max_reobservations','max_corrections','search_limit','max_views')):
            raise ValueError('mobile budgets must be integers')
        if not 2<=self.max_views<=24 or self.base_speed_mps>.15 or self.yaw_speed_radps>.3:
            raise ValueError('unsupported view budget / approach speed')
        if (self.hug_drift_error_m>.01 or self.hug_body_position_tolerance_m>.005
                or self.hug_body_rotation_tolerance_rad>.02):
            raise ValueError('hug body allowance exceeds the bounded transport footprint profile')
