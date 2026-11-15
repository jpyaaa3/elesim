"""Observed-only upright-cylinder estimates with conservative association bounds."""
import numpy as np
from scipy.optimize import least_squares
from .contracts import TargetEstimate


def fit_observed_cylinder(points,*,minimum_arc_rad=np.pi/3):
    p=np.asarray(points,float)
    if p.ndim!=2 or p.shape[1]!=3 or not 60<=len(p)<=100000 or not np.isfinite(p).all():
        raise ValueError('insufficient finite target depth')
    if len(p)>6000:p=p[np.linspace(0,len(p)-1,6000,dtype=int)]
    z=float(np.median(p[:,2]));xy=p[:,:2]
    a=np.column_stack((2*xy,np.ones(len(xy))))
    if np.linalg.cond(a)>1e6:raise ValueError('target circle is poorly constrained')
    cx,cy,k=np.linalg.lstsq(a,np.sum(xy*xy,axis=1),rcond=None)[0]
    radius=np.sqrt(max(0,k+cx*cx+cy*cy))
    if not .035<radius<.065:raise ValueError('target diameter unsupported')
    def residual(v):
        center=np.array(v[:2])+(p[:,2:3]-z)*np.array(v[2:4])
        return np.linalg.norm(xy-center,axis=1)-v[4]
    fit=least_squares(residual,[cx,cy,0,0,radius],loss='soft_l1',f_scale=.002,
        bounds=([cx-.05,cy-.05,-.087,-.087,.035],[cx+.05,cy+.05,.087,.087,.065]),max_nfev=80)
    cx,cy,tx,ty,r=fit.x
    error=abs(residual(fit.x));angles=np.sort(np.arctan2(p[:,1]-cy,p[:,0]-cx))
    coverage=2*np.pi-max(np.diff(np.r_[angles,angles[0]+2*np.pi]))
    if not fit.success or coverage<minimum_arc_rad or np.quantile(error,.95)>.004:
        raise ValueError('target surface coverage / fit residual inadequate')
    axis=np.array([tx,ty,1.]);axis/=np.linalg.norm(axis)
    if np.linalg.norm(axis[:2])>np.sin(np.deg2rad(5)):raise ValueError('target cylinder tilted beyond profile')
    return np.array([cx,cy,z]),axis,float(2*r),float(np.quantile(error,.95)),float(coverage)


class TargetTrack:
    def __init__(self,target_id='target',minimum_error=.003):
        self.target_id=target_id;self.minimum_error=minimum_error;self.estimate=None
    def update(self,points,*,stamp,source,pose_error=.003):
        origin,axis,diameter,residual,coverage=fit_observed_cylinder(points)
        old=self.estimate
        if old is not None:
            if tuple(source)!=old.source:raise RuntimeError('target source/calibration changed')
            if stamp<=old.stamp:raise RuntimeError('replayed target observation')
            previous=old.at_height(origin[2])
            if np.linalg.norm(origin-previous)>.04+old.error_m:
                raise RuntimeError('target identity / stationary-object assumption changed')
        # This is an error allowance, not a calibrated probabilistic confidence.
        # Limited angular support amplifies depth/FK residual uncertainty.
        error=max(self.minimum_error,(residual+pose_error)/max(np.sin(min(coverage,np.pi)/2),.25))
        self.estimate=TargetEstimate(self.target_id,origin,axis,diameter,error,float(stamp),
            1 if old is None else old.revision+1,tuple(source),np.asarray(points))
        return self.estimate
    def error_after_motion(self,distance,rotation,settings):
        if self.estimate is None:raise RuntimeError('no target estimate')
        return self.estimate.error_m+abs(distance)*settings.drift_per_m+abs(rotation)*settings.drift_per_rad
