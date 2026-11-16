"""Bounded continuous camera-goal refinement; this module never commands motion."""
from __future__ import annotations

import time
import numpy as np
from scipy.optimize import minimize
from elesim_pilot.vision.floor_calibration import camera_transform


def frustum_clearances(pose,points,intrinsics,*,padding_fraction=.05):
    """Signed frustum inequalities in camera metres, including depth limits."""
    pose=camera_transform(pose);points=np.asarray(points,float)
    fx,fy,cx,cy,width,height=np.asarray(intrinsics,float)
    local=(points-pose[:3,3])@pose[:3,:3];x,y,z=local.T
    left=(padding_fraction*width-cx)/fx;right=((1-padding_fraction)*width-cx)/fx
    top=(padding_fraction*height-cy)/fy;bottom=((1-padding_fraction)*height-cy)/fy
    return np.r_[z-.1,2.-z,x-left*z,right*z-x,y-top*z,bottom*z-y]


def refine_view(current,points,bounds,camera_pose,clearances,intrinsics,*,motion_rates,
                seeds=(),admissible=lambda q,pose:True,cancelled=lambda:False,
                clock=time.monotonic,max_seconds=30.,max_evaluations=1024):
    """Minimize squared joint travel time subject to view and clearance bounds.

    Numerical optimizer success is not acceptance. Every returned candidate is
    checked again against all inequalities; the caller still owes a live endpoint
    check and a continuous certificate from actual measured joints.
    """
    current=np.asarray(current,float);points=np.asarray(points,float);bounds=np.asarray(bounds,float)
    intrinsics=np.asarray(intrinsics,float);rates=np.asarray(motion_rates,float)
    seeds=np.asarray(seeds,float).reshape(-1,4)
    if (bounds.shape!=(4,2) or not np.isfinite(bounds).all()
            or np.any(bounds[:,0]>=bounds[:,1])):raise ValueError('invalid camera joint bounds')
    if (current.shape!=(4,) or not np.isfinite(current).all()
            or np.any(current<bounds[:,0]) or np.any(current>bounds[:,1])):
        raise ValueError('invalid camera anchor')
    if (points.ndim!=2 or points.shape[1:]!=(3,) or not 0<len(points)<=64
            or not np.isfinite(points).all()):raise ValueError('invalid camera target points')
    if (intrinsics.shape!=(6,) or not np.isfinite(intrinsics).all()
            or np.any(intrinsics[:2]<=0) or np.any(intrinsics[4:]<1)):
        raise ValueError('invalid camera intrinsics')
    if rates.shape!=(4,) or not np.isfinite(rates).all() or np.any(rates<=0):
        raise ValueError('invalid camera motion rates')
    if (len(seeds)>64 or not np.isfinite(seeds).all() or np.any(seeds<bounds[:,0])
            or np.any(seeds>bounds[:,1])):raise ValueError('invalid camera seeds')
    if (not np.isfinite(max_seconds) or max_seconds<=0 or isinstance(max_seconds,bool)
            or not isinstance(max_evaluations,int) or isinstance(max_evaluations,bool)
            or max_evaluations<1):raise ValueError('invalid camera refinement budget')
    began=clock();cache={};trials=0;best=None

    class Exhausted(Exception):pass

    def evaluate(q):
        nonlocal best
        if cancelled():raise RuntimeError('camera view refinement cancelled')
        if clock()-began>=max_seconds:raise Exhausted()
        q=np.clip(q,bounds[:,0],bounds[:,1]);key=tuple(q)
        if key in cache:return cache[key]
        if len(cache)>=max_evaluations:raise Exhausted()
        time.sleep(.001)  # Keep source-fenced telemetry reception schedulable.
        pose=camera_transform(camera_pose(q));raw=np.asarray(clearances(q),float).reshape(-1)
        if not len(raw) or np.isnan(raw).any() or np.isneginf(raw).any():
            raise ValueError('invalid camera collision clearances')
        # Positive infinity means no observed surface. The finite replacement
        # is only for the numerical solver; final acceptance uses raw margins.
        bounded=np.where(np.isposinf(raw),1.,raw)
        view=frustum_clearances(pose,points,intrinsics)
        constraints=np.r_[view,bounded-1e-6]
        best=float(constraints.min()) if best is None else max(best,float(constraints.min()))
        value=(float(np.sum(((q-current)/(rates*10.))**2)),constraints,raw,pose)
        cache[key]=value;return value

    def report(status,**extra):
        return dict(status=status,elapsed_s=clock()-began,evaluations=len(cache),
            trials=trials,best_constraint_m=best,**extra)

    try:
        for seed in [current,*seeds]:
            trials+=1
            result=minimize(lambda q:evaluate(q)[0],seed,method='SLSQP',bounds=bounds,
                constraints=[dict(type='ineq',fun=lambda q:evaluate(q)[1])],
                options=dict(maxiter=60,ftol=1e-8,eps=1e-6))
            q=np.clip(result.x,bounds[:,0],bounds[:,1]);_,constraints,raw,pose=evaluate(q)
            if (constraints.min()>=-1e-8 and np.all(raw>0) and admissible(q,pose)):
                margin=float(raw.min())
                return q.copy(),report('found',optimizer_success=bool(result.success),
                    minimum_clearance_m=margin if np.isfinite(margin) else None)
    except Exhausted:
        return None,report('budget_exhausted')
    return None,report('not_found')
