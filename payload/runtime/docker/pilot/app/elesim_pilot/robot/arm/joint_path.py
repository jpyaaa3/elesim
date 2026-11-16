"""Bounded four-axis RRT-Connect; every accepted edge needs a full certificate.

This local planner only proposes canonical joint waypoints. It owns no motion,
uses no simulator state, and cannot replace the caller's live final validation.
"""
from __future__ import annotations

import time
import numpy as np


def shorten_joint_path(path,certify,*,max_seconds=30.,max_checks=1024,
                       cancelled=lambda:False,clock=time.monotonic):
    """Shortcut a certified polyline; keep its certified tail on budget expiry.

    The caller must certify the input and revalidate the returned path against
    live feedback. Every newly introduced edge needs its own full certificate.
    """
    path=np.array(path,float,copy=True)
    if path.ndim!=2 or path.shape[1:]!=(4,) or len(path)<2 or not np.isfinite(path).all():
        raise ValueError('shortening requires a finite certified four-axis path')
    if (not np.isfinite(max_seconds) or max_seconds<=0 or not isinstance(max_checks,int)
            or isinstance(max_checks,bool) or max_checks<1):
        raise ValueError('invalid path shortening budget')
    began=clock();checks=0;exhausted=False;shortened=[path[0]];i=0
    while i<len(path)-1:
        if cancelled():raise RuntimeError('joint path shortening cancelled')
        chosen=i+1  # This original adjacent edge is already certified.
        for j in range(len(path)-1,i+1,-1):
            if cancelled():raise RuntimeError('joint path shortening cancelled')
            if checks>=max_checks or clock()-began>=max_seconds:
                exhausted=True;break
            time.sleep(.001)
            checks+=1
            if certify(path[i],path[j])['accepted']:
                chosen=j;break
        if exhausted:
            shortened.extend(path[i+1:]);break
        shortened.append(path[chosen]);i=chosen
    return np.array(shortened),dict(edge_checks=checks,elapsed_s=clock()-began,
        original_segments=len(path)-1,segments=len(shortened)-1,budget_exhausted=exhausted)


def connect_joint_path(start, goals, bounds, certify, *, samples, seeds=(),
                       cancelled=lambda: False, clock=time.monotonic,
                       max_seconds=30., max_nodes=512, max_checks=1024):
    bounds=np.asarray(bounds,float);start=np.asarray(start,float)
    goals=np.asarray(goals,float);samples=np.asarray(samples,float)
    seeds=np.asarray(seeds,float).reshape(-1,4)
    if (bounds.shape!=(4,2) or not np.isfinite(bounds).all()
            or np.any(bounds[:,1]<=bounds[:,0])):
        raise ValueError('joint search requires finite four-axis bounds')
    if start.shape!=(4,) or goals.ndim!=2 or goals.shape[1:]!=(4,) or not len(goals):
        raise ValueError('joint search requires a start and nonempty four-axis goals')
    if samples.ndim!=2 or samples.shape[1:]!=(4,) or not len(samples):
        raise ValueError('joint search requires bounded four-axis samples')
    for values in (start,goals,samples,seeds):
        if (not np.isfinite(values).all() or np.any(values<bounds[:,0])
                or np.any(values>bounds[:,1])):
            raise ValueError('joint search input outside finite bounds')
    if (not np.isfinite(max_seconds) or max_seconds<=0
            or not isinstance(max_nodes,int) or not isinstance(max_checks,int)
            or isinstance(max_nodes,bool) or isinstance(max_checks,bool)
            or max_nodes<=len(goals)+1 or max_checks<1):
        raise ValueError('invalid joint search budget')
    began=clock();deadline=began+max_seconds;checks=0;iterations=0;cache={}
    span=bounds[:,1]-bounds[:,0];rng=np.random.default_rng(43)
    trees=[dict(q=[start.copy()],parent=[-1],root=[0],start=True),
           dict(q=[q.copy() for q in goals],parent=[-1]*len(goals),
                root=list(range(len(goals))),start=False)]

    def budget():
        if cancelled():raise RuntimeError('joint path search cancelled')
        return clock()<deadline and checks<max_checks

    def clear(a,b):
        nonlocal checks
        key=(tuple(a),tuple(b))
        if key in cache:return cache[key]
        if not budget():return False
        checks+=1
        cache[key]=bool(certify(a,b)['accepted'])
        return cache[key]

    def result(path=None,goal_index=None,*,strategy='rrt_connect'):
        if path is not None and len(path)==1:path=[path[0],path[0].copy()]
        report=dict(status='found' if path is not None else 'budget_exhausted',
            elapsed_s=clock()-began,iterations=iterations,edge_checks=checks,
            tree_nodes=[len(t['q']) for t in trees],goal_index=goal_index,strategy=strategy)
        return (None if path is None else np.array(path)),report

    if not clear(start,start):return result()
    # Known poses seed the start tree only after a fresh complete certificate.
    for q in seeds:
        if not budget() or len(trees[0]['q'])+len(goals)>=max_nodes:break
        if not np.array_equal(start,q) and clear(start,q):
            trees[0]['q'].append(q.copy());trees[0]['parent'].append(0);trees[0]['root'].append(0)
            # A useful clearance pose may already connect to a goal. Check
            # that complete bridge before spending the same bounded budget
            # growing short random edges around an existing solution.
            for goal_index,goal in enumerate(goals):
                if not budget():break
                if clear(q,goal):
                    return result([start,q,goal],goal_index,strategy='certified_seed_bridge')

    def extend(tree,target):
        values=np.array(tree['q'])
        i=int(np.argmin(np.linalg.norm((values-target)/span,axis=1)))
        delta=target-values[i];length=float(np.linalg.norm(delta/span))
        if np.array_equal(target,values[i]):return i,True
        if sum(len(t['q']) for t in trees)>=max_nodes:return None,False
        q=target.copy() if length<=.08 else values[i]+delta*(.08/length)
        if not clear(values[i],q):return None,False
        tree['q'].append(q);tree['parent'].append(i);tree['root'].append(tree['root'][i])
        return len(tree['q'])-1,length<=.08

    def trace(tree,i):
        path=[]
        while i>=0:path.append(tree['q'][i]);i=tree['parent'][i]
        return path[::-1]

    while budget() and sum(len(t['q']) for t in trees)<max_nodes:
        time.sleep(.001)  # Keep the independent telemetry receiver schedulable.
        a,b=trees[iterations%2],trees[(iterations+1)%2];iterations+=1
        sample=(b['q'][int(rng.integers(len(b['q'])))] if rng.random()<.2
                else samples[int(rng.integers(len(samples)))])
        ia,_=extend(a,sample)
        if ia is None:continue
        for _ in range(16):
            if not budget():break
            ib,reached=extend(b,a['q'][ia])
            if ib is None:break
            if not reached:continue
            pa,pb=trace(a,ia),trace(b,ib)
            path=pa+pb[-2::-1] if a['start'] else pb+pa[-2::-1]
            goal_index=b['root'][ib] if a['start'] else a['root'][ia]
            # Remove redundant bends only when the entire shortcut certifies.
            shortened=[path[0]];i=0
            while i<len(path)-1:
                if not budget():shortened.extend(path[i+1:]);break
                j=next((j for j in range(len(path)-1,i,-1) if clear(path[i],path[j])),i+1)
                shortened.append(path[j]);i=j
            return result(shortened,goal_index)
    return result()
