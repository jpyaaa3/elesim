"""Sensor-only AdaPoinTr trial helpers; no runtime or actuator integration."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class CloudFrame:
    center: np.ndarray
    meters_per_unit: float


def pcn_axes(points):
    """Involutive Z-up/PCN Y-up axis ordering from upstream NormalizeObjectPose.

    This orders coordinates as (x,z,y), exactly as the published KITTI-to-PCN
    transform. No ground-truth object heading or bounding box is consulted.
    """
    points=np.asarray(points)
    if points.ndim!=2 or points.shape[1]!=3:raise ValueError('invalid point axes')
    return points[:,[0,2,1]].copy()


def normalize_partial(points, *, radius=.5):
    """Declare an observed-centroid frame without consulting complete geometry."""
    points=np.asarray(points,dtype=np.float64)
    if (points.ndim!=2 or points.shape[1]!=3 or not 32<=len(points)<=100_000
            or not np.all(np.isfinite(points)) or not 0<radius<=1):
        raise ValueError('unsupported partial point cloud')
    center=points.mean(axis=0)
    extent=np.linalg.norm(points-center,axis=1).max()
    if extent<1e-6:raise ValueError('degenerate partial point cloud')
    frame=CloudFrame(center.copy(),float(extent/radius))
    return ((points-center)/frame.meters_per_unit).astype(np.float32),frame


def restore_completion(points,frame):
    points=np.asarray(points)
    if points.ndim!=2 or points.shape[1]!=3 or not np.all(np.isfinite(points)):
        raise ValueError('invalid completion')
    return points.astype(np.float64)*frame.meters_per_unit+frame.center


def sample_input(points, *, count=2048, seed=0):
    points=np.asarray(points,dtype=np.float32)
    if points.ndim!=2 or points.shape[1]!=3 or not len(points) or count<1:
        raise ValueError('invalid sample input')
    rng=np.random.default_rng(seed)
    if len(points)>=count:return points[rng.choice(len(points),count,replace=False)].copy()
    indices=np.concatenate([np.arange(len(points)),rng.choice(len(points),count-len(points),replace=True)])
    rng.shuffle(indices)
    return points[indices].copy()


def fps_reference(points,count):
    """PointNet++ FPS semantics, including near-zero exclusion and lane ties.

    Independent CPU reference for the published sampling_gpu.cu algorithm.
    This is a test oracle for the operator, never a shape-completion oracle.
    """
    points=np.asarray(points,dtype=np.float32)
    n=len(points);threads=min(512,2**(n.bit_length()-1))
    order=np.lexsort((np.arange(n),np.arange(n)%threads))
    distance=np.full(n,1e10,dtype=np.float32)
    magnitude=(points[:,0]*points[:,0]+points[:,1]*points[:,1])+points[:,2]*points[:,2]
    valid=magnitude>np.float32(1e-3)
    result=np.zeros(count,dtype=np.int32)
    for j in range(1,count):
        d=points-points[result[j-1]]
        squared=(d[:,0]*d[:,0]+d[:,1]*d[:,1])+d[:,2]*d[:,2]
        distance=np.minimum(distance,squared)
        candidates=np.where(valid,distance,np.float32(-1))
        result[j]=order[np.argmax(candidates[order])] if np.any(valid) else 0
    return result
