"""Independent completion quality against private Genesis box geometry.

Truth is read only here, after sensor-only inference. No ICP, oracle pose or
ground-truth normalization is used by the input/preprocessing/model stages.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def box_distance(points,center,size,rotation):
    q=np.abs((np.asarray(points)-center)@rotation)-size/2
    signed=np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(np.max(q,axis=1),0)
    return np.abs(signed)


def sample_box(center,size,rotation,*,count=12000):
    rng=np.random.default_rng(329)
    area=np.array([size[1]*size[2],size[0]*size[2],size[0]*size[1]])
    probability=np.repeat(area,2);probability/=probability.sum()
    faces=rng.choice(6,count,p=probability)
    local=(rng.random((count,3))-.5)*size
    local[np.arange(count),faces//2]=np.where(faces%2,1.,-1.)*size[faces//2]/2
    return local@rotation.T+center


def quality(observed,predicted,centers,sizes,rotations):
    costs=[np.median(box_distance(observed,c,s,r)) for c,s,r in zip(centers,sizes,rotations)]
    chosen=int(np.argmin(costs));center,size,rotation=centers[chosen],sizes[chosen],rotations[chosen]
    truth=sample_box(center,size,rotation)
    surface=box_distance(predicted,center,size,rotation)
    observed_surface=box_distance(observed,center,size,rotation)
    observed_to_pred=cKDTree(predicted).query(observed)[0]
    gt_to_observed=cKDTree(observed).query(truth)[0]
    gt_to_pred=cKDTree(predicted).query(truth)[0]
    threshold=.01
    precision=float(np.mean(surface<=threshold));recall=float(np.mean(gt_to_pred<=threshold))
    partial_precision=float(np.mean(observed_surface<=threshold))
    partial_recall=float(np.mean(gt_to_observed<=threshold))
    f1=lambda p,r:2*p*r/(p+r) if p+r else 0.
    hidden=gt_to_observed>threshold
    corners=np.array(np.meshgrid(*[[-1,1]]*3)).reshape(3,-1).T*size/2
    world_corners=corners@rotation.T+center
    size_error=np.abs(np.ptp(predicted,axis=0)-np.ptp(world_corners,axis=0))
    return {'assigned_box':chosen,'observed_single_box_fraction_10mm':partial_precision,
            'surface_p95_mm':float(np.percentile(surface,95)*1000),
            'observed_surface_p95_mm':float(np.percentile(observed_surface,95)*1000),
            'observed_to_prediction_p95_mm':float(np.percentile(observed_to_pred,95)*1000),
            'precision_10mm':precision,'recall_10mm':recall,'f1_10mm':f1(precision,recall),
            'partial_precision_10mm':partial_precision,'partial_recall_10mm':partial_recall,
            'partial_f1_10mm':f1(partial_precision,partial_recall),
            'hidden_recall_10mm':float(np.mean(gt_to_pred[hidden]<=threshold)) if np.any(hidden) else None,
            'aabb_size_error_mm':(size_error*1000).tolist()}


def evaluate(truth_root,predictions,output):
    truth_root,predictions,output=map(Path,(truth_root,predictions,output))
    recording=json.loads((predictions/'result.json').read_text())
    rows=[]
    for entry in recording['objects']:
        case_id=int(entry['capture'].split('-')[1])
        with np.load(truth_root/f'case-{case_id:04d}.npz',allow_pickle=False) as raw:
            centers,sizes,rotations=[raw[k] for k in ('object_centers','object_sizes','object_rotations')]
        with np.load(predictions/entry['output_file'],allow_pickle=False) as raw:
            observed,predicted=raw['observed'],raw['predicted']
        rows.append({**entry,**quality(observed,predicted,centers,sizes,rotations)})
    if not rows:raise ValueError('no inferred objects')
    metrics={key:float(np.median([r[key] for r in rows])) for key in (
        'surface_p95_mm','observed_surface_p95_mm','observed_to_prediction_p95_mm',
        'precision_10mm','recall_10mm','f1_10mm','partial_f1_10mm','hidden_recall_10mm',
        'forward_wall_ms_median')}
    criteria={'median_surface_p95_mm_max':20.,'median_observed_to_prediction_p95_mm_max':10.,
              'median_precision_10mm_min':.8,'median_hidden_recall_10mm_min':.5,
              'median_f1_improves_over_partial':True,'initial_forward_wall_ms_median_max':100.}
    geometry=(metrics['surface_p95_mm']<=20 and metrics['observed_to_prediction_p95_mm']<=10
              and metrics['precision_10mm']>=.8 and metrics['hidden_recall_10mm']>=.5
              and metrics['f1_10mm']>metrics['partial_f1_10mm'])
    result={'schema_version':1,'complete':True,'objects':rows,'metrics':metrics,'criteria':criteria,
            'geometry_accepted':bool(geometry),'initial_latency_accepted':bool(metrics['forward_wall_ms_median']<=100),
            'original_observations_preserved':all(r['observed_unchanged'] for r in rows),
            'mixed_or_misaligned_components':sum(r['observed_single_box_fraction_10mm']<.9 for r in rows),
            'alignment':'none','scope':'small independent box fixtures; partial centroid normalization; PCN checkpoint; not general-object or grasp acceptance'}
    output.write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('truth',type=Path);parser.add_argument('predictions',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=evaluate(args.truth,args.predictions,args.output)
    print(json.dumps({k:v for k,v in result.items() if k!='objects'}))


if __name__=='__main__':main()
