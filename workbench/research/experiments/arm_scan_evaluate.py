"""Independent scan evaluation in the declared first optical frame; no ICP fit.

Only this stage reads true object geometry and true first-camera extrinsics.
Those extrinsics convert coordinates for evaluation; no fitted alignment or
complete-object normalization is applied. Nominal world placement is separate.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from .adapointr_evaluate import box_distance, sample_box


def sample_shape(shape, count=24000):
    center,size,rotation=[np.asarray(shape[k]) for k in ('center','size','rotation')]
    if shape['kind']=='box':return sample_box(center,size,rotation,count=count)
    rng=np.random.default_rng(991)
    radius,height=size[0]/2,size[2]
    side_probability=2*np.pi*radius*height/(2*np.pi*radius*height+2*np.pi*radius**2)
    side=rng.random(count)<side_probability
    angle=rng.uniform(0,2*np.pi,count)
    radial=np.where(side,radius,radius*np.sqrt(rng.random(count)))
    z=np.where(side,rng.uniform(-height/2,height/2,count),rng.choice([-1,1],count)*height/2)
    local=np.column_stack([radial*np.cos(angle),radial*np.sin(angle),z])
    return local@rotation.T+center


def surface_distance(points, shape):
    center,size,rotation=[np.asarray(shape[k]) for k in ('center','size','rotation')]
    if shape['kind']=='box':return box_distance(points,center,size,rotation)
    local=(points-center)@rotation
    q=np.column_stack([np.linalg.norm(local[:,:2],axis=1)-size[0]/2,np.abs(local[:,2])-size[2]/2])
    return np.abs(np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(q.max(axis=1),0))


def cloud_metrics(points, shape):
    points=np.asarray(points,float)
    if len(points)<3:return {'surface_p95_mm':None,'recall_5mm':0.,'precision_5mm':0.,'points':len(points)}
    truth=sample_shape(shape)
    surface=surface_distance(points,shape)
    local=(points-np.asarray(shape['center']))@np.asarray(shape['rotation'])
    extent=np.percentile(local,99,axis=0)-np.percentile(local,1,axis=0)
    expected=np.asarray(shape['size']).copy()
    if shape['kind']=='cylinder':expected[1]=expected[0]
    return {'surface_p95_mm':float(np.percentile(surface,95)*1000),
        'recall_5mm':float(np.mean(cKDTree(points).query(truth)[0]<=.005)),
        'precision_5mm':float(np.mean(surface<=.005)),'points':len(points),
        'extent_mm':(extent*1000).tolist(),'dimension_error_mm':(np.abs(extent-expected)*1000).tolist()}


def collision_audit(truth_root, data_root):
    """Check true motion against spheres, with independent actual-URDF FK parity."""
    import hashlib
    from elesim_pilot.robot.arm.self_collision import ArmSelfCollision
    from .vision_arm_scene import ArmUrdf
    data_root=Path(data_root);truth_root=Path(truth_root)
    guard=ArmSelfCollision(data_root/'models/planning/zed-mini')
    assert hashlib.sha256((data_root/'models/assemblies/zed-mini/arm.urdf').read_bytes()).hexdigest()==guard.source_hashes['arm.urdf']
    reference=ArmUrdf(data_root/'models/assemblies/zed-mini/arm.urdf',data_root/'calibration/cameras/zed_mini.hand_eye.json',base_height_m=0)
    result=[];fk_error=0.
    for record in json.loads((truth_root/'manifest.json').read_text())['records']:
        with np.load(truth_root/record['file'],allow_pickle=False) as packet:
            start,end=packet['actual_path_start_q'],packet['actual_q']
        planned=guard.link_poses(end);actual=reference.link_poses(end)
        for name in guard.names:fk_error=max(fk_error,float(np.abs(planned[name]-actual[name]).max()))
        certificate=guard.certify_segment(start,end)
        result.append({'file':record['file'],**certificate})
    return {'accepted':bool(result and fk_error<1e-9 and all(r['accepted'] for r in result)),
            'segments':len(result),'rejected':sum(not r['accepted'] for r in result),'sphere_count':guard.sphere_count,
            'minimum_certified_margin_m':min((r['minimum_margin_m'] for r in result),default=0),
            'independent_fk_max_abs':fk_error,'records':result,
            'scope':'continuous actual joint segments against shipped arm/open-claw sphere model; excludes external objects'}


def evaluate(truth_root, scans, output, *, data_root=None):
    truth_root,scans,output=map(Path,(truth_root,scans,output))
    private=json.loads((truth_root/'manifest.json').read_text())
    recording=json.loads((scans/'result.json').read_text())
    episodes=[]
    for result in recording['episodes']:
        episode=result['episode'];folder=scans/f'episode-{episode:02d}'
        shape=private['shape_parameters'][episode]
        accepted=[v for v in result['rows'] if v['accepted']]
        row={'episode':episode,'kind':shape['kind'],'views_attempted':result['views_attempted'],
             'views_accepted':result['views_accepted'],'stop_reason':result['stop_reason']}
        row['additional_view_changes']=result.get('additional_view_changes',max(result['views_attempted']-1,0))
        if not accepted:
            episodes.append({**row,'accepted':False,'reason':'no_observation'});continue
        first_seq=accepted[0]['seq']
        with np.load(truth_root/f'view-{first_seq:05d}.npz',allow_pickle=False) as packet:
            first_true,first_nominal=packet['true_camera_pose'],packet['nominal_camera_pose']
        with np.load(folder/'cloud.npz',allow_pickle=False) as packet:
            for label in ('first','fused','fk_only'):
                local=packet[label]
                world=local@first_true[:3,:3].T+first_true[:3,3]
                row[label]=cloud_metrics(world,shape)
        row['coverage_gain']=row['fused']['recall_5mm']-row['first']['recall_5mm']
        row['nominal_anchor_translation_error_mm']=float(np.linalg.norm(first_nominal[:3,3]-first_true[:3,3])*1000)
        row['nominal_anchor_angle_error_deg']=float(np.degrees(Rotation.from_matrix(first_nominal[:3,:3].T@first_true[:3,:3]).magnitude()))
        directions=[]
        for view in accepted:
            with np.load(truth_root/f"view-{view['seq']:05d}.npz",allow_pickle=False) as packet:
                direction=packet['true_camera_pose'][:3,3]-shape['center']
                directions.append(direction/np.linalg.norm(direction))
        directions=np.asarray(directions)
        row['camera_direction_span_deg']=float(np.degrees(np.arccos(np.clip((directions@directions.T).min(),-1,1))))
        clean=episode<3
        surface_limit=4. if clean else 7.
        row['accepted']=bool(result['views_accepted']>=4 and row['coverage_gain']>=.12 and
            row['fused']['recall_5mm']>=.60 and row['fused']['surface_p95_mm']<=surface_limit and
            row['fused']['precision_5mm']>=.90 and max(row['fused']['dimension_error_mm'])<=12.)
        episodes.append(row)
    final={'complete':True,'accepted':bool(episodes and all(v['accepted'] for v in episodes)),
        'episodes':episodes,'genesis_fk_max_abs':private['fk_max_abs'],'backend':private['backend'],
        'criteria':{'accepted_views_min':4,'full_surface_recall_5mm_min':.60,'coverage_gain_min':.12,
            'surface_p95_mm_max_clean':4.,'surface_p95_mm_max_joint_error':7.,'precision_5mm_min':.90,
            'max_dimension_error_mm':12.},
        'evaluation_alignment':'none; use true first optical frame to express independent truth in the declared scan frame',
        'world_placement_accuracy_claimed':False,
        'scope':'six static box/cylinder fixtures, fixed arm base, red segmentation; unobservable backsides remain unknown'}
    final['collision_audit']=None if data_root is None else collision_audit(truth_root,data_root)
    if final['collision_audit'] is not None:final['accepted'] &= final['collision_audit']['accepted']
    output.write_text(json.dumps(final,indent=2)+'\n')
    return final


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('truth',type=Path);parser.add_argument('scans',type=Path);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--data-root',type=Path)
    args=parser.parse_args();print(json.dumps(evaluate(args.truth,args.scans,args.output,data_root=args.data_root)))


if __name__=='__main__':main()
