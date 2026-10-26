"""Evaluate arm-induced visual tilt independently of plane height or identity."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def angle_deg(first,second):
    return float(np.degrees(np.arccos(np.clip(np.dot(first,second),-1.,1.))))


def visible_object_normals(truth,predicted):
    """Evaluate held-out object top faces, never points used to fit the reference."""
    uv=predicted['pixels_uv']
    if len(uv)<40:return []
    u,v=uv[:,0],uv[:,1]
    z=truth['ideal_depth'][v,u];fx,fy,cx,cy=truth['true_intrinsics'][:4]
    ideal=np.column_stack(((u-cx)*z/fx,(v-cy)*z/fy,z))
    actual=truth['true_camera_pose']
    world=ideal @ actual[:3,:3].T + actual[:3,3]
    results=[]
    for object_id,(center,size,rotation) in enumerate(zip(truth['object_centers'],truth['object_sizes'],truth['object_rotations'])):
        local=(world-center) @ rotation
        top=(np.abs(local[:,2]-size[2]/2)<.002) & np.all(np.abs(local[:,:2])<size[:2]/2-.005,axis=1)
        if np.count_nonzero(top)<40 or np.min(np.ptp(local[top,:2],axis=0))<.02:continue
        normals=[]
        for key in ('points_before','points_after'):
            points=predicted[key][top]
            if not len(points):break
            centered=points-points.mean(axis=0)
            _,basis=np.linalg.eigh(centered.T @ centered/len(points))
            normal=basis[:,0]
            if normal[2]<0:normal=-normal
            normals.append(normal)
        if len(normals)==2:results.append((object_id,*normals))
    return results


def evaluate_tilt(truth_directory,predictions):
    truth_directory,predictions=Path(truth_directory),Path(predictions)
    truth=json.loads((truth_directory/'manifest.json').read_text())
    if truth['error_model']!='measured vs actual joints; camera mount fixed':
        raise ValueError('visual tilt acceptance requires actual joint errors and an exact camera mount')
    replay=json.loads((predictions/'result.json').read_text())
    if replay['correction_mode']!='orientation':raise ValueError('visual tilt acceptance forbids height correction')
    frames={row['name']:row for row in replay['frames']}
    expected={row['name'] for row in truth['frames']}
    if set(frames)!=expected:raise ValueError('missing or unexpected sensor results')
    last={row['case_id']:row for row in truth['frames'] if row['repetition']==2}
    rows=[];groups={};by_object={}
    for case in truth['cases']:
        name=last[case['case_id']]['name'];result=frames[name]
        group=groups.setdefault(case['condition'],{'cases':0,'corrected':0,'reasons':{},
            'before':[],'after':[],'translation':[],'object_before':[],'object_after':[],
            'cross_before':[],'cross_after':[]})
        group['cases']+=1
        reason=result['floor_reason'] or 'tracking'
        group['reasons'][reason]=group['reasons'].get(reason,0)+1
        row={'case_id':case['case_id'],'pose_id':case['pose_id'],'condition':case['condition'],
             'error_seed':case['error_seed'],'frame':name,'q_measured':case['q'],
             'q_actual':case['actual_q'],'joint_error':case['joint_offset'],
             'valid':result['floor_valid'],'reason':reason,'object_points':result['points']}
        with np.load(truth_directory/case['truth_file'],allow_pickle=False) as actual:
            nominal=np.asarray(result['predicted_pose']);true_pose=actual['true_camera_pose']
            before=angle_deg((nominal[:3,:3] @ true_pose[:3,:3].T) @ [0,0,1],[0,0,1])
            row['before_tilt_deg']=before
            if result['floor_valid']:
                corrected=np.asarray(result['corrected_pose'])
                after=angle_deg((corrected[:3,:3] @ true_pose[:3,:3].T) @ [0,0,1],[0,0,1])
                shift=float(np.linalg.norm(corrected[:3,3]-nominal[:3,3]))
                row.update(after_tilt_deg=after,camera_origin_shift_m=shift)
                group['corrected']+=1;group['before'].append(before);group['after'].append(after);group['translation'].append(shift)
                with np.load(predictions/name,allow_pickle=False) as cloud:
                    normals=visible_object_normals(actual,cloud)
                row['object_top_faces']=len(normals)
                for object_id,n_before,n_after in normals:
                    group['object_before'].append(angle_deg(n_before,[0,0,1]))
                    group['object_after'].append(angle_deg(n_after,[0,0,1]))
                    key=(case['condition'],case['boot'],object_id)
                    previous=by_object.setdefault(key,[])
                    for pose_id,b,a in previous:
                        if pose_id!=case['pose_id']:
                            group['cross_before'].append(angle_deg(b,n_before));group['cross_after'].append(angle_deg(a,n_after))
                    previous.append((case['pose_id'],n_before,n_after))
        rows.append(row)
    metrics={}
    for name,g in groups.items():
        metrics[name]={'cases':g['cases'],'corrected_cases':g['corrected'],
            'availability':g['corrected']/g['cases'],'reasons':g['reasons'],
            'camera_origin_shift_max_m':max(g['translation'],default=0.),
            'object_top_face_observations':len(g['object_after']),'cross_view_pairs':len(g['cross_after'])}
        for key,label in [('before','tilt_before'),('after','tilt_after'),('object_before','object_normal_before'),
                          ('object_after','object_normal_after'),('cross_before','cross_view_before'),('cross_after','cross_view_after')]:
            metrics[name][label+'_p95_deg']=float(np.percentile(g[key],95)) if g[key] else None
        metrics[name]['tilt_after_max_deg']=max(g['after'],default=None)
    # Availability is reported for every sampled pose. The angle criterion
    # applies only when a supported trusted direction was actually observed.
    sufficient=all(g['corrected']>=8 for g in groups.values())
    accurate=all(m['tilt_after_p95_deg'] is not None and m['tilt_after_p95_deg']<1. for m in metrics.values())
    bad=sum(row.get('after_tilt_deg',0)>3. for row in rows if row['valid'])
    fixed_origins=all(m['camera_origin_shift_max_m']==0. for m in metrics.values())
    qs=np.array([row['q'] for row in truth['cases'] if row['condition']=='clean' and row['error_seed']==0])
    return {'schema_version':1,'complete':True,'accepted':bool(sufficient and accurate and not bad and fixed_origins),
        'backend':truth['backend'],'cases':len(rows),'frames':len(frames),'unique_poses':len(qs),
        'q_min':qs.min(axis=0).tolist(),'q_max':qs.max(axis=0).tolist(),'metrics':metrics,
        'bad_direction_accepts_above_3deg':int(bad),'all_camera_origins_unchanged':fixed_origins,
        'camera_mount_perturbed':False,'genesis_reference_fk_max_abs':truth['genesis_reference_fk_max_abs'],
        'criteria':{'supported_case_tilt_p95_below_deg':1.,'bad_direction_above_deg':3.,'bad_direction_cases':0,
                    'minimum_supported_cases_per_condition':8,'camera_origin_shift_m':0.},
        'scope':'static imperfect arm, exact fixed camera mount, trusted horizontal surfaces at arbitrary heights; no full joint/position/yaw identification',
        'evaluation_alignment':'none','rows':rows}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('truth',type=Path);parser.add_argument('predictions',type=Path)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--report-only',action='store_true')
    args=parser.parse_args();result=evaluate_tilt(args.truth,args.predictions)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},allow_nan=False))
    if not args.report_only and not result['accepted']:raise SystemExit(1)


if __name__=='__main__':main()
