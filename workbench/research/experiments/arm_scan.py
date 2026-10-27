"""Observation-driven 4DOF scan consumer; no simulator/catalog/truth imports."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from elesim_pilot.vision.multiview import ScanMap, ViewPlanner, transform, refine_joint_scan
from elesim_pilot.vision.partial_cloud import ArmCameraModel, SensorObservation, reconstruct_view, VoxelCloud


def object_mask(observation):
    hsv=cv2.cvtColor(observation.color_bgr,cv2.COLOR_BGR2HSV)
    red=(((hsv[:,:,0]<=12)|(hsv[:,:,0]>=168))&(hsv[:,:,1]>=100)&(hsv[:,:,2]>=60)).astype(np.uint8)
    count,labels,stats,_=cv2.connectedComponentsWithStats(red,connectivity=8)
    if count<=1:return np.zeros(red.shape,bool)
    chosen=1+np.argmax(stats[1:,cv2.CC_STAT_AREA])
    return cv2.erode((labels==chosen).astype(np.uint8),np.ones((3,3),np.uint8))>0


def scan_episode(capture, model, planner, output, *, episode=0, max_views=12, start_q=None):
    if type(max_views) is not int or not 4<=max_views<=24:
        raise ValueError('scan view budget must be 4..24')
    safety=planner.collision_checker
    current=np.zeros(4) if start_q is None else np.asarray(start_q,float)
    if not safety.configuration_free(current):
        raise ValueError('unsafe initial arm configuration; scan will not move')
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    mapper=ScanMap(max_views=max_views)
    boots=[np.array(q) for q in ([-.132063,.382158,-.33934,.558898],[-.13,0.,-.05,.34],[-.10,.15,-.05,.30])]
    q=next((candidate for candidate in boots if safety.certify_segment(current,candidate)['accepted']),None)
    if q is None:raise ValueError('no collision-free acquisition pose')
    adaptive=planner.policy=='information'
    tried=[];accepted_poses=[];rows=[];frames=[];frame_rows=[];choices=[];raw_union=[]
    first=None;first_q=None;joint_fit=None;working=np.empty((0,3));stalls=0;last_sequence=-1
    stop='view_budget'
    for view_index in range(max_views):
        certificate=safety.certify_segment(current,q)
        if not certificate['accepted']:
            stop='unsafe_path_rejected';break
        observation=capture(episode,view_index,q)
        if not np.allclose(observation.encoder_q,q,atol=1e-9,rtol=0) or observation.seq<=last_sequence:
            raise ValueError('capture identity/encoder sequence mismatch')
        last_sequence=observation.seq;current=np.asarray(observation.encoder_q).copy()
        nominal=model.camera_pose(current)
        partial=reconstruct_view(observation,object_mask(observation),nominal,max_points=20000)
        identity=(observation.source_id,observation.source_boot_id,observation.calibration_id)
        if mapper.identity is not None and identity!=mapper.identity:
            stop='source_identity_changed';break
        result=mapper.add(partial.points_camera,nominal,identity=identity,seq=observation.seq)
        if len(partial.points_camera)>=60:
            frames.append((current.copy(),partial.points_camera.copy()));frame_rows.append(view_index)
        accepted=bool(result['accepted'])
        if accepted:
            if first is None:first=partial.points_camera.copy();first_q=current.copy()
            raw_union.append(transform(partial.points_camera,np.linalg.inv(mapper.reference_pose)@nominal))
            accepted_poses.append(mapper.reference_pose@result['camera_in_reference'])
        row={'view':view_index,'seq':observation.seq,'q':current.tolist(),'accepted':accepted,
             'object_points':len(partial.points_camera),'path_certificate':certificate,
             **{k:v for k,v in result.items() if k not in ('aligned_points','camera_in_reference','transform','accepted')}}
        rows.append(row);tried.append(q.copy())
        np.savez_compressed(output/f'view-{view_index:03d}.npz',color_bgr=observation.color_bgr,
            mask=object_mask(observation),observed_camera=partial.points_camera,nominal_pose=nominal,
            camera_in_reference=result.get('camera_in_reference',np.empty((0,4))),aligned_ref=result.get('aligned_points',np.empty((0,3))))
        if first is None:
            if len(tried)>=len(boots):stop='target_not_acquired';break
            q=next((candidate for candidate in boots if not any(np.allclose(candidate,t) for t in tried)
                    and safety.certify_segment(current,candidate)['accepted']),None)
            if q is None:stop='no_safe_acquisition_pose';break
            continue
        previous=working
        working=mapper.points()
        if len(frames)>=4:
            fitted=refine_joint_scan(model,frames)
            if fitted['accepted']:
                joint_fit=fitted;working=fitted['points']
                accepted_poses=[mapper.reference_pose@fitted['camera_in_reference'][i] for i in fitted['used_frames']]
        if len(previous) and len(working):
            from scipy.spatial import cKDTree
            fresh=float(np.mean(cKDTree(previous).query(working)[0]>.006))
            extent_change=float(np.max(np.abs(np.percentile(working,[2,98],axis=0)-np.percentile(previous,[2,98],axis=0))))
            stalls=stalls+1 if fresh<.06 and extent_change<.003 else 0
            row.update(new_geometry_fraction=fresh,extent_change_m=extent_change)
        print(json.dumps({'episode':episode,'view':view_index,'accepted':accepted,'reason':result['reason'],
                          'map_points':len(working),'stalls':stalls}),flush=True)
        if view_index==max_views-1:break
        choice=planner.choose(transform(working,mapper.reference_pose),current,accepted_poses,tried)
        if choice is None:
            stop='no_safe_distinct_view';break
        row['next_expected_visibility_gain']=choice['expected_visibility_gain']
        if adaptive and joint_fit is not None and len(joint_fit['used_frames'])>=4 and stalls>=1 and choice['expected_visibility_gain']<.015:
            stop='marginal_information_saturated';break
        choices.append({k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in choice.items()})
        q=choice['q']
        # Only the comparison policy preserves the old closing capture. The
        # ordinary information policy does not spend a view revisiting home.
        if not adaptive and view_index==max_views-2 and safety.certify_segment(current,first_q)['accepted']:
            q=first_q.copy()
    joint_fit=refine_joint_scan(model,frames) if len(frames)>=4 else None
    fused=mapper.points();accepted_count=mapper.accepted_views
    if joint_fit is not None and joint_fit['accepted']:
        fused=joint_fit['points'];accepted_count=len(joint_fit['used_frames']);used=set(joint_fit['used_frames']);raw_union=[]
        for i,(q,points) in enumerate(frames):
            row_index=frame_rows[i];rows[row_index]['incremental_accepted']=rows[row_index]['accepted'];rows[row_index]['accepted']=i in used
            if i not in used:continue
            path=output/f'view-{row_index:03d}.npz'
            with np.load(path,allow_pickle=False) as packet:values=dict(packet)
            values['camera_in_reference']=joint_fit['camera_in_reference'][i];values['aligned_ref']=transform(points,joint_fit['camera_in_reference'][i])
            np.savez_compressed(path,**values)
            raw_union.append(transform(points,np.linalg.inv(mapper.reference_pose)@model.camera_pose(q)))
    fk_cloud=VoxelCloud(voxel_m=.003,max_voxels=50000)
    for points in raw_union:fk_cloud.add(points)
    np.savez_compressed(output/'cloud.npz',first=np.empty((0,3)) if first is None else first,fused=fused,
        icp_control=mapper.points(),fk_only=fk_cloud.points(),reference_nominal=np.empty((0,4)) if mapper.reference_pose is None else mapper.reference_pose)
    summary={'episode':episode,'views_attempted':len(rows),'views_accepted':accepted_count,'additional_view_changes':max(len(rows)-1,0),
        'start_q':(np.zeros(4) if start_q is None else np.asarray(start_q)).tolist(),'last_q':current.tolist(),
        'stop_reason':stop,'rows':rows,'view_choices':choices,'view_policy':planner.policy,'reference_frame':'first accepted optical frame',
        'geometry_complete_claimed':False,'segmentation':'red largest component fixture; no general object detector',
        'registration':'local point-to-plane preview; coherent shared bend-offset batch fit for final fusion',
        'joint_fit':None if joint_fit is None else {k:v for k,v in joint_fit.items() if k not in ('points','camera_in_reference')},
        'self_collision':{'sphere_count':safety.sphere_count,'margin_m':safety.margin_m,'joint_uncertainty':safety.uncertainty.tolist(),
                          'all_motion_certified':all(r['path_certificate']['accepted'] for r in rows)},
        'fusion_voxel_m':.003,'source_identity':mapper.identity,'truth_access':False}
    (output/'result.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('requests','sensors','output'):parser.add_argument(name,type=Path)
    parser.add_argument('--arm-model',type=Path,required=True);parser.add_argument('--hand-eye',type=Path,required=True)
    parser.add_argument('--episodes',type=int,default=6);parser.add_argument('--max-views',type=int,default=12)
    parser.add_argument('--collision-model',type=Path,required=True)
    parser.add_argument('--view-policy',choices=['information','angular'],default='information')
    parser.add_argument('--candidates',type=int,default=12000)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    base=np.eye(4);base[2,3]=.8
    model=ArmCameraModel(args.arm_model,args.hand_eye,base_world_transform=base)
    from elesim_pilot.robot.arm.self_collision import ArmSelfCollision
    safety=ArmSelfCollision(args.collision_model,joint_uncertainty=[.004,.06,.018,.018])
    planner=ViewPlanner(model,collision_checker=safety,candidates=args.candidates,policy=args.view_policy)
    sequence=0
    def capture(episode,view,q):
        nonlocal sequence
        req={'seq':sequence,'episode':episode,'view':view,'q':np.asarray(q).tolist()}
        path=args.requests/f'request-{sequence:05d}.json';tmp=path.with_suffix('.tmp')
        tmp.write_text(json.dumps(req)+'\n');tmp.replace(path)
        sensor=args.sensors/f'view-{sequence:05d}.npz';began=time.monotonic()
        while not sensor.is_file():
            if time.monotonic()-began>90:raise TimeoutError('scan sensor capture timed out')
            time.sleep(.02)
        observation=SensorObservation.load(sensor)
        if observation.seq!=sequence or not np.allclose(observation.encoder_q,q,atol=1e-9,rtol=0):
            raise ValueError('capture/encoder response does not match request')
        sequence+=1
        return observation
    episodes=[];current=np.zeros(4)
    try:
        for episode in range(args.episodes):
            result=scan_episode(capture,model,planner,args.output/f'episode-{episode:02d}',episode=episode,max_views=args.max_views,start_q=current)
            episodes.append(result);current=np.array(result['last_q'])
    finally:
        (args.requests/'stop.json').write_text(json.dumps({'stop':True})+'\n')
    result={'complete':True,'episodes':episodes,'input_fields':'RGB-D, intrinsics, measured q and source/time identity only',
            'frame':'first optical frame per episode; nominal world placement is not validated',
            'scope':'static object/arm base; kinematic acquisition experiment, no runtime motion wiring'}
    (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
