"""Observation-driven 4DOF scan consumer; no simulator/catalog/truth imports."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from elesim_pilot.vision.multiview import ScanMap, ViewPlanner, transform, voxel_points, refine_joint_scan
from elesim_pilot.vision.partial_cloud import ArmCameraModel, SensorObservation, reconstruct_view, VoxelCloud


def object_mask(observation):
    hsv=cv2.cvtColor(observation.color_bgr,cv2.COLOR_BGR2HSV)
    red=(((hsv[:,:,0]<=12)|(hsv[:,:,0]>=168))&(hsv[:,:,1]>=100)&(hsv[:,:,2]>=60)).astype(np.uint8)
    count,labels,stats,_=cv2.connectedComponentsWithStats(red,connectivity=8)
    if count<=1:return np.zeros(red.shape,bool)
    chosen=1+np.argmax(stats[1:,cv2.CC_STAT_AREA])
    return cv2.erode((labels==chosen).astype(np.uint8),np.ones((3,3),np.uint8))>0


def scan_episode(capture, model, planner, output, *, episode=0, max_views=20):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    mapper=ScanMap(max_views=max_views)
    q=np.array([-.132063,.382158,-.33934,.558898]);tried=[];accepted_poses=[];rows=[];first=None;raw_union=[];choices=[]
    first_q=None;closing=False;frames=[];frame_rows=[];joint_fit=None
    stop='view_budget';stalls=0
    for view_index in range(max_views):
        if first_q is not None and view_index==max_views-1:
            q=first_q.copy();closing=True
        observation=capture(episode,view_index,q)
        nominal=model.camera_pose(observation.encoder_q)
        mask=object_mask(observation)
        partial=reconstruct_view(observation,mask,nominal,max_points=20000)
        identity=(observation.source_id,observation.source_boot_id,observation.calibration_id)
        result=mapper.add(partial.points_camera,nominal,identity=identity,seq=observation.seq)
        if len(partial.points_camera)>=60 and (mapper.identity is None or identity==mapper.identity):
            frames.append((np.asarray(observation.encoder_q),partial.points_camera.copy()))
            frame_rows.append(view_index)
        accepted=bool(result['accepted'])
        if accepted:
            if first is None:
                first=partial.points_camera.copy();first_q=np.asarray(observation.encoder_q).copy()
            raw_union.append(transform(partial.points_camera,np.linalg.inv(mapper.reference_pose)@nominal))
            accepted_poses.append(mapper.reference_pose@result['camera_in_reference'])
            stalls=stalls+1 if result['new_fraction']<.03 else 0
        row={'view':view_index,'seq':observation.seq,'q':list(observation.encoder_q),'accepted':accepted,
             'object_points':len(partial.points_camera),'new_voxels':result['new_voxels'],
             **{k:v for k,v in result.items() if k not in ('aligned_points','camera_in_reference','transform','new_voxels','accepted')}}
        rows.append(row);tried.append(q.copy())
        np.savez_compressed(output/f'view-{view_index:03d}.npz',color_bgr=observation.color_bgr,mask=mask,
            observed_camera=partial.points_camera,nominal_pose=nominal,
            camera_in_reference=result.get('camera_in_reference',np.empty((0,4))),
            aligned_ref=result.get('aligned_points',np.empty((0,3))))
        print(json.dumps({'episode':episode,'view':view_index,'accepted':accepted,'reason':result['reason'],
                          'map_points':len(mapper.points())}),flush=True)
        if closing:
            stop='revisited_first_view';break
        if first is None:
            if view_index>=2:stop='target_not_acquired';break
            q=np.array([-.13,0.,-.05,.34]) if view_index==0 else np.array([-.10,.15,-.05,.30])
            continue
        # Several low-yield views do not establish completeness. Continue
        # through reachable proposals until the view budget or angular
        # coverage is exhausted, then revisit the anchor.
        base_points=transform(mapper.points(),mapper.reference_pose)
        if len(frames)>=4 and view_index%4==3:
            fitted=refine_joint_scan(model,frames)
            if fitted['accepted']:
                joint_fit=fitted
                base_points=transform(fitted['points'],mapper.reference_pose)
        choice=planner.choose(base_points,q,accepted_poses,tried)
        if choice is None:
            closing=True;q=first_q.copy();continue
        choices.append({k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in choice.items()})
        q=choice['q']
    if len(frames)>=4:
        joint_fit=refine_joint_scan(model,frames)
    fused=mapper.points();accepted_count=mapper.accepted_views
    if joint_fit is not None and joint_fit['accepted']:
        fused=joint_fit['points'];accepted_count=len(joint_fit['used_frames'])
        used=set(joint_fit['used_frames']);raw_union=[]
        for i,(q,points) in enumerate(frames):
            row_index=frame_rows[i];rows[row_index]['incremental_accepted']=rows[row_index]['accepted']
            rows[row_index]['accepted']=i in used
            if i not in used:continue
            path=output/f'view-{row_index:03d}.npz'
            with np.load(path,allow_pickle=False) as packet:values=dict(packet)
            values['camera_in_reference']=joint_fit['camera_in_reference'][i]
            values['aligned_ref']=transform(points,joint_fit['camera_in_reference'][i])
            np.savez_compressed(path,**values)
            raw_union.append(transform(points,np.linalg.inv(mapper.reference_pose)@model.camera_pose(q)))
    fk_cloud=VoxelCloud(voxel_m=.003,max_voxels=50000)
    for points in raw_union:fk_cloud.add(points)
    np.savez_compressed(output/'cloud.npz',first=np.empty((0,3)) if first is None else first,
        fused=fused,icp_control=mapper.points(),fk_only=fk_cloud.points(),
        reference_nominal=np.empty((0,4)) if mapper.reference_pose is None else mapper.reference_pose)
    summary={'episode':episode,'views_attempted':len(rows),'views_accepted':accepted_count,
        'stop_reason':stop,'rows':rows,'view_choices':choices,'reference_frame':'first accepted optical frame',
        'geometry_complete_claimed':False,'segmentation':'red largest component fixture; no general object detector',
        'registration':'local point-to-plane preview; coherent shared bend-offset batch fit for final fusion',
        'joint_fit':None if joint_fit is None else {k:v for k,v in joint_fit.items() if k not in ('points','camera_in_reference')},
        'fusion_voxel_m':.003,'source_identity':mapper.identity,'truth_access':False}
    (output/'result.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('requests','sensors','output'):parser.add_argument(name,type=Path)
    parser.add_argument('--arm-model',type=Path,required=True);parser.add_argument('--hand-eye',type=Path,required=True)
    parser.add_argument('--episodes',type=int,default=6);parser.add_argument('--max-views',type=int,default=20)
    parser.add_argument('--candidates',type=int,default=12000)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    base=np.eye(4);base[2,3]=.8
    model=ArmCameraModel(args.arm_model,args.hand_eye,base_world_transform=base)
    planner=ViewPlanner(model,candidates=args.candidates)
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
    episodes=[]
    try:
        for episode in range(args.episodes):
            episodes.append(scan_episode(capture,model,planner,args.output/f'episode-{episode:02d}',episode=episode,max_views=args.max_views))
    finally:
        (args.requests/'stop.json').write_text(json.dumps({'stop':True})+'\n')
    result={'complete':True,'episodes':episodes,'input_fields':'RGB-D, intrinsics, measured q and source/time identity only',
            'frame':'first optical frame per episode; nominal world placement is not validated',
            'scope':'static object/arm base; kinematic acquisition experiment, no runtime motion wiring'}
    (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
