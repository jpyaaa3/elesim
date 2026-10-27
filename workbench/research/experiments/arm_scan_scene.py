"""Private kinematic Genesis/analytic 4DOF RGB-D producer for scan requests.

This is a static-object camera acquisition experiment. Joint-limit-respecting
waypoints move the actual URDF, but dynamics/collision-free motion are not proved.
Only sensors/ is mounted into the scan consumer. The producer never imports Pilot.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
import uuid

import numpy as np
from scipy.spatial.transform import Rotation

from .vision_arm_scene import ArmUrdf


def fixtures(seed=835):
    rng = np.random.default_rng(seed)
    values = []
    for index in range(6):
        kind = 'cylinder' if index % 3 == 2 else 'box'
        size = rng.uniform([.09, .09, .10], [.13, .14, .15])
        if index % 3 == 1:
            size[0] *= .55
        center = np.array([.30, .005, .52+size[2]/2]) + rng.uniform([-.015,-.012,0],[.015,.012,0])
        error = np.zeros(4) if index < 3 else np.array([.003, np.radians(3.), np.radians(.8), np.radians(-.6)])
        values.append({'kind': kind, 'center': center, 'size': size,
                       'rotation': Rotation.from_euler('z', rng.uniform(-.7, .7)).as_matrix(),
                       'joint_error': error, 'noise_m': .0006 if index < 3 else .0015,
                       'dropout': 0. if index < 3 else .1})
    return values


def analytic_rgbd(pose, intrinsics, shape):
    fx, fy, cx, cy, width, height = intrinsics
    v, u = np.mgrid[:int(height), :int(width)]
    rays = np.stack(((u-cx)/fx, (v-cy)/fy, np.ones_like(u)), axis=-1) @ pose[:3, :3].T
    origin = pose[:3, 3]
    with np.errstate(divide='ignore', invalid='ignore'):
        depth = (.52-origin[2])/rays[...,2]
    depth = np.where(depth > .05, depth, 100.)
    color = np.full((*depth.shape, 3), 150, dtype=np.uint8)
    local_origin = (origin-shape['center']) @ shape['rotation']
    d = rays @ shape['rotation']
    size = shape['size']
    with np.errstate(divide='ignore', invalid='ignore'):
        if shape['kind'] == 'box':
            a, b = (-size/2-local_origin)/d, (size/2-local_origin)/d
            near, far = np.minimum(a,b).max(-1), np.maximum(a,b).min(-1)
            hit = (far >= near) & (near > .05) & (near < depth)
        else:
            radius, half = size[0]/2, size[2]/2
            aa = np.sum(d[...,:2]**2,axis=-1)
            bb = 2*np.sum(d[...,:2]*local_origin[:2],axis=-1)
            cc = np.sum(local_origin[:2]**2)-radius**2
            disc = bb**2-4*aa*cc
            side = (-bb-np.sqrt(np.maximum(disc,0)))/(2*aa)
            valid = (disc >= 0) & (np.abs(local_origin[2]+side*d[...,2]) <= half)
            near = np.where(valid & (side > .05), side, 100.)
            for z in (-half,half):
                cap = (z-local_origin[2])/d[...,2]
                radial = local_origin[:2]+d[...,:2]*cap[...,None]
                valid = (np.sum(radial**2,axis=-1) <= radius**2) & (cap > .05)
                near = np.minimum(near,np.where(valid,cap,100.))
            hit = (near > .05) & (near < depth)
    depth[hit] = near[hit]
    color[hit] = [12,24,230]
    return color, depth.astype(np.float32)


def serve(requests, output, *, data_root, backend='genesis', width=480, height=360, seed=835, timeout=900):
    requests, output, data_root = map(Path,(requests,output,data_root))
    sensors, truth = output/'sensors', output/'truth'
    from workbench.research.support.output import fresh_directory
    fresh_directory(sensors);truth.mkdir()
    arm = ArmUrdf(data_root/'models/assemblies/zed-mini/arm.urdf', data_root/'calibration/cameras/zed_mini.hand_eye.json',base_height_m=.8)
    shapes = fixtures(seed)
    context=json.loads((data_root/'models/arm/default.json').read_text())['context']
    configured=context['limit']['value'];bend=np.radians(configured['bend_deg'])
    limits=np.array([[context['linear_min_m'],np.radians(configured['roll_min_deg']),-bend,-bend],
                     [context['linear_max_m'],np.radians(configured['roll_max_deg']),bend,bend]])
    scene = entity = camera = observer = None
    focal = height/2/np.tan(np.radians(30))
    intr = np.array([focal,focal,(width-1)/2,(height-1)/2,width,height])
    objects = [];indices = names = None
    if backend == 'genesis':
        import genesis as gs
        from elesim_sim.vision.render_device import check_camera_renderer
        from elesim_sim.vision.sim_camera.mount import intrinsics_from_fov
        gs.init(backend=gs.gpu,logging_level='warning')
        scene = gs.Scene(show_viewer=False,
            sim_options=gs.options.SimOptions(dt=.01,substeps=1,gravity=(0,0,0)),
            rigid_options=gs.options.RigidOptions(enable_collision=False,enable_self_collision=False,enable_joint_limit=False,disable_constraint=True),
            vis_options=gs.options.VisOptions(ambient_light=(.7,.7,.7)))
        scene.add_entity(gs.morphs.Plane(),surface=gs.surfaces.Default(color=(.5,.5,.5)))
        scene.add_entity(gs.morphs.Box(size=(.9,1.,.04),pos=(.3,0,.5),fixed=True),surface=gs.surfaces.Default(color=(.6,.6,.6)))
        for i,shape in enumerate(shapes):
            kwargs={'pos':(0,0,10.+i),'fixed':True}
            morph = (gs.morphs.Cylinder(radius=float(shape['size'][0]/2),height=float(shape['size'][2]),**kwargs)
                     if shape['kind']=='cylinder' else gs.morphs.Box(size=tuple(shape['size']),quat=tuple(Rotation.from_matrix(shape['rotation']).as_quat()[[3,0,1,2]]),**kwargs))
            objects.append(scene.add_entity(morph,surface=gs.surfaces.Default(color=(.9,.08,.04))))
        entity = scene.add_entity(gs.morphs.URDF(file=str(arm.path),fixed=True,pos=(0,0,.8),merge_fixed_links=False))
        camera = scene.add_camera(res=(width,height),fov=60,GUI=False,debug=False)
        observer = scene.add_camera(res=(800,600),pos=(1.1,-1.,1.35),lookat=(.25,0,.7),fov=45,GUI=False,debug=False)
        scene.build()
        check_camera_renderer(scene.visualizer.rasterizer._renderer,use_gpu=True,label='arm-scan')
        k = intrinsics_from_fov(width=width,height=height,fov_deg=60)
        intr[:4] = [k.fx,k.fy,k.cx,k.cy]
        names = list(arm.joint_values([0,0,0,0]))
        indices = [int(np.asarray(entity.get_joint(name).dofs_idx_local).reshape(-1)[0]) for name in names]
    first_poses = {};boots={};records=[];last_q=np.zeros(4);last_command=np.zeros(4);last_episode=None;fk_error=0.
    began=time.monotonic();sequence=0
    try:
        while time.monotonic()-began < timeout:
            path=requests/f'request-{sequence:05d}.json'
            if not path.is_file():
                if (requests/'stop.json').is_file():break
                time.sleep(.02);continue
            if path.stat().st_size>4096:raise ValueError('excessive scan request')
            req=json.loads(path.read_text())
            if set(req)!={'seq','episode','view','q'} or req['seq']!=sequence:raise ValueError('invalid scan request')
            episode=int(req['episode']);q=np.asarray(req['q'],float)
            if not 0<=episode<len(shapes) or q.shape!=(4,) or not np.all(np.isfinite(q)) or np.any(q<limits[0]-1e-9) or np.any(q>limits[1]+1e-9):raise ValueError('scan joint request exceeds limits')
            shape=shapes[episode];actual_q=np.clip(q+shape['joint_error'],*limits)
            actual=arm.camera_pose(actual_q)
            if camera is None:
                color,depth=analytic_rgbd(actual,intr,shape)
            else:
                import cv2
                import torch
                from elesim_sim.vision.sim_camera.mount import _render_camera
                if last_episode!=episode:
                    for i,obj in enumerate(objects):obj.set_pos(tuple(shape['center']) if i==episode else (0,0,10.+i))
                # Kinematic interpolation exercises all URDF joints, without
                # claiming force control or continuous collision-free motion.
                for fraction in np.linspace(0,1,8):
                    values=arm.joint_values(last_q*(1-fraction)+actual_q*fraction)
                    entity.set_dofs_position(torch.as_tensor([values[n] for n in names],device=gs.device,dtype=gs.tc_float),dofs_idx_local=indices)
                link=entity.get_link(arm.parent)
                live=np.eye(4)
                live[:3,3]=link.get_pos().detach().cpu().numpy().reshape(3)
                quat=link.get_quat().detach().cpu().numpy().reshape(4)
                live[:3,:3]=Rotation.from_quat(quat[[1,2,3,0]]).as_matrix()
                discrepancy=float(np.abs(live-arm.link_poses(actual_q)[arm.parent]).max())
                fk_error=max(fk_error,discrepancy)
                if discrepancy>2e-5:raise RuntimeError('Genesis arm FK mismatch')
                actual=live@arm.mount
                camera.set_pose(transform=actual@np.diag([1,-1,-1,1]))
                rgb,depth,_,_=_render_camera(camera,rgb=True,depth=True,force_render=True)
                if hasattr(rgb,'cpu'):rgb=rgb.cpu().numpy()
                if hasattr(depth,'cpu'):depth=depth.cpu().numpy()
                color=np.ascontiguousarray(np.asarray(rgb)[...,:3][...,::-1],dtype=np.uint8)
                depth=np.asarray(depth,dtype=np.float32)
                if int(req['view']) in (0,3,7,11):
                    rgb,_,_,_=_render_camera(observer,rgb=True,depth=False,force_render=True)
                    if hasattr(rgb,'cpu'):rgb=rgb.cpu().numpy()
                    cv2.imwrite(str(truth/f'observer-{sequence:05d}.png'),np.asarray(rgb)[...,:3][...,::-1])
            boots.setdefault(episode,uuid.uuid4().hex)
            first_poses.setdefault(episode,actual.copy())
            rng=np.random.default_rng(seed+sequence*193)
            noisy=depth+rng.normal(0,shape['noise_m'],depth.shape).astype(np.float32)
            noisy[(depth<.05)|(depth>5)|~np.isfinite(depth)]=0
            noisy[rng.random(depth.shape)<shape['dropout']]=0
            filename=f'view-{sequence:05d}.npz';part=sensors/('pending-'+filename)
            np.savez_compressed(part,schema_version=np.array(1),color_bgr=color,depth_raw=noisy,
                depth_scale=np.array(1.),intrinsics=intr,seq=np.array(sequence),ts=np.array(1000.+sequence*.2),
                source_id=np.array('arm-scan-camera'),source_boot_id=np.array(boots[episode]),
                calibration_id=np.array('arm4-zed-v1'),encoder_q=q)
            np.savez_compressed(truth/filename,true_camera_pose=actual,nominal_camera_pose=arm.camera_pose(q),
                actual_q=actual_q,actual_path_start_q=last_q,command_path_start_q=last_command,
                object_center=shape['center'],object_size=shape['size'],object_rotation=shape['rotation'])
            part.replace(sensors/filename)
            records.append({'file':filename,'episode':episode,'view':int(req['view']),'q':q.tolist(),
                            'kind':shape['kind'],'noise_m':shape['noise_m'],'joint_error':shape['joint_error'].tolist()})
            last_q,last_episode=actual_q,episode
            last_command=q.copy()
            sequence+=1
            (truth/'progress.json').write_text(json.dumps({'frames':sequence,'episode':episode,'view':req['view']})+'\n')
    finally:
        if scene is not None:scene.destroy()
        manifest={'schema_version':1,'backend':backend,'records':records,'fk_max_abs':fk_error,
            'first_camera_pose':{str(k):v.tolist() for k,v in first_poses.items()},
            'scope':'Static objects, fixed bench arm base, interpolated kinematic poses; no actuator dynamics or collision safety',
            'shape_parameters':[{'kind':s['kind'],'center':s['center'].tolist(),'size':s['size'].tolist(),'rotation':s['rotation'].tolist()} for s in shapes]}
        (truth/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return {'frames':sequence,'backend':backend,'fk_max_abs':fk_error}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('requests',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--data-root',type=Path,required=True)
    parser.add_argument('--backend',choices=['analytic','genesis'],default='genesis')
    parser.add_argument('--width',type=int,default=480);parser.add_argument('--height',type=int,default=360)
    args=parser.parse_args()
    print(json.dumps(serve(args.requests,args.output,data_root=args.data_root,backend=args.backend,width=args.width,height=args.height)))


if __name__=='__main__':main()
