"""Sim ports for the mobile workflow. Inference never sends actuator commands."""
from concurrent.futures import ThreadPoolExecutor
import time
import uuid
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree

from elesim_pilot.reconstruction.hug import HugModel
from elesim_pilot.reconstruction.models import load_model
from elesim_pilot.reconstruction.contracts import Measurement,Task
from elesim_pilot.reconstruction.scan_collision import ScanCollision
from elesim_pilot.robot.arm.self_collision import ArmSelfCollision
from elesim_pilot.robot.arm.iklib.kinematics import _forward_link_tf,base_world_transform_from_context
from elesim_pilot.vision.partial_cloud import ArmCameraModel,SensorObservation,reconstruct_view
from elesim_pilot.vision.perception.sim_rendered_camera import SimRenderedCamera
from .contracts import Settings,TargetEstimate,planar,angle
from .feedback import Feedback
from .target_track import TargetTrack
from .local_map import LocalMap,transport_footprint
from .base_planner import BasePlanner,body_frame,solver_xy
from .base_executor import BaseExecutor


class SimPorts:
    def __init__(self,connection,service,state,scan,*,simulation_status,model_root,settings=None,clock=time.monotonic):
        self.connection,self.service,self.state,self.scan=connection,service,state,scan
        self.simulation_status=simulation_status;self.model_root=Path(model_root)
        self.s=settings or Settings();self.clock=clock;self.feedback=Feedback(clock)
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='mobile-plan')
        self.camera=None;self.model=None;self.grid=None;self.track=None;self.last_send=0.;self.operation=None
        self.generation=0;self.history=[];self.last_observation_base=None;self.scan_source=None
        self.last_command=None;self.stopped=False;self._summary={};self._latest={}
    def route(self):return self.scan.route()
    def context(self):
        s=self.simulation_status()
        if s is None:raise RuntimeError('no live Sim scene')
        return (self.route()[:2],s.epoch)
    def accept_telemetry(self,payload,**source):
        route=self.connection.motion_route()
        identity=(source['source_id'],source['source_boot_id'])
        if identity!=route[:2]:return
        if identity!=self.feedback.source:self._latest.clear()
        self.feedback.accept(payload,identity);self._latest.update(payload)
    def preflight(self,target_id):
        from elesim_protocol.scene_artifact import identifier
        identifier(target_id)
        if self.operation and self.operation.get('future') and not self.operation['future'].done():
            raise RuntimeError('previous mobile worker is still stopping')
        descriptor=next(d for d in self.connection.endpoints if d.get('endpoint_id')==self.route()[0])
        if 'simulation.timed_arm.v1' not in descriptor.get('capabilities',()):
            raise RuntimeError('Update Sim: simulation.timed_arm.v1 required')
        if getattr(self.service,'_go2_arm_mount',None) is None:raise RuntimeError('mobile task needs a mounted Go2 arm')
        if self.scan.active or self.scan._busy():
            raise RuntimeError('stop competing arm workflows')
        self.guard()
        self.model=HugModel(self.model_root);self.model.validate_robot(self.scan.model_dir/'arm.urdf')
        self.model.validate_physics(self._latest.get('physics_profile'))
        snapshot=self.feedback.snapshot(self.route()[:2]);q=snapshot['joint']['q']
        if np.any(abs(q-self.model.meta['home'])>[.001,.015,.015,.015]):raise RuntimeError('Start mobile task at measured Home')
        self.target_id=target_id;self.stopped=False
        self.grid=None;self.track=None;self.history=[];self.last_observation_base=None
        self.scan_source=None;self._summary={};self.operation=None;self.last_frame_seq=-1
        if hasattr(self,'camera_cache'):del self.camera_cache
        if self.camera is not None:self.camera.stop();self.camera=None
    def guard(self):
        host=self.connection.state_sink.get_state();s=self.simulation_status()
        self.feedback.snapshot(self.route()[:2],self.s.feedback_timeout_s)
        if s is None or s.paused or not host.connected or not host.torque_enabled or host.claw_closed is not False:
            raise RuntimeError('running Sim, torque ON and open claws required')
        if host.safety_fault or not host.reply_ok:raise RuntimeError(host.safety_fault or host.reply_reason or 'Sim rejected command')
        b=self.feedback.bases[-1]
        if max(abs(b['rpy'][:2]))>.07:raise RuntimeError('Go2 body tilt outside flat-floor profile')
        if self._latest.get('go2_controller_fault'):raise RuntimeError(self._latest['go2_controller_fault'])
        if self.model is not None and self.operation is not None and not self.stopped:
            self.model.validate_physics(self._latest.get('physics_profile'))
    def summary(self):return dict(self._summary)
    def _send(self,grant,ownership,**body):
        channels=set()
        if 'q' in body or 'timed_arm' in body:channels.add('arm')
        if 'go2_vel' in body:channels.add('base')
        if not channels:raise ValueError('mobile command has no motion channel')
        for channel in channels:
            ownership.require(grant,self.route(),channel)
        self.connection.send_workflow_motion(dict(command='target',source='mobile_manipulation',**body),self.route())
    def _initialize(self):
        if self.grid is not None:return
        host=self.connection.state_sink.get_state();context=self.service._ik_context_for_host(host)
        self.base=base_world_transform_from_context(context)
        self.arm=ArmSelfCollision(self.scan.model_dir,joint_uncertainty=[.001,.015,.015,.015])
        self.collision=ScanCollision(self.arm,self.base)
        self.camera_model=ArmCameraModel(self.scan.model_dir.parents[1]/'arm/default.json',self.scan.hand_eye,
            base_world_transform=self.base)
        snap=self.feedback.snapshot(self.route()[:2]);b=snap['base'];body=self._measured_body(snap)
        node_pos,node_rot=_forward_link_tf(context,[0.,0.,0.,0.])['node0']
        node=np.eye(4);node[:3,3]=node_pos;node[:3,:3]=node_rot
        self.node_in_body=np.linalg.inv(body)@node;self.arm_in_body=np.linalg.inv(body)@self.base
        self.grid=LocalMap();self.track=TargetTrack(self.target_id,self.s.min_position_error_m)
        # Conservative 3D spheres projected with the actual transport posture,
        # plus GO2 body/leg sweep envelope from this flat-floor task profile.
        home=np.asarray(self.model.meta['home']);centers=self.collision._centers(home)
        centers=(centers-b['pos'])@body[:3,:3]
        radii=self.collision._radii
        self.footprint=transport_footprint(centers,radii)
        self.planner=BasePlanner(self.model,self.grid,self.footprint,self.node_in_body,b['pos'][2],self.s)
        self.planner.roll_pitch=b['rpy'][:2].copy()
        # The occupied starting stance is the only unobserved seed: the flat
        # Sim profile has just verified stationary measured standing support.
        t=planar(snap['pose']);seed=self.footprint@t[:2,:2].T+t[:2,3]
        self.grid.free.update(self.grid.cell(p) for p in seed)
        self.last_command=home.copy();self.history=[home.copy()]
    def _measured_body(self,snapshot):
        b=snapshot['base']
        return body_frame(snapshot['pose'],b['pos'][2],b['rpy'][:2])
    def _grasp_goal(self):
        snapshot=self.feedback.snapshot(self.route()[:2])
        self.planner.body_z=snapshot['base']['pos'][2]
        self.planner.roll_pitch=snapshot['base']['rpy'][:2].copy()
        return dict(goal=self.planner.grasp(self.track.estimate,snapshot['pose']))
    def _capture(self):
        self._initialize()
        if self.camera is None:
            descriptor=next(d for d in self.connection.endpoints if d['endpoint_id']==self.route()[0])
            stream=descriptor.get('streams',{}).get('rgbd',{})
            if stream.get('format')!='encoded-rgbd-v1':raise RuntimeError('encoded RGB-D required')
            self.camera=SimRenderedCamera(topic=stream['endpoint'],timeout_ms=100,
                endpoint_id=self.connection.pilot_id+'-mobile',dds_settings=self.connection.dds_settings,
                expected_source_id=self.route()[0],expected_boot_id=self.route()[1],wire_format=stream['format'])
            self.camera.start()
        deadline=self.clock()+5.
        while self.clock()<deadline and not self.stopped:
            try:frame=self.camera.capture(retries=1,retry_sleep_s=0.)
            except RuntimeError:continue
            if frame.arm_q is None or frame.seq<=self.last_frame_seq:continue
            try:b=self.feedback.associate(frame,self.route()[:2])
            except RuntimeError:continue
            current=self.feedback.snapshot(self.route()[:2])
            age=current['joint']['stamp']-frame.ts
            if not -.05<=age<=.5:continue
            if self.scan_source is not None and (frame.source_id,frame.source_boot_id,frame.calibration_id)!=self.scan_source:
                raise RuntimeError('camera source/calibration changed')
            self.scan_source=(frame.source_id,frame.source_boot_id,frame.calibration_id)
            mount=self.service._go2_arm_mount
            base=mount.arm_base_world_transform(b['pos'],b['rpy'])
            model=ArmCameraModel(self.scan.model_dir.parents[1]/'arm/default.json',self.scan.hand_eye,base_world_transform=base)
            pose=model.camera_pose(frame.arm_q)
            sensor=SensorObservation(frame.color_bgr,frame.depth_raw,frame.depth_scale,frame.intrinsics,
                frame.seq,frame.ts,frame.source_id,frame.source_boot_id,frame.calibration_id,tuple(frame.arm_q))
            self.last_frame_seq=frame.seq
            return Measurement(sensor,base_world_transform=base),pose,base
        raise RuntimeError('no fresh stationary RGB-D capture')
    def _observe(self,allow_prediction=False):
        generation=self.generation
        measurement,pose,base=self._capture()
        if self.stopped or generation!=self.generation:raise RuntimeError('observation cancelled')
        config=yaml.safe_load(self.scan.models_path.read_text())
        segmenter=load_model('segmenter',config['models']['segmenter'],self.scan.models_path.parent).implementation
        masks=segmenter.segment(measurement,Task('mobile',self.target_id))
        # Map every observed surface, removing the arm's own conservative volume.
        self.collision.base=base.copy()
        full=reconstruct_view(measurement.sensor,np.ones(measurement.sensor.depth_raw.shape,bool),pose,max_points=6000).points_before
        centers=self.collision._centers(measurement.sensor.encoder_q)
        kept=[]
        for start in range(0,len(full),256):
            block=full[start:start+256]
            gaps=np.linalg.norm(block[:,None]-centers[None],axis=2)-self.collision._radii
            kept.append(block[np.all(gaps>.015,axis=1)])
        environment=np.concatenate(kept) if kept else np.empty((0,3))
        snap=self.feedback.snapshot(self.route()[:2]);b=snap['base']
        body_local=(environment-b['pos'])@Rotation.from_euler('xyz',b['rpy']).as_matrix()
        self_body=(abs(body_local[:,0])<.4)&(abs(body_local[:,1])<.27)&(body_local[:,2]>-.4)&(body_local[:,2]<.25)
        environment=environment[~self_body]
        self.collision.cloud.add(environment)
        fused=self.collision.cloud.points()
        self.collision.tree=cKDTree(fused) if len(fused) else None
        self.grid.observe(environment)
        if not masks:
            if allow_prediction and self.track.estimate is not None and self.last_observation_base is not None:
                snap=self.feedback.snapshot(self.route()[:2]);previous=self.last_observation_base
                distance=np.linalg.norm(snap['pose'][:2]-previous[:2]);turn=angle(snap['pose'][2]-previous[2])
                err=self.track.error_after_motion(distance,turn,self.s)
                if self.clock()-self.track.estimate.stamp<=self.s.target_age_s:
                    report=self.planner.robust_solution(self.track.estimate,snap['pose'],err+self.s.hug_drift_error_m,self._measured_body(snap))
                    if report is not None:
                        self._summary.update(predicted_target=True,target_error_m=err)
                        self.predicted_error=err;return dict(visible=True,predicted=True)
            if (not allow_prediction and self.track.estimate is not None
                    and self.clock()-self.track.estimate.stamp<=self.s.target_age_s):
                # A deliberate floor look can temporarily hide a previously
                # acquired stationary target. It may guide observation planning
                # but never passes the stricter final grasp handoff above.
                return dict(visible=True,observation_only=True)
            return dict(visible=False)
        cloud=reconstruct_view(measurement.sensor,masks[0].mask,pose).points_before
        if len(cloud)<60:
            return dict(visible=False)
        # The broad self-body mask must never erase an identified nearby
        # target from the collision map along with the robot's own surfaces.
        self.collision.cloud.add(cloud)
        self.collision.tree=cKDTree(self.collision.cloud.points())
        self.grid.observe(cloud)
        try:
            latest=self.feedback.snapshot(self.route()[:2])['joint']
            estimate=self.track.update(cloud,stamp=latest['received']-max(0.,latest['stamp']-measurement.sensor.ts),source=self.scan_source)
        except ValueError as exc:
            if self.track.estimate is not None:
                self._summary['observation_reason']=str(exc)
                return dict(visible=False)
            # A partial first view can locate the task's known cylinder before
            # its diameter is measurable. This prior only guides observation;
            # the post-scan fitted diameter/error gates the actual grasp.
            center=np.median(cloud,axis=0)
            toward=center-pose[:3,3];toward[2]=0
            toward/=max(np.linalg.norm(toward),1e-9)
            center+=toward*.05
            estimate=TargetEstimate(self.target_id,center,np.array([0.,0.,1.]),.1,.05,
                self.clock(),1,self.scan_source,cloud)
            self.track.estimate=estimate
            self._summary['diameter_prior_only']=True
        self.last_observation_base=self.feedback.snapshot(self.route()[:2])['pose'].copy()
        self.predicted_error=estimate.error_m
        self._summary.update(target_world_m=estimate.origin.tolist(),diameter_mm=estimate.diameter*1000,
            target_error_m=estimate.error_m,predicted_target=False,observed_points=len(cloud))
        return dict(visible=True)
    def _observation_goal(self):
        # FK cache in arm-base coordinates; transformed per candidate body pose.
        if not hasattr(self,'camera_cache'):
            from scipy.stats import qmc
            raw=qmc.Halton(4,scramble=True,seed=41).random(1536)
            self.camera_q=raw*(self.arm.bounds[:,1]-self.arm.bounds[:,0])+self.arm.bounds[:,0]
            local_model=ArmCameraModel(self.scan.model_dir.parents[1]/'arm/default.json',self.scan.hand_eye,
                base_world_transform=np.eye(4))
            self.camera_cache=np.array([local_model.camera_pose(q) for q in self.camera_q])
        snapshot=self.feedback.snapshot(self.route()[:2]);current=snapshot['pose']
        self.planner.body_z=snapshot['base']['pos'][2]
        self.planner.roll_pitch=snapshot['base']['rpy'][:2].copy()
        def evaluate(pose,target):
            body=body_frame(pose,self.planner.body_z,self.planner.roll_pitch);base=body@self.arm_in_body
            center=target.at_height(self.planner.frame(pose)[2,3])
            poses=base[None]@self.camera_cache
            v=center-poses[:,:3,3];camera=np.einsum('nji,nj->ni',poses[:,:3,:3],v)
            uv=abs(camera[:,:2])/np.maximum(camera[:,2:3],1e-9)
            ids=np.flatnonzero((camera[:,2]>.25)&(camera[:,2]<1.)&(uv[:,0]<.4)&(uv[:,1]<.25))
            accepted=[];directions=[]
            collision=ScanCollision(self.arm,base);collision.cloud=self.collision.cloud;collision.tree=self.collision.tree
            for i in ids[np.argsort(np.linalg.norm(uv[ids],axis=1))]:
                direction=v[i]/np.linalg.norm(v[i])
                if directions and max(float(direction@d) for d in directions)>np.cos(np.deg2rad(10)):continue
                q=self.camera_q[i]
                if not collision.certify_segment(np.array(self.model.meta['home']),q)['accepted']:continue
                # ROI corners, not a hallucinated full 1.1 m object, must fit.
                offsets=np.array([[x,y,z] for x in [-.06,.06] for y in [-.06,.06] for z in [-self.s.scan_roi_height_m/2,self.s.scan_roi_height_m/2]])
                roi=(center+offsets-poses[i,:3,3])@poses[i,:3,:3]
                if np.any(roi[:,2]<.1) or np.any(abs(roi[:,:2])/roi[:,2:3]>[.65,.45]):continue
                accepted.append(i);directions.append(direction)
                if len(accepted)>=4:break
            if len(accepted)<3:return None
            return dict(views=len(accepted),camera_q=self.camera_q[accepted[0]].tolist())
        try:return dict(goal=self.planner.observation(self.track.estimate,current,evaluate))
        except ValueError as exc:
            self._summary['planning_reason']=str(exc)
            return dict(needs_mapping=True)
    def _map_search_prepare(self,index):
        from scipy.stats import qmc
        sample=qmc.Halton(4,scramble=True,seed=17).random(256)
        qs=sample*(self.arm.bounds[:,1]-self.arm.bounds[:,0])+self.arm.bounds[:,0]
        host=self.connection.state_sink.get_state()
        base=base_world_transform_from_context(self.service._ik_context_for_host(host))
        model=ArmCameraModel(self.scan.model_dir.parents[1]/'arm/default.json',self.scan.hand_eye,base_world_transform=base)
        candidates=[]
        for q in qs:
            t=model.camera_pose(q);direction=t[:3,2]
            if direction[2]>=-.2:continue
            floor=t[:3,3]-direction*(t[2,3]/direction[2])
            distance=np.linalg.norm(floor[:2]-base[:2,3])
            if .25<distance<1.2:candidates.append((floor[0],floor[1],q))
        candidates.sort(key=lambda row:np.arctan2(row[1]-base[1,3],row[0]-base[0,3]))
        if not candidates:raise RuntimeError('no downward camera pose for local floor mapping')
        for row in candidates[index%len(candidates):]+candidates[:index%len(candidates)]:
            try:return self._arm_prepare([row[2]])
            except RuntimeError:continue
        raise RuntimeError('floor observation arm paths are blocked')
    def _arm_prepare(self,waypoints):
        # Recheck each segment at the current base and observed scene.
        base=base_world_transform_from_context(self.service._ik_context_for_host(self.connection.state_sink.get_state()))
        self.collision.base=base.copy()
        start=self.feedback.snapshot(self.route()[:2])['joint']['q']
        points=[start.copy()]+[np.asarray(q,float) for q in waypoints]
        for a,b in zip(points,points[1:]):
            if not self.collision.certify_segment(a,b)['accepted']:raise RuntimeError('arm transport/search path blocked')
        return points
    def begin(self,phase,grant,ownership,**args):
        self.generation+=1;self.stopped=False
        op=dict(phase=phase,grant=grant,ownership=ownership,args=args,started=self.clock())
        self.operation=op
        if phase in {'acquire','reacquire','post_scan_acquire','plan_observation','plan_grasp',
                'transport','search','aim_observation','map_search','check_grasp'}:
            op['hold_q']=self.feedback.snapshot(self.route()[:2])['joint']['q'].copy()
        if phase in {'acquire','reacquire','post_scan_acquire'}:op['future']=self.pool.submit(self._observe,phase=='reacquire')
        elif phase=='plan_observation':op['future']=self.pool.submit(self._observation_goal)
        elif phase=='plan_grasp':op['future']=self.pool.submit(self._grasp_goal)
        elif phase in {'transport','search','aim_observation','map_search'}:
            self._initialize()
            if phase=='map_search':
                op['future']=self.pool.submit(self._map_search_prepare,args['index'])
                return op
            if phase=='transport':waypoints=list(reversed(self.history))+[np.array(self.model.meta['home'])]
            elif phase=='aim_observation':waypoints=[args['goal'].report['camera_q']]
            else:
                q=self.feedback.snapshot(self.route()[:2])['joint']['q'].copy()
                q[1]=np.clip(q[1]+(.18 if args['index']%2 else -.36),self.arm.bounds[1,0],self.arm.bounds[1,1])
                q[2]+=(-1 if args['index']%2 else 1)*.06
                waypoints=[np.clip(q,self.arm.bounds[:,0],self.arm.bounds[:,1])]
            op['future']=self.pool.submit(self._arm_prepare,waypoints)
        elif phase in {'move_observation','move_grasp'}:
            self._summary['base_goal']=dict(pose=args['goal'].pose.tolist(),kind=args['goal'].report['kind'],
                path=args['goal'].path.tolist())
            op['executor']=BaseExecutor(self.s,self.clock)
            snap=self.feedback.snapshot(self.route()[:2]);op['executor'].start(args['goal'],snap['pose'],self.grid.revision)
        elif phase=='scan':
            self.scan.start(self.target_id,self.s.max_views,roi_height_m=self.s.scan_roi_height_m,retain_lease=True,tracking_uncertainty=[.001,.015,.015,.015])
        elif phase=='check_grasp':op['future']=self.pool.submit(self._check_grasp)
        elif phase=='hug':
            self.model.validate_robot(self.scan.model_dir/'arm.urdf')
            self.model.validate_physics(self._latest.get('physics_profile'))
            row=self._selected_hug;commands=self.model.commands(row);ident=uuid.uuid4().hex
            self._send(grant,ownership,timed_arm=dict(id=ident,operation='start',dt_s=self.model.meta['dt_s'],q=commands.tolist()))
            op['timed_id']=ident
            self.hug_execution_id=ident
            self.hug_final_tick=len(commands)-1
            self.hug_evaluation_tick=self.hug_final_tick
            op['body_pose']=self.feedback.snapshot(self.route()[:2])['pose'].copy()
            op['body_frame']=self._measured_body(self.feedback.snapshot(self.route()[:2]))
            op['target']=self.track.estimate
            op['solver_xy']=solver_xy(op['target'],op['body_frame']@self.node_in_body,self.planner.offset)
        elif phase=='verify':
            report=self._latest.get('manipulation_evaluation',{})
            valid=(report.get('id')==self.hug_execution_id
                and report.get('tick',-1)>=getattr(self,'hug_evaluation_tick',self.hug_final_tick))
            op['result']=dict(verified=valid and report.get('success') is True,
                evaluated_failure=valid and report.get('success') is not True,
                reason='' if valid and report.get('success') else 'Physical five-second hold was not verified',
                evidence=report if valid else {})
        else:raise RuntimeError('unknown mobile operation')
        return op
    def _check_grasp(self):
        snap=self.feedback.snapshot(self.route()[:2]);target=self.track.estimate
        if target is None or self.clock()-target.stamp>self.s.target_age_s:return dict(accepted=False)
        if np.any(abs(snap['joint']['q']-self.model.meta['home'])>[.001,.015,.015,.015]):return dict(accepted=False)
        row=self.planner.robust_solution(target,snap['pose'],getattr(self,'predicted_error',target.error_m)+self.s.hug_drift_error_m,self._measured_body(snap))
        if row is None:return dict(accepted=False)
        self._selected_hug=row;self._summary.update(solver_xy_mm=row['xy_mm'],q_deg=row['q_deg'],weights=row['weights'])
        return dict(accepted=True)
    def poll(self,op):
        if op is not self.operation:raise RuntimeError('stale mobile operation')
        phase=op['phase'];now=self.clock();snap=self.feedback.snapshot(self.route()[:2])
        grant,ownership=op['grant'],op['ownership']
        if now-op['started']>180 and phase!='scan':raise RuntimeError('mobile phase timeout')
        if 'result' in op:return op['result']
        if 'future' in op:
            if not op['future'].done():
                self._hold(op,op['hold_q']);return None
            result=op['future'].result();op.pop('future')
            if phase not in {'transport','search','aim_observation','map_search'}:return result
            if snap['joint']['sim_time'] is None:raise RuntimeError('arm approach requires Sim-time feedback')
            op.update(path=result,index=1,last=snap['joint']['q'].copy(),stable=None,last_at=snap['joint']['sim_time'])
        if phase in {'transport','search','aim_observation','map_search'}:
            path=op['path'];i=op['index'];goal=path[i]
            sim_time=snap['joint']['sim_time']
            if sim_time is None or sim_time<op['last_at']:raise RuntimeError('arm approach Sim clock changed')
            dt=min(.1,max(0.,sim_time-op['last_at']));op['last_at']=sim_time
            rates=np.array([.005,.04,.04,.04])
            q=op['last']+np.clip(goal-op['last'],-rates*dt,rates*dt)
            if np.any(abs(snap['joint']['q']-op['last'])>[.001,.015,.015,.015]):raise RuntimeError('arm did not follow approach')
            self._send(grant,ownership,q=q.tolist());op['last']=q;self.last_command=q
            close=np.all(abs(snap['joint']['q']-goal)<[.001,.015,.015,.015]) and np.allclose(q,goal,atol=1e-8)
            if close:
                if op['stable'] is None:op['stable']=now
                if now-op['stable']>.3:
                    if i<len(path)-1:op['index']+=1;op['stable']=None
                    else:
                        if phase=='transport':self.history=[np.array(self.model.meta['home'])]
                        else:self.history.append(goal.copy())
                        return dict(purpose=op['args'].get('purpose',''))
            else:op['stable']=None
        elif phase.startswith('move_'):
            b=snap['base'];velocity,done=op['executor'].step(snap['pose'],np.linalg.norm(b['velocity']),np.linalg.norm(b['angular']),
                lambda a,b:self.grid.segment_free(a,b,self.footprint))
            self._send(grant,ownership,go2_vel=velocity.tolist())
            self._hold(op,np.array(self.model.meta['home']))
            if done:return dict()
        elif phase=='scan':
            status=self.scan.status()
            if status['active']:return None
            if status['phase']!='completed':raise RuntimeError(status.get('reason','scan failed'))
            root=Path(status['scene_manifest']).parent
            with np.load(root/(self.target_id+'.npz'),allow_pickle=False) as d:points=d['observed'].copy()
            frame_files=sorted(root.parent.glob('frames/*.npz'))
            if not frame_files:raise RuntimeError('scan has no capture provenance')
            with np.load(frame_files[-1],allow_pickle=False) as d:
                latest=self.feedback.snapshot(self.route()[:2])['joint']
                stamp=latest['received']-max(0.,latest['stamp']-float(d['ts']))
            self.track.update(points,stamp=stamp,source=self.scan_source)
            self.last_observation_base=snap['pose'].copy();self.predicted_error=self.track.estimate.error_m
            self.history+=list(map(np.array,status.get('return_q_path',[])))
            return dict(quality_ok=self.track.estimate.error_m<=.012 and abs(self.track.estimate.diameter-.1)<=.002)
        elif phase=='hug':
            measured=self._measured_body(snap);initial=op['body_frame']
            movement=measured[:3,3]-initial[:3,3]
            turn=Rotation.from_matrix(initial[:3,:3].T@measured[:3,:3]).magnitude()
            if (np.linalg.norm(movement)>self.s.hug_body_position_tolerance_m
                    or turn>self.s.hug_body_rotation_tolerance_rad):
                raise RuntimeError('body moved beyond the stationary hug allowance')
            xy=solver_xy(op['target'],measured@self.node_in_body,self.planner.offset)
            drift=float(np.linalg.norm(xy-op['solver_xy'])/1000)
            if drift>self.s.hug_drift_error_m:raise RuntimeError('body used the reserved solver-coordinate drift allowance')
            self._summary['hug_solver_drift_m']=drift
            report=self._latest.get('timed_arm',{})
            evidence=self._latest.get('manipulation_evaluation',{})
            # Twenty seconds is a deadline, not an instruction to keep running
            # after the independent five-second loaded hold has succeeded.
            # Never use an old success from another path or an earlier tick.
            if (report.get('id')==op['timed_id'] and report.get('state') in {'executing','completed'}
                    and evidence.get('id')==op['timed_id'] and evidence.get('tick',-1)==report.get('tick')
                    and evidence.get('success') is True and evidence.get('hold_s',0.)>=5.):
                self.hug_evaluation_tick=report['tick']
                self._summary['verified_hold_tick']=report['tick']
                self._send(grant,ownership,timed_arm=dict(id=op['timed_id'],operation='cancel'))
                return dict()
            if now-self.last_send>.1:
                self._send(grant,ownership,timed_arm=dict(id=op['timed_id'],operation='keepalive'));self.last_send=now
            if report.get('id')==op['timed_id']:
                if report['state']=='cancelled':raise RuntimeError('timed arm: '+report.get('reason','cancelled'))
                if report['state']=='completed':
                    evidence=self._latest.get('manipulation_evaluation',{})
                    if evidence.get('id')==op['timed_id'] and evidence.get('tick',-1)>=report['tick']:
                        return dict()
                    if 'finished_at' not in op:op['finished_at']=now
                    if now-op['finished_at']>1.:
                        return dict()
        return None
    def _hold(self,op,q):
        if self.clock()-self.last_send<.08:return
        actual=self.feedback.snapshot(self.route()[:2])['joint']['q']
        if np.any(abs(actual-q)>[.001,.015,.015,.015]):raise RuntimeError('arm did not hold its checked posture')
        self._send(op['grant'],op['ownership'],q=np.asarray(q).tolist());self.last_send=self.clock()
    def stop(self,reason):
        self.stopped=True;self.generation+=1
        op=self.operation
        if op is None:return
        if op is not None and op['phase']=='scan':self.scan.request_cancel(reason)
        try:
            if op is not None and op.get('timed_id'):
                self.connection.send_workflow_motion(dict(command='target',timed_arm=dict(id=op['timed_id'],operation='cancel')),op['grant'].route)
            self.connection.send_workflow_motion(dict(command='target',go2_vel=[0.,0.,0.]),op['grant'].route)
        finally:
            if self.connection.motion_route()==op['grant'].route:self.connection.clear_pending_motion()
    def close(self):
        self.stopped=True;self.pool.shutdown(wait=True,cancel_futures=True)
        if self.camera is not None:self.camera.stop()
