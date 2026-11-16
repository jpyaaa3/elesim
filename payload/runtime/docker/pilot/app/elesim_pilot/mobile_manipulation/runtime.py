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
from elesim_pilot.robot.arm.approach import JointApproach
from elesim_pilot.robot.arm.iklib.kinematics import _forward_link_tf,base_world_transform_from_context
from elesim_pilot.vision.partial_cloud import ArmCameraModel,SensorObservation,reconstruct_view
from elesim_pilot.vision.multiview import ViewPlanner
from elesim_pilot.vision.perception.sim_rendered_camera import SimCameraTimeout,SimRenderedCamera
from .contracts import Settings,TargetEstimate,planar,angle
from .feedback import Feedback
from .target_track import TargetTrack
from .local_map import LocalMap,transport_footprint,floor_cell_points
from .base_planner import BasePlanner,FloorObservationRequired,body_frame,solver_xy
from .base_executor import BaseExecutor


class ArmPathBlocked(RuntimeError):
    """The current collision model rejected a complete arm segment."""


class SimPorts:
    def __init__(self,connection,service,state,scan,*,simulation_status,model_root,settings=None,clock=time.monotonic):
        self.connection,self.service,self.state,self.scan=connection,service,state,scan
        self.simulation_status=simulation_status;self.model_root=Path(model_root)
        self.s=settings or Settings();self.clock=clock;self.feedback=Feedback(clock)
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='mobile-plan')
        self.camera=None;self.model=None;self.grid=None;self.track=None;self.last_send=0.;self.operation=None
        self.generation=0;self.history=[];self.last_observation_base=None;self.scan_source=None
        self.last_command=None;self.stopped=False;self._summary={};self._latest={}
        self.last_target_q=None;self.mapping_views=[];self.mapping_intrinsics=None
        self.mapping_margin_m=0.
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
        self._require_home_start(q)
        self.target_id=target_id;self.stopped=False
        self.grid=None;self.track=None;self.history=[];self.last_observation_base=None
        self.scan_source=None;self._summary={};self.operation=None;self.last_frame_seq=-1
        self.last_target_q=None;self.mapping_views=[];self.mapping_intrinsics=None
        self.mapping_margin_m=0.
        if hasattr(self,'mapping_goal_xy'):del self.mapping_goal_xy
        if hasattr(self,'mapping_goal_pose'):del self.mapping_goal_pose
        if hasattr(self,'camera_cache'):del self.camera_cache
        if self.camera is not None:self.camera.stop();self.camera=None
    def _require_home_start(self,q):
        from elesim_protocol import ControlU,control_u_to_sim_q
        offsets=self.state.offset_values()[:4]
        display=control_u_to_sim_q(ControlU(*(v+o for v,o in
            zip((180.,180.,10.,10.),offsets))),self.service.control_mapping())
        calibrated=np.array([display.linear_m,display.roll_rad,display.theta1_rad,display.theta2_rad])
        # The UI Home preset includes operator calibration. It is a valid
        # initial view; transport still certifies every segment back to the
        # policy's exact canonical Home before any base move or learned path.
        if not any(np.all(abs(q-home)<=[.001,.015,.015,.015])
                   for home in (self.model.meta['home'],calibrated)):
            raise RuntimeError('Start mobile task at measured Home')
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
        self.collision=ScanCollision(self.arm,self.base,reserve_tracking=True)
        self.camera_model=ArmCameraModel(self.scan.model_dir.parents[1]/'arm/default.json',self.scan.hand_eye,
            base_world_transform=self.base)
        snap=self.feedback.snapshot(self.route()[:2]);b=snap['base'];body=self._measured_body(snap)
        node_pos,node_rot=_forward_link_tf(context,[0.,0.,0.,0.])['node0']
        node=np.eye(4);node[:3,3]=node_pos;node[:3,:3]=node_rot
        self.node_in_body=np.linalg.inv(body)@node;self.arm_in_body=np.linalg.inv(body)@self.base
        map_resolution=.01
        self.grid=LocalMap(resolution=map_resolution,origin_xy=b['pos'][:2]-map_resolution/2)
        self.track=TargetTrack(self.target_id,self.s.min_position_error_m)
        # Conservative 3D spheres projected with the actual transport posture,
        # plus GO2 body/leg sweep envelope from this flat-floor task profile.
        home=np.asarray(self.model.meta['home']);centers=self.collision._centers(home)
        centers=(centers-b['pos'])@body[:3,:3]
        radii=self.collision._radii
        spacing=self.grid.resolution/2
        self.footprint=transport_footprint(centers,radii,spacing=spacing)
        self.planner=BasePlanner(self.model,self.grid,self.footprint,self.node_in_body,b['pos'][2],self.s)
        self.planner.roll_pitch=b['rpy'][:2].copy()
        # The occupied starting stance is the only unobserved seed: the flat
        # Sim profile has just verified stationary measured standing support.
        start=snap['joint']['q'].copy()
        occupied=transport_footprint((self.collision._centers(start)-b['pos'])@body[:3,:3],radii,spacing=spacing)
        t=planar(snap['pose']);seed=occupied@t[:2,:2].T+t[:2,3]
        self.grid.free.update(self.grid.cell(p) for p in seed)
        self.last_command=start.copy();self.history=[start.copy()]
    def _measured_body(self,snapshot):
        b=snapshot['base']
        return body_frame(snapshot['pose'],b['pos'][2],b['rpy'][:2])
    def _grasp_goal(self):
        snapshot=self.feedback.snapshot(self.route()[:2])
        self.planner.body_z=snapshot['base']['pos'][2]
        self.planner.roll_pitch=snapshot['base']['rpy'][:2].copy()
        self._summary['map_current']=self.grid.footprint_status(snapshot['pose'],self.footprint,
            margin_m=self.planner.clearance_m)
        try:return dict(goal=self.planner.grasp(self.track.estimate,snapshot['pose']))
        except FloorObservationRequired as exc:
            return self._mapping_required(exc)
        finally:
            self._summary['grasp_planning']=dict(self.planner.last_grasp_report)
    def _mapping_required(self,exc):
        self.mapping_goal_xy=exc.pose[:2].copy();self.mapping_goal_pose=exc.pose.copy()
        self.mapping_margin_m=self.planner.clearance_m
        self._summary.update(planning_reason=str(exc),mapping_goal_xy=self.mapping_goal_xy.tolist(),
            mapping_clearance_m=self.mapping_margin_m,
            map_goal=self.grid.footprint_status(exc.pose,self.footprint,margin_m=self.mapping_margin_m))
        return dict(needs_mapping=True)
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
        deadline=self.clock()+5.;rejections={}
        def rejected(reason):
            rejections[reason]=rejections.get(reason,0)+1
            self._summary['capture_rejections']=dict(rejections)
        while self.clock()<deadline and not self.stopped:
            try:frame=self.camera.capture(retries=1,retry_sleep_s=0.)
            except SimCameraTimeout:
                rejected('no_frame');continue
            if frame.arm_q is None or frame.seq<=self.last_frame_seq:
                rejected('missing_encoder_or_replayed_frame');continue
            try:b=self.feedback.associate(frame,self.route()[:2])
            except RuntimeError as exc:
                rejected(str(exc));continue
            current=self.feedback.snapshot(self.route()[:2])
            age=current['joint']['stamp']-frame.ts
            if not -.05<=age<=.5:
                rejected('capture/joint timestamp gap');continue
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
    def _missing_target(self,*,allow_mapping):
        if allow_mapping and hasattr(self,'mapping_goal_pose'):
            status=self.grid.footprint_status(self.mapping_goal_pose,self.footprint,
                margin_m=getattr(self,'mapping_margin_m',0.))
            self._summary['map_goal']=status
            if status['unknown_cells'] and not status['blocked_cells']:
                return dict(visible=False,needs_mapping=True)
        return dict(visible=False)
    def _observe(self,allow_prediction=False):
        generation=self.generation
        measurement,pose,base=self._capture()
        if self.stopped or generation!=self.generation:raise RuntimeError('observation cancelled')
        self.mapping_intrinsics=measurement.sensor.intrinsics
        config=yaml.safe_load(self.scan.models_path.read_text())
        segmenter=load_model('segmenter',config['models']['segmenter'],self.scan.models_path.parent).implementation
        masks=segmenter.segment(measurement,Task('mobile',self.target_id))
        # Map every observed surface, removing the arm's own conservative volume.
        self.collision.base=base.copy()
        full=reconstruct_view(measurement.sensor,np.ones(measurement.sensor.depth_raw.shape,bool),pose,max_points=6000).points_before
        centers=self.collision._centers(measurement.sensor.encoder_q)
        snap=self.feedback.snapshot(self.route()[:2]);b=snap['base']
        goal_xy=getattr(self,'mapping_goal_xy',b['pos'][:2])
        self.grid.recenter((b['pos'][:2]+goal_xy)/2)
        def exclude_robot(points):
            kept=[]
            for start in range(0,len(points),256):
                block=points[start:start+256]
                gaps=np.linalg.norm(block[:,None]-centers[None],axis=2)-self.collision._radii
                kept.append(block[np.all(gaps>.015,axis=1)])
            environment=np.concatenate(kept) if kept else np.empty((0,3))
            body_local=(environment-b['pos'])@Rotation.from_euler('xyz',b['rpy']).as_matrix()
            self_body=(abs(body_local[:,0])<.4)&(abs(body_local[:,1])<.27)&(body_local[:,2]>-.4)&(body_local[:,2]<.25)
            return environment[~self_body]
        environment=exclude_robot(full)
        self.collision.cloud.add(environment)
        fused=self.collision.cloud.points()
        self.collision.tree=cKDTree(fused) if len(fused) else None
        floor=exclude_robot(floor_cell_points(measurement.sensor,pose,
            resolution=self.grid.resolution,max_cells=self.grid.max_cells,cell_window=self.grid.window,
            origin_xy=self.grid.origin))
        self.grid.observe(np.concatenate([environment,floor]))
        self._summary['floor_cells_in_capture']=len(floor)
        if not masks:
            missing=self._missing_target(allow_mapping=not allow_prediction)
            if missing.get('needs_mapping'):return missing
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
            return missing
        cloud=reconstruct_view(measurement.sensor,masks[0].mask,pose).points_before
        if len(cloud)<60:
            return self._missing_target(allow_mapping=not allow_prediction)
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
                return self._missing_target(allow_mapping=not allow_prediction)
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
        self.last_target_q=np.array(measurement.sensor.encoder_q,float)
        self.predicted_error=estimate.error_m
        self._summary.update(target_world_m=estimate.origin.tolist(),diameter_mm=estimate.diameter*1000,
            target_error_m=estimate.error_m,predicted_target=False,observed_points=len(cloud))
        return dict(visible=True)
    def _observation_goal(self):
        generation=self.generation
        def cooperate():
            # Small NumPy/SciPy FK calls can retain the GIL across the whole
            # batch. Give the DDS thread a turn; never relax feedback age.
            time.sleep(.001)
            if self.stopped or generation!=self.generation:raise RuntimeError('observation planning cancelled')
        # FK cache in arm-base coordinates; transformed per candidate body pose.
        if not hasattr(self,'camera_cache'):
            local_model=ArmCameraModel(self.scan.model_dir.parents[1]/'arm/default.json',self.scan.hand_eye,
                base_world_transform=np.eye(4))
            views=ViewPlanner(local_model,collision_checker=self.collision,candidates=12000,
                operating_bounds=self.arm.bounds,cancelled=lambda:self.stopped or generation!=self.generation)
            self.camera_q,self.camera_cache=views.q,views.poses
        snapshot=self.feedback.snapshot(self.route()[:2]);current=snapshot['pose']
        self.planner.body_z=snapshot['base']['pos'][2]
        self.planner.roll_pitch=snapshot['base']['rpy'][:2].copy()
        diagnostics=dict(current_footprint_free=self.grid.footprint_free(current,self.footprint),
            free_cells=len(self.grid.free),blocked_cells=len(self.grid.blocked),
            sampled_camera_poses=len(self.camera_q),evaluated=[])
        def evaluate(pose,target):
            cooperate()
            body=body_frame(pose,self.planner.body_z,self.planner.roll_pitch);base=body@self.arm_in_body
            center=target.at_height(self.planner.frame(pose)[2,3])
            poses=base[None]@self.camera_cache
            v=center-poses[:,:3,3];camera=np.einsum('nji,nj->ni',poses[:,:3,:3],v)
            uv=abs(camera[:,:2])/np.maximum(camera[:,2:3],1e-9)
            ids=np.flatnonzero((camera[:,2]>.25)&(camera[:,2]<1.)&(uv[:,0]<.4)&(uv[:,1]<.25))
            accepted=[];directions=[]
            tested=dict(pose=pose.tolist(),in_view=len(ids),accepted=0,rejections={})
            diagnostics['evaluated'].append(tested)
            collision=ScanCollision(self.arm,base,reserve_tracking=True);collision.cloud=self.collision.cloud;collision.tree=self.collision.tree
            for i in ids[np.argsort(np.linalg.norm(uv[ids],axis=1))]:
                cooperate()
                direction=v[i]/np.linalg.norm(v[i])
                if directions and max(float(direction@d) for d in directions)>np.cos(np.deg2rad(10)):continue
                q=self.camera_q[i]
                if not collision.configuration_free(q):
                    reason='arrival_tracking_margin'
                    tested['rejections'][reason]=tested['rejections'].get(reason,0)+1
                    continue
                certificate=collision.certify_segment(np.array(self.model.meta['home']),q)
                if not certificate['accepted']:
                    reason=certificate.get('reason','collision')
                    tested['rejections'][reason]=tested['rejections'].get(reason,0)+1
                    continue
                # ROI corners, not a hallucinated full 1.1 m object, must fit.
                offsets=np.array([[x,y,z] for x in [-.06,.06] for y in [-.06,.06] for z in [-self.s.scan_roi_height_m/2,self.s.scan_roi_height_m/2]])
                roi=(center+offsets-poses[i,:3,3])@poses[i,:3,:3]
                if np.any(roi[:,2]<.1) or np.any(abs(roi[:,:2])/roi[:,2:3]>[.65,.45]):continue
                accepted.append(i);directions.append(direction)
                tested['accepted']=len(accepted)
                if len(accepted)>=4:break
            if len(accepted)<3:return None
            return dict(views=len(accepted),camera_q=self.camera_q[accepted[0]].tolist())
        try:result=dict(goal=self.planner.observation(self.track.estimate,current,evaluate))
        except FloorObservationRequired as exc:result=self._mapping_required(exc)
        except ValueError as exc:
            self._summary['planning_reason']=str(exc)
            result=dict(needs_mapping=True)
        self._summary['observation_planning']=diagnostics
        return result
    def _refine_floor_prepare(self,current,probes,model,seeds):
        from elesim_pilot.robot.arm.view_refinement import refine_view,frustum_clearances
        from elesim_pilot.arm_trajectory import (arm_base_pose_delta,require_stable_arm_base,
            _ARM_BASE_POSITION_TOLERANCE_M,_ARM_BASE_ANGLE_TOLERANCE_RAD)
        intr=getattr(self,'mapping_intrinsics',None)
        if intr is None or not len(probes):return None
        low,high=np.min(probes[:,:2],axis=0),np.max(probes[:,:2],axis=0)
        # A camera frustum is convex. Covering this rectangle also covers
        # every requested floor point without an unbounded constraint array.
        points=np.array([[x,y,0.] for x in (low[0],high[0]) for y in (low[1],high[1])])
        bounds=self.arm.bounds.copy()
        bounds[:,0]+=self.arm.uncertainty;bounds[:,1]-=self.arm.uncertainty
        anchor=np.clip(current,bounds[:,0],bounds[:,1])
        starts=np.asarray([row[1] for row in seeds[:64]],float).reshape(-1,4)
        starts=np.clip(starts,bounds[:,0],bounds[:,1])
        intrinsic=np.array([intr.fx,intr.fy,intr.cx,intr.cy,intr.width,intr.height])
        def admissible(q,pose):
            direction=pose[:3,2]
            if direction[2]>=-.2:return False
            floor=pose[:3,3]-direction*(pose[2,3]/direction[2])
            if not .25<np.linalg.norm(floor[:2]-self.collision.base[:2,3])<1.2:return False
            return not any(np.linalg.norm(pose[:3,3]-p[:3,3])<.03
                and direction@p[:3,2]>np.cos(np.deg2rad(10)) for p in self.mapping_views)
        generation=self.generation
        reference_base=self.collision.base.copy()
        def margins(q):
            return self.collision.configuration_margins(q,
                base_position_error_m=_ARM_BASE_POSITION_TOLERANCE_M,
                base_angle_error_rad=_ARM_BASE_ANGLE_TOLERANCE_RAD)
        q,report=refine_view(anchor,points,bounds,model.camera_pose,
            margins,intrinsic,motion_rates=JointApproach.rates,
            seeds=starts,admissible=admissible,
            cancelled=lambda:self.stopped or self.generation!=generation)
        self._summary['floor_search']['view_refinement']=report
        if q is None:return None
        report['target_q']=q.tolist()
        try:result=self._arm_prepare([q])
        except ArmPathBlocked as exc:
            report.update(path_accepted=False,path_reason=str(exc));return None
        # The body can move slightly during optimization. Recheck the same
        # endpoint reserve and original frustum at the freshly measured base.
        live=ArmCameraModel(self.scan.model_dir.parents[1]/'arm/default.json',self.scan.hand_eye,
            base_world_transform=self.collision.base)
        pose=live.camera_pose(q)
        position_delta,angle_delta=arm_base_pose_delta(reference_base,self.collision.base)
        try:require_stable_arm_base(reference_base,self.collision.base);stable=True
        except RuntimeError:stable=False
        endpoint=bool(self.collision.configuration_free(q));view=bool(admissible(q,pose))
        frustum=float(np.min(frustum_clearances(pose,points,intrinsic,padding_fraction=0.)))
        margin=float(np.min(self.collision.configuration_margins(q)))
        report.update(base_position_delta_m=position_delta,base_rotation_delta_rad=angle_delta,
            base_stable=stable,endpoint_admitted=endpoint,view_admitted=view,
            live_frustum_margin_m=frustum,live_minimum_clearance_m=margin if np.isfinite(margin) else None)
        failed=[name for name,ok in [('base',stable),('endpoint',endpoint),('view',view),('frustum',frustum>=0)] if not ok]
        if failed:
            report.update(path_accepted=False,path_reason='refined live admission failed: '+', '.join(failed))
            return None
        report['path_accepted']=True
        self._summary['floor_search']['path_strategy']='refined_direct'
        self.mapping_views.append(pose.copy());return result
    def _map_search_prepare(self,index):
        from scipy.stats import qmc
        host=self.connection.state_sink.get_state()
        base=base_world_transform_from_context(self.service._ik_context_for_host(host))
        self.collision.base=base.copy()
        model=ArmCameraModel(self.scan.model_dir.parents[1]/'arm/default.json',self.scan.hand_eye,base_world_transform=base)
        cached=hasattr(self,'camera_cache')
        if cached:
            qs=self.camera_q;poses=base[None]@self.camera_cache
        else:
            sample=qmc.Halton(4,scramble=True,seed=17).random(256)
            qs=sample*(self.arm.bounds[:,1]-self.arm.bounds[:,0])+self.arm.bounds[:,0]
            poses=None
        snapshot=self.feedback.snapshot(self.route()[:2]);current=snapshot['joint']['q']
        start=snapshot['pose'][:2];goal=getattr(self,'mapping_goal_xy',start)
        lo=np.minimum(start,goal)-.45;hi=np.maximum(start,goal)+.45
        xy=np.array(np.meshgrid(*[np.arange(a,b,self.grid.resolution) for a,b in zip(lo,hi)])).reshape(2,-1).T
        unknown=np.array([p for p in xy if self.grid.cell(p) not in self.grid.free
            and self.grid.cell(p) not in self.grid.blocked]).reshape(-1,2)
        focus=False
        if hasattr(self,'mapping_goal_pose'):
            cells=self.grid._footprint_cells(self.mapping_goal_pose,self.footprint,
                margin_m=getattr(self,'mapping_margin_m',0.))
            missing=[c for c in cells if c not in self.grid.free and c not in self.grid.blocked]
            if missing:
                unknown=self.grid.cell_centers(missing);focus=True
        probes=np.column_stack([unknown,np.zeros(len(unknown))])
        intr=getattr(self,'mapping_intrinsics',None)
        candidates=[]
        for i,q in enumerate(qs):
            if i%16==0:time.sleep(.001)
            t=model.camera_pose(q) if poses is None else poses[i];direction=t[:3,2]
            if direction[2]>=-.2:continue
            # A floor look that did not reveal the needed cells must not win
            # repeatedly on its predicted field of view alone.
            if any(np.linalg.norm(t[:3,3]-p[:3,3])<.03
                   and direction@p[:3,2]>np.cos(np.deg2rad(10))
                   for p in self.mapping_views):continue
            floor=t[:3,3]-direction*(t[2,3]/direction[2])
            distance=np.linalg.norm(floor[:2]-base[:2,3])
            if not .25<distance<1.2:continue
            local=(probes-t[:3,3])@t[:3,:3]
            normalized=local[:,:2]/np.maximum(local[:,2:3],1e-9)
            visible=(local[:,2]>.1)&(local[:,2]<2.)
            if intr is None:
                visible&=np.all(abs(normalized)<[.65,.45],axis=1)
            else:
                uv=normalized*[intr.fx,intr.fy]+[intr.cx,intr.cy]
                visible&=np.all((uv>=[0,0])&(uv<[intr.width,intr.height]),axis=1)
            if not np.any(visible):continue
            duration=np.max(abs(q-current)/[.005,.04,.04,.04])
            score=float(visible.sum())/(1.+duration*.03)
            candidates.append((score,q,t))
        candidates.sort(key=lambda row:-row[0])
        self._summary['floor_search']=dict(cached=cached,sampled=len(qs),
            goal_footprint_focus=focus,unknown_probes=len(probes),candidates=len(candidates),blocked_paths=0,
            arrival_rejections=0,detour_attempts=0)
        detours=[]
        for _,q,t in candidates:
            time.sleep(.001)
            if not self.collision.configuration_free(q):
                self._summary['floor_search']['arrival_rejections']+=1
                continue
            try:result=self._arm_prepare([q])
            except ArmPathBlocked as exc:
                self._summary['floor_search']['blocked_paths']+=1
                self._summary['floor_search']['last_rejection']=str(exc)
                detours.append((q,t))
                continue
            self._summary['floor_search']['path_strategy']='direct'
            self.mapping_views.append(t.copy());return result
        # A straight joint interpolation can sweep through the object even
        # when both views are clear. Prefer every direct candidate before
        # reusing the bounded, fully re-certified return/history routes.
        if detours:
            goals=detours[:64]
            # A high-scoring view may need a long history replay, while a
            # later view has a short certified retract/transform/extend path.
            # Compare the simple routes within the existing goal-root set
            # before committing to the first view's history.
            for q,t in goals:
                time.sleep(.001)
                self._summary['floor_search']['detour_attempts']+=1
                try:result=self._return_prepare(q,use_history=False)
                except ArmPathBlocked:continue
                self._summary['floor_search']['path_strategy']=self._summary['arm_return']['strategy']
                self.mapping_views.append(t.copy());return result
            q,t=detours[0]
            self._summary['floor_search']['detour_attempts']+=1
            try:result=self._return_prepare(q)
            except ArmPathBlocked as exc:
                self._summary['floor_search']['last_rejection']=str(exc)
            else:
                self._summary['floor_search']['path_strategy']=self._summary['arm_return']['strategy']
                self.mapping_views.append(t.copy());return result
            from elesim_pilot.robot.arm.joint_path import connect_joint_path
            generation=self.generation
            retracted=current.copy()
            retracted[0]=min(current[0],self.arm.bounds[0,0]+self.arm.uncertainty[0])
            path,report=connect_joint_path(current,[q for q,t in goals],self.arm.bounds,
                self.collision.certify_segment,samples=qs,seeds=[retracted,*self.history[:1]],
                cancelled=lambda:self.stopped or self.generation!=generation)
            self._summary['floor_search']['joint_search']=report
            if path is not None:
                # The search used a stationary planning snapshot. Certify the
                # entire proposal again from fresh feedback and the live base.
                result=self._arm_prepare(path[1:])
                if not self.collision.configuration_free(path[-1]):
                    raise ArmPathBlocked('floor observation endpoint lost its tracking reserve')
                self._summary['floor_search']['path_strategy']='certified_joint_search'
                self.mapping_views.append(goals[report['goal_index']][1].copy());return result
        if not detours:
            result=self._refine_floor_prepare(current,probes,model,candidates)
            if result is not None:return result
        raise RuntimeError('floor observation arm paths are blocked' if candidates
            else 'no downward camera pose for local floor mapping')
    def _arm_prepare(self,waypoints):
        # Recheck each segment at the current base and observed scene.
        base=base_world_transform_from_context(self.service._ik_context_for_host(self.connection.state_sink.get_state()))
        self.collision.base=base.copy()
        start=self.feedback.snapshot(self.route()[:2])['joint']['q']
        points=[start.copy()]+[np.asarray(q,float) for q in waypoints]
        for a,b in zip(points,points[1:]):
            report=self.collision.certify_segment(a,b)
            if not report['accepted']:raise ArmPathBlocked('arm transport/search path blocked: '+report.get('reason','collision'))
        return points
    def _return_prepare(self,goal,*,use_history=True):
        history=list(reversed(self.history))
        current=self.feedback.snapshot(self.route()[:2])['joint']['q'].copy()
        def candidates():
            yield 'direct',[goal]
            # Try the smaller retraction first. If changing shape while
            # extending still sweeps an obstacle, keep BOTH endpoint shapes
            # fully retracted for the transform, then extend to the same goal.
            if goal[0]<current[0]-1e-9:
                retracted=current.copy();retracted[0]=goal[0]
                yield 'retract_first',[retracted,goal]
            linear=min(current[0],goal[0],self.arm.bounds[0,0]+self.arm.uncertainty[0])
            if linear<max(current[0],goal[0])-1e-9:
                before=current.copy();after=goal.copy();before[0]=after[0]=linear
                yield 'retract_transform_extend',[before,after,goal]
            if use_history:
                for count in range(1,len(history)+1):
                    yield 'history_prefix',history[:count]+[goal]
        for index,(strategy,waypoints) in enumerate(candidates()):
            if index:time.sleep(.001)
            try:points=self._arm_prepare(waypoints)
            except ArmPathBlocked:continue
            self._summary['arm_return']=dict(strategy=strategy,
                segments=len(points)-1,goal_q=np.asarray(goal).tolist())
            return points
        raise ArmPathBlocked('arm transport/search path blocked')
    def begin(self,phase,grant,ownership,**args):
        self.generation+=1;self.stopped=False
        op=dict(phase=phase,grant=grant,ownership=ownership,args=args,started=self.clock())
        self.operation=op
        if phase in {'plan_observation','plan_grasp'}:
            for field in ('mapping_goal_pose','mapping_goal_xy'):
                if hasattr(self,field):delattr(self,field)
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
            if phase=='transport':
                op['future']=self.pool.submit(self._return_prepare,np.array(self.model.meta['home']))
                return op
            elif phase=='aim_observation':waypoints=[args['goal'].report['camera_q']]
            else:
                q=self.feedback.snapshot(self.route()[:2])['joint']['q'].copy()
                last_seen=getattr(self,'last_target_q',None)
                if last_seen is not None and np.any(abs(q-last_seen)>[.001,.015,.015,.015]):
                    # A floor look can hide the target for longer than its
                    # prediction budget. Recheck the return route and look
                    # from the last successful view before blind perturbations.
                    op['future']=self.pool.submit(self._return_prepare,last_seen.copy())
                    return op
                else:
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
            self.scan.start(self.target_id,self.s.max_views,roi_height_m=self.s.scan_roi_height_m,
                retain_lease=True,tracking_uncertainty=[.001,.015,.015,.015],
                finish_drift_bounds=(self.s.hug_body_position_tolerance_m,self.s.hug_body_rotation_tolerance_rad),
                finish_when=self._scan_observation_ready)
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
    @staticmethod
    def _scan_quality(estimate):
        return estimate.error_m<=.012 and abs(estimate.diameter-.1)<=.002
    def _scan_observation_ready(self,points):
        try:
            estimate=TargetTrack(self.target_id).update(points,stamp=self.clock(),source=self.scan_source)
        except ValueError:return False
        return self._scan_quality(estimate)
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
            op.update(path=result,index=1,approach=JointApproach(result[0],result[1]),
                stable=None,last_at=snap['joint']['sim_time'])
        if phase in {'transport','search','aim_observation','map_search'}:
            path=op['path'];i=op['index'];goal=path[i]
            sim_time=snap['joint']['sim_time']
            if sim_time is None or sim_time<op['last_at']:raise RuntimeError('arm approach Sim clock changed')
            dt=min(.1,max(0.,sim_time-op['last_at']));op['last_at']=sim_time
            q=op['approach'].step(snap['joint']['q'],dt)
            if self.last_command is None or not np.array_equal(q,self.last_command) or now-self.last_send>=.08:
                self._send(grant,ownership,q=q.tolist());self.last_command=q;self.last_send=now
            close=np.all(abs(snap['joint']['q']-goal)<[.001,.015,.015,.015]) and np.allclose(q,goal,atol=1e-8)
            if close:
                if op['stable'] is None:op['stable']=now
                if now-op['stable']>.3:
                    if i<len(path)-1:
                        op['index']+=1;op['stable']=None
                        op['approach']=JointApproach(goal,path[i+1])
                    else:
                        if phase=='transport':self.history=[np.array(self.model.meta['home'])]
                        else:self.history.extend(q.copy() for q in path[1:])
                        return dict(purpose=op['args'].get('purpose',''))
            else:op['stable']=None
        elif phase.startswith('move_'):
            def segment_free(a,b):
                free=self.grid.segment_free(a,b,self.footprint)
                if not free:
                    self._summary['base_path_check']=dict(pose=np.asarray(a).tolist(),
                        current=self.grid.footprint_status(a,self.footprint),
                        next=self.grid.footprint_status(b,self.footprint))
                return free
            b=snap['base'];velocity,done=op['executor'].step(snap['pose'],np.linalg.norm(b['velocity']),np.linalg.norm(b['angular']),
                segment_free)
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
            return dict(quality_ok=self._scan_quality(self.track.estimate))
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
