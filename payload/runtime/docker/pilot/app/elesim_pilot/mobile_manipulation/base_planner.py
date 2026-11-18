"""Observation goals and C-band placement; all geometry stays in Pilot."""
import numpy as np
import time
from scipy.spatial.transform import Rotation
from .contracts import BaseGoal,planar,angle


class FloorObservationRequired(ValueError):
    def __init__(self,pose,*,footprint=None,margin_m=0.):
        super().__init__('no observed-free local path to a supported C-band goal')
        self.pose=np.array(pose,float,copy=True)
        self.footprint=None if footprint is None else np.array(footprint,float,copy=True)
        self.margin_m=float(margin_m)


def body_frame(base_pose,body_z,roll_pitch=(0.,0.)):
    t=planar(base_pose);t[2,3]=body_z
    t[:3,:3]=Rotation.from_euler('xyz',[*roll_pitch,base_pose[2]]).as_matrix()
    return t


def node_frame(base_pose,node_in_body,body_z,roll_pitch=(0.,0.)):
    t=body_frame(base_pose,body_z,roll_pitch)
    return t@node_in_body


def solver_xy(target,frame,offset_mm=144.5):
    n=frame[:3,2];den=float(n@target.axis)
    if abs(den)<.9:raise ValueError('unsupported cylinder/arm-plane alignment')
    point=target.origin+target.axis*float(n@(frame[:3,3]-target.origin)/den)
    local=frame[:3,:3].T@(point-frame[:3,3])
    return np.array([-local[1]*1000,local[0]*1000+offset_mm])


def inflated_errors(radius):
    # Declared deterministic error probes, not a continuous robustness proof.
    return np.array([[0.,0.]]+[[np.cos(a)*radius,np.sin(a)*radius] for a in np.linspace(0,2*np.pi,16,endpoint=False)])


class BasePlanner:
    def __init__(self,model,grid,footprint,node_in_body,body_z,settings):
        self.model,self.grid,self.footprint=model,grid,footprint
        self.node_in_body=np.asarray(node_in_body,float);self.body_z=body_z;self.s=settings
        self.offset=model.meta.get('solver_origin_offset_mm',144.5)
        self.rows=None
        self.roll_pitch=(0.,0.)
        radius=0. if footprint is None else float(np.max(np.linalg.norm(footprint,axis=1)))
        # An actual tracking error must fit measured floor around the route,
        # including its starting stance. This never changes arrival criteria.
        self.clearance_m=(settings.base_position_tolerance_m
            +2*radius*np.sin(settings.base_yaw_tolerance_rad/2))
    def frame(self,pose):return node_frame(pose,self.node_in_body,self.body_z,self.roll_pitch)
    def robust_solution(self,target,pose,error_m,measured_body=None):
        frame=self.frame(pose) if measured_body is None else measured_body@self.node_in_body
        xy=solver_xy(target,frame,self.offset)
        points=xy+inflated_errors(error_m*1000)
        rows=self.model.solve_many(points)
        if any(r is None for r in rows):return None
        # All perturbations must remain on the same analytic contact branch.
        if len({(r['feature'],r['root']) for r in rows})!=1:return None
        return rows[0]
    def _require_current_clearance(self,current_pose):
        if not self.grid.footprint_free(current_pose,self.footprint,margin_m=self.clearance_m):
            if self.grid.footprint_status(current_pose,self.footprint,margin_m=self.clearance_m)['blocked_cells']:
                raise ValueError('current transport footprint intersects an observed obstacle')
            raise FloorObservationRequired(current_pose)
    def grasp(self,target,current_pose):
        self.last_grasp_report=dict(stage='checking_start',blocked_goals=0,unknown_goals=0,
            observed_goals=0,route_rejections=0)
        if abs(target.diameter-.1)>.002:raise ValueError('diameter outside D100 profile')
        self._require_current_clearance(current_pose)
        if self.rows is None:self.rows=self.model.sample_triangle(20)
        error=target.error_m+self.s.arrival_error_m+self.s.hug_drift_error_m
        # Reject narrow/boundary targets once in a single solver batch, before
        # expanding each target into many possible body yaws.
        probes=inflated_errors(error*1000)
        batch=np.concatenate([np.asarray(row['xy_mm'])+probes for row in self.rows])
        solutions=[]
        for begin in range(0,len(batch),4000):solutions.extend(self.model.solve_many(batch[begin:begin+4000]))
        interior=[]
        for i,row in enumerate(self.rows):
            neighbourhood=solutions[i*len(probes):(i+1)*len(probes)]
            if all(r is not None for r in neighbourhood) and len({(r['feature'],r['root']) for r in neighbourhood})==1:
                interior.append(row)
        if not interior:raise ValueError('no supported C-band interior at measured-error allowance')
        self.last_grasp_report.update(stage='checking_goals',interior_targets=len(interior))
        possible=[];unobserved=[]
        for index,row in enumerate(interior):
            if index%8==0:time.sleep(.001)
            xy=np.asarray(row['xy_mm']);local=np.array([(xy[1]-self.offset)/1000,-xy[0]/1000,0.])
            for yaw in np.arange(current_pose[2]-np.pi,current_pose[2]+np.pi,.2617993878):
                rotation=body_frame([0,0,yaw],self.body_z,self.roll_pitch)
                nf=rotation@self.node_in_body
                relative=nf[:3,:3]@local+nf[:3,3]
                point=target.at_height(relative[2])
                pose=np.array([*(point-relative)[:2],yaw])
                distance=np.linalg.norm(pose[:2]-current_pose[:2]);turn=abs(angle(yaw-current_pose[2]))
                if not self.grid.footprint_free(pose,self.footprint,margin_m=self.clearance_m):
                    status=self.grid.footprint_status(pose,self.footprint,margin_m=self.clearance_m)
                    # More floor views cannot clear a measured obstacle.
                    if status['blocked_cells']:
                        self.last_grasp_report['blocked_goals']+=1
                    elif status['unknown_cells']:
                        self.last_grasp_report['unknown_goals']+=1
                        unobserved.append((distance+.2*turn,pose))
                    continue
                # Geometric neighbourhood must have room in XY, not just q.
                possible.append((distance+.2*turn,pose,row,distance,turn))
        self.last_grasp_report['observed_goals']=len(possible)
        for _,pose,row,distance,turn in sorted(possible,key=lambda x:x[0])[:48]:
            # Arrival is followed by a fresh observation. Travel drift is
            # charged separately only when a blind final handoff is needed.
            error=target.error_m+self.s.arrival_error_m+self.s.hug_drift_error_m
            report=self.robust_solution(target,pose,error)
            if report is None:continue
            try:path=self.grid.path(current_pose,pose,self.footprint,margin_m=self.clearance_m)
            except ValueError:
                self.last_grasp_report['route_rejections']+=1;continue
            self.last_grasp_report['stage']='path_found'
            return BaseGoal(pose,path,np.array(self.model.meta['home']),dict(
                kind='grasp',**report,error_allowance_m=error,map_revision=self.grid.revision,
                hug_drift_error_m=self.s.hug_drift_error_m,
                target_reach_m=float(np.linalg.norm(target.origin[:2]-pose[:2])),
                base_clearance_m=self.clearance_m,
                robustness='16 boundary probes plus centre; not a continuous certificate'))
        if possible or unobserved:
            self.last_grasp_report['stage']='needs_actual_floor'
            closest=min([(cost,pose) for cost,pose,*_ in possible]+unobserved,key=lambda row:row[0])[1]
            raise FloorObservationRequired(closest)
        self.last_grasp_report['stage']='no_unblocked_goal'
        raise ValueError('no reachable C-band goal with measured-error allowance')
    def observation(self,target,current_pose,camera_evaluator):
        self._require_current_clearance(current_pose)
        candidates=[np.asarray(current_pose,float)]
        center=target.origin[:2]
        bearing=np.arctan2(current_pose[1]-center[1],current_pose[0]-center[0])
        for radius in (.65,.85,1.05):
            for a in bearing+np.deg2rad([-60,-30,0,30,60]):
                p=center+radius*np.array([np.cos(a),np.sin(a)])
                candidates.append(np.r_[p,angle(a+np.pi)])
        ranked=[]
        for pose in candidates:
            if not self.grid.footprint_free(pose,self.footprint,margin_m=self.clearance_m):continue
            report=camera_evaluator(pose,target)
            if report is None or report['views']<3:continue
            cost=np.linalg.norm(pose[:2]-current_pose[:2])+.2*abs(angle(pose[2]-current_pose[2]))
            ranked.append((-report['views'],cost,pose,report))
        for _,_,pose,report in sorted(ranked,key=lambda x:x[:2]):
            try:path=self.grid.path(current_pose,pose,self.footprint,margin_m=self.clearance_m)
            except ValueError:continue
            return BaseGoal(pose,path,np.array(self.model.meta['home']),dict(kind='observation',**report))
        raise ValueError('no observed-free base pose with three distinct camera views')
