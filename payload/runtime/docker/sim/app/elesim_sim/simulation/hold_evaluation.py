"""Independent D100 physical evaluation; object truth never enters Pilot planning."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from .contact_metrics import camera_side_axes_from_frames, classify_faces,assembly_contact_pairs,arm_contact_faults
from .grasp_metrics import HoldTracker, cylinder_clearance

SETTINGS=dict(dt=.01,hold_duration_s=5.,lift_clearance_m=.02,hold_slip_m=.01,
    hold_contact_fraction=.8,lift_roll_target_rad=0.,lift_roll_tolerance_rad=math.radians(5),
    face_force_min_n=.01,face_margin_m=.001,face_normal_tolerance=.15)


def array(raw):
    if hasattr(raw,'detach'):raw=raw.detach().cpu().numpy()
    return np.asarray(raw)


def rotation(quaternion):
    w,x,y,z=np.asarray(quaternion).reshape(4)
    return Rotation.from_quat([x,y,z,w]).as_matrix()


class HoldEvaluation:
    def __init__(self):self.ident=None
    def reset(self,ident):
        self.ident=ident;self.count=0;self.hits=0;self.reference=None;self.confirm=0
        self.grasp=False;self.fault=False;self.succeeded=False
        self.wrong_face=False;self.physical_fault=False;self.joint_indices=None
        self.hold=None
        self.report=dict(id=ident,scope='Sim contact evaluation; unavailable on Robot',success=False)
    def update(self,*,dt,clearance,floor,touch,proper,actual,command,relative,forbidden=False,physical_fault=False):
        self.wrong_face=bool(forbidden);self.physical_fault|=bool(physical_fault)
        self.fault|=bool(physical_fault)
        grasp=all(touch) and proper and not forbidden and actual[2]<0 and actual[3]<0 and not self.fault
        self.confirm=self.confirm+1 if grasp else 0
        self.grasp|=self.confirm*dt>=.2
        if self.hold is None:self.hold=HoldTracker(1,dict(SETTINGS,dt=dt))
        valid_linear=abs(actual[0]*1500)<=3 and abs(command[0]*1500)<=3
        result=self.hold.update(np.array([clearance]),np.array([floor]),np.array([touch]),
            np.asarray([actual]),np.array([self.fault]),np.asarray([relative]),np.array([not valid_linear or forbidden]))
        self.count=int(self.hold.count[0]);fraction=float(self.hold.fraction()[0])
        slip=float(self.hold.maximum_slip[0])
        self.succeeded=bool(self.grasp and result[0])
        self.report.update(success=self.succeeded and not self.fault,hold_s=self.count*dt,
            contact_fraction=fraction,clearance_m=float(clearance),slip_m=slip,forbidden=self.fault,
            wrong_face=self.wrong_face,physical_fault=self.physical_fault)
        return dict(self.report)
    def sample(self,robot,obj,actual,command,dt):
        positions=array(robot.get_links_pos()).reshape(-1,3)
        quats=array(robot.get_links_quat()).reshape(-1,4)
        names={link.name:link for link in robot.links}
        if 'camera' not in names or 'node9' not in names or 'gripper_base' not in names:
            raise RuntimeError('hold evaluator needs camera/node9/gripper model')
        frames={}
        for name,link in names.items():
            frame=np.eye(4);frame[:3,:3]=rotation(quats[link.idx_local]);frame[:3,3]=positions[link.idx_local]
            frames[name]=frame
        axes=camera_side_axes_from_frames(frames)
        robot_links={link.idx:link for link in robot.links};object_links={link.idx for link in obj.links}
        raw=obj.get_contacts(exclude_self_contact=False,is_padded=True)
        values={k:array(v) for k,v in raw.items()}
        valid=values['valid_mask'].reshape(-1);la=values['link_a'].reshape(-1);lb=values['link_b'].reshape(-1)
        normal=values['normal'].reshape(-1,3);position=values['position'].reshape(-1,3)
        force=np.linalg.norm(values['force_a'].reshape(-1,3),axis=1)
        touch=[False,False];wrong=False;floor=False
        for i in np.flatnonzero(valid):
            if force[i]<=1e-4:continue
            a,b=int(la[i]),int(lb[i])
            if a in robot_links and b in object_links:link=robot_links[a];outward=-normal[i]
            elif b in robot_links and a in object_links:link=robot_links[b];outward=normal[i]
            else:floor=True;continue
            if force[i]<=SETTINGS['face_force_min_n']:continue
            name=link.name
            role=(0 if name.startswith('node') and name[4:].isdigit() and int(name[4:])<5 else
                1 if (name.startswith('node') and name[4:].isdigit()) or name.startswith('gripper') else -1)
            if role<0:wrong=True;continue
            proper,bad=classify_faces(position[i],outward,positions[link.idx_local],quats[link.idx_local],axes[name],
                margin=SETTINGS['face_margin_m'],normal_tolerance=SETTINGS['face_normal_tolerance'])
            wrong|=bool(bad);touch[role]|=bool(proper)
        rr={k:array(v) for k,v in robot.get_contacts(exclude_self_contact=False,is_padded=True).items()}
        mask=rr['valid_mask'].reshape(-1)
        ids=[*robot_links,*object_links]
        for key in ('link_a','link_b'):
            if mask.any():ids.extend(rr[key].reshape(-1)[mask].tolist())
        size=max(ids)+1;roles=np.zeros(size,dtype=int);objects=np.zeros(size,dtype=bool)
        objects[list(object_links)]=True
        by_name={link.name:link.idx for link in robot.links}
        for name,index in by_name.items():
            roles[index]=(1 if name.startswith('node') and name[4:].isdigit() and int(name[4:])<5 else
                2 if name in axes else 4 if name=='camera' else 3)
        faults=bool(arm_contact_faults(rr,roles,objects,assembly_contact_pairs(by_name)))
        if faults and 'fault_contacts' not in self.report:
            # Derived fault evidence only. Never publish contact/object XYZ to
            # the planner through this independent evaluation channel.
            detail=[]
            for i in np.flatnonzero(mask):
                one={k:rr[k].reshape(-1,3)[i:i+1] for k in ('normal','position','force_a')}
                one.update({k:rr[k].reshape(-1)[i:i+1] for k in ('link_a','link_b','valid_mask')})
                if 'penetration' in rr:one['penetration']=rr['penetration'].reshape(-1)[i:i+1]
                if not arm_contact_faults(one,roles,objects,assembly_contact_pairs(by_name)):continue
                labels=[]
                for key in ('link_a','link_b'):
                    index=int(one[key][0]);link=robot_links.get(index)
                    labels.append(link.name if link else 'target' if index in object_links else 'environment')
                detail.append(dict(links=labels,force_n=float(np.linalg.norm(one['force_a'])),
                    penetration_m=float(one.get('penetration',np.zeros(1))[0])))
                if len(detail)>=8:break
            self.report['fault_contacts']=detail
        if self.joint_indices is None:
            joints=['j_plate_housing','j_housing_wedge','j_wedge_node0']+[f'j_node{i}_node{i+1}' for i in range(9)]
            self.joint_indices=[int(np.asarray(robot.get_joint(name).dofs_idx_local).item()) for name in joints]
        q=array(robot.get_dofs_position(dofs_idx_local=self.joint_indices)).reshape(12)
        spread=np.ptp(q[2:].reshape(2,5),axis=1)
        joint_fault=bool(not np.isfinite(q).all() or bool(np.any(spread>.05))
            or bool(np.any(abs(q[2:])>math.pi/5+.025)) or not -.173333333334-.003<=q[0]<=.003
            or abs(q[1])>math.pi/2+.03)
        self.report.update(joint_spread_rad=spread.tolist(),joint_bounds_fault=joint_fault)
        faults|=joint_fault
        pos=array(obj.get_pos()).reshape(3);quat=array(obj.get_quat()).reshape(4)
        clearance=float(cylinder_clearance(pos,quat,.05,1.1))
        anchor=names['gripper_base'].idx_local
        relative=rotation(quats[anchor]).T@(pos-positions[anchor])
        return self.update(dt=dt,clearance=clearance,floor=floor,touch=touch,proper=not wrong,
            actual=actual,command=command,relative=relative,forbidden=wrong,physical_fault=faults)
