"""Free four-DoF policy from the real Home preset, through C grasp and lift.

One fixed case per runner, parallel independent replicas. Policy controls every joint command from Home; lift timing is event-driven.
"""
from dataclasses import replace
import math
from pathlib import Path

import elesim_sim.rl  # NumPy before Torch
import numpy as np
import torch
from tensordict import TensorDict

from elesim_sim.rl.arm_kinematics import ArmWaypointMapper
from elesim_sim.rl.execution import MotionClock
from elesim_sim.rl.scene import PhysicsConfig, create_scene, init_genesis
from .configure import fk, materialize_robot, OBSERVATIONS, CONTROL_MODE, DEFAULT_NUM_ENVS
from .configure import CHANNELS
from workbench.research.experiments.wrap_grasp.configs.loader import load_config
from workbench.research.experiments.wrap_grasp.scene import WrapGraspScene


class Scene(WrapGraspScene):
    """Reuse robot indexing/gains only; this task owns entities and materials."""
    def build_task(self, experiment, case, asset_dir, *, video=False):
        import genesis as gs
        s = experiment['settings']
        init_genesis(self.cfg.runtime.backend, s['seed'])
        self.scene = create_scene(PhysicsConfig(s['dt'], s['solver_substeps'], s['max_collision_pairs'],
                                               self.cfg.scene.show_viewer, max_contacts=s['max_contacts']))
        self.floor = self.scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(friction=s['floor_friction']))
        robot_path = materialize_robot(Path(asset_dir)/'robot.urdf')
        self.robot = self.scene.add_entity(gs.morphs.URDF(
            file=str(robot_path), pos=(0, 0, s['spawn_height_m']), fixed=True,
            merge_fixed_links=False, default_armature=0., decompose_robot_error_threshold=.15),
            material=gs.materials.Rigid(friction=s['friction']))
        radius = case['diameter_mm']/2000
        self.object = self.scene.add_entity(
            gs.morphs.Cylinder(radius=radius, height=s['height_m'], pos=case['object_xyz_m'], fixed=False),
            # Genesis mixes pair friction using max: low object/floor friction,
            # high robot friction gives the desired arm/object coefficient.
            material=gs.materials.Rigid(rho=s['mass_kg']/(math.pi*radius**2*s['height_m']), friction=s['floor_friction']))
        self.cameras = {}
        if video:
            self.cameras['report'] = self.scene.add_camera(
                res=(960, 720), pos=(1.35, 1.6, 1.45), lookat=(.40, .05, .5),
                up=(0., 0., 1.), fov=50, GUI=False)
        self.scene.build(n_envs=self.n_envs, env_spacing=(3., 3.), center_envs_at_origin=False)
        self._built = True
        self.robot.set_friction(s['friction'])
        self.object.set_friction(s['floor_friction'])
        self.object.set_mass(s['mass_kg'])
        if not torch.allclose(self.object.get_mass(), torch.full_like(self.object.get_mass(), s['mass_kg']), rtol=1e-4):
            raise RuntimeError('Physical object mass differs from experiment')
        self.floor.set_friction(s['floor_friction'])
        # Do not call legacy friction override: floor friction is deliberately separate.
        from elesim_sim.rl.arm_kinematics import resolve_dof_index
        self.arm_dofs = resolve_dof_index(self.robot, self.cfg.arm)
        self.links = self._resolve_links()
        self._configure_arm_gains(); self._configure_go2_legs()
        self.initial_state = self.scene.get_state()
        return self


class HugEnv:
    num_actions=4

    def __init__(self,experiment,case,asset_dir,*,n_envs=DEFAULT_NUM_ENVS,backend='gpu',viewer=False,
                 auto_reset=True,video=False):
        from .control import action_rates
        from .metrics import HoldTracker,local_link_index
        from .rewards import RewardLedger
        from .contacts import camera_side_axes
        self.experiment,self.case=experiment,case;self.s=s=experiment['settings']
        self.num_envs,self.auto_reset=n_envs,auto_reset
        self.cfg={'task':CONTROL_MODE,'settings':s,'case':case}
        cfg=load_config()
        arm=replace(cfg.arm,limits=replace(cfg.arm.limits,curl_limit_per_node_rad=None,
                    linear_m=(s['linear_min_m'],s['linear_max_m']),roll_rad=(s['roll_min_rad'],s['roll_max_rad']),
                    bend_per_node_rad=s['bend_per_node_rad']),
                    gains=replace(cfg.arm.gains,kp=s['kp'],kv=s['kv'],force_range=s['torque_limit']))
        cfg=replace(cfg,arm=arm,runtime=replace(cfg.runtime,n_envs=n_envs,backend=backend,
                    torch_device='cuda' if backend=='gpu' else 'cpu'),
                    scene=replace(cfg.scene,show_viewer=viewer,friction=None,
                                  go2=replace(cfg.scene.go2,spawn_xyz=(0.,0.,s['spawn_height_m']))))
        self.scene=Scene(cfg,n_envs=n_envs).build_task(experiment,case,asset_dir,video=video)
        self.device=self.scene.device
        self.mapper=ArmWaypointMapper(arm,n_envs=n_envs,device=self.device,rate_limit=action_rates(s))
        self.dofs=self.scene.arm_dofs.all_indices;self.max_episode_length=s['episode_steps']
        self.clock=MotionClock(s['dt'],s['action_ticks'],1.)
        self.episode_length_buf=torch.zeros(n_envs,dtype=torch.long,device=self.device)
        self.ticks=torch.zeros_like(self.episode_length_buf)
        self.fault=torch.zeros(n_envs,dtype=torch.bool,device=self.device)
        self.success=torch.zeros_like(self.fault)
        self.grasp_tick=torch.full_like(self.ticks,-1)
        self.grasp_ticks=torch.zeros_like(self.ticks)
        self.hold=HoldTracker(n_envs,s,torch,self.device)
        self.ledger=RewardLedger(n_envs,s,torch,self.device)
        self.last_touch=torch.zeros((n_envs,2),device=self.device)
        self.last_fault=torch.zeros_like(self.fault)
        self.last_wrong_face=torch.zeros_like(self.fault);self.last_floor=torch.zeros_like(self.fault)
        self.drop_armed=torch.zeros_like(self.fault)
        self.macro_wrong=torch.zeros(n_envs,device=self.device);self.macro_drops=torch.zeros_like(self.macro_wrong)
        self.record=None;self.record_env_indices=None;self.tick_monitor=None
        names=self.scene.links.name_by_index
        self.proximal=self._indices([k for k,n in names.items() if n in {f'node{i}' for i in range(5)}])
        self.distal=self._indices([k for k,n in names.items() if n in {f'node{i}' for i in range(5,10)} or n.startswith('gripper')])
        self.moving=torch.cat((self.proximal,self.distal))
        self.protected=self._indices([k for k,n in names.items() if n in ('housing','wedge','camera') or k in self.scene.links.go2])
        self.camera=self._indices([k for k,n in names.items() if n=='camera'])
        self.obj=self._indices(self.scene.links.object_);self.floor=self._indices(self.scene.links.floor)
        size=max(names)+1
        self.contact_local=torch.zeros(size,dtype=torch.long,device=self.device)
        self.contact_axes=torch.zeros((size,3),device=self.device)
        self.contact_roles=torch.zeros(size,dtype=torch.long,device=self.device)
        self.contact_objects=torch.zeros(size,dtype=torch.bool,device=self.device);self.contact_objects[self.obj]=True
        axes=camera_side_axes()
        for link in self.scene.robot.links:
            self.contact_local[link.idx]=link.idx_local
            if link.name in axes:self.contact_axes[link.idx]=torch.tensor(axes[link.name],device=self.device)
        self.contact_roles[self.proximal]=1;self.contact_roles[self.distal]=2
        self.anchor=local_link_index(self.scene.robot,'gripper_base')
        from .workspace import marker_geometry
        self.workspace_owners,markers=marker_geometry()
        self.workspace_markers=torch.tensor(markers,device=self.device,dtype=torch.float32)
        self.workspace_links=torch.tensor([local_link_index(self.scene.robot,name) for name in self.workspace_owners],device=self.device)
        self.workspace_exposure=torch.zeros(n_envs,device=self.device)
        self.workspace_peak=torch.zeros(n_envs,device=self.device)
        by_name={v:k for k,v in names.items()}
        pairs=[(by_name[f'node{i}'],by_name[f'node{i+1}']) for i in range(9)]
        pairs.append((by_name['wedge'],by_name['node0']))
        rigid=[by_name[n] for n in ('node9','gripper_base','gripper_claw_left','gripper_claw_right','camera') if n in by_name]
        pairs += [(a,b) for i,a in enumerate(rigid) for b in rigid[i+1:]]
        self.assembly_pairs=pairs
        self.set_case(case);self.reset();self.verify_fk()

    def _indices(self, values):
        return torch.tensor(list(values),dtype=torch.long,device=self.device)


    def set_case(self, case):
        if case['diameter_mm']!=self.case['diameter_mm']:
            raise ValueError('Rebuild scene for a different cylinder diameter')
        self.case=case;self.cfg['case']=case
        self.start=torch.tensor(case['start_command'],device=self.device,dtype=torch.float32)
        self.goal=torch.tensor(case['goal_command'],device=self.device,dtype=torch.float32)
        self.origin=torch.tensor(case['object_xyz_m'],device=self.device,dtype=torch.float32)
        from .workspace import envelope
        self.workspace_polygon=torch.tensor(envelope(case,self.s,self.workspace_owners,
            self.workspace_markers.detach().cpu().numpy()),device=self.device,dtype=torch.float32)


    def reset(self, ids=None):
        ids=torch.arange(self.num_envs,device=self.device) if ids is None else ids;n=len(ids)
        self.scene.scene.reset(self.scene.initial_state,envs_idx=ids)
        self.mapper.waypoint[ids]=self.start
        targets=self.mapper.joint_targets()[ids]
        self.scene.robot.set_dofs_position(targets,dofs_idx_local=self.dofs,envs_idx=ids,zero_velocity=True)
        self.scene.robot.control_dofs_position(targets,dofs_idx_local=self.dofs,envs_idx=ids)
        legs=self.scene.go2_leg_dofs
        if legs:
            pose=torch.tensor(self.scene.cfg.scene.go2.leg_pose_rad,device=self.device).repeat(len(legs)//3).expand(n,len(legs))
            self.scene.robot.set_dofs_position(pose,dofs_idx_local=legs,envs_idx=ids,zero_velocity=True)
            self.scene.robot.control_dofs_position(pose,dofs_idx_local=legs,envs_idx=ids)
        self.scene.object.set_pos(self.origin.expand(n,3),envs_idx=ids)
        self.scene.object.set_quat(torch.tensor([1.,0,0,0],device=self.device).expand(n,4),envs_idx=ids)
        self.scene.object.set_dofs_velocity(torch.zeros((n,6),device=self.device),envs_idx=ids)
        for a in (self.episode_length_buf,self.ticks,self.fault,self.success,self.last_touch,self.last_fault,
                  self.grasp_ticks,self.last_wrong_face,self.last_floor,self.drop_armed,self.macro_wrong,self.macro_drops):a[ids]=0
        self.workspace_exposure[ids]=0;self.workspace_peak[ids]=0
        self.grasp_tick[ids]=-1;self.hold.reset(ids);self.ledger.reset(ids,self.start.expand(n,4),self.goal)
        return self.get_observations(),{}


    def verify_fk(self):
        import xml.etree.ElementTree as ET
        from .configure import BUNDLE
        root=ET.parse(BUNDLE/'robot.urdf').getroot()
        joints={self.scene.cfg.arm.linear_joint:float(self.start[0]),self.scene.cfg.arm.roll_joint:float(self.start[1])}
        joints.update({name:float(self.start[2 if i<5 else 3]) for i,name in enumerate(self.scene.cfg.arm.bend_joints)})
        pos=self.scene.robot.get_links_pos()[0].detach().cpu().numpy()
        for link in self.scene.robot.links:
            if link.name in ('node0','node4','node9'):
                expected=fk(root,link.name,joints)[:3,3]+[0,0,self.s['spawn_height_m']]
                if np.linalg.norm(pos[link.idx_local]-expected)>.002:raise RuntimeError(f'Home FK mismatch: {link.name}')


    def actual(self):
        q=self.scene.robot.get_dofs_position(dofs_idx_local=self.dofs)
        return torch.stack((q[:,0],q[:,1],q[:,2:7].mean(1),q[:,7:12].mean(1)),dim=1)


    def get_observations(self):
        scale=torch.tensor([1.,1.,5.,5.],device=self.device)
        policy=torch.cat((self.actual()*scale,self.mapper.waypoint*scale,
            (self.goal[2:]*5).expand(self.num_envs,2),self.relative_object(),self.scene.object.get_vel(),
            torch.full((self.num_envs,1),self.case['diameter_mm']/1000,device=self.device),
            (self.grasp_tick>=0).float()[:,None],
            (self.hold.count/self.hold.required_ticks).clamp(0,1)[:,None],
            (self.ticks/(self.max_episode_length*self.s['action_ticks']))[:,None]),dim=1)
        if policy.shape[1]!=len(OBSERVATIONS) or not torch.isfinite(policy).all():raise RuntimeError('Invalid C-hug observation')
        privileged=torch.cat((self.last_touch,self.last_wrong_face.float()[:,None],
            (self.grasp_ticks/(self.s['grasp_confirm_s']/self.s['dt'])).clamp(0,1)[:,None],
            self.ledger.best_lift[:,None],self.ledger.best_hold[:,None],
            self.ledger.grasp_paid.float()[:,None],self.drop_armed.float()[:,None]),dim=1)
        return TensorDict({'policy':policy,'privileged':privileged},batch_size=[self.num_envs])

    def contacts(self):
        raw=self.scene.robot.get_contacts(exclude_self_contact=False,is_padded=True)
        valid=raw['valid_mask'].to(self.device)
        if valid.shape[-1]==0:return torch.zeros_like(self.last_touch),torch.zeros_like(self.fault),torch.zeros_like(self.fault)
        a,b=raw['link_a'].to(self.device).long(),raw['link_b'].to(self.device).long()
        force=raw['force_a'].to(self.device).norm(dim=-1);active=valid&(force>1e-4)
        def pair(l,r):return (torch.isin(a,l)&torch.isin(b,r))|(torch.isin(b,l)&torch.isin(a,r))
        from .contacts import object_contact_faces
        touch,wrong=object_contact_faces(raw,self.scene.robot.get_links_pos(),self.scene.robot.get_links_quat(),
            self.contact_local,self.contact_axes,self.contact_roles,self.contact_objects,self.s,torch)
        assembly=torch.zeros_like(valid)
        for l,r in self.assembly_pairs:assembly|=((a==l)&(b==r))|((a==r)&(b==l))
        forbidden=pair(self.moving,self.protected)|pair(self.moving,self.moving)|pair(self.obj,self.protected)
        forbidden|=pair(self.moving,self.floor)|pair(self.camera,self.floor)|pair(self.camera,self.protected)
        relevant=torch.isin(a,self.moving)|torch.isin(b,self.moving)|torch.isin(a,self.camera)|torch.isin(b,self.camera)|pair(self.obj,self.protected)
        bad=(active&forbidden&~assembly).any(1)
        bad|=(valid&relevant&~assembly&(raw['penetration'].to(self.device)>self.s['max_penetration_m'])).any(1)
        bad|=(valid&~torch.isfinite(force)).any(1)
        bad|=(active&pair(self.moving,self.obj)&(~torch.isfinite(raw['normal']).all(-1)
              |~torch.isfinite(raw['position']).all(-1)|(raw['normal'].norm(dim=-1)<.5))).any(1)
        return touch.float(),bad,wrong


    def object_clearance(self):
        from .metrics import cylinder_clearance
        return cylinder_clearance(self.scene.object.get_pos(),self.scene.object.get_quat(),
                                  self.case['diameter_mm']/2000,self.s['height_m'],torch)


    def relative_object(self):
        anchor=self.scene.robot.get_links_pos()[:,self.anchor];quat=self.scene.robot.get_links_quat()[:,self.anchor]
        v=self.scene.object.get_pos()-anchor;u=-quat[:,1:];t=2*torch.cross(u,v,dim=1)
        return v+quat[:,0:1]*t+torch.cross(u,t,dim=1)


    def floor_contact(self):
        raw=self.scene.object.get_contacts(exclude_self_contact=False,is_padded=True)
        v=raw['valid_mask'].to(self.device)
        a,b=raw['link_a'].to(self.device).long(),raw['link_b'].to(self.device).long()
        f=raw['force_a'].to(self.device).norm(dim=-1)
        return (v&(f>1e-4)&(torch.isin(a,self.floor)|torch.isin(b,self.floor))).any(1)


    def tick(self,command):
        self.mapper.waypoint[:]=command
        self.scene.robot.control_dofs_position(self.mapper.joint_targets(),dofs_idx_local=self.dofs)
        self.scene.step();self.ticks+=1
        from .workspace import transform_markers,workspace_excess,workspace_fraction
        markers=transform_markers(self.scene.robot.get_links_pos()[:,self.workspace_links],
            self.scene.robot.get_links_quat()[:,self.workspace_links],self.workspace_markers,torch)
        excess=workspace_excess(markers,self.workspace_polygon,self.s,torch)
        self.workspace_exposure+=workspace_fraction(excess,self.s,torch)
        self.workspace_peak[:]=torch.maximum(self.workspace_peak,excess)
        touch,bad,wrong=self.contacts();self.last_touch,self.last_fault,self.last_wrong_face=touch,bad,wrong
        self.macro_wrong+=wrong.float()
        raw=self.scene.robot.get_dofs_position(dofs_idx_local=self.dofs)
        bad|=(raw[:,2:].abs()>self.s['bend_per_node_rad']+.025).any(1)
        bad|=(raw[:,0]<self.s['linear_min_m']-.003)|(raw[:,0]>self.s['linear_max_m']+.003)
        bad|=(raw[:,1]<self.s['roll_min_rad']-.03)|(raw[:,1]>self.s['roll_max_rad']+.03)
        for group in (raw[:,2:7],raw[:,7:12]):bad|=group.max(1).values-group.min(1).values>self.s['max_joint_spread_rad']
        pos=self.scene.object.get_pos()
        if not torch.isfinite(pos).all() or not torch.isfinite(raw).all() or not torch.isfinite(self.scene.object.get_quat()).all():raise RuntimeError('Nonfinite C-hug physics')
        bad|=(pos-self.origin).norm(dim=1)>1.
        self.fault|=bad
        actual=self.actual();c_shape=(actual[:,2]<0)&(actual[:,3]<0)
        grasp=(touch[:,0]>0)&(touch[:,1]>0)&c_shape&~self.fault&~wrong
        self.grasp_ticks=torch.where(grasp,self.grasp_ticks+1,torch.zeros_like(self.grasp_ticks))
        confirmed=self.grasp_ticks>=round(self.s['grasp_confirm_s']/self.s['dt'])
        self.grasp_tick=torch.where(confirmed&(self.grasp_tick<0),self.ticks,self.grasp_tick)
        self.last_floor=self.floor_contact();clearance=self.object_clearance()
        valid_hold=self.hold.update(clearance,self.last_floor,touch,actual,self.fault,self.relative_object(),wrong)
        dropped=self.drop_armed&self.hold.interrupted
        self.macro_drops+=dropped.float();self.drop_armed&=~dropped
        self.drop_armed|=confirmed&(clearance>=self.s['lift_clearance_m'])&~self.last_floor&~self.fault&~wrong
        self.success[:]=valid_hold&(self.grasp_tick>=0)
        if self.record is not None:
            recorded=command[0] if self.record_env_indices is None else command[self.record_env_indices]
            self.record.append(recorded.detach().cpu().numpy().copy())
        if self.tick_monitor is not None:self.tick_monitor(self)

    def outcome(self):
        from .control import pose_error
        from .metrics import lift_roll_error,lift_posture_complete,lift_posture_fraction
        actual=self.actual()
        return dict(success=self.success&~self.fault,fault=self.fault,
            current_hold_duration_s=self.hold.count*self.s['dt'],
            minimum_hold_clearance_m=self.hold.minimum_clearance,
            maximum_hold_slip_m=self.hold.maximum_slip,hold_contact_fraction=self.hold.fraction(),
            goal_error_normalized=pose_error(actual,self.goal,self.s,torch),
            lift_roll_error_rad=lift_roll_error(actual,self.s,torch),
            lift_posture_complete=lift_posture_complete(actual,self.s,torch),
            lift_posture_fraction=lift_posture_fraction(actual,self.s,torch),
            wrong_face_contact=self.last_wrong_face,grasp_confirmed=self.grasp_ticks>=round(self.s['grasp_confirm_s']/self.s['dt']),
            wrong_face_contact_any=self.macro_wrong>0,
            highest_lift_fraction=self.ledger.best_lift,highest_hold_fraction=self.ledger.best_hold,
            workspace_peak_excess_m=self.workspace_peak,
            reward_total=sum(self.ledger.totals.values()))

    def step(self,actions):
        from .control import advance_command
        if actions.shape!=(self.num_envs,4) or not torch.isfinite(actions).all():raise RuntimeError('Expected finite four-DoF policy actions')
        self.begin_action()
        start=self.mapper.waypoint.clone();end=advance_command(start,actions,self.s,torch)
        for _,command in self.clock.targets(start,end):self.tick(command)
        reward,done,terms=self.finish_action(start,actions)
        result=self.outcome()
        extras={'time_outs':torch.zeros_like(done),'log':{'c_hug/success':result['success'].float().mean(),
                'c_hug/fault':self.fault.float().mean(),'c_hug/hold_s':(self.hold.count.float()*self.s['dt']).mean(),
                'c_hug/wrong_face_fraction':(self.macro_wrong/self.s['action_ticks']).mean(),
                **{'reward/'+k:v.float().mean() for k,v in terms.items()}}}
        if self.auto_reset and done.any():self.reset(done.nonzero(as_tuple=False).flatten())
        return self.get_observations(),reward,done,extras

    def begin_action(self):
        self.macro_wrong.zero_();self.macro_drops.zero_();self.workspace_exposure.zero_()

    def finish_action(self,start,requested_actions=None):
        from .control import action_rates
        self.episode_length_buf+=1
        self.success&=(self.macro_wrong==0)&~self.fault
        confirmed=self.grasp_ticks>=round(self.s['grasp_confirm_s']/self.s['dt'])
        lift=(self.object_clearance()/self.s['lift_clearance_m']).clamp(0,1)
        lift=torch.where(~self.last_floor,lift,torch.zeros_like(lift))
        hold=(self.hold.count/self.hold.required_ticks).clamp(0,1)
        hold*=((self.hold.fraction()>=self.s['hold_contact_fraction'])&(self.grasp_tick>=0)).float()
        applied=(self.mapper.waypoint-start)/start.new_tensor(action_rates(self.s))
        # Policy rollouts charge requested effort even if joint bounds clip it.
        # A command-only trajectory replay cannot reconstruct rejected requests.
        effort=applied if requested_actions is None else requested_actions
        self.effort_basis='applied-command lower bound' if requested_actions is None else 'requested-policy-action'
        return self.ledger.step(self.actual(),self.goal,effort,confirmed_grasp=confirmed,
            lift_fraction=lift,hold_fraction=hold,wrong_fraction=self.macro_wrong/self.s['action_ticks'],
            fault=self.fault,success=self.success,timeout=self.episode_length_buf>=self.max_episode_length,drops=self.macro_drops,
            workspace_fraction=self.workspace_exposure/self.s['action_ticks'])

    def measure(self):
        result={k:v[0].detach().cpu().item() for k,v in self.outcome().items()}
        if not math.isfinite(result['minimum_hold_clearance_m']):result['minimum_hold_clearance_m']=None
        result.update(object_xyz_m=self.scene.object.get_pos()[0].detach().cpu().tolist(),
            actual_command=self.actual()[0].detach().cpu().tolist(),clearance_m=float(self.object_clearance()[0]),
            action_cost_basis=getattr(self,'effort_basis','no policy step'))
        result['reward_components']={k:float(v[0]) for k,v in self.ledger.totals.items()}
        return result

    def metadata(self):
        return dict(self.case,simulation_context=self.experiment['simulation_context'],control_mode=CONTROL_MODE,
            duration_s=int(self.ticks[0])*self.s['dt'],episode_limit_s=self.max_episode_length*self.clock.duration,
            grasp_time_s=None if int(self.grasp_tick[0])<0 else int(self.grasp_tick[0])*self.s['dt'])


class ReplayAdapter:
    def __init__(self,env,offset_xy_mm=(0.,0.)):
        self.env=env;self.command_tick=-1;self.previous=None
        offset=np.asarray(offset_xy_mm,float)
        if offset.shape!=(2,) or not np.isfinite(offset).all():raise ValueError('Invalid object XY offset')
        self.object_offset=[offset[1]/1000,-offset[0]/1000,0.]

    def reset(self,metadata):
        from .configure import episode_duration
        if metadata.get('control_mode')!=CONTROL_MODE or metadata['simulation_context']!=self.env.experiment['simulation_context']:raise ValueError('Replay context/control-mode mismatch')
        if not 0<metadata['duration_s']<=episode_duration(self.env.s)+1e-8:raise ValueError('Invalid replay duration')
        case=next(c for c in self.env.experiment['cases'] if c['case_id']==metadata['case_id'])
        for key in ('q_deg','xy_mm','diameter_mm','goal_command','start_command'):
            if metadata[key]!=case[key]:raise ValueError(f'Replay case mismatch: {key}')
        self.env.set_case(case);self.env.reset();self.env.verify_fk();self.command_tick=-1;self.previous=None
        offset=torch.tensor(self.object_offset,device=self.env.device,dtype=torch.float32)
        self.env.scene.object.set_pos((self.env.origin+offset).expand(self.env.num_envs,3))

    def command(self,channels,values):
        from .configure import validate_command
        if tuple(channels)!=CHANNELS:raise ValueError('Unknown command convention')
        self.command_tick+=1
        value=validate_command(self.env.case,self.env.s,self.command_tick,values,self.previous)
        self.previous=value.copy()
        self.commanded=torch.tensor(value,device=self.env.device,dtype=torch.float32).reshape(1,4)

    def step(self):
        if int(self.env.ticks[0])%self.env.s['action_ticks']==0:
            self.env.begin_action();self.action_start=self.env.mapper.waypoint.clone()
        self.env.tick(self.commanded)
        if int(self.env.ticks[0])%self.env.s['action_ticks']==0:self.env.finish_action(self.action_start)
    def measure(self):return self.env.measure()
    def evaluate(self,samples):return self.env.measure()
