"""Two-action, fixed-roll cylinder hug MDP, outside the EleSim payload.

One fixed case per runner, parallel independent replicas. Training freezes the
command for the final hold window; collection and replay use identical ticks.
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
from .configure import fk, materialize_robot
from .interpolate import CHANNELS
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
                res=(960, 720), pos=(.55, 0., 1.9), lookat=(.55, 0., .52),
                up=(0., 1., 0.), fov=50, GUI=False)
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
    num_actions = 2

    def __init__(self, experiment, case, asset_dir, *, n_envs=256, backend='gpu', viewer=False, auto_reset=True,
                 video=False):
        self.experiment, self.case = experiment, case
        self.s = s = experiment['settings']
        self.cfg = {'task': 'hug_interpolation', 'settings': s, 'case': case}
        self.num_envs, self.auto_reset = n_envs, auto_reset
        cfg = load_config()
        arm = replace(cfg.arm, limits=replace(cfg.arm.limits, curl_limit_per_node_rad=None,
                      bend_per_node_rad=math.pi/5), gains=replace(cfg.arm.gains,
                      kp=s['kp'], kv=s['kv'], force_range=s['torque_limit']))
        cfg = replace(cfg, arm=arm, runtime=replace(cfg.runtime, n_envs=n_envs, backend=backend,
                      torch_device='cuda' if backend == 'gpu' else 'cpu'),
                      scene=replace(cfg.scene, show_viewer=viewer, friction=None,
                                    go2=replace(cfg.scene.go2, spawn_xyz=(0., 0., s['spawn_height_m']))))
        self.scene = Scene(cfg, n_envs=n_envs).build_task(experiment, case, asset_dir, video=video)
        self.device = self.scene.device
        self.mapper = ArmWaypointMapper(arm, n_envs=n_envs, device=self.device, rate_limit=(0, 0, s['action_delta_rad'], s['action_delta_rad']))
        self.dofs = self.scene.arm_dofs.all_indices
        self.max_episode_length = s['approach_steps']+s['hold_steps']
        self.episode_length_buf = torch.zeros(n_envs, dtype=torch.long, device=self.device)
        self.ticks = torch.zeros_like(self.episode_length_buf)
        self.clock = MotionClock(s['dt'], s['action_ticks'], 1.)
        self.fault = torch.zeros(n_envs, dtype=torch.bool, device=self.device)
        self.hold_hits = torch.zeros(n_envs, device=self.device)
        self.hold_pos = torch.zeros((n_envs, 3), device=self.device)
        self.last_touch = torch.zeros((n_envs, 2), device=self.device)
        self.last_fault = torch.zeros(n_envs, dtype=torch.bool, device=self.device)
        self.record = None
        self.tick_monitor = None
        names = self.scene.links.name_by_index
        self.proximal = self._indices([k for k, name in names.items() if name in {f'node{i}' for i in range(5)}])
        self.distal = self._indices([k for k, name in names.items() if name in {f'node{i}' for i in range(5, 10)} or name.startswith('gripper')])
        self.moving = torch.cat((self.proximal, self.distal))
        self.protected = self._indices([k for k, name in names.items() if name in ('housing', 'wedge', 'camera') or k in self.scene.links.go2])
        self.camera = self._indices([k for k, name in names.items() if name == 'camera'])
        self.obj = self._indices(self.scene.links.object_)
        self.floor = self._indices(self.scene.links.floor)
        self.set_case(case)
        self.reset()
        self.verify_fk()

    def _indices(self, values):
        return torch.tensor(list(values), dtype=torch.long, device=self.device)

    def set_case(self, case):
        if case['diameter_mm'] != self.case['diameter_mm']:
            raise ValueError('Rebuild scene for a different diameter; never relabel the existing cylinder')
        self.case = case
        self.cfg['case'] = case
        self.start = torch.tensor(case['start_command'], device=self.device, dtype=torch.float32)
        self.goal = torch.tensor(case['goal_command'], device=self.device, dtype=torch.float32)
        self.origin = torch.tensor(case['object_xyz_m'], device=self.device, dtype=torch.float32)

    def reset(self, ids=None):
        ids = torch.arange(self.num_envs, device=self.device) if ids is None else ids
        n = len(ids)
        # Restore solver state as well as visible poses; previous-case contact
        # history must not leak into replay or a new training episode.
        self.scene.scene.reset(self.scene.initial_state, envs_idx=ids)
        self.mapper.waypoint[ids] = self.start
        targets = self.mapper.joint_targets()[ids]
        self.scene.robot.set_dofs_position(targets, dofs_idx_local=self.dofs, envs_idx=ids, zero_velocity=True)
        self.scene.robot.control_dofs_position(targets, dofs_idx_local=self.dofs, envs_idx=ids)
        legs = self.scene.go2_leg_dofs
        if legs:
            pose = torch.tensor(self.scene.cfg.scene.go2.leg_pose_rad, device=self.device).repeat(len(legs)//3)
            pose = pose.expand(n, len(legs))
            self.scene.robot.set_dofs_position(pose, dofs_idx_local=legs, envs_idx=ids, zero_velocity=True)
            self.scene.robot.control_dofs_position(pose, dofs_idx_local=legs, envs_idx=ids)
        self.scene.object.set_pos(self.origin.expand(n, 3), envs_idx=ids)
        self.scene.object.set_quat(torch.tensor([1., 0, 0, 0], device=self.device).expand(n, 4), envs_idx=ids)
        # No permissive fallback: residual velocity corrupts the experiment.
        self.scene.object.set_dofs_velocity(torch.zeros((n, 6), device=self.device), envs_idx=ids)
        self.episode_length_buf[ids] = 0; self.ticks[ids] = 0
        self.fault[ids] = False; self.hold_hits[ids] = 0
        self.hold_pos[ids] = self.origin
        self.last_touch[ids] = 0; self.last_fault[ids] = False
        return self.get_observations(), {}

    def verify_fk(self):
        """Measured neutral node frames must match URDF at each roll before learning."""
        import xml.etree.ElementTree as ET
        from .configure import BUNDLE
        root = ET.parse(BUNDLE/'robot.urdf').getroot()
        joints = {self.scene.cfg.arm.roll_joint: float(self.start[1])}
        joints.update({name: float(self.start[2 if i < 5 else 3])
                       for i, name in enumerate(self.scene.cfg.arm.bend_joints)})
        positions = self.scene.robot.get_links_pos()[0].detach().cpu().numpy()
        for link in self.scene.robot.links:
            if link.name not in ('node0', 'node4', 'node9'):
                continue
            expected = fk(root, link.name, joints)[:3, 3]+[0, 0, self.s['spawn_height_m']]
            if np.linalg.norm(positions[link.idx_local]-expected) > .002:
                raise RuntimeError(f'Genesis/URDF FK mismatch at {link.name}; do not train')

    def actual(self):
        q = self.scene.robot.get_dofs_position(dofs_idx_local=self.dofs)
        return torch.stack((q[:, 2:7].mean(1), q[:, 7:12].mean(1)), dim=1)

    def get_observations(self):
        q = self.actual()
        policy = torch.cat((q, self.mapper.waypoint[:, 2:], self.goal[2:].expand(self.num_envs, 2),
                            (self.episode_length_buf/self.max_episode_length).unsqueeze(1)), dim=1)
        privileged = torch.cat((self.scene.object.get_pos()-self.origin,
                                self.scene.object.get_vel(), self.last_touch), dim=1)
        if not torch.isfinite(policy).all() or not torch.isfinite(privileged).all():
            raise RuntimeError('Nonfinite observation; physics run aborted')
        return TensorDict({'policy': policy, 'privileged': privileged}, batch_size=[self.num_envs])

    def contacts(self):
        raw = self.scene.robot.get_contacts(exclude_self_contact=False, is_padded=True)
        valid = raw['valid_mask'].to(self.device)
        if valid.shape[-1] == 0:
            return torch.zeros_like(self.last_touch), torch.zeros_like(self.fault)
        a, b = raw['link_a'].to(self.device).long(), raw['link_b'].to(self.device).long()
        # Positive normal load, not just membership in the broad-phase buffer.
        force = raw['force_a'].to(self.device).norm(dim=-1)
        active = valid & (force > 1e-4)
        def pair(left, right):
            return ((torch.isin(a, left) & torch.isin(b, right)) |
                    (torch.isin(b, left) & torch.isin(a, right)))
        touch = torch.stack(((active & pair(self.proximal, self.obj)).any(1),
                             (active & pair(self.distal, self.obj)).any(1)), dim=1).float()
        forbidden = pair(self.moving, self.protected) | pair(self.obj, self.protected)
        forbidden |= pair(self.moving, self.floor) | pair(self.moving, self.moving)
        forbidden |= pair(self.camera, self.floor) | pair(self.camera, self.protected)
        penetration = raw['penetration'].to(self.device)
        relevant = torch.isin(a, self.moving) | torch.isin(b, self.moving)
        relevant |= torch.isin(a, self.camera) | torch.isin(b, self.camera) | pair(self.obj, self.protected)
        bad = (active & forbidden).any(1) | (valid & relevant & (penetration > self.s['max_penetration_m'])).any(1)
        bad |= (~torch.isfinite(force) & valid).any(1)
        return touch, bad

    def tick(self, command):
        s = self.s
        hold_start = s['approach_steps']*s['action_ticks']
        begin = self.ticks == hold_start
        ids = begin.nonzero(as_tuple=False).flatten()
        if len(ids):
            self.hold_pos[ids] = self.scene.object.get_pos()[ids]
            velocity = self.scene.object.get_dofs_velocity()[ids].clone()
            velocity[:, 1] += s['kick_velocity_m_s']
            self.scene.object.set_dofs_velocity(velocity, envs_idx=ids)
        self.mapper.waypoint[:] = command
        self.scene.robot.control_dofs_position(self.mapper.joint_targets(), dofs_idx_local=self.dofs)
        self.scene.step()
        self.ticks += 1
        touch, bad = self.contacts()
        self.last_touch, self.last_fault = touch, bad
        pos, q = self.scene.object.get_pos(), self.actual()
        quat = self.scene.object.get_quat()
        axis_z = 1-2*(quat[:, 1]**2+quat[:, 2]**2)
        bad |= (pos-self.origin).norm(dim=1) > s['max_displacement_m']
        bad |= axis_z < math.cos(math.radians(s['max_tilt_deg']))
        raw_q = self.scene.robot.get_dofs_position(dofs_idx_local=self.dofs)
        bad |= (raw_q[:, 2:].abs() > math.pi/5+.025).any(1)
        bad |= raw_q[:, 0].abs() > .003
        bad |= (raw_q[:, 1]-self.start[1]).abs() > .03
        for group in (raw_q[:, 2:7], raw_q[:, 7:12]):
            bad |= group.max(1).values-group.min(1).values > s['max_joint_spread_rad']
        if not torch.isfinite(pos).all() or not torch.isfinite(q).all():
            raise RuntimeError('Nonfinite physics state; abort rather than train on invalid observations')
        bad |= q[:, 1] > .025
        self.fault |= bad
        holding = self.ticks > hold_start
        self.hold_hits += holding.float()*(touch.min(1).values > 0).float()
        if self.record is not None:
            self.record.append(command[0].detach().cpu().numpy().copy())
        if self.tick_monitor is not None:
            self.tick_monitor(self)

    def outcome(self):
        s = self.s
        error = (self.actual()-self.goal[2:]).abs().max(1).values
        retention = (self.scene.object.get_pos()-self.hold_pos).norm(dim=1)
        fraction = self.hold_hits/(s['hold_steps']*s['action_ticks'])
        complete = self.ticks >= self.max_episode_length*s['action_ticks']
        success = complete & ~self.fault & (error <= s['goal_tolerance_rad'])
        success &= (retention <= s['hold_displacement_m']) & (fraction >= s['hold_contact_fraction'])
        return {'success': success, 'fault': self.fault, 'goal_error_rad': error,
                'hold_displacement_m': retention, 'hold_contact_fraction': fraction}

    def step(self, actions):
        if not torch.isfinite(actions).all():
            raise RuntimeError('Nonfinite policy action')
        before = (self.actual()-self.goal[2:]).norm(dim=1)
        start = self.mapper.waypoint.clone(); end = start.clone()
        delta = actions.to(self.device).clamp(-1, 1)*self.s['action_delta_rad']
        delta *= (self.episode_length_buf < self.s['approach_steps']).unsqueeze(1)
        end[:, 2:] = (end[:, 2:]+delta).clamp(-math.pi/5, math.pi/5)
        end[:, 3].clamp_(max=0.)
        for _, command in self.clock.targets(start, end):
            self.tick(command)
        self.episode_length_buf += 1
        after = (self.actual()-self.goal[2:]).norm(dim=1)
        result = self.outcome()
        done = self.episode_length_buf >= self.max_episode_length
        reward = 10*(before-after)+.1*torch.exp(-after/.1)+.05*self.last_touch.sum(1)
        reward -= .01*(actions**2).sum(1)+self.fault.float()
        reward += done.float()*result['success'].float()*10
        extras = {'time_outs': torch.zeros_like(done), 'log': {'hug/goal_error': after.mean(),
                  'hug/fault': self.fault.float().mean(), 'hug/success': result['success'].float().mean()}}
        if self.auto_reset and done.any():
            self.reset(done.nonzero(as_tuple=False).flatten())
        return self.get_observations(), reward, done, extras

    def measure(self):
        result = {key: value[0].detach().cpu().item() for key, value in self.outcome().items()}
        q = self.scene.robot.get_dofs_position(dofs_idx_local=self.dofs)[0].detach().cpu().numpy()
        result.update(object_xyz_m=self.scene.object.get_pos()[0].detach().cpu().tolist(),
                      actual_linear_m=float(q[0]), actual_roll_rad=float(q[1]),
                      segment_spread_rad=[float(np.ptp(q[2:7])), float(np.ptp(q[7:12]))],
                      actual_bend_rad=self.actual()[0].detach().cpu().tolist())
        return result

    def metadata(self):
        return dict(self.case, simulation_context=self.experiment['simulation_context'],
                    duration_s=self.max_episode_length*self.clock.duration,
                    motion_end_s=self.s['approach_steps']*self.clock.duration)


class ReplayAdapter:
    def __init__(self, env):
        self.env = env

    def reset(self, metadata):
        if metadata['simulation_context'] != self.env.experiment['simulation_context']:
            raise ValueError('Replay context mismatch')
        case = next(c for c in self.env.experiment['cases'] if c['case_id'] == metadata['case_id'])
        for key in ('xy_mm', 'diameter_mm', 'roll_deg', 'goal_command', 'start_command'):
            if metadata[key] != case[key]:
                raise ValueError(f'Replay case mismatch: {key}')
        duration = self.env.max_episode_length*self.env.clock.duration
        if abs(metadata['duration_s']-duration) > 1e-8:
            raise ValueError('Replay duration differs from experiment')
        self.env.set_case(case); self.env.reset(); self.env.verify_fk()

    def command(self, channels, values):
        if channels != CHANNELS:
            raise ValueError('Unknown command convention')
        value = np.asarray(values)
        if abs(value[0]) > 1e-8 or abs(value[1]-self.env.case['start_command'][1]) > 1e-6:
            raise ValueError('Replay moved fixed linear/roll')
        if np.any(abs(value[2:]) > math.pi/5+1e-6) or value[3] > 1e-8:
            raise ValueError('Replay bend bound/ZED violation')
        self.commanded = torch.tensor(value, dtype=torch.float32, device=self.env.device).reshape(1, 4)

    def step(self):
        self.env.tick(self.commanded)

    def measure(self):
        return self.env.measure()

    def evaluate(self, samples):
        return self.env.measure()
