"""Physics adapter. Forward PPO rewards and forward termination are not used."""
import numpy as np
from .criteria import admission_good,violations,error,Tracker,REASONS
from .prepare import save


def sample(env):
    import torch
    def a(x):return x.detach().cpu().numpy().copy()
    robot=env.scene.robot;obj=env.scene.object
    raw=obj.get_contacts(exclude_self_contact=False,is_padded=True)
    ids=torch.tensor([link.idx for link in robot.links],device=env.device)
    valid=raw['valid_mask'].to(env.device)
    pair=valid&(torch.isin(raw['link_a'].to(env.device).long(),ids)|torch.isin(raw['link_b'].to(env.device).long(),ids))
    force=raw['force_a'].to(env.device).norm(dim=-1)
    penetration=raw['penetration'].to(env.device)
    if not torch.isfinite(force[valid]).all() or not torch.isfinite(penetration[valid]).all():
        raise RuntimeError('Invalid contact data')
    def maximum(value):
        return torch.where(pair,value,torch.zeros_like(value)).amax(1) if value.shape[1] else torch.zeros(env.num_envs,device=env.device)
    qd=robot.get_dofs_velocity(dofs_idx_local=env.dofs)
    actual_vel=torch.stack((qd[:,0],qd[:,1],qd[:,2:7].mean(1),qd[:,7:12].mean(1)),dim=1)
    result=dict(actual=a(env.actual()),actual_vel=a(actual_vel),
        joint_q=a(robot.get_dofs_position()),joint_qd=a(robot.get_dofs_velocity()),
        object_pos=a(obj.get_pos()),object_quat=a(obj.get_quat()),object_vel=a(obj.get_vel()),object_ang=a(obj.get_ang()),
        touch=a(env.last_touch),floor=a(env.last_floor),fault=a(env.fault),wrong=a(env.last_wrong_face),
        contact_force=a(maximum(force)),contact_penetration=a(maximum(penetration)))
    if any(not np.isfinite(v).all() for v in result.values()):raise RuntimeError('Nonfinite physics state')
    return result


def restore_leg_hold(env, ids=None):
    """Genesis snapshot reset clears PD modes; reapply the standing target.

    Do not teleport joints or zero their velocities. Restore only the controller.
    """
    legs=env.scene.go2_leg_dofs
    if not legs:return
    import torch
    n=env.num_envs if ids is None else len(ids)
    pose=torch.tensor(env.scene.cfg.scene.go2.leg_pose_rad,device=env.device,dtype=torch.float32)
    pose=pose.repeat(len(legs)//3).expand(n,len(legs))
    env.scene.robot.control_dofs_position(pose,dofs_idx_local=legs,envs_idx=ids)


def arrays(env):
    from workbench.research.experiments.hug_c.control import command_bounds,action_rates
    lower,upper=command_bounds(env.s)
    return lower,upper,action_rates(env.s)/env.s['action_ticks']


def tick(env,command):
    import torch
    # Environment tick replaces bookkeeping tensors; reset mutates them later.
    # Inference tensors cannot be mutated after leaving their inference scope.
    with torch.no_grad():
        env.tick(torch.as_tensor(command,device=env.device,dtype=torch.float32))
    return sample(env)


def archive(path,frames,commands,active,metadata):
    import json
    np.savez_compressed(path,**{k:np.asarray([s[k] for s in frames]) for k in frames[0]},
        command=np.asarray(commands),active=np.asarray(active),
        time_s=np.arange(len(frames))*metadata['dt'],metadata_json=np.asarray(json.dumps(metadata)))


def admit(env,directory):
    """Nominal placement is reset once; never rebase drift after settling."""
    directory.mkdir(parents=True,exist_ok=False);env.reset()
    goal=np.asarray(env.case['goal_command']);command=np.tile(goal,(env.num_envs,1))
    lower,upper,rate=arrays(env);dt=env.s['dt']
    count=np.zeros(env.num_envs,int);bad=np.zeros(env.num_envs,bool)
    frames=[sample(env)];frames[0]['violation_mask']=np.zeros((env.num_envs,len(REASONS)),bool);masks=[];peak_xy=np.zeros(env.num_envs);peak_tilt=peak_xy.copy()
    for _ in range(round(1/dt)):
        s=tick(env,command);mask,xy,angle=violations(s,command,command,lower,upper,rate,env.case['object_xyz_m'])
        bad|=mask.any(-1);peak_xy=np.maximum(peak_xy,xy);peak_tilt=np.maximum(peak_tilt,angle)
        good=admission_good(s,goal)&~bad;count=np.where(good,count+1,0)
        s['violation_mask']=mask;frames.append(s);masks.append(mask)
    passed=(count>=round(.2/dt))&~bad
    report=dict(status='admitted' if passed.all() else 'initial_grasp_invalid',passed=passed.tolist(),
        final_good_ticks=count.tolist(),peak_xy_m=peak_xy.tolist(),peak_tilt_rad=peak_tilt.tolist(),
        goal_error=error(s['actual'],goal).tolist(),proper_contacts=s['touch'].tolist(),
        floor=s['floor'].tolist(),object_speed=np.linalg.norm(s['object_vel'],axis=-1).tolist(),
        object_angular_speed=np.linalg.norm(s['object_ang'],axis=-1).tolist(),
        violation_any=np.any(masks,axis=0).tolist(),reference='nominal object pose before settling')
    save(directory/'report.json',report)
    archive(directory/'states.npz',frames,[command]*len(frames),[np.ones(env.num_envs,bool)]*len(frames),
        dict(dt=dt,case=env.case,simulation_context=env.experiment['simulation_context'],phase='admission'))
    return passed.all()


def evaluate(env,paths,directory,reset_state,baseline=None):
    """Restore complete admitted Genesis state, never just arm q."""
    env.reset();env.scene.scene.reset(reset_state)
    restore_leg_hold(env)
    env.mapper.waypoint[:]=env.start
    n=env.num_envs;home=np.asarray(env.case['home_command']);lower,upper,rate=arrays(env)
    initial=sample(env)
    if baseline is not None:
        for key in ('joint_q','joint_qd','object_pos','object_quat','object_vel','object_ang'):
            if not np.allclose(initial[key],baseline[key],rtol=1e-5,atol=1e-6):
                raise RuntimeError('Complete-state restore mismatch: '+key)
    initial['violation_mask']=np.zeros((n,len(REASONS)),bool)
    tracker=Tracker(n,error(initial['actual'],home),env.s['dt'])
    frames=[initial];commands=[paths[:,0].copy()];active=[tracker.active.copy()]
    previous=paths[:,0].copy()
    for i in range(1,paths.shape[1]):
        command=np.where(tracker.active[:,None],paths[:,i],previous)
        s=tick(env,command)
        s['violation_mask']=violations(s,command,previous,lower,upper,rate,env.case['object_xyz_m'])[0]
        valid=tracker.update(s,command,previous,home,lower,upper,rate,env.case['object_xyz_m'])
        frames.append(s);commands.append(command.copy());active.append(valid);previous=command.copy()
        if not tracker.active.any():break
    reports=tracker.reports();save(directory/'report.json',dict(candidates=reports))
    archive(directory/'states.npz',frames,commands,active,dict(dt=env.s['dt'],case=env.case,
        simulation_context=env.experiment['simulation_context'],direction='grasp-to-home',
        reset_method='full Genesis snapshot during search; independently admitted prefix replay for curriculum',
        note='States after lane termination are invalid; success is not a forward or lift certificate'))
    return reports
