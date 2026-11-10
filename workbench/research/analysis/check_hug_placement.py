"""Diagnose a C-hug target in an independent scene; never trains or changes a run.

Direct placement separates endpoint feasibility from Home-to-grasp reachability.
Small closing perturbations test whether exact tangent geometry lacks preload.
A prescribed roll sweep is diagnostic only, never part of the training policy.
"""
import argparse
import json
import math
from pathlib import Path
import numpy as np


def command_grid(case):
    rows=[]
    for linear in (0.,-.001,-.002):
        for q3 in (0.,-1.,-3.,-5.,-8.,-12.):
            for q4 in (0.,-.75,-1.5):
                command=np.asarray(case['goal_command']).copy()
                command[0]+=linear
                command[2:]+=np.deg2rad([q3,q4])/5
                command[2:]=np.clip(command[2:],-math.pi/5,math.pi/5)
                rows.append(dict(delta_linear_mm=linear*1000,delta_q_total_deg=[q3,q4],command=command.tolist()))
    return rows


def contacts(env,index):
    """Loaded robot/object contacts, with actual local positions and face label."""
    import torch
    from workbench.research.experiments.hug_c.contacts import inverse_rotate,classify_faces
    raw=env.scene.robot.get_contacts(exclude_self_contact=False,is_padded=True)
    positions=env.scene.robot.get_links_pos();quats=env.scene.robot.get_links_quat()
    result=[]
    for i in raw['valid_mask'][index].nonzero().flatten().tolist():
        a,b=int(raw['link_a'][index,i]),int(raw['link_b'][index,i])
        obj=env.scene.links.object_
        if not (a in obj or b in obj):continue
        robot=b if a in obj else a
        if robot not in env.scene.links.name_by_index:continue
        name=env.scene.links.name_by_index[robot]
        if name not in {f'node{k}' for k in range(10)} and not name.startswith('gripper'):continue
        local=int(env.contact_local[robot]);origin=positions[index,local];quat=quats[index,local]
        point=raw['position'][index,i];normal=-raw['normal'][index,i] if a==robot else raw['normal'][index,i]
        proper,wrong=classify_faces(point,normal,origin,quat,env.contact_axes[robot],
            margin=env.s['face_margin_m'],normal_tolerance=env.s['face_normal_tolerance'],xp=torch)
        force=float(raw['force_a'][index,i].norm())
        result.append(dict(link=name,force_n=force,loaded=force>env.s['face_force_min_n'],
            position_mm=(point*1000).cpu().tolist(),local_position_mm=(inverse_rotate(quat,point-origin)*1000).cpu().tolist(),
            proper_face=bool(proper),wrong_face=bool(wrong),penetration_mm=float(raw['penetration'][index,i])*1000))
    return result


def run(args):
    import torch
    from workbench.research.experiments.hug_c.configure import load_experiment,write_json
    from workbench.research.experiments.hug_c.environment import HugEnv
    exp=load_experiment(args.experiment)
    case=next(c for c in exp['cases'] if c['case_id']==args.case)
    args.output.mkdir(parents=True,exist_ok=False)
    grid=command_grid(case)
    env=HugEnv(exp,case,args.output/'assets',n_envs=len(grid),backend='gpu',auto_reset=False)
    commands=torch.tensor([r['command'] for r in grid],device=env.device,dtype=torch.float32)
    env.mapper.waypoint[:]=commands
    targets=env.mapper.joint_targets()
    env.scene.robot.set_dofs_position(targets,dofs_idx_local=env.dofs,zero_velocity=True)
    env.scene.robot.control_dofs_position(targets,dofs_idx_local=env.dofs)
    ever_grasp=torch.zeros(len(grid),dtype=torch.bool,device=env.device)
    max_hold=torch.zeros(len(grid),device=env.device)
    max_clear=torch.full((len(grid),),-1.,device=env.device)
    max_both=torch.zeros(len(grid),dtype=torch.long,device=env.device)
    phase_counts={};snapshots={}
    for phase,duration in (('settle',1.),('roll-lift',2.),('hold',5.5)):
        ticks=round(duration/env.s['dt']);counts=torch.zeros(len(grid),device=env.device)
        for tick in range(ticks):
            if phase=='roll-lift':commands[:,1]=-math.pi/2+(math.pi/2)*(tick+1)/ticks
            env.tick(commands)
            both=env.last_touch.bool().all(1)&~env.last_wrong_face&~env.fault
            counts+=both.float()
            ever_grasp|=(env.grasp_tick>=0)&~env.fault
            max_hold=torch.maximum(max_hold,env.hold.count.float()*env.s['dt'])
            max_clear=torch.maximum(max_clear,env.object_clearance())
            max_both=torch.maximum(max_both,env.grasp_ticks)
        phase_counts[phase]=(counts/ticks).cpu().tolist()
        score=(env.grasp_ticks.float()+ever_grasp.float()*1000-env.fault.float()*10000)
        best=int(score.argmax());snapshots[phase]=dict(best_index=best,nominal_contacts=contacts(env,0),best_contacts=contacts(env,best))
        print(json.dumps(dict(phase=phase,confirmed_grasp_candidates=int(ever_grasp.sum()),
            current_both_contacts=int(both.sum()),fault_candidates=int(env.fault.sum()),
            lift_success_candidates=int(env.success.sum()),best_index=best)),flush=True)
        if phase=='settle':
            static=[dict(grid[i],index=i,confirmed_grasp=bool(ever_grasp[i]),fault=bool(env.fault[i]),
                         both_contact_fraction=phase_counts[phase][i],object_xyz_m=env.scene.object.get_pos()[i].cpu().tolist()) for i in range(len(grid))]
            write_json(args.output/'static.json',dict(case=case,endpoint_candidates=static,snapshot=snapshots[phase]))
    objects=env.scene.object.get_pos().cpu().numpy()
    outcomes=[dict(grid[i],index=i,static=static[i],ever_confirmed_grasp=bool(ever_grasp[i]),
        longest_both_contact_s=float(max_both[i])*env.s['dt'],max_clearance_m=float(max_clear[i]),
        max_hold_s=float(max_hold[i]),success=bool(env.success[i]),fault=bool(env.fault[i]),
        object_xyz_m=objects[i].tolist(),phase_both_contact_fractions={p:values[i] for p,values in phase_counts.items()}) for i in range(len(grid))]
    report=dict(schema='elesim-c-hug-placement-diagnostic',schema_version=1,case=case,
        simulation_context=exp['simulation_context'],training_run_modified=False,optimizer_used=False,
        start='direct placement at geometric endpoint plus declared small perturbations',
        home_approach_tested=False,roll_sweep_degrees=[-90,0],results=outcomes,contact_snapshots=snapshots)
    write_json(args.output/'report.json',report)
    print(json.dumps(dict(report=str(args.output/'report.json'),static_grasps=sum(r['static']['confirmed_grasp'] and not r['static']['fault'] for r in outcomes),lift_successes=sum(r['success'] for r in outcomes))),flush=True)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--experiment',type=Path,required=True);p.add_argument('--case',required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args(argv)
    with __import__('torch').inference_mode():run(a)


if __name__=='__main__':main()
