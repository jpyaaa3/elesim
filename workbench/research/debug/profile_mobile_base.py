"""Isolate one measured mobile-base goal in the production Genesis plant.

No DDS, Robot, floor-map admission or grasp success is claimed. Limits and the
relative goal come from the recorded mobile case; the physical arm stays Home.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import importlib
import importlib.util
import inspect
import json
from pathlib import Path
import sys
import time

import numpy as np


def goal_command(pose,goal,settings,reach):
    """The mobile executor's proportional command and final-arrival condition."""
    pose=np.asarray(pose,float);delta=np.asarray(goal,float)-pose
    delta[2]=(delta[2]+np.pi)%(2*np.pi)-np.pi
    distance=float(np.linalg.norm(delta[:2]))
    close=(distance<settings['base_position_tolerance_m']
        and abs(delta[2])<settings['base_yaw_tolerance_rad']
        and distance+reach*abs(delta[2])<=settings['arrival_error_m'])
    if close:return np.zeros(3),True
    c,s=np.cos(pose[2]),np.sin(pose[2]);v=np.array([[c,s],[-s,c]])@delta[:2]*.8
    norm=np.linalg.norm(v)
    if norm>settings['base_speed_mps']:v*=settings['base_speed_mps']/norm
    return np.r_[v,np.clip(delta[2]*.8,-settings['yaw_speed_radps'],settings['yaw_speed_radps'])],False


class ProbeFinished(Exception):
    def __init__(self,success,reason):self.success,self.reason=success,reason


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',type=Path,required=True)
    parser.add_argument('--config',required=True)
    parser.add_argument('--model-bundle',required=True)
    parser.add_argument('--controller-source',type=Path,
        help='explicit read-only implementation prototype; never an installed acceptance run')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if any(args.output.with_suffix(suffix).exists() for suffix in
            ('.run.json','.summary.json','.state.jsonl','.profile.jsonl')):
        parser.error('probe output already exists; inspect it before starting another experiment')
    case=json.loads(args.case.read_text());settings=case['settings']
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.with_suffix('.run.json').open('x') as marker:
        json.dump(dict(started_epoch_s=time.time(),
            case_sha256=hashlib.sha256(args.case.read_bytes()).hexdigest(),
            runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),marker)
    module_name='elesim_sim.robot.go2.pympc_controller'
    if args.controller_source:
        spec=importlib.util.spec_from_file_location(module_name,args.controller_source)
        module=importlib.util.module_from_spec(spec);sys.modules[module_name]=module;spec.loader.exec_module(module)
        setattr(importlib.import_module('elesim_sim.robot.go2'),'pympc_controller',module)
    controller=importlib.import_module(module_name)
    controller_sha=hashlib.sha256(Path(inspect.getfile(controller)).read_bytes()).hexdigest()
    from scipy.spatial.transform import Rotation
    from elesim_sim.simulation.genesis.utils import to_numpy_1d
    from workbench.research.debug import profile_sim_walking as walking
    original=walking.advance_controlled_step;started=None;settled=None;goal=None
    maximum=np.zeros(2);best=float('inf');last_progress=None;latest={};result=None
    args.output.parent.mkdir(parents=True,exist_ok=True)
    states=args.output.with_suffix('.state.jsonl').open('w')
    def step(scene,command,arm_q,**kwargs):
        nonlocal started,settled,goal,best,last_progress,latest
        arm_q=replace(arm_q,**dict(zip(['linear_m','roll_rad','theta1_rad','theta2_rad'],case['home_q'])))
        if np.linalg.norm(command)>0:
            base=scene.go2._entity.get_link('base');pos=to_numpy_1d(base.get_pos())[:3]
            quat=to_numpy_1d(base.get_quat())[:4];rpy=Rotation.from_quat(quat[[1,2,3,0]]).as_euler('xyz')
            velocity=to_numpy_1d(base.get_vel())[:3];angular=to_numpy_1d(base.get_ang())[:3]
            pose=np.r_[pos[:2],rpy[2]];now=time.monotonic()
            if started is None:
                started=last_progress=now
                turn=rpy[2]-case['planned_start_yaw'];c,s=np.cos(turn),np.sin(turn)
                delta=np.asarray(case['goal_offset']);xy=np.array([[c,-s],[s,c]])@delta[:2]
                goal=pose+np.r_[xy,delta[2]]
            maximum[:]=np.maximum(maximum,abs(rpy[:2]))
            desired,close=goal_command(pose,goal,settings,case['target_reach_m'])
            delta=goal-pose;delta[2]=(delta[2]+np.pi)%(2*np.pi)-np.pi
            metric=float(np.linalg.norm(delta[:2])+.2*abs(delta[2]))
            if metric<best-.0002:best=metric;last_progress=now
            latest=dict(elapsed_s=now-started,pose=pose.tolist(),goal=goal.tolist(),
                rpy=rpy.tolist(),command=desired.tolist(),linear_speed=float(np.linalg.norm(velocity)),
                angular_speed=float(np.linalg.norm(angular)),
                combined_arrival_error_m=float(np.linalg.norm(delta[:2])+case['target_reach_m']*abs(delta[2])))
            states.write(json.dumps(latest)+'\n');states.flush()
            if maximum.max()>.07:raise ProbeFinished(False,'body_tilt')
            if now-started>settings['base_timeout_s']:raise ProbeFinished(False,'base_timeout')
            if close and np.linalg.norm(velocity)<.01 and np.linalg.norm(angular)<.05:
                if settled is None:settled=now
                if now-settled>=settings['settle_s']:raise ProbeFinished(True,'arrived_and_settled')
            else:settled=None
            if not close and now-last_progress>10.:raise ProbeFinished(False,'no_measured_progress')
            command=desired
        return original(scene,command,arm_q,**kwargs)
    walking.advance_controlled_step=step;previous=sys.argv
    sys.argv=['profile_sim_walking',args.config,args.model_bundle,'--command','.08','0','0',
        '--duration','90','--settle','2','--stop','0','--repeat','1',
        '--output',str(args.output.with_suffix('.profile.jsonl'))]
    try:
        walking.main()
        result=dict(success=False,reason='simulation_horizon_ended')
    except ProbeFinished as done:result=dict(success=done.success,reason=done.reason)
    finally:
        walking.advance_controlled_step=original;sys.argv=previous;states.close()
        if result is not None:
            result.update(diagnostic_only=True,no_dds_or_map_or_grasp_acceptance=True,
                controller_source_sha256=controller_sha,implementation_override=bool(args.controller_source),
                maximum_abs_roll_pitch_rad=maximum.tolist(),last=latest,case=case)
            temporary=args.output.with_suffix('.summary.partial.json')
            temporary.write_text(json.dumps(result,indent=2)+'\n')
            temporary.replace(args.output.with_suffix('.summary.json'))
            print(json.dumps(result),flush=True)
    return 0 if result and result['success'] else 1


if __name__=='__main__':
    raise SystemExit(main())
