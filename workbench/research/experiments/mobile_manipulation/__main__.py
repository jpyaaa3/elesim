"""Probe installed Sim construction and simulation-time replay without DDS.

This is a physics probe, not a complete mobile RGB-D/DDS acceptance run. Object
coordinates are fixture truth used only to set up and evaluate this one scene.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import time
import subprocess
import sys
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--config',type=Path,default=Path('payload/config/sim/config.yaml'))
    p.add_argument('--data-root',type=Path,default=Path('payload/data'))
    p.add_argument('--steps',type=int,default=40)
    p.add_argument('--locomotion',choices=('configured','raibert'),default='configured')
    selection=p.add_mutually_exclusive_group()
    selection.add_argument('--anchor',type=int,choices=(0,1,2))
    selection.add_argument('--anchors',type=int,choices=(0,1,2),nargs='+')
    selection.add_argument('--all-anchors',action='store_true')
    selection.add_argument('--fixture',type=Path,help='analytic placement JSON from the planning probe')
    selection.add_argument('--suite-fixture',type=Path,help='three active anchors, fixed-open anchor 0 and this interior fixture')
    p.add_argument('--base',choices=('fixed','free'),default='free')
    p.add_argument('--fixed-open-claws',action='store_true')
    p.add_argument('--video',action='store_true')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    if a.all_anchors or a.anchors or a.suite_fixture:
        reports=[];indices=a.anchors or [0,1,2]
        if len(indices)!=len(set(indices)):p.error('duplicate anchor')
        requested=[(f'anchor-{i}',['--anchor',str(i)]) for i in indices]
        if a.suite_fixture:
            if a.fixed_open_claws:p.error('suite controls its own claw variants')
            requested=[('active-anchor-0',['--anchor','0']),
                ('active-anchor-1',['--anchor','1']),('active-anchor-2',['--anchor','2']),
                ('fixed-open-anchor-0',['--anchor','0','--fixed-open-claws']),
                ('selected-interior',['--fixture',str(a.suite_fixture)])]
        for name,selection_args in requested:
            command=[sys.executable,'-m','workbench.research.experiments.mobile_manipulation',
                '--output',str(a.output/name),'--config',str(a.config),
                '--base',a.base,'--locomotion',a.locomotion,*selection_args]
            command+=['--data-root',str(a.data_root)]
            if a.fixed_open_claws:command.append('--fixed-open-claws')
            if a.video:command.append('--video')
            with (a.output/(name+'.log')).open('w') as log:
                result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=600)
            path=a.output/name/'report.json'
            report=json.loads(path.read_text()) if path.is_file() else dict(accepted=False,error='no child report')
            reports.append(dict(report,name=name,exit_code=result.returncode))
            (a.output/'report.json').write_text(json.dumps(dict(anchors=reports,complete=len(reports)==len(requested),
                accepted=len(reports)==len(requested) and all(r['accepted'] for r in reports)),indent=2)+'\n')
            print(name,'accepted',report['accepted'],flush=True)
        return 0 if all(r['accepted'] for r in reports) else 1
    from elesim_sim.config import load_app_config
    from elesim_sim.runtime import GenesisApp,RuntimePrep,AssetProcessor
    from elesim_sim.control_state import SimulationStateSource
    from elesim_protocol import SimQ
    b=load_app_config(a.config)
    model_root=a.data_root/'policies/hug-d100'
    meta=json.loads((model_root/'manifest.json').read_text())
    case=None;weights=None
    if a.anchor is not None:
        case=meta['anchors'][a.anchor];weights=np.eye(3)[a.anchor]
    elif a.fixture is not None:
        case=json.loads(a.fixture.read_text());weights=np.asarray(case['weights'],float)
        xy=np.asarray(case['xy_mm'],float)
        if (case.get('source_manifest_sha256')!=hashlib.sha256((model_root/'manifest.json').read_bytes()).hexdigest()
                or weights.shape!=(3,) or xy.shape!=(2,) or not np.isfinite(weights).all()
                or not np.isfinite(xy).all() or np.any(weights<0) or abs(weights.sum()-1)>1e-8
                or np.linalg.norm(xy)>500 or xy[0]>0 or xy[1]<0
                or not np.allclose(weights@np.asarray([c['q_deg'] for c in meta['anchors']]),case['q_deg'],atol=1e-7)):
            raise ValueError('placement fixture does not match the installed analytic triangle')
    cfg=replace(b.sim_config,enable_viewer=False,sim_camera_enable=False,sim_observer_camera_enable=False,
        build_dir=str((a.data_root/'models/assemblies/zed-mini').resolve()))
    state=SimulationStateSource(b.mapping_config)
    app=GenesisApp(params=replace(b.sim_param,realtime=False),cfg=cfg,limit=b.joint_limit,
        model=replace(b.spawn_config,sim_target_enable=False),
        urdf_export_cfg=b.urdf_export_config,go2_locomotion_config=(replace(b.go2_locomotion_config,mode='raibert') if a.locomotion=='raibert' else b.go2_locomotion_config),
        mapping_cfg=b.mapping_config,state_source=state)
    report=dict(scope='actual installed-runtime Genesis scene; no DDS / RGB-D acquisition',started=time.time())
    try:
        urdf=AssetProcessor(app).prepare_assets()
        if a.fixed_open_claws:
            from elesim_sim.model_bundle import physical_model_path
            urdf=str(physical_model_path(cfg.build_dir,use_go2=True,fixed_open=True))
        camera={}
        def before_build(scene):
            if a.video:camera['report']=scene.add_camera(res=(960,720),pos=(1.35,-1.4,1.5),
                lookat=(.35,.05,.5),up=(0.,0.,1.),fov=50,GUI=False)
        prep=RuntimePrep(app);prep.init_genesis(urdf,attach_scene_cameras=False,
            diagnostic_base_fixed=a.base=='fixed',before_build=before_build)
        scene=app.sim_scene
        entity=scene.mock_object_entities.get('cylinder_100mm')
        assert entity is not None
        entity.set_pos([2.,2.,.55]);entity.zero_all_dofs_velocity()
        for _ in range(a.steps):
            scene.apply_sim_q(state.estimate_q());scene.step()
        actual=scene.mover.measured_4dof_q()
        begin=np.array([actual.linear_m,actual.roll_rad,actual.theta1_rad,actual.theta2_rad])
        goal=np.array(meta['home'])
        n=int(np.ceil(max(abs(goal-begin)/[.02,.15,.15,.15])/.01))
        for fraction in np.linspace(0,1,n+1):
            scene.apply_sim_q(SimQ(*(begin+(goal-begin)*fraction)));scene.step()
        for _ in range(100):scene.apply_sim_q(SimQ(*goal));scene.step()
        fixture=[.65,.15,.55]
        if case is not None:
            from elesim_sim.simulation.model_frames import neutral_frame
            from scipy.spatial.transform import Rotation
            body=np.eye(4);body[:3,3]=scene.go2_entity.get_pos().detach().cpu().numpy().reshape(3)
            quat=scene.go2_entity.get_quat().detach().cpu().numpy().reshape(4)
            body[:3,:3]=Rotation.from_quat(quat[[1,2,3,0]]).as_matrix()
            node=body@neutral_frame(urdf,'node0');x,y=case['xy_mm']
            point=node@np.array([(y-meta.get('solver_origin_offset_mm',144.5))/1000,-x/1000,0.,1.])
            fixture=[float(point[0]),float(point[1]),.55]
        entity.set_pos(fixture);entity.zero_all_dofs_velocity()
        report['fixture_start_world_m']=fixture
        report['placement']='solver XY at measured settled body; upright on floor'
        if case is not None:report['solver_case']=dict(xy_mm=case['xy_mm'],weights=weights.tolist(),q_deg=case['q_deg'])
        actual=scene.mover.measured_4dof_q();home=[actual.linear_m,actual.roll_rad,actual.theta1_rad,actual.theta2_rad]
        report['start_q']=home
        report['base_mode']=a.base;report['fixed_open_claws']=a.fixed_open_claws
        report['physics']=dict(dt=b.sim_param.dt,substeps=b.sim_param.substeps,
            arm_control=cfg.arm_control_mode,arm_friction=cfg.arm_friction,floor_friction=cfg.floor_friction)
        report['body_start_m']=scene.go2_entity.get_pos().detach().cpu().numpy().reshape(-1).tolist() if scene.go2_entity is not None else None
        # Check that RGB-D replica geometry and encoder metadata describe the
        # same measured state, rather than mixing commanded and actual joints.
        scene.camera_joint_names=tuple(scene.mover.joint_names)
        scene.camera_hand_eye_joint_names=scene.camera_joint_names
        snapshot=scene._camera_state_snapshot(arm_q=tuple(home),sim_time_s=scene.sim_time_s(.01))
        assert np.allclose(snapshot.hand_eye_robot_joint_positions,scene.mover.get_dofs_position(),atol=1e-8)
        assert np.allclose(snapshot.arm_q,home,atol=1e-8)
        if report['body_start_m'] is not None:assert np.allclose(snapshot.root_pos,report['body_start_m'],atol=1e-8)
        report['measured_render_snapshot_coherent']=True
        state.seed_estimate_q(actual);state.apply_command(dict(command='torque_on'))
        path=np.repeat([home],21,axis=0).tolist()
        if case is not None:
            with np.load(model_root/'anchors.npz',allow_pickle=False) as d:path=np.tensordot(weights,d['commands'],axes=1)
            path=np.concatenate((path,np.repeat(path[-1:],2001-len(path),axis=0))).tolist()
        state.apply_command(dict(command='target',timed_arm=dict(id='probe',operation='start',dt_s=.01,q=path)))
        max_error=np.zeros(4);max_command_lag=np.zeros(4);trace=[];completed_hold=False
        if a.video:camera['report'].start_recording(save_to_filename=str((a.output/'replay.mp4').resolve()),fps=20)
        for i in range(len(path)):
            state.apply_command(dict(command='target',timed_arm=dict(id='probe',operation='keepalive')))
            actual=scene.mover.measured_4dof_q()
            command=state.estimate_q()
            actual_vector=np.array([actual.linear_m,actual.roll_rad,actual.theta1_rad,actual.theta2_rad])
            command_vector=np.array([command.linear_m,command.roll_rad,command.theta1_rad,command.theta2_rad])
            max_error=np.maximum(max_error,abs(actual_vector-command_vector))
            body=scene.go2_entity
            trace.append(dict(tick=i,command=command_vector.tolist(),actual=actual_vector.tolist(),
                body_pos=body.get_pos().detach().cpu().numpy().reshape(-1).tolist() if body is not None else None,
                body_quat=body.get_quat().detach().cpu().numpy().reshape(-1).tolist() if body is not None else None,
                object_pos=entity.get_pos().detach().cpu().numpy().reshape(-1).tolist(),
                evaluation=state.manipulation_evaluation_status()))
            state.advance_sim_time(i*.01,actual,.01)
            if state.timed_arm_status()['state']=='cancelled':break
            scene.apply_sim_q(state.estimate_q())
            applied=scene.mover.current_4dof_q();desired=state.estimate_q()
            max_command_lag=np.maximum(max_command_lag,np.abs(
                np.array([applied.linear_m,applied.roll_rad,applied.theta1_rad,applied.theta2_rad])-
                np.array([desired.linear_m,desired.roll_rad,desired.theta1_rad,desired.theta2_rad])))
            scene.step()
            actual=scene.mover.measured_4dof_q()
            state.sample_manipulation_evaluation(scene.mover.entity,entity,[actual.linear_m,actual.roll_rad,actual.theta1_rad,actual.theta2_rad],.01)
            # Genesis records at its simulation-time fps cadence from step().
            evaluation=state.manipulation_evaluation_status()
            if i%500==0:print('tick',i,'evaluation',evaluation,flush=True)
            if case is not None and evaluation.get('success') is True:
                completed_hold=True
                state.apply_command(dict(command='target',timed_arm=dict(id='probe',operation='cancel')))
                break
        if a.video:camera['report'].stop_recording()
        report.update(timed_arm=state.timed_arm_status(),evaluation=state.manipulation_evaluation_status(),
            cylinder_xyz=entity.get_pos().detach().cpu().numpy().reshape(-1).tolist(),
            replay_completed=state.timed_arm_status()['state']=='completed',
            max_tracking_error=max_error.tolist(),last_measured_q=[actual.linear_m,actual.roll_rad,actual.theta1_rad,actual.theta2_rad],
            last_command_q=command_vector.tolist())
        report['completed_hold']=completed_hold
        report['maximum_command_limiter_lag']=max_command_lag.tolist()
        report['accepted']=report['replay_completed'] if case is None else completed_hold
        report['completion']=('verified_five_second_hold' if completed_hold else
            'command_horizon' if report['replay_completed'] else 'interrupted')
        report['anchor']=a.anchor
        report['locomotion']=a.locomotion
        report['body_end_m']=scene.go2_entity.get_pos().detach().cpu().numpy().reshape(-1).tolist() if scene.go2_entity is not None else None
    except Exception as exc:
        report.update(accepted=False,error=repr(exc));raise
    finally:
        report['elapsed_s']=time.time()-report['started']
        (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        if 'trace' in locals():(a.output/'trace.json').write_text(json.dumps(trace)+'\n')
    print(json.dumps(report,indent=2))
    return 0 if report['accepted'] else 1


if __name__=='__main__':raise SystemExit(main())
