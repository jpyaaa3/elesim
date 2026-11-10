"""Replay a declared Home approach in a separate scene, with actual object motion.

No optimizer or policy is involved. A projected-geometry proposal is not marked
collision-free unless the real replay confirms it. The training run is untouched.
"""
import argparse
import json
from pathlib import Path
import numpy as np


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--experiment',type=Path,required=True);p.add_argument('--case',required=True)
    motion=p.add_mutually_exclusive_group()
    motion.add_argument('--path',type=Path,help='Optional found waypoint JSON; default is direct Home-to-goal')
    motion.add_argument('--preset',choices=('extend_arm',),help='Replay the actual UI preset from Home')
    p.add_argument('--output',type=Path,required=True);a=p.parse_args(argv)
    from workbench.research.experiments.hug_c.configure import load_experiment,CHANNELS,CONTROL_MODE,write_json
    from workbench.research.experiments.hug_c.control import action_rates
    from workbench.research.experiments.hug_c.environment import HugEnv,ReplayAdapter
    from workbench.research.experiments.hug_interpolation.video import record_video
    from elesim_sim.rl.trajectory import Trajectory
    from elesim_sim.rl.replay import replay
    import torch
    exp=load_experiment(a.experiment);case=next(c for c in exp['cases'] if c['case_id']==a.case)
    waypoints=[case['start_command'],case['goal_command']]
    waypoint_source='case geometric goal'
    if a.preset:
        from workbench.research.experiments.wrap_grasp.app_config import load_mechanism,UI_PRESETS
        mechanism=load_mechanism()
        if not np.allclose(case['start_command'],mechanism.presets['home'],atol=1e-10):
            raise ValueError('Case start differs from the actual Home preset')
        waypoints=[case['start_command'],list(mechanism.presets[a.preset])]
        waypoint_source=dict(preset=a.preset,ui_controls=UI_PRESETS[a.preset],
                             mechanism_config=mechanism.source,display_offsets=[0.,0.,0.,0.])
    if a.path:
        raw=json.loads(a.path.read_text())
        if not raw.get('found'):raise ValueError('No proposed route to replay')
        waypoints=[[q[0]/1000,np.deg2rad(q[1]),*np.deg2rad(np.array(q[2:])/5)] for q in raw['waypoints']]
    s=exp['settings'];times=[0.]
    for before,after in zip(waypoints,waypoints[1:]):
        steps=max(1,int(np.ceil(np.max(abs(np.asarray(after)-before)/action_rates(s)))))
        times.append(times[-1]+steps*s['action_ticks']*s['dt'])
    waypoints.append(waypoints[-1]);times.append(times[-1]+1.)
    trace=Trajectory(np.array(times),np.array(waypoints),CHANNELS,
        dict(case,control_mode=CONTROL_MODE,simulation_context=exp['simulation_context'],duration_s=times[-1],
             diagnostic_only=True,controller='declared rate-bounded Home approach'),dict(physics_validated=False))
    a.output.mkdir(parents=True,exist_ok=False);trace.save(a.output/'commands.npz')
    env=HugEnv(exp,case,a.output/'assets',n_envs=1,backend='gpu',auto_reset=False,video=True)
    class ContactReplayAdapter(ReplayAdapter):
        def measure(self):
            state=super().measure()
            raw=self.env.scene.robot.get_contacts(exclude_self_contact=False,is_padded=True)
            active=raw['valid_mask'][0] & (raw['force_a'][0].norm(dim=-1)>1e-4)
            contacts={}
            object_ids=set(self.env.obj.detach().cpu().tolist())
            names=self.env.scene.links.name_by_index
            for index in active.nonzero(as_tuple=False).flatten().detach().cpu().tolist():
                left,right=int(raw['link_a'][0,index]),int(raw['link_b'][0,index])
                other=right if left in object_ids else left if right in object_ids else None
                if other is not None:
                    name=names.get(other,str(other))
                    contacts[name]=contacts.get(name,0.)+float(raw['force_a'][0,index].norm())
            state['object_contact_forces_n']=contacts
            return state
    with torch.inference_mode(),record_video(env,a.output/'approach.mp4'):
        result=replay(trace,ContactReplayAdapter(env),dt=s['dt'])
    origin=np.asarray(case['object_xyz_m']);samples=result['samples']
    displacement=np.array([np.linalg.norm(np.array(row['state']['object_xyz_m'])[:2]-origin[:2])*1000 for row in samples])
    first_movement=next((row['time_s'] for row,d in zip(samples,displacement) if d>1.),None)
    first_bad=next((row['time_s'] for row in samples if row['state']['fault']),None)
    first_wrong=next((row['time_s'] for row in samples if row['state']['wrong_face_contact']),None)
    first_contact=next((dict(time_s=row['time_s'],forces_n=row['state']['object_contact_forces_n'])
                        for row in samples if row['state']['object_contact_forces_n']),None)
    summary=dict(schema='elesim-c-hug-Home-approach-replay',schema_version=1,case=case,
        started_at_home=True,optimizer_used=False,training_modified=False,declared_waypoints=waypoints,
        duration_s=times[-1],max_object_displacement_mm=float(displacement.max()),
        first_object_motion_over_1mm_s=first_movement,first_fault_s=first_bad,first_wrong_face_s=first_wrong,
        waypoint_source=waypoint_source,first_object_contact=first_contact,
        evaluation=result['evaluation'],video=str(a.output/'approach.mp4'))
    write_json(a.output/'replay.json',result);write_json(a.output/'report.json',summary)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
