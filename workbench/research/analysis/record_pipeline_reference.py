"""Record the exact forward GA reference used for PPO resets, without learning."""
import argparse
import json
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--backend',choices=('gpu','cpu'),default='gpu')
    a=p.parse_args()
    from workbench.research.experiments.hug_pipeline.learning import load
    from workbench.research.experiments.hug_c.environment import HugEnv
    from workbench.research.experiments.hug_c.experiment import check_versions
    from workbench.research.experiments.hug_interpolation.video import record_video
    from workbench.research.experiments.hug_reverse_ga.physics import sample,tick
    from workbench.research.experiments.hug_reverse_ga.prepare import save
    from workbench.research.experiments.hug_reverse_rl.bank import compare
    from workbench.research.experiments.hug_pipeline.reference import validate
    check_versions();plan,path=load(a.plan)
    a.output.mkdir(parents=True,exist_ok=False)
    exp=dict(settings=plan['settings'],simulation_context=plan['simulation_context'],cases=[plan['case']])
    env=HugEnv(exp,plan['case'],a.output/'assets',n_envs=1,backend=a.backend,auto_reset=False,video=True)
    env.scene.cameras['report'].set_pose(pos=(.85,-1.,1.25),lookat=(.35,.05,.55),up=(0.,0.,1.))
    env.reset();frames=[sample(env)]
    errors=compare(frames[0],{k:v[0] for k,v in path.items()},plan['state_tolerances'])
    with record_video(env,a.output/'forward-reference.mp4'):
        for i,command in enumerate(path['command'][1:],1):
            state=tick(env,command[None]);frames.append(state)
            now=compare(state,{k:v[i] for k,v in path.items()},plan['state_tolerances'])
            errors={k:max(errors[k],now[k]) for k in errors}
    measured={k:np.asarray([s[k][0] for s in frames]) for k in frames[0]}
    measured.update(command=path['command'],time_s=path['time_s'])
    validate(measured,plan)
    np.savez_compressed(a.output/'measured-reference.npz',**measured)
    selected=[]
    for seconds in (0,2,4,6,7.47,8.04,8.62,10,12,14,16,float(path['time_s'][-1])):
        i=int(np.argmin(abs(path['time_s']-seconds)))
        q=measured['actual'][i]
        selected.append(dict(time_s=float(path['time_s'][i]),linear_mm=float(q[0]*1000),roll_deg=float(np.rad2deg(q[1])),
            q3_total_deg=float(np.rad2deg(q[2])*5),q4_total_deg=float(np.rad2deg(q[3])*5),
            contacts=measured['touch'][i].tolist(),object_xy_displacement_mm=float(1000*np.linalg.norm(measured['object_pos'][i,:2]-np.asarray(plan['case']['object_xyz_m'])[:2])),
            actual_velocity=measured['actual_vel'][i].tolist(),fault=bool(measured['fault'][i]),wrong_face=bool(measured['wrong'][i])))
    report=dict(reference_sha256=plan['reference_path_sha256'],case=plan['case'],duration_s=float(path['time_s'][-1]),
        measured_state_max_errors=errors,samples=selected,training_updates=0,
        role='Actual forward GA commands used only for reset-state construction; PPO does not imitate these actions')
    save(a.output/'reference-report.json',report);print(json.dumps(report),flush=True)
    return 0

if __name__=='__main__':raise SystemExit(main())
