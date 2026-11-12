"""Record one verified reverse-GA winner, without changing search or PPO."""
import argparse
import json
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--allow-unverified',action='store_true',
                   help='Also record a candidate that succeeded in search but failed independent verification')
    args=p.parse_args()
    from workbench.research.experiments.hug_reverse_ga.prepare import digest,save
    from workbench.research.experiments.hug_reverse_ga.physics import admit,evaluate,sample
    from workbench.research.experiments.hug_c.environment import HugEnv
    from workbench.research.experiments.hug_interpolation.video import record_video
    plan=json.loads((args.source/'plan.json').read_text());curr=json.loads((args.source/'run/curriculum.json').read_text())
    allowed=('reverse_path_verified','verification_failed') if args.allow_unverified else ('reverse_path_verified',)
    if plan['source_sha256']!=digest() or curr['status'] not in allowed:raise ValueError('Unverified or changed source')
    winner=curr['candidate'];gen=curr['generation']
    with np.load(args.source/f'run/generation-{gen:04d}/candidates.npz',allow_pickle=False) as d:commands=d['commands'][winner:winner+1].copy()
    args.output.mkdir(parents=True,exist_ok=False)
    env=HugEnv(plan,plan['cases'][0],args.output/'assets',n_envs=1,backend='gpu',auto_reset=False,video=True)
    env.scene.cameras['report'].set_pose(pos=(.85,-1.0,1.25),lookat=(.35,.05,.55),up=(0.,0.,1.))
    if not admit(env,args.output/'admission'):raise RuntimeError('Initial grasp failed')
    state=env.scene.scene.get_state();baseline=sample(env)
    with record_video(env,args.output/'winner.mp4'):
        reports=evaluate(env,commands,args.output,state,baseline)
    with np.load(args.output/'states.npz',allow_pickle=False) as states:
        legs=env.scene.go2_leg_dofs
        target=np.tile(env.scene.cfg.scene.go2.leg_pose_rad,len(legs)//3)
        leg_q=states['joint_q'][:,0][:,legs]
        leg_qd=states['joint_qd'][:,0][:,legs]
        leg_report=dict(max_target_error_rad=float(np.max(abs(leg_q-target))),
                        max_speed_rad_s=float(np.max(abs(leg_qd)))) if legs else {}
    save(args.output/'video-report.json',dict(source=str(args.source),candidate=winner,generation=gen+1,
        verification_status=curr['status'],verification_replays=curr['replays'],result=reports[0],leg_hold=leg_report,training=False))
    print(json.dumps(reports[0]))
    return 0 if reports[0]['success'] else 1


if __name__=='__main__':raise SystemExit(main())
