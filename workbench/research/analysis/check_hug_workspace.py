"""Direct-placement GPU check of the compact-footprint cost across replicas.

This does not test a Home approach or grasp/lift success. Teleporting to an
extended pose is only a numerical/geometry diagnostic, never demonstration data.
"""
import argparse
from pathlib import Path


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--experiment',type=Path,required=True)
    parser.add_argument('--case',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--num-envs',type=int,default=4)
    args=parser.parse_args(argv)
    if args.num_envs<2 or args.output.exists():
        parser.error('Use >=2 environments and a new report output')
    import torch
    from workbench.research.experiments.hug_c.configure import load_experiment,write_json
    from workbench.research.experiments.hug_c.environment import HugEnv
    exp=load_experiment(args.experiment)
    case=next(c for c in exp['cases'] if c['case_id']==args.case)
    env=HugEnv(exp,case,args.output.parent/'workspace-assets',n_envs=args.num_envs,
               backend='gpu',auto_reset=False)
    report=dict(schema='elesim-c-hug-batch-workspace-check',schema_version=1,
                simulation_context=exp['simulation_context'],n_envs=args.num_envs,
                teleport_diagnostic=True,approach_validated=False,samples={})
    with torch.inference_mode():
        for name,command in [('home',case['start_command']),('goal',case['goal_command']),
                             ('extended',[0.,0.,0.,0.])]:
            env.reset()
            q=torch.tensor(command,device=env.device,dtype=torch.float32).expand(env.num_envs,4)
            env.mapper.waypoint[:]=q
            targets=env.mapper.joint_targets()
            env.scene.robot.set_dofs_position(targets,dofs_idx_local=env.dofs,zero_velocity=True)
            env.scene.robot.control_dofs_position(targets,dofs_idx_local=env.dofs)
            env.begin_action()
            env.tick(q)
            peak=env.workspace_peak
            assert bool(torch.isfinite(peak).all())
            report['samples'][name]=dict(excess_m=peak.cpu().tolist(),
                                         cost_fraction=env.workspace_exposure.cpu().tolist())
    tolerance=.001
    report['ready']=(max(report['samples']['home']['excess_m'])<=tolerance and
                     max(report['samples']['goal']['excess_m'])<=tolerance and
                     min(report['samples']['extended']['excess_m'])>.05)
    write_json(args.output,report)
    print(report,flush=True)
    return 0 if report['ready'] else 1


if __name__=='__main__':
    raise SystemExit(main())
