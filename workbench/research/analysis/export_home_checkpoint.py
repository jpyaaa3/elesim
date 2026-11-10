"""Export a paired Home-prefix checkpoint for report-only NPZ/video replay."""
import argparse
from pathlib import Path
import json
from types import SimpleNamespace


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True);p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--backend',choices=('cpu','gpu'),default='gpu')
    args=p.parse_args(argv)
    from workbench.research.experiments.hug_home_rl.data import load_plan,sha
    from workbench.research.experiments.hug_home_rl.experiment import create_env,resume
    from workbench.research.experiments.hug_c.experiment import runner
    from workbench.research.experiments.hug_c.policy import export_policy
    from workbench.research.experiments.hug_reverse_ga.prepare import save
    plan,path=load_plan(args.plan);args.output.mkdir(parents=True,exist_ok=False)
    env=create_env(SimpleNamespace(num_envs=1,backend=args.backend,video=False),plan,path,args.output)
    learner=runner(env,None);resume(learner,env,plan,args.checkpoint)
    export_policy(learner.alg.get_policy(),args.output/'policy.npz',dict(env.metadata(),checkpoint_sha256=sha(args.checkpoint)))
    record=dict(checkpoint=str(args.checkpoint),checkpoint_sha256=sha(args.checkpoint),
        frontier=env.schedule.frontier,prefix_time_s=plan['pool_ticks'][env.schedule.frontier]*plan['settings']['dt'],training_updates=0)
    save(args.output/'export.json',record);print(json.dumps(record),flush=True)
    return 0


if __name__=='__main__':raise SystemExit(main())
