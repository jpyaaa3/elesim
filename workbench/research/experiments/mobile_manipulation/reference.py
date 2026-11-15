"""Replay packaged commands in the original research environment; never train."""
import argparse
import json
from pathlib import Path
import time
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--anchor',type=int,choices=(0,1,2),required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--data-root',type=Path,default=Path('payload/data'))
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    from workbench.research.experiments.hug_c.configure import DEFAULTS
    from workbench.research.experiments.hug_pipeline.task import LinearHugEnv
    import torch
    root=a.data_root/'policies/hug-d100';meta=json.loads((root/'manifest.json').read_text())
    case=meta['anchors'][a.anchor]
    with np.load(root/'anchors.npz',allow_pickle=False) as data:path=data['commands'][a.anchor].copy()
    path=np.concatenate((path,np.repeat(path[-1:],2001-len(path),axis=0)))
    report=dict(scope='original fixed-body task; command replay only; no optimization',anchor=a.anchor,accepted=False)
    start=time.monotonic();trace=[]
    try:
        env=LinearHugEnv(dict(settings=dict(DEFAULTS)),case,a.output/'assets',n_envs=1,auto_reset=False)
        for index,q in enumerate(path):
            env.tick(torch.tensor(q,device=env.device,dtype=torch.float32)[None])
            raw=env.scene.robot.get_dofs_position(dofs_idx_local=env.dofs)[0]
            trace.append(dict(tick=index,actual=env.actual()[0].detach().cpu().tolist(),
                fault=bool(env.fault[0]),wrong_face=bool(env.last_wrong_face[0]),hold_s=float(env.hold.count[0])*.01,
                joint_spread=[float(torch.ptp(group)) if hasattr(torch,'ptp') else float(group.max()-group.min()) for group in (raw[2:7],raw[7:12])]))
            if index%500==0:print('tick',index,trace[-1],flush=True)
            if bool(env.success[0]) and not bool(env.fault[0]):break
        report.update(accepted=bool(env.success[0]) and not bool(env.fault[0]),
            final=trace[-1],linear=env.linear_report(),first_fault=next((r for r in trace if r['fault']),None))
    except Exception as exc:report['error']=repr(exc);raise
    finally:
        report['elapsed_s']=time.monotonic()-start
        (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        (a.output/'trace.json').write_text(json.dumps(trace)+'\n')
    print(json.dumps(report,indent=2))
    return 0 if report['accepted'] else 1


if __name__=='__main__':raise SystemExit(main())
