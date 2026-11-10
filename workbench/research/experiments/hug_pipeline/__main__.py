"""Run a pinned cylinder study; no training occurs during plan/status/resolve."""
import argparse
import json
from pathlib import Path
import numpy as np
from .contracts import load_study,save,sha


def resolve(root,xy,output):
    from .contracts import make_case
    from .evaluation import blend
    from workbench.research.experiments.hug_c.source import read_kernel,inverse
    from workbench.research.experiments.hug_c.triangle import weights
    plan=load_study(root);publication=json.loads((root/'publication.json').read_text())
    manifest=json.loads((root/'manifest.json').read_text())
    if manifest.get('publication_sha256')!=sha(root/'publication.json'):raise ValueError('Publication changed')
    if not publication.get('ready') or publication['evaluation_sha256']!=sha(root/'interpolation.json'):
        raise ValueError('Study has no validated publication')
    if publication['shared_context']!=plan['shared_context']:raise ValueError('Publication context')
    kernel,_=read_kernel(root/'mapper.html',require_preparation=True)
    anchors=publication['anchors'];v=np.array([a['case']['q_deg'] for a in anchors])
    rows=inverse(kernel,plan['diameter_mm'],xy,[a['case'] for a in anchors],mu_max=plan['mu_max'])
    rows=[r for r in rows if np.min(weights(v,r['q_deg']))>=-1e-9]
    if not rows:raise ValueError('No feasible in-triangle root; no diameter fallback or extrapolation')
    target=min(rows,key=lambda r:r['mu_min']);paths=[]
    for anchor in anchors:
        file=root/'alignment'/(anchor['case']['case_id']+'.npz')
        if sha(file)!=anchor['aligned_sha256'] or sha(anchor['policy'])!=anchor['policy_sha256']:raise ValueError('Published anchor changed')
        with np.load(file,allow_pickle=False) as raw:paths.append(raw['commands'].copy())
    commands,w=blend(paths,v,target['q_deg'],plan)
    report=dict(target=target,diameter_mm=plan['diameter_mm'],weights=w.tolist(),anchors=anchors,
        target_physics_verified=False,interpolation='phase-aligned open-loop command candidate',
        note='Sampled study validation does not certify this previously untested point')
    output.mkdir(parents=True,exist_ok=False);np.savez_compressed(output/'candidate.npz',commands=commands,metadata_json=np.asarray(json.dumps(report)))
    save(output/'resolution.json',report);return report


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    q=sub.add_parser('plan');q.add_argument('--mapper',type=Path,required=True);q.add_argument('--output',type=Path,required=True)
    q.add_argument('--forward-seed',type=Path,action='append',default=[],help='Optional command NPZ proposal, always revalidated in physics');q.add_argument('--reuse-reverse',type=Path,action='append',default=[],help='Explicit verified reverse-plan directory; exact physical case match required');q.add_argument('--diameter',type=float,required=True);q.add_argument('--mu-max',type=float,default=.4);q.add_argument('--step',type=float,default=1.)
    for name,default in (('seed',42),('population',128),('generations',100),('repair-generations',100),('num-envs',512),('iterations',2000),('max-reselections',5),('validation',30),('test',100)):
        q.add_argument('--'+name,type=int,default=default)
    q.add_argument('--backend',choices=('gpu','cpu'),default='gpu')
    for name in ('run','resume','status','resolve'):
        q=sub.add_parser(name);q.add_argument('root',type=Path)
        if name in ('run','resume'):q.add_argument('--stop-after',choices=('screen','reverse','forward','check','train','policy'))
        if name=='resolve':q.add_argument('--x',type=float,required=True);q.add_argument('--y',type=float,required=True);q.add_argument('--output',type=Path,required=True)
    a=p.parse_args(argv)
    if a.command=='plan':
        from .planning import prepare
        result=prepare(a);print(json.dumps(result,indent=2));return 0
    if a.command=='status':
        path=a.root/'manifest.json';print(path.read_text() if path.exists() else json.dumps({'status':'planned'}));return 0
    if a.command=='resolve':print(json.dumps(resolve(a.root,[a.x,a.y],a.output),indent=2));return 0
    from .coordinator import Coordinator,lock
    with lock(a.root):
        coordinator=Coordinator(a.root)
        try:coordinator.run(a.stop_after)
        except Exception as exc:
            if coordinator.state['status']=='running':
                coordinator.state.update(status='execution_error',error=repr(exc));coordinator.write()
            raise
        if a.stop_after:
            coordinator.state.update(status='diagnostic_stop',stopped_after=a.stop_after);coordinator.write()
    return 0

if __name__=='__main__':raise SystemExit(main())
