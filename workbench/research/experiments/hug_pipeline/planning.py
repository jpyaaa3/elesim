"""Function-only catalog, immutable mapper copy, exact q/root preserving plans."""
from pathlib import Path
import json
from .contracts import save,sha,digest,shared,make_case,files_hashes
from workbench.research.experiments.hug_reverse_ga.motion_budget import contract


def prepare(args):
    from workbench.research.experiments.hug_c.prepare import prepare as select
    from workbench.research.experiments.hug_c.configure import DEFAULTS,load_mechanism
    if args.output.exists():raise FileExistsError(args.output)
    if not 0<args.mu_max<DEFAULTS['friction']:raise ValueError('Require friction margin')
    for key in ('population','generations','num_envs','iterations','max_reselections'):
        if getattr(args,key)<1:raise ValueError('Positive budget required: '+key)
    from .reverse import configuration
    reverse_backend=configuration(args)
    args.output.mkdir(parents=True)
    # Store only the executable geometry, not the user's multi-megabyte DATA cloud.
    from workbench.research.experiments.hug_c.source import read_kernel
    kernel,_=read_kernel(args.mapper,require_preparation=True)
    split=kernel.index('const CONTACT_OVERLAP_TOL_MM=')
    html=kernel[:split]+'let current=null;\n'+kernel[split:]+'function muLimit(){}\n'
    (args.output/'mapper.html').write_text(html)
    pilot=select(args.output/'mapper.html',diameter=args.diameter,mu_max=args.mu_max,step=args.step,
                 seed=args.seed,validation=args.validation,test=args.test,catalog=args.output/'catalog.json')
    save(args.output/'pilot.json',pilot)
    settings=dict(DEFAULTS,seed=args.seed);home=list(load_mechanism().presets['home'])
    plan=dict(schema='hug-pipeline',version=1,motion_budget=contract(),source_sha256=digest(),shared_context=shared(settings),
        settings=settings,home=home,diameter_mm=args.diameter,mu_max=args.mu_max,step_deg=args.step,
        seed=args.seed,mapper_sha256=sha(args.output/'mapper.html'),
        budgets={k:getattr(args,k) for k in ('population','generations','num_envs','iterations','max_reselections')},
        validation=args.validation,test=args.test,knots=6,backend=args.backend,
        reverse_planner=reverse_backend,
        reverse_inputs=[dict(directory=str(p.resolve()),artifacts=files_hashes(p)) for p in args.reuse_reverse],
        interpolation='phase-aligned command trajectories; not averaged network weights',
        readiness='planned; no physics or learning executed')
    if getattr(args,'reuse_selection',None):
        raw=args.reuse_selection.read_bytes()
        (args.output/'fixed-selection.json').write_bytes(raw)
        plan['fixed_selection_sha256']=sha(args.output/'fixed-selection.json')
    save(args.output/'plan.json',plan)
    return plan


def choose(root,plan,excluded,screen):
    from workbench.research.experiments.hug_c.prepare import prepare as select
    if plan.get('fixed_selection_sha256'):
        return fixed_selection(root,plan,excluded,screen)
    pilot=select(root/'mapper.html',diameter=plan['diameter_mm'],mu_max=plan['mu_max'],step=plan['step_deg'],
        seed=plan['seed'],validation=plan['validation'],test=plan['test'],excluded=excluded,catalog_preflight=screen)
    return pilot,[make_case(row,plan['diameter_mm'],plan['home']) for row in pilot['cases']]


def reverse_plan(plan,case,output,mapper):
    from workbench.research.experiments.hug_reverse_ga.prepare import digest as reverse_digest
    from workbench.research.experiments.hug_c.source import read_kernel,evaluate
    kernel,source=read_kernel(mapper,require_preparation=True)
    check=evaluate(kernel,case['diameter_mm'],[case['q_deg']],mu_max=plan['mu_max'])[0]
    if check is None or any(check[k]!=case[k] for k in ('q_deg','xy_mm','feature','root','mu_min')):
        raise ValueError('Selected geometric root changed')
    grasp=dict(case,start_command=case['goal_command'])
    result=dict(schema='hug-reverse-ga-plan',version=1,motion_budget=contract(),source=source,
        simulation_context=dict(plan['shared_context'],code_sha256=reverse_digest(),direction='grasp-to-home'),
        settings=plan['settings'],cases=[grasp],solutions=[check],selection='exact pipeline anchor; no XY re-solve',
        input_xyD_mm=[*case['xy_mm'],case['diameter_mm']],home=plan['home'],
        population=plan['budgets']['population'],knots=plan['knots'],generations=plan['budgets']['generations'],
        seed=plan['seed'],parade_every=10,source_sha256=reverse_digest())
    output.mkdir(parents=True,exist_ok=False);save(output/'plan.json',result);(output/'mapper-kernel.js').write_text(kernel)
    return result


def fixed_selection(root,plan,excluded,screen):
    """Keep original anchors/held-out points; physical admission is still required."""
    from .contracts import canonical
    from .reuse import compatible
    from workbench.research.experiments.hug_c.source import read_kernel,evaluate
    from workbench.research.experiments.hug_c.triangle import weights
    import numpy as np
    file=root/'fixed-selection.json'
    if sha(file)!=plan['fixed_selection_sha256'] or excluded:
        raise ValueError('Fixed selection changed or failed; no automatic reselection')
    selection=json.loads(file.read_text());cases=selection['cases']
    if len(cases)!=3+plan['validation']+plan['test']:
        raise ValueError('Fixed selection sample count mismatch')
    if [c['split'] for c in cases]!=['train']*3+['validation']*plan['validation']+['test']*plan['test']:
        raise ValueError('Fixed selection splits mismatch')
    kernel,_=read_kernel(root/'mapper.html',require_preparation=True)
    rows=evaluate(kernel,plan['diameter_mm'],[c['q_deg'] for c in cases],mu_max=plan['mu_max'])
    vertices=np.asarray([c['q_deg'] for c in cases[:3]])
    if len({canonical(c['q_deg']) for c in cases[:3]})!=3:raise ValueError('Repeated anchors')
    for c,r in zip(cases,rows):
        if r is None or any(c[k]!=r[k] for k in ('q_deg','xy_mm','feature','root','mu_min')):
            raise ValueError('Fixed case no longer satisfies mapper')
        rebuilt=make_case(c,plan['diameter_mm'],plan['home'])
        if rebuilt!=c or np.min(weights(vertices,c['q_deg'])) < -1e-8:
            raise ValueError('Fixed case convention or triangle mismatch')
    screened=json.loads(Path(screen).read_text())
    for c in cases[:3]:
        if not any(r['q_deg']==c['q_deg'] and r['home_passed'] for r in screened['results']):
            raise ValueError('Fixed anchor Home screen failed')
        if not any(compatible(plan,c,json.loads((Path(i['directory'])/'plan.json').read_text()))
                   for i in plan.get('reverse_inputs',[])):
            raise ValueError('Fixed anchor requires matching verified reverse evidence')
    return selection['pilot'],cases
