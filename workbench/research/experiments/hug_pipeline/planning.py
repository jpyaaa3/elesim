"""Function-only catalog, immutable mapper copy, exact q/root preserving plans."""
from pathlib import Path
import json
import shutil
from .contracts import save,sha,digest,shared,make_case,files_hashes


def prepare(args):
    from workbench.research.experiments.hug_c.prepare import prepare as select
    from workbench.research.experiments.hug_c.configure import DEFAULTS,load_mechanism
    if args.output.exists():raise FileExistsError(args.output)
    if not 0<args.mu_max<DEFAULTS['friction']:raise ValueError('Require friction margin')
    for key in ('population','generations','repair_generations','num_envs','iterations','max_reselections'):
        if getattr(args,key)<1:raise ValueError('Positive budget required: '+key)
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
    seed_files=[]
    for i,file in enumerate(args.forward_seed):
        target=args.output/f'forward-seed-{i:03d}.npz';shutil.copyfile(file,target)
        seed_files.append(dict(file=target.name,sha256=sha(target)))
    plan=dict(schema='hug-pipeline',version=1,source_sha256=digest(),shared_context=shared(settings),
        settings=settings,home=home,diameter_mm=args.diameter,mu_max=args.mu_max,step_deg=args.step,
        seed=args.seed,mapper_sha256=sha(args.output/'mapper.html'),
        budgets={k:getattr(args,k) for k in ('population','generations','repair_generations','num_envs','iterations','max_reselections')},
        validation=args.validation,test=args.test,knots=6,backend=args.backend,
        reverse_inputs=[dict(directory=str(p.resolve()),artifacts=files_hashes(p)) for p in args.reuse_reverse],forward_seeds=seed_files,
        interpolation='phase-aligned command trajectories; not averaged network weights',
        readiness='planned; no physics or learning executed')
    save(args.output/'plan.json',plan)
    return plan


def choose(root,plan,excluded,screen):
    from workbench.research.experiments.hug_c.prepare import prepare as select
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
    result=dict(schema='hug-reverse-ga-plan',version=1,source=source,
        simulation_context=dict(plan['shared_context'],code_sha256=reverse_digest(),direction='grasp-to-home'),
        settings=plan['settings'],cases=[grasp],solutions=[check],selection='exact pipeline anchor; no XY re-solve',
        input_xyD_mm=[*case['xy_mm'],case['diameter_mm']],home=plan['home'],
        population=plan['budgets']['population'],knots=plan['knots'],generations=plan['budgets']['generations'],
        seed=plan['seed'],parade_every=10,source_sha256=reverse_digest())
    output.mkdir(parents=True,exist_ok=False);save(output/'plan.json',result);(output/'mapper-kernel.js').write_text(kernel)
    return result
