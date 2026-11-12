"""Exact mapper inverse at requested XYD; no triangle or GPU required."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from .motion_budget import contract
from .criteria import EVALUATION_POLICY


def digest():
    from workbench.research.experiments.hug_c.configure import digest_code
    from workbench.research.experiments.hug_ga.experiment import source_digest
    h=hashlib.sha256((digest_code()+source_digest()).encode())
    for p in sorted(Path(__file__).parent.glob('*.py')):
        h.update(p.name.encode());h.update(p.read_bytes())
    return h.hexdigest()


def save(path,data):
    path=Path(path);temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(data,indent=2,allow_nan=False));temp.replace(path)


def prepare(args):
    from workbench.research.experiments.hug_c.source import read_kernel,evaluate,inverse
    from workbench.research.experiments.hug_c.configure import DEFAULTS,asset_digest,BUNDLE,fk,load_mechanism,VERSIONS
    if (not all(math.isfinite(v) for v in (args.x,args.y,args.diameter,args.mu_max,args.seed_step))
        or args.diameter<=0 or not 0<args.mu_max<DEFAULTS['friction'] or not 0<args.seed_step<=30
        or args.population<2 or args.generations<1 or args.knots<1):
        raise ValueError('Invalid coordinates, diameter, friction, grid or search budget')
    if args.output.exists():raise FileExistsError(args.output)
    kernel,source=read_kernel(args.mapper,require_preparation=True)
    seeds=evaluate(kernel,args.diameter,step=args.seed_step,mu_max=args.mu_max)
    # Forward sampling supplies root/feature seeds; final q solves exact input XY.
    seeds.sort(key=lambda r:np.linalg.norm(np.array(r['xy_mm'])-[args.x,args.y]))
    solutions=inverse(kernel,args.diameter,[args.x,args.y],seeds,mu_max=args.mu_max)
    args.output.mkdir(parents=True)
    if not solutions:
        save(args.output/'completion.json',dict(status='solver_no_solution',input_xyD_mm=[args.x,args.y,args.diameter],
            source=source,note='Local inverse search failure, not an impossibility proof; C-band and preparation CAD filters apply'))
        return False
    solutions.sort(key=lambda r:(r['mu_min'],r['q_deg']))
    selected=solutions[0];home=list(load_mechanism().presets['home'])
    goal=[0.,-math.pi/2,*np.deg2rad(np.asarray(selected['q_deg'])/5).tolist()]
    frame=fk(ET.parse(BUNDLE/'robot.urdf').getroot(),'node0')
    if not np.allclose(frame[:3,:3],np.eye(3),atol=1e-8):raise ValueError('Unsupported mounting frame')
    case=dict(selected,case_id='reverse-xyD',split='train',diameter_mm=args.diameter,band=1,roll_deg=-90,
        start_command=goal,goal_command=goal,home_command=home,
        object_xyz_m=[float(frame[0,3])+(args.y-144.5)/1000,float(frame[1,3])-args.x/1000,DEFAULTS['height_m']/2])
    settings=dict(DEFAULTS,seed=args.seed)
    context=dict(model_sha256=asset_digest(),code_sha256=digest(),versions=VERSIONS,settings=settings,
        direction='grasp-to-home',criterion='docs/hug-reverse-ga-criteria.md',
        command_convention='linear m, roll rad, bends per-node rad; public q3/q4 = 5*bend',
        frame='mapper mm: x=-world Y, y=world X; housing centre')
    plan=dict(schema='hug-reverse-ga-plan',version=1,motion_budget=contract(),evaluation_policy=EVALUATION_POLICY,source=source,simulation_context=context,
        settings=settings,cases=[case],solutions=solutions,selection='minimum mu_min among exact inverse solutions',
        input_xyD_mm=[args.x,args.y,args.diameter],home=home,population=args.population,knots=args.knots,
        generations=args.generations,seed=args.seed,parade_every=10,source_sha256=digest(),
        preparation_filter='Inherited mapper static and Home-to-Extend CAD checks; not a dynamic path certificate')
    save(args.output/'plan.json',plan)
    (args.output/'mapper-kernel.js').write_text(kernel)
    return True
