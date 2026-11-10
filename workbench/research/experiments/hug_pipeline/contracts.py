"""Portable study contracts. Shared physics and individual case identities differ."""
from pathlib import Path
import hashlib
import json
import numpy as np
from workbench.research.experiments.hug_reverse_ga.prepare import save


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def canonical(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()


def digest():
    from workbench.research.experiments.hug_home_rl.data import digest as base
    h=hashlib.sha256(base().encode())
    for p in sorted(Path(__file__).parent.glob('*.py')):
        h.update(p.name.encode());h.update(p.read_bytes())
    return h.hexdigest()


def shared(settings):
    from workbench.research.experiments.hug_c.configure import asset_digest,VERSIONS
    return dict(model_sha256=asset_digest(),code_sha256=digest(),versions=VERSIONS,settings=settings,
        control_mode='pipeline-free-four-dof-v1',reset_contract='forward-full-state-v1',
        reward_contract='bounded-milestones-lift90-v5+bounded-command-variation-v1',
        command_convention='linear m, roll rad, bends per-node rad; public q3/q4 = 5*bend')


def make_case(row,diameter,home):
    import xml.etree.ElementTree as ET
    from workbench.research.experiments.hug_c.configure import BUNDLE,fk,DEFAULTS
    frame=fk(ET.parse(BUNDLE/'robot.urdf').getroot(),'node0')
    if not np.allclose(frame[:3,:3],np.eye(3),atol=1e-8):raise ValueError('Unsupported mount')
    # Barycentric coordinates belong to a triangle, not to an anchor case.
    # Removing a failed neighbour must not invalidate an unchanged learned case.
    row={k:v for k,v in row.items() if k!='q_barycentric'}
    x,y=row['xy_mm'];goal=[0.,-np.pi/2,*np.deg2rad(np.asarray(row['q_deg'])/5).tolist()]
    identity={k:row[k] for k in ('q_deg','xy_mm','feature','root')}
    identity['diameter_mm']=diameter
    return dict(row,case_id='c-'+canonical(identity)[:20],diameter_mm=diameter,band=1,roll_deg=-90,
        branch='c-function',split=row.get('split','train'),start_command=list(home),home_command=list(home),goal_command=goal,
        object_xyz_m=[float(frame[0,3])+(y-144.5)/1000,float(frame[1,3])-x/1000,DEFAULTS['height_m']/2])


def load_study(root):
    from workbench.research.experiments.hug_c.configure import load_mechanism
    root=Path(root);p=json.loads((root/'plan.json').read_text())
    if (p.get('schema')!='hug-pipeline' or p.get('version')!=1 or p['source_sha256']!=digest()
        or p['shared_context']!=shared(p['settings']) or p['home']!=list(load_mechanism().presets['home'])
        or sha(root/'mapper.html')!=p['mapper_sha256']):raise ValueError('Study source/model/mapper mismatch')
    return p


def files_hashes(directory):
    return {str(p.relative_to(directory)):sha(p) for p in sorted(Path(directory).rglob('*'))
            if p.is_file() and p.suffix in ('.json','.npz','.pt','.js')}


def verify_files(directory,hashes):
    for name,value in hashes.items():
        p=Path(directory)/name
        if not p.is_file() or sha(p)!=value:raise ValueError('Artifact changed: '+str(p))
