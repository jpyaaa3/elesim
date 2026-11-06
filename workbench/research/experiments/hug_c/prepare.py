"""Choose three C anchors by covered solution area, exclusively from mapper functions."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from .source import read_kernel, evaluate
from .triangle import select, weights
from .configure import write_json, asset_digest


def prepare(source, *, diameter=150., mu_max=.4, step=1., seed=42, validation=30, test=100, excluded=None, preview=None, catalog=None,
            catalog_preflight=None):
    if not (0<diameter<=1000 and 0<mu_max<5 and .1<=step<=10 and 180/step==round(180/step)):
        raise ValueError('Invalid diameter, friction cutoff, or angle resolution')
    if validation<1 or test<1:
        raise ValueError('Require nonempty validation and test splits')
    kernel,provenance=read_kernel(source)
    rows=evaluate(kernel,diameter,step=step,mu_max=mu_max)
    original_count=len(rows); screened=None
    if catalog_preflight is not None:
        raw=Path(catalog_preflight).read_bytes();screened=json.loads(raw)
        if (screened.get('schema')!='elesim-c-hug-catalog-preflight'
            or screened['source']['kernel_sha256']!=provenance['kernel_sha256']
            or screened['diameter_mm']!=diameter or screened['step_deg']!=step
            or screened['mu_max']!=mu_max
            or screened['simulation_context']['model_sha256']!=asset_digest()):
            raise ValueError('Catalog preflight does not match source/model/grid')
        ready={tuple(r['q_deg']) for r in screened['results'] if r['static_ready']}
        rows=[r for r in rows if tuple(r['q_deg']) in ready]
        provenance['catalog_preflight_sha256']=hashlib.sha256(raw).hexdigest()
        provenance['catalog_physics_settings']=screened['simulation_context']['settings']
        ready_cells={tuple(np.floor((np.array(q)+180)/step).astype(int)) for q in ready}
    excluded=excluded or []
    if excluded:
        rows=[r for r in rows if not any(np.linalg.norm(np.array(r['q_deg'])-q)<step/2 for q in excluded)]
    vertices,selection=select(rows,step,seed=seed)
    anchors=[rows[i] for i in vertices];v=np.array([r['q_deg'] for r in anchors])
    rng=np.random.default_rng(seed);held=[];attempts=0
    while len(held)<validation+test and attempts<20:
        uv=rng.dirichlet([1,1,1],max(256,(validation+test-len(held))*3));qs=uv@v
        sampled=evaluate(kernel,diameter,qs.tolist(),mu_max=mu_max)
        held.extend(r for r in sampled if r is not None and
                    (screened is None or tuple(np.floor((np.array(r['q_deg'])+180)/step).astype(int)) in ready_cells));attempts+=1
    if len(held)<validation+test:
        raise ValueError('Insufficient valid triangle interior; increase sampling budget or inspect the chart')
    branch='c-q-triangle-'+provenance['kernel_sha256'][:12]
    cases=[]
    for i,(row,split) in enumerate([(r,'train') for r in anchors]+[(r,'validation' if i<validation else 'test') for i,r in enumerate(held[:validation+test])]):
        c=dict(row,case_id=f'c-{split}-{i:03d}-d{diameter:g}',split=split,diameter_mm=diameter,
               branch=branch,q_barycentric=weights(v,np.array(row['q_deg'])).tolist())
        cases.append(c)
    selection.update(mu_max=mu_max,diameter_mm=diameter,roll_deg=-90,shape='C: q3*q4>0; q4<0',
                     excluded_q_total_deg=excluded,continuous_interior_assumed_valid=False,
                     original_geometric_samples=original_count,static_catalog_screened=screened is not None,
                     coverage_denominator='static-ready raster cells' if screened else 'geometric raster cells')
    if preview is not None:
        from .preview import write_preview
        write_preview(preview,rows,v,selection)
    if catalog is not None:
        write_json(catalog,dict(schema='elesim-c-hug-catalog',schema_version=1,source=provenance,
                               diameter_mm=diameter,step_deg=step,mu_max=mu_max,rows=rows))
    return dict(schema='elesim-c-hug-pilot',schema_version=1,source=provenance,selection=selection,
                cases=cases,triangle=dict(branch=branch,anchor_ids=[c['case_id'] for c in cases[:3]],
                                         coordinate_space='q3/q4 total degrees'),
                readiness='geometry-only; Home/approach/lift require Genesis preflight')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mapper',type=Path)
    p.add_argument('--diameter-mm',type=float,default=150);p.add_argument('--mu-max',type=float,default=.4)
    p.add_argument('--step-deg',type=float,default=1.);p.add_argument('--seed',type=int,default=42)
    p.add_argument('--exclude-q',type=Path,help='JSON list of failed preflight [q3,q4] total degrees')
    p.add_argument('--preview',type=Path,help='Write a standalone q triangle SVG/HTML review')
    p.add_argument('--catalog-output',type=Path,help='Save function samples for screening and exact XY inverse seeds')
    p.add_argument('--catalog-preflight',type=Path,help='Use batched Home/goal screening to select physically valid anchor candidates')
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=prepare(a.mapper,diameter=a.diameter_mm,mu_max=a.mu_max,step=a.step_deg,seed=a.seed,
                   excluded=json.loads(a.exclude_q.read_text()) if a.exclude_q else None,preview=a.preview,catalog=a.catalog_output,
                   catalog_preflight=a.catalog_preflight)
    write_json(a.output,result)
    print(json.dumps(result['selection'],indent=2));print(f'Wrote {len(result["cases"])} cases; no physics or RL executed.')

if __name__=='__main__':main()
