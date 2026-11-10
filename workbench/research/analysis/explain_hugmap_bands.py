"""Read-only study of gaga's contact families and cardinal boundary cells.

Diagnoses describe tested contact branches, never a global impossibility proof.
Output is an independent snapshot; neither source JSON nor production solver changes.
"""
import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import hashlib
import itertools
import json
import math
from pathlib import Path
import time

import numpy as np
from numpy.polynomial import polynomial as poly
from workbench.research.analysis import hugmap_algebraic_inverse as inv
from workbench.research.analysis import hugmap_contact_geometry as geom
from workbench.research.support import hugmap as h


def extended_angles(center,radius,spec):
    constant,xterm,yterm,den=inv.feature_polynomials(tuple(spec));x,y=center
    term=x*x+y*y-radius*radius if spec[1]=='v' else -radius
    coefficients=constant+x*xterm+y*yterm+term*den;scale=np.max(np.abs(coefficients))
    if scale<1e-13:return [],True
    coefficients=coefficients/scale
    while len(coefficients)>1 and abs(coefficients[-1])<1e-12:coefficients=coefficients[:-1]
    result=[]
    for root in poly.polyroots(coefficients):
        if abs(root.imag)>1e-7:continue
        angle=10*math.atan(float(root.real))
        if abs(angle)>math.radians(200):continue
        if not any(abs(angle-q)<1e-7 for q in result):result.append(angle)
    return sorted(result),False


def constraints(q,center,radius,mu_allow):
    """Same acceptance tests as inspect_pose, but return every violation.

    An extra +/-20 deg search is used only to expose joint-limit counterfactuals.
    Never report those poses as valid solutions.
    """
    surfaces,bodies,_,_=h.robot_polygons(q)
    gaps=np.array([h.circle_gap(p,radius,center) for p in surfaces]);violations=[];details=[]
    if np.max(np.abs(q))>math.pi+1e-12:violations.append('joint')
    penetration=[h.PART_NAMES[i] for i in range(len(surfaces)) if gaps[i]<-h.TOL]
    if gaps[h.CAMERA_INDEX]<=h.CLEARANCE:penetration.append('camera_clearance')
    penetration.extend(h.BODY_NAMES[i] for i,p in enumerate(bodies) if h.circle_gap(p,radius,center)<=h.CLEARANCE)
    if penetration:violations.append('object_collision');details.extend(penetration)
    # Self-collision is independent of the circle and checked even if object collision exists.
    if h.self_gaps(surfaces,bodies).min() < -h.TOL:violations.append('self_collision')
    contact_ids=np.flatnonzero(np.abs(gaps[:h.CAMERA_INDEX])<=h.TOL);best=None
    for i,j in itertools.combinations(contact_ids,2):
        pts=np.array([h.nearest_boundary([center],surfaces[k])[1][0] for k in (i,j)])
        if np.linalg.norm(pts[0]-pts[1])<1e-8:continue
        score=h.friction_score(pts,center-pts)['mu_critical']
        if math.isfinite(score) and (best is None or score<best):best=float(score)
    if best is None or best>=mu_allow:violations.append('friction')
    return dict(violations=violations,obstacles=sorted(set(details)),mu_min=best,
        seg1_deg=float(np.degrees(q[0])),seg2_deg=float(np.degrees(q[1])))


_CATALOG=None
_MU=None

def initialize(catalog,mu):
    global _CATALOG,_MU
    _CATALOG=catalog;_MU=mu


def probe(job):
    diameter,x,y,interior=job;center=h.local_center(x,y);radius=diameter/2000
    first_angles={};second_angles={};seen=set();candidates=[];degenerate=False;raw_roots=0
    for specs in _CATALOG:
        first,second=map(tuple,specs)
        if first not in first_angles:first_angles[first]=extended_angles(center,radius,first)
        q1s,bad=first_angles[first];degenerate|=bad
        for q1 in q1s:
            key=(q1,second)
            if key not in second_angles:second_angles[key]=extended_angles(inv.second_center(q1,center),radius,second)
            q2s,bad=second_angles[key];degenerate|=bad
            for q2 in q2s:
                raw_roots+=1;q=np.array([q1,q2])
                if not geom.on_features(q,specs,center,radius):continue
                key=(round(q1,8),round(q2,8))
                if key in seen:continue
                seen.add(key)
                c=constraints(q,center,radius,_MU);c['features']=specs;candidates.append(c)
    valid=[c for c in candidates if not c['violations']]
    if valid:
        # Cross-check each new candidate with the unchanged production validator.
        for c in valid:
            assert h.inspect_pose(np.radians([c['seg1_deg'],c['seg2_deg']]),center,_MU,radius) is not None
        best=min(valid,key=lambda c:c['mu_min']);label='recovered';reasons=[]
    elif candidates:
        minimum=min(len(c['violations']) for c in candidates)
        simplest=[c for c in candidates if len(c['violations'])==minimum]
        reason_sets=sorted(set(tuple(c['violations']) for c in simplest))
        label=reason_sets[0][0] if len(reason_sets)==1 and len(reason_sets[0])==1 else 'mixed'
        reasons=[list(s) for s in reason_sets]
        best=min(simplest,key=lambda c:(max(abs(c['seg1_deg']),abs(c['seg2_deg'])),c['mu_min'] if c['mu_min'] is not None else 999))
    else:label='unresolved';best=None;reasons=[]
    return dict(D=diameter,x=x,y=y,interior=interior,label=label,candidate_count=len(candidates),raw_roots=raw_roots,
        degenerate=bool(degenerate),reasons=reasons,best=best,
        recovered=[min((c for c in valid if (1 if c['seg2_deg']>0 else 2)==band),key=lambda c:c['mu_min'])
                   for band in (1,2) if any((1 if c['seg2_deg']>0 else 2)==band for c in valid)])


def main():
    ap=argparse.ArgumentParser();ap.add_argument('source',type=Path);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--workers',type=int,default=4);args=ap.parse_args()
    raw=args.source.read_bytes();doc=json.loads(raw)
    if doc['summary']['status']!='complete' or doc['metadata']['geometry']!=h.REFERENCE:raise ValueError('Expected complete current-geometry table')
    cfg=doc['metadata']['config'];dx=cfg['dx_mm'];dy=cfg['dy_mm']
    if dx!=5 or dy!=5:raise ValueError('This explanation viewer expects a 5 mm grid')
    catalog=set();maps=[];jobs=[]
    for entry in doc['maps']:
        records=[];found={(r['x_mm'],r['y_mm']) for r in entry['results'] if r['status']=='found'}
        for row in entry['results']:
            x,y=row['x_mm'],row['y_mm']
            if row['status']=='found':
                q=np.radians([row['seg1_deg'],row['seg2_deg']]);specs=tuple(geom.identify(q,n,np.array(p)) for n,p in zip(row['contact_parts'],row['contacts_node0_m']));catalog.add(specs)
                records.append(dict(x=x,y=y,band=1 if row['seg2_deg']>0 else 2,q1=row['seg1_deg'],q2=row['seg2_deg'],mu=row['mu_min'],features=specs,
                    joint_margin_deg=180-max(abs(row['seg1_deg']),abs(row['seg2_deg'])),friction_margin=cfg['mu_allow']-row['mu_min']))
            elif row['status']=='not_found':
                ns=[(x+sx,y+sy) in found for sx,sy in [(dx,0),(-dx,0),(0,dy),(0,-dy)]]
                if any(ns):jobs.append((entry['diameter_mm'],x,y,bool((ns[0] and ns[1]) or (ns[2] and ns[3]))))
        maps.append(dict(D=entry['diameter_mm'],found=records,probes=[]))
    catalog=sorted(catalog);print('found',sum(len(m['found']) for m in maps),'catalog',len(catalog),'probes',len(jobs),flush=True)
    start=time.perf_counter();by_d={m['D']:m for m in maps};results=[]
    with ProcessPoolExecutor(max_workers=args.workers,initializer=initialize,initargs=(catalog,cfg['mu_allow'])) as pool:
        for i,result in enumerate(pool.map(probe,jobs,chunksize=8)):
            results.append(result);by_d[result['D']]['probes'].append(result)
            if (i+1)%200==0:print(i+1,'/',len(jobs),round(time.perf_counter()-start,1),'s',dict(Counter(r['label'] for r in results)),flush=True)
    summary=[]
    for m in maps:
        bands={}
        for band in (1,2):
            rows=[r for r in m['found'] if r['band']==band]
            bands[band]=dict(count=len(rows),node_counts=dict(Counter(r['features'][0][0] for r in rows)),
                q2_abs=geom.percentiles([abs(r['q2']) for r in rows]),joint_within_2deg=sum(r['joint_margin_deg']<2 for r in rows),
                mu_above_point45=sum(r['mu']>.45 for r in rows))
        summary.append(dict(D=m['D'],bands=bands,probes=dict(Counter(r['label'] for r in m['probes'])),
            interior=dict(Counter(r['label'] for r in m['probes'] if r['interior']))))
    report=dict(source_sha256=hashlib.sha256(raw).hexdigest(),mu_allow=cfg['mu_allow'],elapsed_s=time.perf_counter()-start,catalog=catalog,
        scope='Every not_found cell cardinally adjacent to an observed success. Exact contacts; empirical 28-pair catalog; joint counterfactuals limited to 200 deg. No universal impossibility claims.',
        labeling='A recovered candidate passes original inspect_pose. Other labels summarize the least-violating candidates; mixed means different or simultaneous failures. Unresolved means no finite-feature candidate in the searched catalog.',
        summaries=summary,maps=maps,geometry=dict(profiles=h.REFERENCE['profiles'],body_center=h.BODY_CENTER.tolist()))
    args.output.mkdir(parents=True,exist_ok=True);(args.output/'explanation.json').write_text(json.dumps(report,separators=(',',':'))+'\n')
    template=Path(__file__).parents[1]/'support'/'hugmap_explanation_template.html'
    (args.output/'index.html').write_text(template.read_text().replace('__EXPLANATION_DATA__',json.dumps(report,separators=(',',':')).replace('<','\\u003c')))
    print('DONE',round(report['elapsed_s'],2),'s',dict(Counter(r['label'] for r in results)),flush=True)

if __name__=='__main__':main()
