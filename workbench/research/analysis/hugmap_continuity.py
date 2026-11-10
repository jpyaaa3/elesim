"""Piecewise algebraic contact branches and sampled continuity audits.

Static contact continuation, not a dynamics or approach stability proof.
The empirical contact catalog and floating-point roots are not exhaustive.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import time
import numpy as np
from workbench.research.support import hugmap as h

# Found by expanding all 4,920 seg1/distal feature pairs at missing samples.
# This is still a finite empirical catalog, not an exhaustiveness claim.
CONTINUATION_CATALOG=tuple(sorted(set(h.CATALOG)|{
    (('node1','v',4),('gripper_claw_left','e',6)),
    (('node0','v',5),('gripper_claw_left','e',6)),
    (('node0','e',4),('gripper_claw_left','e',6)),
    *((('node4','e',4),(f'node{i}','e',4)) for i in (5,6,7)),
}))


def branches(x, y, D, roll, catalog=None):
    """All discovered valid root branches, before applying a friction cutoff.

    Coordinates are mm, angles degrees. Identity is a contact feature pair;
    multiple roots with that identity are matched by proximity, not root index.
    """
    if roll not in (-90,90) or D<=0 or not all(map(math.isfinite,(x,y,D))):
        raise ValueError('Finite coordinates, positive D and roll +/-90 required')
    center=h.local_center(x*roll/90,y);radius=D/2000
    if x>0 or y<0 or x*x+y*y>500**2+1e-8 or h.exclusion(center,radius):
        return dict(candidates=[],degeneracies=0)
    catalog=CONTINUATION_CATALOG if catalog is None else catalog
    firsts={};seconds={};seen=set();out=[];degeneracies=0
    for a,b in catalog:
        try:
            if a not in firsts:firsts[a]=h.feature_angles(center,radius,a)
            for q1 in firsts[a]:
                key=(q1,b)
                if key not in seconds:seconds[key]=h.feature_angles(h.second_center(q1,center),radius,b)
                for q2 in seconds[key]:
                    q=np.array([q1,q2]);pose=tuple(np.round(q,8))
                    if q2>=0 or pose in seen or not h.contact_matches(q,(a,b),center,radius):continue
                    seen.add(pose)
                    sol=h.inspect_pose(q,center,math.inf,radius)
                    if sol is None:continue
                    out.append(dict(q_deg=np.degrees(q).tolist(),mu=sol['mu_min'],features=[a,b],
                                    contact_parts=sol['contact_parts']))
        except (ArithmeticError,np.linalg.LinAlgError):degeneracies+=1
    return dict(candidates=sorted(out,key=lambda r:r['mu']),degeneracies=degeneracies)


def required_mu(x,y,D,roll):
    """Geometry-derived scalar f_roll(x,y,D); None means no catalog solution."""
    candidates=branches(x,y,D,roll)['candidates']
    return candidates[0]['mu'] if candidates else None


def bottleneck_path(layers, start_q=None, end_q=None):
    """Minimize largest adjacent joint jump over all sampled candidate paths.

    No arbitrary angular threshold is baked into the result. A positive jump
    that persists under refinement is evidence against this sampled path,
    not proof that no alternative curve in (x,y,D) exists.
    """
    if not layers or any(not a for a in layers):return None
    qs=[np.array([c['q_deg'] for c in a]) for a in layers]
    cost=np.zeros(len(qs[0]))
    if start_q is not None:
        cost[:]=np.inf;cost[np.argmin(np.max(abs(qs[0]-start_q),axis=1))]=0
    parents=[]
    for previous,current in zip(qs,qs[1:]):
        distance=np.max(abs(previous[:,None,:]-current[None,:,:]),axis=2)
        proposal=np.maximum(cost[:,None],distance)
        parent=np.argmin(proposal,axis=0);cost=proposal[parent,np.arange(len(current))]
        parents.append(parent)
    index=int(np.argmin(cost)) if end_q is None else int(np.argmin(np.max(abs(qs[-1]-end_q),axis=1)))
    jump=float(cost[index]);indices=[index]
    for parent in reversed(parents):indices.append(int(parent[indices[-1]]))
    indices.reverse();path=[layer[i] for layer,i in zip(layers,indices)]
    return dict(max_step_deg=jump,path=path,max_mu=max(c['mu'] for c in path))


def trace(start,end,roll,steps=40,mu_allow=.5):
    points=np.linspace(start,end,steps+1)
    results=[branches(*p,roll) for p in points]
    layers=[r['candidates'] for r in results]
    feasible=[[c for c in a if c['mu']<mu_allow] for a in layers]
    minima=[a[0] if a else None for a in layers]
    jumps=[float(np.max(abs(np.array(a['q_deg'])-b['q_deg']))) for a,b in zip(minima,minima[1:]) if a and b]
    endpoints=(minima[0]['q_deg'],minima[-1]['q_deg']) if minima[0] and minima[-1] else (None,None)
    return dict(start=list(start),end=list(end),roll=roll,steps=steps,
                no_geometric_root=sum(not a for a in layers),infeasible_samples=sum(not a for a in feasible),
                degeneracies=sum(r['degeneracies'] for r in results),
                best_max_step_deg=max(jumps,default=None),
                geometry_path=bottleneck_path(layers),
                feasible_path=bottleneck_path(feasible),
                best_endpoint_path=bottleneck_path(feasible,*endpoints),
                samples=[dict(point=p.tolist(),minimum=m,candidate_count=len(a)) for p,m,a in zip(points,minima,layers)])


def _job(item):
    tag,start,end,roll,steps=item
    return dict(tag=tag,**trace(start,end,roll,steps))


def write_viewer(report,output):
    template=Path(__file__).resolve().parents[1]/'support'/'hugmap_continuity_template.html'
    output.write_text(template.read_text().replace('__DATA__',json.dumps(report).replace('<','\\u003c')))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('source',type=Path);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--workers',type=int,default=4);args=ap.parse_args()
    raw=args.source.read_bytes();doc=json.loads(raw)
    if doc['metadata']['geometry']!=h.REFERENCE:raise ValueError('Geometry mismatch')
    if doc['metadata']['config']['mu_allow']!=.5:raise ValueError('This audit expects mu=0.5')
    rows={}
    for m in doc['maps']:
        for r in m['results']:
            for sol in r.get('roll_solutions',[]):
                rows[(r['x_mm'],r['y_mm'],m['diameter_mm'],sol['roll_deg'])]=sol
    edges=[];stats={}
    for roll in (-90,90):
        selected=[]
        for p,a in rows.items():
            if p[3]!=roll:continue
            for axis in range(3):
                other=list(p);other[axis]+=5;other=tuple(other)
                if other not in rows:continue
                b=rows[other];jump=max(abs(a[k]-b[k]) for k in ('seg1_deg','seg2_deg'))
                selected.append((jump,axis,p,other))
        vals=np.array([r[0] for r in selected]);stats[str(roll)]=dict(edges=len(vals),joint_jump_deg_percentiles=np.percentile(vals,[50,90,95,99,100]).tolist(),over_20_deg=int(sum(vals>20)))
        # Worst and representative adjacent edges in each spatial/size direction.
        for axis in range(3):
            subset=sorted([r for r in selected if r[1]==axis],reverse=True)
            for idx in sorted({0,1,len(subset)//2}):
                jump,_,a,b=subset[idx];edges.append((f'neighbor_axis{axis}_jump{jump:.3f}',a[:3],b[:3],roll,40))
    # Cross-sections through the visible band2 split, keeping above-threshold
    # geometric branches so a friction gap is not mistaken for lost geometry.
    for d in (140,150,160,170):
        ys=[p[1] for p in rows if p[0]==-100 and p[2]==d and p[3]==90]
        edges.append((f'split_D{d}',(-100,min(ys),d),(-100,max(ys),d),90,100))
    for roll in (-90,90):edges.append(('diameter_sweep',(-100,370,130),(-100,370,180),roll,100))
    start=time.monotonic();args.output.mkdir(parents=True,exist_ok=True)
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        results=[]
        for r in pool.map(_job,edges):
            results.append(r);print(r['tag'],r['roll'],'best',r['best_max_step_deg'],'path',r['feasible_path']['max_step_deg'] if r['feasible_path'] else None,flush=True)
    # Refine worst-edge traces 4x to distinguish discretization from switching.
    refinements=[(r['tag']+'_refined',r['start'],r['end'],r['roll'],160) for r in results if r['tag'].startswith('neighbor') and r['best_max_step_deg'] is not None and r['best_max_step_deg']>.2]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        refined=list(pool.map(_job,refinements))
    report=dict(source_sha256=hashlib.sha256(raw).hexdigest(),neighbor_statistics=stats,traces=results,refined=refined,
                elapsed_s=time.monotonic()-start,catalog_size=len(CONTINUATION_CATALOG),
                limitations='Sampled straight-line continuation of static contact roots. No dynamics, global infeasibility or full-domain continuity certificate. Catalog is empirical; root degeneracies are not resolved.')
    (args.output/'continuity.json').write_text(json.dumps(report,indent=2))
    write_viewer(report,args.output/'index.html')
    print(json.dumps(dict(stats=stats,elapsed_s=report['elapsed_s'],refinements=len(refined))),flush=True)

if __name__=='__main__':main()
