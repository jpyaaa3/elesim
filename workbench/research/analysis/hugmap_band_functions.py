"""Independent geometry-based band functions of x, y, D.

Piecewise algebraic root functions, NOT elementary closed-form boundaries.
No learned coefficients or source JSON needed for a query. The physical seg2 angle is negative (away from ZED); roll is -90 for band1
and +90 for band2. The contact catalog includes opposite-side features and is empirical; a missing root is not a proof of global impossibility.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
from workbench.research.analysis import hugmap_algebraic_inverse as inv
from workbench.research.analysis import hugmap_contact_geometry as geom
from workbench.research.support import hugmap as h

OBSERVED_CATALOG = ((('node0', 'e', 1), ('gripper_claw_left', 'v', 0)), (('node0', 'v', 1), ('gripper_claw_left', 'v', 0)), (('node1', 'e', 1), ('gripper_claw_left', 'e', 0)), (('node1', 'e', 1), ('gripper_claw_left', 'v', 0)), (('node1', 'v', 1), ('gripper_claw_left', 'e', 0)), (('node1', 'v', 1), ('gripper_claw_left', 'v', 0)), (('node1', 'v', 2), ('gripper_claw_left', 'v', 0)), (('node2', 'e', 1), ('gripper_claw_left', 'e', 0)), (('node2', 'e', 1), ('gripper_claw_left', 'v', 0)), (('node2', 'e', 4), ('gripper_claw_left', 'v', 7)), (('node2', 'v', 1), ('gripper_claw_left', 'v', 0)), (('node2', 'v', 2), ('gripper_claw_left', 'e', 0)), (('node2', 'v', 2), ('gripper_claw_left', 'v', 0)), (('node2', 'v', 5), ('gripper_claw_left', 'e', 6)), (('node2', 'v', 5), ('gripper_claw_left', 'v', 7)), (('node3', 'e', 4), ('gripper_base', 'e', 3)), (('node3', 'e', 4), ('gripper_claw_left', 'e', 6)), (('node3', 'e', 4), ('gripper_claw_left', 'v', 6)), (('node3', 'e', 4), ('gripper_claw_left', 'v', 7)), (('node3', 'v', 4), ('gripper_claw_left', 'e', 6)), (('node3', 'v', 4), ('gripper_claw_left', 'v', 7)), (('node3', 'v', 5), ('gripper_base', 'e', 3)), (('node3', 'v', 5), ('gripper_claw_left', 'e', 6)), (('node3', 'v', 5), ('gripper_claw_left', 'v', 6)), (('node4', 'e', 4), ('node9', 'e', 4)), (('node4', 'v', 4), ('gripper_base', 'e', 3)), (('node4', 'v', 4), ('gripper_base', 'v', 3)), (('node4', 'v', 4), ('node9', 'e', 4)))


def mirror_feature(spec):
    """Corresponding opposite-side feature, not a transformed collision model."""
    name,kind,index=spec;vertices=h.PROFILES[h.PART_NAMES.index(name)]
    axis=(vertices[:,0].min()+vertices[:,0].max())/2
    reflected=vertices.copy();reflected[:,0]=2*axis-reflected[:,0]
    mapping=[int(np.argmin(np.linalg.norm(vertices-point,axis=1))) for point in reflected]
    if max(np.linalg.norm(vertices[j]-point) for point,j in zip(reflected,mapping))>1e-8:
        raise ValueError('Contact profile is not laterally symmetric about its own center')
    if kind=='v':return name,kind,mapping[index]
    # Reflection reverses polygon winding.
    return name,kind,mapping[(index+1)%len(vertices)]


CATALOG=tuple(sorted(set(OBSERVED_CATALOG)|{tuple(mirror_feature(s) for s in pair) for pair in OBSERVED_CATALOG}))


def evaluate(x,y,D,mu_allow=.5):
    """Return both independent mu requirements and signed friction margins.

    f_k = mu_allow - min(mu_critical over collision-free, joint-limited exact
    contact roots with physical q2<0, at roll=-90/+90). No candidate => score=-infinity in band1/2;
    the serializable result uses None and explicit coverage status instead.
    """
    if not all(math.isfinite(v) for v in (x,y,D,mu_allow)) or D<=0 or mu_allow<=0:
        raise ValueError('Finite coordinates, positive diameter and friction required')
    if x>0 or y<0 or x*x+y*y>500**2+1e-8:
        return dict(bands=[None,None],status='outside_interest',degeneracies=[])
    radius=D/2000;bands=[];degeneracies=[]
    for roll in (-90,90):
        sign=roll/90
        # The whole arm (including gripper and camera) is rigidly mirrored in
        # top view when roll changes. Solve in its original intrinsic frame.
        center=h.local_center(sign*x,y)
        if h.exclusion(center,radius):bands.append(None);continue
        result=inv.solve_catalog(center,radius,CATALOG,math.inf,q2_negative_only=True)
        degeneracies.extend(dict(r,roll_deg=roll) for r in result['degeneracies'])
        candidates=result['solutions']
        if candidates:
            best=min(candidates,key=lambda r:r['mu_min'])
            bands.append(dict(mu_required=best['mu_min'],score=mu_allow-best['mu_min'],roll_deg=roll,
                seg1_deg=best['seg1_deg'],seg2_deg=best['seg2_deg'],
                planar_seg1_deg=sign*best['seg1_deg'],planar_seg2_deg=sign*best['seg2_deg'],contact_parts=best['contact_parts']))
        else:bands.append(None)
    status='evaluated' if any(bands) else 'body_excluded' if h.exclusion(h.local_center(x,y),radius) else 'no_catalog_solution'
    return dict(bands=bands,status=status,degeneracies=degeneracies)


def band1(x,y,D,mu_allow=.5):
    value=evaluate(x,y,D,mu_allow)['bands'][0]
    return value['score'] if value is not None else -math.inf


def band2(x,y,D,mu_allow=.5):
    value=evaluate(x,y,D,mu_allow)['bands'][1]
    return value['score'] if value is not None else -math.inf


def friction_boundary(q1_deg,D,mu_allow=.5):
    """Closed-form curve for node3 side / gripper side at the friction cutoff.

    No root search or fitting. Other inequalities determine the valid subarcs.
    """
    delta=-2*math.atan(1/mu_allow)
    q1=math.radians(q1_deg);q2=delta-q1/5
    if max(abs(q1),abs(q2))>math.pi:return None
    q=np.array([q1,q2]);specs=[('node3','e',4),('gripper_claw_left','e',6)]
    fs=[geom.feature(q,*s) for s in specs]
    cs=geom.centers(*fs,D/2000)
    if not cs:return None
    center=cs[0];xy=(center-h.BODY_CENTER)*1000
    if xy[0]>0 or xy[1]<0 or np.linalg.norm(xy)>500:return None
    if not geom.on_features(q,specs,center,D/2000):return None
    valid=h.inspect_pose(q,center,mu_allow+1e-7,D/2000) is not None
    return dict(x=float(xy[0]),y=float(xy[1]),q1=q1_deg,q2=math.degrees(q2),valid_subarc=valid)


def camera_margin(phi_deg,D,vertex_index=7):
    """Camera clearance for a chosen claw corner in the rigid gripper frame."""
    radius=D/2000;vertex=np.asarray(h.REFERENCE['profiles']['gripper_claw_left'][vertex_index])
    phi=math.radians(phi_deg);center=vertex+radius*np.array([math.cos(phi),math.sin(phi)])
    return float(h.signed_distances([center],np.asarray(h.REFERENCE['profiles']['camera']))[0])-radius-h.CLEARANCE


def camera_boundary(D,vertex_index=7):
    """Elementary line/circle constructions; no joint solving or angle search.

    phi lies in the chosen gripper vertex's outward normal cone.
    Returns contact-direction boundaries and camera-clear angular intervals.
    """
    if vertex_index not in (0,7):raise ValueError('Only the two top claw corners are supported')
    lower,upper=(90.,180.) if vertex_index==0 else (0.,90.)
    radius=D/2000;clearance=h.CLEARANCE;vertex=np.asarray(h.REFERENCE['profiles']['gripper_claw_left'][vertex_index])
    camera=np.asarray(h.REFERENCE['profiles']['camera']);candidates=[]
    def add(angle,feature):
        angle=angle%(2*np.pi)
        if not math.radians(lower)-1e-10<=angle<=math.radians(upper)+1e-10:return
        degrees=math.degrees(angle)
        if abs(camera_margin(degrees,D,vertex_index))>1e-8:return
        if not any(abs(degrees-r['phi_deg'])<1e-6 for r in candidates):candidates.append(dict(phi_deg=degrees,feature=feature))
    for i,w in enumerate(camera):
        vector=w-vertex;length=np.linalg.norm(vector)
        value=(length*length-clearance*clearance-2*radius*clearance)/(2*radius*length)
        if abs(value)<=1:
            origin=math.atan2(vector[1],vector[0]);offset=math.acos(value)
            add(origin+offset,'vertex'+str(i));add(origin-offset,'vertex'+str(i))
    area=np.sum(camera[:,0]*np.roll(camera[:,1],-1)-camera[:,1]*np.roll(camera[:,0],-1))
    for i,a in enumerate(camera):
        delta=camera[(i+1)%len(camera)]-a;length=np.linalg.norm(delta);t=delta/length;n=np.sign(area)*np.array([t[1],-t[0]])
        value=1+(clearance-n@(vertex-a))/radius
        if abs(value)<=1:
            origin=math.atan2(n[1],n[0]);offset=math.acos(value)
            for angle in (origin+offset,origin-offset):
                center=vertex+radius*np.array([math.cos(angle),math.sin(angle)])
                along=(center-a)@t
                if -1e-10<=along<=length+1e-10:add(angle,'edge'+str(i))
    candidates.sort(key=lambda r:r['phi_deg']);cuts=[lower]+[r['phi_deg'] for r in candidates]+[upper]
    allowed=[[a,b] for a,b in zip(cuts,cuts[1:]) if camera_margin((a+b)/2,D,vertex_index)>0]
    return dict(boundaries=candidates,camera_clear_intervals_deg=allowed,
        vertex_index=vertex_index,semantics='Camera only; positive clearance in the chosen corner normal cone. Not the full band condition.')


def job(item):
    D,x,y=item
    return D,x,y,evaluate(x,y,D)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path);ap.add_argument('--output',type=Path);ap.add_argument('--workers',type=int,default=4);ap.add_argument('--grid-step',type=int,default=10);ap.add_argument('--diameters',nargs='+',type=float,default=[100,130,150,190,200]);ap.add_argument('--point',nargs=3,type=float,metavar=('X','Y','D'));args=ap.parse_args()
    if args.point:
        print(json.dumps(evaluate(*args.point),indent=2,allow_nan=False));return
    if args.source is None or args.output is None:ap.error('--source and --output required for grid study')
    raw=args.source.read_bytes();source=json.loads(raw)
    if source['metadata']['geometry']!=h.REFERENCE or source['summary']['status']!='complete':raise ValueError('Expected complete matching geometry')
    maps=[];jobs=[]
    for m in source['maps']:
        D=m['diameter_mm']
        if D not in args.diameters:continue
        rows=[r for r in m['results'] if abs(r['x_mm']%args.grid_step)<1e-8 and abs(r['y_mm']%args.grid_step)<1e-8]
        observed=[[r['x_mm'],r['y_mm'],1 if r['seg2_deg']>0 else 2] for r in rows if r['status']=='found']
        maps.append(dict(D=D,observed=observed,values=[],boundary=[]))
        jobs.extend((D,r['x_mm'],r['y_mm']) for r in rows)
    by_D={m['D']:m for m in maps};start=time.perf_counter();degenerate=0
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for i,(D,x,y,result) in enumerate(pool.map(job,jobs,chunksize=24)):
            degenerate+=bool(result['degeneracies'])
            if any(result['bands']):
                by_D[D]['values'].append([x,y,*[None if b is None else [b['mu_required'],b['seg1_deg'],b['seg2_deg'],b['roll_deg']] for b in result['bands']]])
            if (i+1)%8000==0:print(i+1,'/',len(jobs),round(time.perf_counter()-start,1),'s',flush=True)
    for m in maps:
        m['camera_boundary']=camera_boundary(m['D'])
        for q1 in np.arange(0,180.01,.25):
            p=friction_boundary(float(q1),m['D'])
            if p is not None and p['valid_subarc']:m['boundary'].append(p)
    report=dict(source_sha256=hashlib.sha256(raw).hexdigest(),formula='f_k = mu_allow - mu_required,k; mu_required,k is the minimum over geometric contact roots of the chosen band, after collision and joint-limit tests',
        scope='Observed contact catalog plus opposite-side features; numerical roots. Physical seg2<0, roll=-90 for band1 and +90 for band2. Legacy gaga lacks this policy. Not an elementary closed form or a proof of completeness.',xy_step_mm=args.grid_step,policy=dict(seg2='negative: away from ZED in the current logical geometry',roll_deg=[-90,90]),legacy_reference=True,
        mu_reference=source['metadata']['config']['mu_allow'],catalog=CATALOG,maps=maps,degenerate_cells=degenerate,elapsed_s=time.perf_counter()-start)
    args.output.mkdir(parents=True,exist_ok=True);(args.output/'functions.json').write_text(json.dumps(report,separators=(',',':'),allow_nan=False)+'\n')
    template=Path(__file__).parents[1]/'support'/'hugmap_band_functions_template.html'
    if template.exists():(args.output/'index.html').write_text(template.read_text().replace('__BAND_FUNCTION_DATA__',json.dumps(report,separators=(',',':')).replace('<','\\u003c')))
    print('DONE',report['elapsed_s'],'degenerate',degenerate,flush=True)

if __name__=='__main__':main()
