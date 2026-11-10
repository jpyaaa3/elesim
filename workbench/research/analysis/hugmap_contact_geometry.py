"""Exact contact-feature constructions for the fixed Hugmap polygon geometry.

No fitted coefficients and no inverse-kinematics optimizer are used here.
Run: python -m workbench.research.analysis.hugmap_contact_geometry SOURCE --output DIR
"""
import argparse
from collections import Counter
import hashlib
import itertools
import json
import math
from pathlib import Path
import time

import numpy as np
from workbench.research.support import hugmap as h


def rotate(p, angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.asarray(p) @ np.array([[c,s],[-s,c]])


def chain_sum(n, angle, pitch=.05):
    """Sum n rotated links by a finite trigonometric-series identity."""
    if n==0:return np.zeros(2)
    factor=n*np.sinc(n*angle/(2*np.pi))/np.sinc(angle/(2*np.pi))
    middle=(n+1)*angle/2
    return pitch*factor*np.array([-math.sin(middle),math.cos(middle)])


def frame(q, part):
    index=min(part,10);a,b=np.asarray(q)/5
    position=chain_sum(min(index,5),a)
    if index>5:position+=rotate(chain_sum(index-5,b),q[0])
    angle=(index+1)*a if index<5 else q[0]+(index-4)*b if index<10 else sum(q)
    return position,angle


def polygon(q, part):
    position,angle=frame(q,part)
    return position+rotate(h.PROFILES[part],angle)


def feature(q, name, kind, index):
    vertices=polygon(q,h.PART_NAMES.index(name))
    if kind=='v':return dict(kind='v',point=vertices[index])
    a=vertices[index];b=vertices[(index+1)%len(vertices)];length=np.linalg.norm(b-a);t=(b-a)/length
    area=np.sum(vertices[:,0]*np.roll(vertices[:,1],-1)-vertices[:,1]*np.roll(vertices[:,0],-1))
    n=np.sign(area)*np.array([t[1],-t[0]])
    return dict(kind='e',point=a,t=t,n=n,length=length)


def identify(q, name, point):
    v=polygon(q,h.PART_NAMES.index(name));distances=np.linalg.norm(v-point,axis=1)
    if distances.min()<1e-7:return (name,'v',int(distances.argmin()))
    delta=np.roll(v,-1,axis=0)-v
    fractions=np.clip(np.sum((point-v)*delta,axis=1)/np.sum(delta*delta,axis=1),0,1)
    return (name,'e',int(np.argmin(np.linalg.norm(v+fractions[:,None]*delta-point,axis=1))))


def centers(a,b,radius):
    """Circle centers touching two features; finite-edge/vertex domains separate."""
    if a['kind']=='e' and b['kind']=='e':
        matrix=np.array([a['n'],b['n']]);det=np.linalg.det(matrix)
        # Parallel offset lines yield no unique center (none or a continuum).
        if abs(det)<1e-12:return []
        return [np.linalg.solve(matrix,[a['n']@a['point']+radius,b['n']@b['point']+radius])]
    if a['kind']=='v' and b['kind']=='v':
        delta=b['point']-a['point'];length=np.linalg.norm(delta)
        if length<1e-12:return []
        squared=radius**2-length**2/4
        if squared< -1e-14:return []
        offset=np.sqrt(max(0,squared))*np.array([-delta[1],delta[0]])/length
        mid=(a['point']+b['point'])/2
        return [mid+offset,mid-offset]
    edge,vertex=(a,b) if a['kind']=='e' else (b,a)
    base=edge['point']+radius*edge['n'];delta=vertex['point']-base
    squared=radius**2-(edge['n']@delta)**2
    if squared< -1e-14:return []
    along=edge['t']@delta;root=np.sqrt(max(0,squared))
    return [base+(along+root)*edge['t'],base+(along-root)*edge['t']]


def contact_point(f,center):
    return f['point'] if f['kind']=='v' else f['point']+((center-f['point'])@f['t'])*f['t']


def on_features(q,specs,center,radius,tolerance=1e-7):
    for spec in specs:
        f=feature(q,*spec);point=contact_point(f,center)
        if f['kind']=='e':
            along=(point-f['point'])@f['t']
            if along < -tolerance or along > f['length']+tolerance:return False
        vertices=polygon(q,h.PART_NAMES.index(spec[0]))
        gap,closest,_=h.nearest_boundary([center],vertices)
        if abs(gap[0]-radius)>tolerance or np.linalg.norm(closest[0]-point)>tolerance:return False
        if h.inside_polygon([center],vertices)[0]:return False
    return True


def circle_mu(p1,p2,radius):
    """Equal-radius point contacts: mu_critical = sqrt(D^2/d^2 - 1)."""
    d=np.linalg.norm(np.asarray(p1)-np.asarray(p2))
    return math.sqrt(max(0,4*radius**2/d**2-1)) if d>1e-12 else math.inf


def percentiles(values):
    return dict(zip(['median','p95','max'],map(float,np.percentile(values,[50,95,100])))) if values else None


def main():
    ap=argparse.ArgumentParser();ap.add_argument('source',type=Path);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    raw=args.source.read_bytes();doc=json.loads(raw)
    if doc['metadata']['geometry']!=h.REFERENCE:raise ValueError('Geometry mismatch')
    report=dict(source_sha256=hashlib.sha256(raw).hexdigest(),bands={});samples={1:[],2:[]}
    for band in (1,2):
        errors=[];mu_errors=[];counts=Counter();kinds=Counter();missing=0;outside=0;special=[];entries=[];missing_entries=[]
        for m in doc['maps']:
            radius=m['diameter_mm']/2000
            for row in m['results']:
                if row['status']!='found' or (1 if row['seg2_deg']>0 else 2)!=band:continue
                q=np.radians([row['seg1_deg'],row['seg2_deg']]);center=h.local_center(row['x_mm'],row['y_mm'])
                specs=[identify(q,name,np.array(point)) for name,point in zip(row['contact_parts'],row['contacts_node0_m'])]
                key=' / '.join(f'{name}:{kind}{index}' for name,kind,index in specs);counts[key]+=1;kinds[''.join(s[1] for s in specs)]+=1
                fs=[feature(q,*spec) for spec in specs];candidates=centers(*fs,radius)
                if not candidates:
                    missing+=1;missing_entries.append(dict(D=m['diameter_mm'],x=row['x_mm'],y=row['y_mm'],q1=row['seg1_deg'],q2=row['seg2_deg'],features=key,contact_gaps_mm=row['contact_gaps_mm']));continue
                chosen=min(candidates,key=lambda c:np.linalg.norm(c-center));error=np.linalg.norm(chosen-center)*1000
                errors.append(error);mu_errors.append(abs(circle_mu(*row['contacts_node0_m'],radius)-row['mu_min']))
                if not on_features(q,specs,chosen,radius):outside+=1
                if key=='node3:e4 / gripper_claw_left:e6':
                    delta=q[0]/5+q[1];special.append(abs(abs(1/math.tan(delta/2))-row['mu_min']))
                entries.append(dict(D=m['diameter_mm'],x=row['x_mm'],y=row['y_mm'],q1=row['seg1_deg'],q2=row['seg2_deg'],features=key,error_mm=float(error)))
                if m['diameter_mm']==150:samples[band].append([row['x_mm'],row['y_mm']])
        report['bands'][band]=dict(count=sum(counts.values()),feature_pairs=counts.most_common(),feature_types=kinds,missing_real_intersection=missing,
            center_error_mm=percentiles(errors),missing_entries=missing_entries,outside_exact_feature_domain=outside,friction_chord_error=percentiles(mu_errors),
            dominant_face_face_count=len(special),dominant_face_face_mu_error=percentiles(special),worst_reconstructions=sorted(entries,key=lambda r:r['error_mm'],reverse=True)[:5])
        print('BAND',band,json.dumps(report['bands'][band]),flush=True)
    # Forward construction independent of stored q values: no solve_placement calls.
    sweeps=[(1,[('node1','e',1),('gripper_claw_left','v',0)],range(-90,91,2),range(120,181,2)),
            (1,[('node2','e',1),('gripper_claw_left','v',0)],range(-90,91,2),range(120,181,2)),
            (2,[('node3','e',4),('gripper_claw_left','e',6)],range(0,121,2),range(-180,-119,2))]
    forward=[];stats=[]
    for band,specs,q1s,q2s in sweeps:
        start=time.perf_counter();n=0;geometric=0;valid=0
        for q1,q2 in itertools.product(q1s,q2s):
            q=np.radians([q1,q2]);fs=[feature(q,*spec) for spec in specs]
            for c in centers(*fs,.075):
                n+=1
                if not on_features(q,specs,c,.075):continue
                points=[contact_point(f,c) for f in fs]
                if circle_mu(*points,.075)>=.5:continue
                xy=(c-h.BODY_CENTER)*1000
                if xy[0]>0 or xy[1]<0 or np.linalg.norm(xy)>500:continue
                geometric+=1
                sol=h.inspect_pose(q,c,.5,.075)
                if sol is None:continue
                valid+=1;forward.append(dict(band=band,x_mm=float(xy[0]),y_mm=float(xy[1]),seg1_deg=q1,seg2_deg=q2,mu_min=sol['mu_min']))
        stats.append(dict(band=band,features=specs,angle_pairs=len(q1s)*len(q2s),circle_centers=n,geometric_and_friction_pass=geometric,full_validator_pass=valid,elapsed_s=time.perf_counter()-start))
        print('FORWARD',stats[-1],flush=True)
    report['forward_D150']=stats
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'audit.json').write_text(json.dumps(report,indent=2)+'\n')
    (args.output/'forward.json').write_text(json.dumps(forward,separators=(',',':'))+'\n')
    # Native vector figure, no raster or external plotting dependency.
    svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="560" viewBox="0 0 1000 560"><rect width="100%" height="100%" fill="white"/>']
    for band in (1,2):
        left=40+(band-1)*500
        svg.append(f'<text x="{left}" y="24" font-family="sans-serif" font-size="17">Band {band}, D = 150 mm</text>')
        svg.append(f'<path d="M {left} 50 V 510 H {left+440}" stroke="#888" fill="none"/>')
        for x,y in samples[band]:svg.append(f'<circle cx="{left-x*.88:.3f}" cy="{510-y*.88:.3f}" r="2.2" fill="#999"/>')
        for r in forward:
            if r['band']==band:svg.append(f'<circle cx="{left-r["x_mm"]*.88:.3f}" cy="{510-r["y_mm"]*.88:.3f}" r="1.5" fill="#e06720"/>')
    svg.append('<text x="40" y="545" font-family="sans-serif" font-size="14">Gray: recorded successes. Orange: forward contact formulas + full collision/friction validation (selected features only).</text></svg>')
    (args.output/'forward.svg').write_text(''.join(svg))

if __name__=='__main__':main()
