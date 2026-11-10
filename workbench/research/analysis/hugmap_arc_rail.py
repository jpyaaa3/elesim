"""A small geometric model of both hug bands: one circular arc and one rail.

Closed-form parametric surrogate, not an exact CAD feasibility certificate.
No fitted coefficients, polynomial roots, inverse solver or source JSON needed
for forward() or seg1_from_center(). All lengths inside the model are metres.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
from collections import Counter
import numpy as np

PITCH=.05
HALF_WIDTH=.035
RAIL_WIDTH=.03546
RAIL_MIN=-.038627811
RAIL_MAX=.089
MOUNT_Y=.1445


def rotation(q):
    c,s=math.cos(q),math.sin(q)
    return np.array([[c,-s],[s,c]])


def endpoint(q):
    """Exact five-link endpoint; sinc handles the straight-arm limit."""
    factor=5*PITCH*np.sinc(q/(2*np.pi))/np.sinc(q/(10*np.pi))
    return factor*np.array([-math.sin(3*q/5),math.cos(3*q/5)])


def seg1_from_center(x,y,D,roll):
    """Approximate q1 in degrees from a single explicit atan rational function.

    Uses the circular envelope of seg1's right side. Polygon corners cause
    approximation error. Returns the principal physical bend, not a fit.
    """
    if roll not in (-90,90) or D<=0:raise ValueError('Positive D and roll +/-90 required')
    X=x*roll/90000;Y=y/1000-MOUNT_Y+PITCH/2;A=D/2000+HALF_WIDTH
    numerator=PITCH*(X-A);denominator=A*A-X*X-Y*Y
    if abs(denominator)<1e-15:
        return None
    return math.degrees(10*math.atan(numerator/denominator))


def _mu(n1,n2):
    d=float(np.clip(n1@n2,-1,1))
    return math.sqrt(max(0,(1+d)/(1-d))) if d<1-1e-14 else math.inf


def forward(q1_deg,q2_deg,D,roll,check_domains=True):
    """Closed-form object centres, indexed by the two physical bend angles.

    Circle--line intersection gives side contact; two circle intersections give
    the rail's rounded endpoint continuations. Arc span and rail domains are
    checked here. CAD collisions/clearances and mechanical feasibility are not.
    """
    if roll not in (-90,90) or D<=0 or not -180<=q1_deg<=180 or not -180<=q2_deg<0:
        raise ValueError('Invalid roll, diameter or joint range')
    q1,q2=map(math.radians,(q1_deg,q2_deg));R=D/2000;A=HALF_WIDTH+R
    G=endpoint(q1)+rotation(q1)@endpoint(q2);rot=rotation(q1+q2)
    n=rot[:,0];t=rot[:,1];out=[]
    straight=abs(q1)<1e-10
    if not straight:
        k=PITCH/(2*math.tan(q1/10));K=np.array([-k,-PITCH/2]);signed_radius=k+A
        if abs(signed_radius)<1e-12:return []
        rho=abs(signed_radius)

    def add(c,kind,normal2):
        normal1=np.array([1.,0.]) if straight else (c-K)/signed_radius
        along=float((c-G)@t)
        if check_domains:
            if straight:
                if not PITCH*.22744378-1e-9<=c[1]<=5*PITCH+1e-9:return
            else:
                theta=math.atan2(normal1[1],normal1[0]);theta+=round((q1*.6-theta)/(2*math.pi))*2*math.pi
                if not min(q1/5,q1)-1e-9<=theta<=max(q1/5,q1)+1e-9:return
            if kind=='side' and not RAIL_MIN-1e-9<=along<=RAIL_MAX+1e-9:return
            if kind=='tip' and (normal2@t< -1e-9 or normal2@n< -1e-9):return
            if kind=='heel' and (normal2@t>1e-9 or normal2@n< -1e-9):return
        out.append(dict(x=float(c[0]*1000*roll/90),y=float((c[1]+MOUNT_Y)*1000),
                        mu=_mu(normal1,normal2),normal_dot=float(np.clip(normal1@normal2,-1,1)),
                        kind=kind,rail_coordinate_mm=along*1000))

    offset=RAIL_WIDTH+R
    if straight:
        if abs(n[1])>1e-12:
            add(np.array([A,(n@G+offset-n[0]*A)/n[1]]),'side',n)
    else:
        d=offset+float(n@(G-K));disc=signed_radius**2-d*d
        if disc>=-1e-13:
            for sign in (-1,1):add(K+d*n+sign*math.sqrt(max(0,disc))*t,'side',n)
    for kind,s in [('tip',RAIL_MAX),('heel',RAIL_MIN)]:
        B=G+rot@np.array([RAIL_WIDTH,s])
        if straight:
            disc=R*R-(A-B[0])**2
            if disc<0:continue
            for sign in (-1,1):
                c=np.array([A,B[1]+sign*math.sqrt(disc)]);add(c,kind,(c-B)/R)
        else:
            delta=B-K;length=float(np.linalg.norm(delta))
            if length<1e-12:continue
            direction=delta/length;along=(rho*rho-R*R+length*length)/(2*length);disc=rho*rho-along*along
            if disc< -1e-13:continue
            tangent=np.array([-direction[1],direction[0]])
            for sign in (-1,1):
                c=K+along*direction+sign*math.sqrt(max(0,disc))*tangent;add(c,kind,(c-B)/R)
    return out


def summary(values):
    return dict(zip(('median','p90','p95','p99','max'),map(float,np.percentile(values,[50,90,95,99,100])))) if values else None


def write_viewer(report,output):
    rows=[]
    for s in report['cases']:
        c=s['reconstruction']
        rows.append([s['D'],s['roll'],s['x'],s['y'],s['q1'],s['q2'],s['mu'],
                     c['x'] if c else None,c['y'] if c else None])
    template=Path(__file__).resolve().parents[1]/'support'/'hugmap_arc_rail_template.html'
    data=json.dumps(dict(stats=report['stats'],transition=report.get('band1_cs_transition'),rows=rows),separators=(',',':')).replace('<','\\u003c')
    output.write_text(template.read_text().replace('__DATA__',data))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('source',type=Path);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    raw=args.source.read_bytes();doc=json.loads(raw);args.output.mkdir(parents=True,exist_ok=True);audit=[]
    for m in doc['maps']:
        for row in m['results']:
            for sol in row.get('roll_solutions',[]):
                D=m['diameter_mm'];roll=sol['roll_deg'];q1=seg1_from_center(row['x_mm'],row['y_mm'],D,roll)
                candidates=forward(sol['seg1_deg'],sol['seg2_deg'],D,roll)
                best=min(candidates,key=lambda c:(c['x']-row['x_mm'])**2+(c['y']-row['y_mm'])**2) if candidates else None
                X=row['x_mm']*roll/90;Y=row['y_mm']-119.5;A=D/2+35
                if abs(sol['seg1_deg'])<1e-9:
                    radial_error=abs(X-A)
                else:
                    k=25/math.tan(math.radians(sol['seg1_deg'])/10)
                    radial_error=abs(math.hypot(X+k,Y)-abs(k+A))
                audit.append(dict(x=row['x_mm'],y=row['y_mm'],D=D,roll=roll,q1=sol['seg1_deg'],q2=sol['seg2_deg'],
                    q1_estimate=q1,q1_error_deg=abs(q1-sol['seg1_deg']) if q1 is not None else None,
                    circle_radial_error_mm=radial_error,mu=sol['mu_min'],reconstruction=best,
                    error_mm=math.hypot(best['x']-row['x_mm'],best['y']-row['y_mm']) if best else None))
    stats={}
    for roll in (-90,90):
        a=[r for r in audit if r['roll']==roll];b=[r for r in a if r['reconstruction']]
        stats[str(roll)]=dict(count=len(a),q1_error_deg=summary([r['q1_error_deg'] for r in a if r['q1_error_deg'] is not None]),
            circle_radial_error_mm=summary([r['circle_radial_error_mm'] for r in a]),
            forward_reconstructed=len(b),forward_error_mm=summary([r['error_mm'] for r in b]),
            within_2_5mm=sum(r['error_mm']<=2.5 for r in b),within_5mm=sum(r['error_mm']<=5 for r in b),
            mu_error=summary([abs(r['mu']-r['reconstruction']['mu']) for r in b]),
            kinds=dict(Counter(r['reconstruction']['kind'] for r in b)))
    report=dict(source_sha256=hashlib.sha256(raw).hexdigest(),model='CAD-derived circular envelope + terminal right rail; no fitted parameters',
                limitation='Approximate geometry. Reconstruction uses saved joint angles; it is not independent full-map coverage or a collision/continuity proof.',stats=stats,cases=audit)
    nonzero=[r for r in audit if r['roll']==-90 and abs(r['q1'])>1e-6]
    straight=[r for r in audit if r['roll']==-90 and abs(r['q1'])<=1e-6]
    report['band1_cs_transition']=dict(equation='abs(x_mm) = D_mm/2 + 35',
        nonzero_cases=len(nonzero),mismatches=sum((r['q1']>0)!=(-r['x']<r['D']/2+35) for r in nonzero),
        straight_cases=len(straight),straight_max_line_error_mm=max((abs(-r['x']-r['D']/2-35) for r in straight),default=0))
    (args.output/'audit.json').write_text(json.dumps(report,indent=2))
    write_viewer(report,args.output/'index.html')
    print(json.dumps(stats,indent=2),flush=True)

if __name__=='__main__':main()
