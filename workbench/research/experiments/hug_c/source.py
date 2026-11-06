"""Use the mapper's actual analytic/CAD JavaScript kernel, without its DATA rows."""
import hashlib
import json
from pathlib import Path
import subprocess
import math


def read_kernel(path):
    html = Path(path).read_text()
    try:
        model = html[html.index('const rot='):html.index('let current=')]
        collision = html[html.index('const CONTACT_OVERLAP_TOL_MM='):html.index('function muLimit()')]
    except ValueError as exc:
        raise ValueError('Unsupported mapper.html kernel layout') from exc
    kernel = model + collision
    if 'const DATA=' in kernel or 'DATA.rows' in kernel:
        raise ValueError('Embedded point data must not enter function sampling')
    return kernel, dict(kind='mapper-function', path=str(Path(path).resolve()),
                       html_sha256=hashlib.sha256(html.encode()).hexdigest(),
                       kernel_sha256=hashlib.sha256(kernel.encode()).hexdigest(),
                       point_data_used=False, collision_tolerance_mm=1.)


def evaluate(kernel, diameter, q_points=None, *, step=1., mu_max=.4):
    """Return only C targets in the same XY quadrant, with mapper CAD checks."""
    payload = dict(diameter=float(diameter), points=q_points, step=float(step), mu=float(mu_max))
    code = r'''
const rows=[],D=INPUT.diameter,roll=-90;
function evaluate(q3,q4){
 if(!(q3<0&&q4<0&&q3>=-180&&q4>=-180))return null;
 const m=model(q3,q4,D,roll);let polys=null,bad=false,best=null;
 for(const c of m.candidates){
  if(!Number.isFinite(c.mu)||c.mu>INPUT.mu||c.x>0||c.y<0||Math.hypot(c.x,c.y)>500)continue;
  if(!polys){polys=collisionGeometry(q3,q4);bad=selfCollision(polys);}
  if(bad||objectCollision(polys,c,D,roll))continue;
  if(!best||c.mu<best.mu_min)best={q_deg:[q3,q4],xy_mm:[c.x,c.y],mu_min:c.mu,feature:c.kind,root:c.branch};
 }
 return best;
}
if(INPUT.points){for(const p of INPUT.points)rows.push(evaluate(p[0],p[1]));}
else{for(let i=0;i<Math.ceil(180/INPUT.step);i++)for(let j=0;j<Math.ceil(180/INPUT.step);j++){
 const row=evaluate(-180+(i+.5)*INPUT.step,-180+(j+.5)*INPUT.step);if(row)rows.push(row);
}}
console.log(JSON.stringify(rows));
'''
    try:
        result = subprocess.run(['node'], input='const INPUT='+json.dumps(payload)+';\n'+kernel+code,
                                text=True, capture_output=True, check=True, timeout=180)
    except FileNotFoundError as exc:
        raise RuntimeError('Node.js is required only to prepare cases from mapper.html') from exc
    return json.loads(result.stdout)


def inverse(kernel, diameter, xy, seeds, *, mu_max=.4):
    """Solve exact XY using sampled feature/root seeds, then recheck forward/CAD.

    Seeds select analytic branches, not an interpolated XY-to-q approximation.
    A missing solution means this local search failed; it is not an impossibility
    proof. Stored mapper DATA never enters this calculation.
    """
    if len(xy)!=2 or not all(math.isfinite(v) for v in (*xy,diameter,mu_max)):
        raise ValueError('Require finite XY, diameter and friction cutoff')
    if diameter<=0 or mu_max<=0 or xy[0]>0 or xy[1]<0 or math.hypot(*xy)>500:
        return []
    payload=dict(diameter=diameter,xy=list(xy),mu=mu_max,
                 seeds=[dict(q4=r['q_deg'][1],kind=r['feature'],branch=r['root']) for r in seeds])
    code=r'''
const out=[],[x,y]=INPUT.xy,D=INPUT.diameter,roll=-90;
for(const seed of INPUT.seeds){
 const hit=inverseContactPose(x,y,D,roll,seed);
 if(!hit||!(hit.q3<0&&hit.q4<0&&hit.q3>=-180&&hit.q4>=-180))continue;
 const m=model(hit.q3,hit.q4,D,roll);
 const c=m.candidates.find(c=>c.kind===seed.kind&&c.branch===seed.branch&&
   Number.isFinite(c.mu)&&c.mu<=INPUT.mu&&Math.hypot(c.x-x,c.y-y)<1e-4);
 if(!c)continue;
 const polys=collisionGeometry(hit.q3,hit.q4);
 if(selfCollision(polys)||objectCollision(polys,c,D,roll))continue;
 if(out.some(r=>Math.hypot(r.q_deg[0]-hit.q3,r.q_deg[1]-hit.q4)<1e-7))continue;
 out.push({q_deg:[hit.q3,hit.q4],xy_mm:[c.x,c.y],mu_min:c.mu,feature:c.kind,root:c.branch});
}
console.log(JSON.stringify(out));
'''
    try:
        result=subprocess.run(['node'],input='const INPUT='+json.dumps(payload)+';\n'+kernel+code,
                              text=True,capture_output=True,check=True,timeout=30)
    except FileNotFoundError as exc:
        raise RuntimeError('Node.js is required for exact mapper-function XY inversion') from exc
    return json.loads(result.stdout)
