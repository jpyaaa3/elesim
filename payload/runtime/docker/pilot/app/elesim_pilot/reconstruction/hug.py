"""Reconstructed upright cylinder -> exact mapper inversion -> timed C-hug.

Only simulation is admitted. This is open-loop trajectory interpolation, not
an averaged neural policy or a claim of measured grasp success.
"""
from pathlib import Path
import hashlib
import json
import subprocess
import numpy as np
from elesim_protocol.scene_artifact import SceneArtifact


def fit_cylinder(vertices):
    p=np.asarray(vertices,float)
    if p.ndim!=2 or p.shape[1]!=3 or not 24<=len(p)<=100000 or not np.isfinite(p).all():
        raise ValueError('Cylinder fitting needs finite reconstructed vertices')
    low,high=np.min(p[:,2]),np.max(p[:,2])
    if not 1.07<=high-low<=1.13 or abs(low)>.015:
        raise ValueError('This model requires an upright 1.1 m cylinder standing on world Z=0')
    # Ignore cap interiors; fit the side surface, never the mock object telemetry.
    side=p[(p[:,2]>low+.01)&(p[:,2]<high-.01)]
    if len(side)<24:
        # OBJ cylinders may have only rim vertices and two cap-centre vertices.
        midpoint=(p[:,:2].min(0)+p[:,:2].max(0))/2
        side=p[np.linalg.norm(p[:,:2]-midpoint,axis=1)>.01]
    if len(side)<24:raise ValueError('Insufficient cylinder side vertices')
    xy=side[:,:2];a=np.column_stack((2*xy,np.ones(len(xy))))
    if np.linalg.cond(a)>1e5:raise ValueError('Insufficient cylinder surface coverage')
    cx,cy,k=np.linalg.lstsq(a,np.sum(xy*xy,axis=1),rcond=None)[0]
    radius=np.sqrt(max(0,k+cx*cx+cy*cy));radial=np.linalg.norm(xy-[cx,cy],axis=1)
    residual=float(np.quantile(abs(radial-radius),.95))
    if residual>.002 or abs(radius-.05)>.001:
        raise ValueError('Reconstruction is not a supported 100 mm cylinder (2 mm residual / diameter tolerance)')
    angles=np.sort(np.arctan2(xy[:,1]-cy,xy[:,0]-cx));gaps=np.diff(np.r_[angles,angles[0]+2*np.pi])
    if 2*np.pi-max(gaps)<np.pi/2:raise ValueError('Cylinder fit needs at least 90 degrees of observed surface')
    return dict(center_world_m=[float(cx),float(cy),float((low+high)/2)],
                diameter_mm=float(radius*2000),height_m=float(high-low),residual_m=residual)


def extract_target(manifest,target_id,expected_revision):
    artifact=SceneArtifact.load(manifest)
    if artifact.revision!=expected_revision:raise ValueError('Reconstruction changed; scan again')
    items=[o for o in artifact.objects if o.object_id==target_id and o.role=='target']
    if len(items)!=1:raise ValueError('Select the reconstructed target object')
    vertices=[list(map(float,line.split()[1:])) for line in items[0].mesh_file.read_text().splitlines() if line.startswith('v ')]
    return fit_cylinder(vertices)


def barycentric(vertices,target):
    v=np.asarray(vertices,float)
    return np.linalg.solve(np.vstack((v.T,np.ones(3))),np.r_[target,1.])


class HugModel:
    def __init__(self,root):
        self.root=Path(root);self.meta=json.loads((self.root/'manifest.json').read_text())
        m=self.meta
        if m.get('schema')!='elesim-hug-interpolation' or m.get('version')!=1 or m.get('scope')!='sim-experimental':
            raise ValueError('Unsupported hug model')
        for name in ('anchors.npz','kernel.js'):
            if hashlib.sha256((self.root/name).read_bytes()).hexdigest()!=m['files'][name]:
                raise ValueError('Hug model checksum mismatch')
        with np.load(self.root/'anchors.npz',allow_pickle=False) as d:self.paths=d['commands'].copy()
        if self.paths.ndim!=3 or self.paths.shape[0]!=3 or self.paths.shape[2]!=4 or not np.isfinite(self.paths).all():
            raise ValueError('Invalid anchor paths')
        if not np.allclose(self.paths[:,0],m['home'],atol=1e-8):raise ValueError('Anchors do not start at Home')
        if (np.any(self.paths<np.asarray(m['lower'])-1e-7) or np.any(self.paths>np.asarray(m['upper'])+1e-7)
                or np.any(abs(np.diff(self.paths,axis=1))>np.asarray(m['max_step'])+1e-6)):
            raise ValueError('Anchor path exceeds position / slew limits')
        if np.any(abs(np.diff(self.paths[:,:,0],axis=1)).sum(1)*1500>200+1e-5):
            raise ValueError('Anchor exceeds linear travel budget')

    def validate_robot(self, planning_urdf):
        if hashlib.sha256(Path(planning_urdf).read_bytes()).hexdigest()!=self.meta['planning_urdf_sha256']:
            raise ValueError('Hug model does not match the configured arm model')

    def resolve(self,xy):
        m=self.meta
        if len(xy)!=2 or not np.isfinite(xy).all():raise ValueError('Invalid solver coordinates')
        data=dict(xy=list(map(float,xy)),diameter=m['diameter_mm'],mu=m['mu_max'],seeds=[dict(q4=c['q_deg'][1],kind=c['feature'],branch=c['root']) for c in m['anchors']])
        code='const INPUT='+json.dumps(data)+';\n'+(self.root/'kernel.js').read_text()+r'''
const out=[]; const [x,y]=INPUT.xy,D=INPUT.diameter,roll=-90;
for(const seed of INPUT.seeds){
 const h=inverseContactPose(x,y,D,roll,seed);if(!h||h.q3>=0||h.q4>=0)continue;
 const c=h.c,p=collisionGeometry(h.q3,h.q4);
 if(c.mu>INPUT.mu||selfCollision(p)||objectCollision(p,c,D,roll)||preparationReport(c,D).blocked)continue;
 out.push({q_deg:[h.q3,h.q4],mu_min:c.mu});
}console.log(JSON.stringify(out));
'''
        try:r=subprocess.run(['node','--max-old-space-size=128'],input=code,text=True,capture_output=True,check=True,timeout=4)
        except FileNotFoundError as exc:raise RuntimeError('Pilot needs Node.js for the bundled exact hug solver') from exc
        vertices=[c['q_deg'] for c in m['anchors']];valid=[]
        for row in json.loads(r.stdout):
            w=barycentric(vertices,row['q_deg'])
            if min(w)>=-1e-9 and max(w)<=1+1e-9:valid.append((row,w))
        if not valid:raise ValueError('Reconstructed cylinder lies outside the supported C-hug triangle')
        row,w=min(valid,key=lambda pair:pair[0]['mu_min']);w=np.maximum(w,0);w/=w.sum()
        commands=np.tensordot(w,self.paths,axes=1)
        ticks=round(m['episode_s']/m['dt_s'])+1
        if len(commands)>ticks:raise ValueError('Anchor path exceeds episode budget')
        commands=np.concatenate((commands,np.repeat(commands[-1:],ticks-len(commands),axis=0)))
        return commands,dict(row,xy_mm=list(map(float,xy)),weights=w.tolist(),
            duration_s=m['episode_s'],scope=m['scope'],physical_success_verified=False)
