"""Search Home-to-target paths against the fixed cylinder using actual meshes.

This conservative XY convex projection is only a path proposal; a found path
must still pass full Genesis robot/self collision and dynamic replay checks.
Search failure is not an impossibility proof. The training source is untouched.
"""
import argparse
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull


def rx(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])


def ry(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])


class ObjectGeometry:
    def __init__(self,experiment,case):
        from workbench.research.experiments.hug_c.configure import BUNDLE,fk
        root=ET.parse(BUNDLE/'robot.urdf').getroot();self.meshes={}
        for name in [*(f'node{i}' for i in range(10)),'gripper_base','gripper_claw_left','gripper_claw_right','camera']:
            link=root.find(f"link[@name='{name}']");mesh=link.find('visual/geometry/mesh')
            p=BUNDLE/mesh.get('filename')
            vertices=np.array([list(map(float,s.split()[1:4])) for s in p.read_text().splitlines() if s.startswith('v ')])
            vertices=vertices[ConvexHull(vertices).vertices]
            if name.startswith('gripper') or name=='camera':
                relative=np.linalg.inv(fk(root,'node9'))@fk(root,name)
                vertices=vertices@relative[:3,:3].T+relative[:3,3]
            self.meshes[name]=vertices
        self.origin=fk(root,'node0')[:3,3]+[0,0,experiment['settings']['spawn_height_m']]
        self.center=np.asarray(case['object_xyz_m'])[:2];self.radius=case['diameter_mm']/2000
        self.calls=0

    def polygons(self,q):
        linear,roll,q3,q4=q
        rotation=rx(math.radians(roll));p=self.origin+np.array([linear/1000,0,0])
        for i in range(10):
            rotation=rotation@ry(math.radians(q3 if i<5 else q4)/5)
            yield f'node{i}',self.meshes[f'node{i}']@rotation.T+p
            if i<9:p=p+rotation@np.array([.05,0,0])
        for name in ('gripper_base','gripper_claw_left','gripper_claw_right','camera'):
            yield name,self.meshes[name]@rotation.T+p

    def gap(self,q,*,first_only=False):
        self.calls+=1;best=1.;which=None
        for name,vertices in self.polygons(q):
            if first_only and name=='node5':break
            if first_only and name not in {f'node{i}' for i in range(5)}:continue
            xy=vertices[:,:2];lo=xy.min(0);hi=xy.max(0)
            box=np.maximum(np.maximum(lo-self.center,self.center-hi),0)
            lower=np.linalg.norm(box)-self.radius
            if lower>.003:continue
            hull=ConvexHull(xy);p=xy[hull.vertices];ab=np.roll(p,-1,axis=0)-p
            alpha=np.clip(((self.center-p)*ab).sum(1)/(ab*ab).sum(1),0,1)
            distance=np.linalg.norm(p+alpha[:,None]*ab-self.center,axis=1).min()
            if np.max(hull.equations[:,:2]@self.center+hull.equations[:,2])<0:distance=-distance
            gap=float(distance-self.radius)-(0.001 if name=='camera' else 0)
            if gap<best:best,which=gap,name
            if gap<-.00002:return best,which
        return best,which


SCALE=np.array([50.,45.,90.,90.])
LOWER=np.array([-173.333333333, -90.,-180.,-180.])
UPPER=np.array([0.,90.,180.,180.])


def segment(a,b,step=.06):
    count=max(1,int(np.ceil(np.max(abs((b-a)/SCALE))/step)))
    return np.linspace(a,b,count+1)[1:]


def search(geometry,start,goal,*,seconds=90,seed=42):
    rng=np.random.default_rng(seed);began=time.monotonic();trees=[([start],[None]),([goal],[None])]
    cache={}
    def valid(q):
        key=tuple(np.round(q,7))
        if key not in cache:cache[key]=geometry.gap(q)[0]>=-.00002
        return cache[key]
    def connect(tree,target,extend=False):
        nodes,parents=tree;near=int(np.argmin(np.linalg.norm((np.asarray(nodes)-target)/SCALE,axis=1)))
        origin=nodes[near];delta=(target-origin)/SCALE;length=np.linalg.norm(delta)
        end=target if not extend or length<=.45 else origin+SCALE*delta/length*.45
        for q in segment(origin,end):
            if not valid(q):return None
        nodes.append(end);parents.append(near);return len(nodes)-1
    def chain(tree,index):
        out=[]
        while index is not None:out.append(tree[0][index]);index=tree[1][index]
        return out[::-1]
    if not valid(start) or not valid(goal):
        return dict(found=False,reason='endpoint rejected by conservative object projection',start_gap=geometry.gap(start),goal_gap=geometry.gap(goal))
    if connect(trees[0],goal) is not None:return dict(found=True,waypoints=[start.tolist(),goal.tolist()],iterations=0)
    for iteration in range(100000):
        if time.monotonic()-began>seconds:break
        a=iteration%2;b=1-a
        target=rng.uniform(LOWER,UPPER)
        if rng.random()<.15:target=trees[b][0][0]
        new=connect(trees[a],target,extend=True)
        if new is None:continue
        end=trees[a][0][new];other=connect(trees[b],end)
        if other is not None:
            left=chain(trees[a],new);right=chain(trees[b],other)
            path=left+right[-2::-1]
            if a: path=path[::-1]
            for _ in range(150):
                i,j=sorted(rng.choice(len(path),2,replace=False))
                if j>i+1 and all(valid(q) for q in segment(path[i],path[j])):path=path[:i+1]+path[j:]
            return dict(found=True,waypoints=[q.tolist() for q in path],iterations=iteration+1,elapsed_s=time.monotonic()-began,geometry_calls=geometry.calls)
    return dict(found=False,reason='finite RRT search exhausted; not an impossibility proof',elapsed_s=time.monotonic()-began,geometry_calls=geometry.calls,tree_sizes=[len(t[0]) for t in trees])


def first_segment_components(geometry,start,goal,*,fine=False):
    """Necessary-condition grid: q4 cannot move seg1 out of this obstacle.

    Disconnection at finite resolution is evidence, never a continuous proof.
    Connectivity also needs edge/whole-robot and dynamic verification.
    """
    from scipy.ndimage import label
    linear=np.unique(np.r_[-173.333333333,np.arange(-170.,1,5 if fine else 10),start[0],goal[0]])
    roll=np.unique(np.r_[np.arange(-90.,91,5 if fine else 10),-89.75,-89.5,-89.,-88.,88.,89.,89.5,89.75,start[1],goal[1]])
    q3=np.unique(np.r_[np.arange(-180.,181,2.5 if fine else 5),-2.,-1.,-.5,-.25,.25,.5,1.,2.,start[2],goal[2]])
    free=np.zeros((len(linear),len(roll),len(q3)),bool);began=time.monotonic()
    for i,l in enumerate(linear):
        for j,r in enumerate(roll):
            for k,q in enumerate(q3):free[i,j,k]=geometry.gap([l,r,q,0],first_only=True)[0]>=-.00002
    labels,n=label(free)
    def index(v):return tuple(int(np.argmin(abs(axis-value))) for axis,value in zip((linear,roll,q3),v[:3]))
    a=index(start);b=index(goal);sa=int(labels[a]);sb=int(labels[b])
    def describe(c):
        points=np.argwhere(labels==c) if c else np.empty((0,3),int)
        return dict(id=c,cells=len(points),q3_range_deg=[float(q3[points[:,2].min()]),float(q3[points[:,2].max()])] if len(points) else None)
    return dict(grid_shape=list(free.shape),total_cells=int(free.size),free_cells=int(free.sum()),
        components=int(n),home_component=describe(sa),goal_component=describe(sb),
        connected=sa>0 and sa==sb,elapsed_s=time.monotonic()-began,
        fixed_object=True,geometry='only seg1 convex XY projections; ignore seg2 and robot body',
        continuous_impossibility_proven=False,fine=fine)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--experiment',type=Path,required=True)
    p.add_argument('--case',required=True);p.add_argument('--seconds',type=float,default=90.)
    p.add_argument('--first-segment-grid',action='store_true')
    p.add_argument('--output',type=Path,required=True);a=p.parse_args(argv)
    exp=json.loads(a.experiment.read_text());case=next(c for c in exp['cases'] if c['case_id']==a.case)
    def convert(command):return np.r_[command[0]*1000,np.rad2deg(command[1]),np.rad2deg(command[2:])*5]
    start=convert(case['start_command']);goal=convert(case['goal_command']);geom=ObjectGeometry(exp,case)
    direct=[]
    for q in np.vstack((start,segment(start,goal,step=.02))):
        gap,link=geom.gap(q)
        if gap<-.00002:direct.append(dict(q=q.tolist(),penetration_lower_bound_mm=-gap*1000,link=link))
    result=search(geom,start,goal,seconds=a.seconds)
    if a.first_segment_grid:result['first_segment_grid']=first_segment_components(geom,start,goal)
    result.update(schema='elesim-c-hug-home-approach-search',schema_version=1,case=case,
        command_units=['linear_mm','roll_deg','q3_total_deg','q4_total_deg'],direct_interpolation_collisions=direct,
        physics_validated=False,includes_self_body_collision=False,training_modified=False)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as stream:stream.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('case','direct_interpolation_collisions')},indent=2))


if __name__=='__main__':main()
