"""Research-only bounded sphere fit; records coverage, never publishes runtime assets."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def fit_inner_shell(mesh, budget):
    """Inward-tangent candidates only; no volume voxelization or interior fill."""
    import trimesh
    # Open source meshes use oriented-face signed distance as an estimate.
    # Metadata records this limitation; the output is visualization-only.
    points, faces = trimesh.sample.sample_surface(mesh, max(4000, budget*30))
    normals = mesh.face_normals[faces]
    # Scale to the part, with local thickness handled by repeated inward refit.
    target = float(np.clip(np.min(mesh.extents)*.18, .001, .02))
    radii = np.full(len(points), target)
    accepted = np.zeros(len(points), bool)
    centers = points.copy()
    for _ in range(7):
        ids = np.flatnonzero(~accepted)
        if not len(ids): break
        c = points[ids]-normals[ids]*radii[ids,None]
        distance = trimesh.proximity.signed_distance(mesh,c)
        good = (distance >= radii[ids]-1e-7) & (distance <= radii[ids]+5e-7)
        centers[ids[good]] = c[good]
        accepted[ids[good]] = True
        radii[ids[~good]] *= .5
    ids = np.flatnonzero(accepted & (radii >= .00025))
    if not len(ids): raise ValueError('No valid inward shell spheres')
    centers, radii = centers[ids], radii[ids] - 1e-6
    # Spend the sphere budget on previously uncovered surface regions.
    chosen=[]; gap=np.full(len(points),np.inf)
    remaining=np.ones(len(centers),bool)
    for _ in range(min(budget,len(centers))):
        if not chosen: index=int(np.argmax(radii))
        else:
            far=points[int(np.argmax(gap))]
            score=np.linalg.norm(centers-far,axis=1)-radii
            score[~remaining]=np.inf
            index=int(np.argmin(score))
        chosen.append(index);remaining[index]=False
        gap=np.minimum(gap,np.linalg.norm(points-centers[index],axis=1)-radii[index])
    centers,radii=centers[chosen],radii[chosen]
    distance=trimesh.proximity.signed_distance(mesh,centers)
    if np.any(distance < radii-1e-7): raise ValueError('Sphere protrudes outside mesh')
    if np.any(distance-radii > 2e-6): raise ValueError('Deep interior sphere rejected')
    return centers,radii


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--model',type=Path,default=Path('payload/data/models/planning/zed-mini'))
    p.add_argument('--assembly',type=Path,default=Path('payload/data/models/assemblies/zed-mini'))
    p.add_argument('--exclude-plate',action='store_true')
    p.add_argument('--allocation-audit',type=Path)
    args=p.parse_args()
    import trimesh
    np.random.seed(42)
    raw=yaml.safe_load((args.model/'robot.yml').read_text());cfg=raw['robot_cfg']['kinematics']
    tree=ET.parse(args.assembly/'arm.urdf').getroot(); audit={}; fitted={}
    budgets={n:len(v) for n,v in cfg['collision_spheres'].items()}
    removed=budgets.pop('plate',0) if args.exclude_plate else 0
    if removed:
        previous=json.loads(args.allocation_audit.read_text())['coverage_audit'] if args.allocation_audit else {}
        names=list(budgets)
        weights=np.array([max(previous.get(n,{}).get('max_sampled_surface_gap_mm',1.),.01) for n in names])
        shares=removed*weights/weights.sum();extra=np.floor(shares).astype(int)
        for i in np.argsort(-(shares-extra))[:removed-int(extra.sum())]:extra[i]+=1
        for n,v in zip(names,extra):budgets[n]+=int(v)
    for name,old in cfg['collision_spheres'].items():
        if name not in budgets: continue
        pieces=[]
        for collision in tree.find(f"link[@name='{name}']").findall('collision'):
            m=collision.find('geometry/mesh'); shape=trimesh.load_mesh(args.assembly/m.get('filename'),process=True)
            shape.apply_scale(np.fromstring(m.get('scale','1 1 1'),sep=' '))
            o=collision.find('origin');t=np.eye(4)
            if o is not None:
                t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
                t[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ')
            shape.apply_transform(t);pieces.append(shape)
        mesh=trimesh.util.concatenate(pieces)
        centers,radii=fit_inner_shell(mesh,budgets[name])
        if centers is None or len(centers)==0:
            audit[name]=dict(spheres=0, status='no_internal_spheres')
            fitted[name]=[]
            continue
        centers=np.asarray(centers);radii=np.asarray(radii)
        samples=np.vstack((mesh.vertices,trimesh.sample.sample_surface(mesh,10000)[0]))
        gap=np.concatenate([np.min(np.linalg.norm(batch[:,None,:]-centers[None,:,:],axis=-1)-radii,axis=1)
                            for batch in np.array_split(samples,32)])
        audit[name]=dict(spheres=len(radii),max_sampled_surface_gap_mm=float(gap.max()*1000),
                         sampled_coverage_fraction=float(np.mean(gap<=0)),inside_only=True,shell_only=True,min_radius_mm=float(radii.min()*1000),max_radius_mm=float(radii.max()*1000),watertight=bool(mesh.is_watertight))
        fitted[name]=[dict(center=c.tolist(),radius=float(r)) for c,r in zip(centers,radii)]
    args.output.mkdir(parents=True,exist_ok=False)
    cfg['collision_spheres']=fitted
    cfg['collision_link_names']=[n for n in cfg['collision_link_names'] if n in fitted]
    cfg['self_collision_ignore']={n:[x for x in v if x in fitted] for n,v in cfg['self_collision_ignore'].items() if n in fitted}
    cfg['self_collision_buffer']={n:v for n,v in cfg['self_collision_buffer'].items() if n in fitted}
    shutil.copyfile(args.model/'arm.urdf',args.output/'arm.urdf')
    (args.output/'robot.yml').write_text(yaml.safe_dump(raw,sort_keys=False))
    meta=json.loads((args.model/'metadata.json').read_text());meta.update(sphere_count=sum(len(v) for v in fitted.values()),
        research_only=True, excluded_collision_links=['plate'] if args.exclude_plate else [], sphere_budgets=budgets, redistributed_plate_spheres=removed, fit='inward tangent shell; part-scaled and local-thickness-adaptive radii; no interior fill',coverage_audit=audit,
        collision_coverage_certified=False, containment_certified=False, visualization_only=True, missing_links=[n for n,v in fitted.items() if not v])
    meta['artifacts']={n:hashlib.sha256((args.output/n).read_bytes()).hexdigest() for n in ('arm.urdf','robot.yml')}
    (args.output/'metadata.json').write_text(json.dumps(meta,indent=2))
    print(json.dumps(audit,indent=2))


if __name__=='__main__':main()
