"""CPU self-collision certificates using the shipped conservative sphere model.

No actuator or CUDA I/O. Directly connected/proven-separated pair exclusions
come only from the digest-checked planning artifact. Segment certificates use
joint-motion Lipschitz bounds, rather than assuming sampled safe endpoints
make the path safe. This covers the modeled arm/open claws, not external objects.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


class ArmSelfCollision:
    def __init__(self, model_dir, *, margin_m=.002, joint_uncertainty=None):
        root = Path(model_dir)
        meta = json.loads((root/'metadata.json').read_text())
        if meta.get('schema_version') != 1:
            raise ValueError('unsupported collision model')
        for name in ('arm.urdf', 'robot.yml'):
            if hashlib.sha256((root/name).read_bytes()).hexdigest() != meta.get('artifacts', {}).get(name):
                raise ValueError('collision model digest mismatch: '+name)
        self.model_dir = root
        self.source_hashes = meta['sources']
        self.bounds = np.asarray(meta['q_bounds'],float)
        if self.bounds.shape != (4,2) or not np.isfinite(self.bounds).all() or np.any(self.bounds[:,0]>=self.bounds[:,1]):
            raise ValueError('invalid collision joint bounds')
        self.margin_m = float(margin_m)
        self.uncertainty = np.zeros(4) if joint_uncertainty is None else np.asarray(joint_uncertainty,float)
        if not 0 <= self.margin_m <= .02 or self.uncertainty.shape != (4,) or not np.isfinite(self.uncertainty).all() or np.any(self.uncertainty < 0):
            raise ValueError('invalid collision uncertainty/margin')
        kin = yaml.safe_load((root/'robot.yml').read_text())['robot_cfg']['kinematics']
        self.base = kin['base_link']
        self.names = list(kin['collision_link_names'])
        self.local = {n:np.array([s['center'] for s in kin['collision_spheres'][n]],float) for n in self.names}
        self.radii = {n:np.array([s['radius'] for s in kin['collision_spheres'][n]],float) for n in self.names}
        self.sphere_count = sum(map(len,self.local.values()))
        if self.sphere_count != meta['sphere_count'] or not 0 < self.sphere_count < 1024:
            raise ValueError('invalid collision sphere budget')
        for n in self.names:
            if self.local[n].shape != (len(self.radii[n]),3) or not np.isfinite(self.local[n]).all() or not np.isfinite(self.radii[n]).all() or np.any(self.radii[n]<=0):
                raise ValueError('invalid collision spheres')
        ignored = {frozenset((a,b)) for a,bs in kin['self_collision_ignore'].items() for b in bs}
        self.pairs = [(a,b) for i,a in enumerate(self.names) for b in self.names[i+1:] if frozenset((a,b)) not in ignored]
        active = {n:i for i,n in enumerate(meta['joint_names'])}
        self.joints = []
        ancestry = {self.base:[]}
        for j in ET.parse(root/'arm.urdf').getroot().findall('joint'):
            parent,child = j.find('parent').get('link'),j.find('child').get('link')
            origin = j.find('origin')
            xyz = np.fromstring(origin.get('xyz','0 0 0') if origin is not None else '0 0 0',sep=' ')
            rpy = np.fromstring(origin.get('rpy','0 0 0') if origin is not None else '0 0 0',sep=' ')
            t = np.eye(4);t[:3,:3] = Rotation.from_euler('xyz',rpy).as_matrix();t[:3,3] = xyz
            axis_node=j.find('axis');axis=np.fromstring(axis_node.get('xyz','1 0 0') if axis_node is not None else '1 0 0',sep=' ')
            mimic=j.find('mimic');name=j.get('name');kind=j.get('type')
            master=name if mimic is None else mimic.get('joint')
            index=None if kind=='fixed' else active[master]
            multiplier=1. if mimic is None else float(mimic.get('multiplier','1'))
            offset=0. if mimic is None else float(mimic.get('offset','0'))
            self.joints.append(dict(parent=parent,child=child,kind=kind,t=t,axis=axis,index=index,multiplier=multiplier,offset=offset))
            ancestry[child]=ancestry[parent]+[len(self.joints)-1]
        self.ancestry=ancestry
        self.local_center={n:self.local[n].mean(0) for n in self.names}
        self.local_bound={n:float(np.max(np.linalg.norm(self.local[n]-self.local_center[n],axis=1)+self.radii[n])) for n in self.names}
        # A rotation's point-speed bound is the total downstream translation
        # length plus the link-local sphere reach. Common ancestor motions
        # cancel from pair distances and are omitted from that pair's bound.
        self.pair_lipschitz=np.zeros((len(self.pairs),4))
        for pair_index,(a,b) in enumerate(self.pairs):
            common=set(ancestry[a]) & set(ancestry[b])
            for name in (a,b):
                chain=ancestry[name]
                reach=float(np.max(np.linalg.norm(self.local[name],axis=1)))
                for joint_index in reversed(chain):
                    joint=self.joints[joint_index];axis_index=joint['index']
                    if axis_index is not None and joint_index not in common:
                        amount=1. if joint['kind']=='prismatic' else reach
                        self.pair_lipschitz[pair_index,axis_index]+=abs(joint['multiplier'])*amount
                    reach+=float(np.linalg.norm(joint['t'][:3,3]))
                    if joint['kind']=='prismatic':reach+=float(np.max(np.abs(self.bounds[axis_index])))
        self.required_gap = self.margin_m+self.pair_lipschitz@self.uncertainty
        self._cache={}

    def _q(self,q):
        q=np.asarray(q,float)
        if q.shape!=(4,) or not np.isfinite(q).all() or np.any(q<self.bounds[:,0]-1e-9) or np.any(q>self.bounds[:,1]+1e-9):
            raise ValueError('invalid collision joint configuration')
        return q

    def link_poses(self,q):
        q=self._q(q);poses={self.base:np.eye(4)}
        for j in self.joints:
            t=j['t'].copy()
            if j['index'] is not None:
                value=q[j['index']]*j['multiplier']+j['offset']
                if j['kind']=='revolute':t[:3,:3]=t[:3,:3]@Rotation.from_rotvec(j['axis']*value).as_matrix()
                elif j['kind']=='prismatic':t[:3,3]+=t[:3,:3]@j['axis']*value
                else:raise ValueError('unsupported collision joint')
            poses[j['child']]=poses[j['parent']]@t
        return poses

    def pair_clearances(self,q):
        q=self._q(q);key=tuple(q)
        if key in self._cache:return self._cache[key].copy()
        poses=self.link_poses(q)
        centers={n:self.local[n]@poses[n][:3,:3].T+poses[n][:3,3] for n in self.names}
        bounds={n:self.local_center[n]@poses[n][:3,:3].T+poses[n][:3,3] for n in self.names}
        result=np.empty(len(self.pairs))
        for i,(a,b) in enumerate(self.pairs):
            lower=np.linalg.norm(bounds[a]-bounds[b])-self.local_bound[a]-self.local_bound[b]
            if lower > self.required_gap[i]+.03:
                result[i]=lower
                continue
            delta=centers[a][:,None,:]-centers[b][None,:,:]
            result[i]=np.min(np.sqrt(np.sum(delta**2,axis=-1))-self.radii[a][:,None]-self.radii[b][None,:])
        if len(self._cache)>=20_000:self._cache.clear()
        self._cache[key]=result.copy()
        return result

    def configuration_free(self,q):
        return bool(np.all(self.pair_clearances(q)>self.required_gap))

    def certify_segment(self,start,goal,*,max_checks=4096):
        """Certify the continuous straight joint segment, including uncertainty.

        At every parameter t, one of the checked segment endpoints is at
        distance <= half the interval. Lipschitz sphere separation bounds
        cover that unsampled motion. An unresolved interval is rejected.
        """
        start,goal=self._q(start),self._q(goal)
        ca,cb=self.pair_clearances(start)-self.required_gap,self.pair_clearances(goal)-self.required_gap
        checks=1 if np.array_equal(start,goal) else 2
        if np.any(ca<=0) or np.any(cb<=0):
            return {'accepted':False,'reason':'self_collision_or_margin','checked_configurations':checks,'minimum_margin_m':float(min(ca.min(),cb.min()))}
        pending=[(start,goal,ca,cb)];certified=float('inf')
        while pending:
            a,b,left,right=pending.pop()
            movement=self.pair_lipschitz@np.abs(b-a)
            bound=np.minimum(left,right)-movement*.5
            if np.all(bound>0):
                certified=min(certified,float(bound.min()));continue
            if checks>=max_checks:
                return {'accepted':False,'reason':'collision_certificate_budget','checked_configurations':checks,'minimum_margin_m':0.}
            middle=(a+b)*.5;cm=self.pair_clearances(middle)-self.required_gap;checks+=1
            if np.any(cm<=0):
                return {'accepted':False,'reason':'self_collision_on_path','checked_configurations':checks,'minimum_margin_m':float(cm.min())}
            pending.extend(((a,middle,left,cm),(middle,b,cm,right)))
        return {'accepted':True,'reason':'continuous_sphere_separation','checked_configurations':checks,
                'minimum_margin_m':certified}
