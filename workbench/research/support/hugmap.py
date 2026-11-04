#!/usr/bin/env python3
"""Hugmap: standalone fixed-placement planar grasp map, Python 3.10+.

Install external libraries: python -m pip install numpy
Run GUI: python hugmap.py (Tkinter required; Ubuntu package python3-tk)
Build table: python hugmap.py --table --output table.json
Resume table: python hugmap.py --resume table.json
Run CLI: python hugmap.py --run --output map.json --dx-mm 20 --dy-mm 20 --mu 0.5

No repository imports or model/data files are required. Geometry and collision
primitives are copied from newhug 0.6.1. Object: circle, default diameter table 50..250 mm in 5 mm steps.
Fixed body origin: center of the embedded 130 x 233 mm HOUSING rectangle, a
planar body proxy, NOT the actual Go2 base/legs. Linear joint fixed at neutral.
Object centers cover the left-forward 500 mm quadrant (x<=0, y>=0), roll +90 deg.
For circles, reflect x and use roll -90 deg with unchanged seg1/seg2 and mu.
Object poses keep (x,y,theta); circle evaluates theta=0 only. Noncircular solving
is not yet implemented.
seg1_deg/seg2_deg are TOTAL five-node group bends, each node receives 1/5.
This is static planar contact search, not a motion or physical grasp guarantee.
"""
from __future__ import annotations
import argparse
from concurrent.futures import ProcessPoolExecutor, wait, FIRST_COMPLETED
from collections import Counter
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
import itertools
import json
import math
import multiprocessing
import os
from pathlib import Path
import queue
import threading
import tempfile
import time
try:
    import numpy as np
except ImportError as exc:
    raise SystemExit("Hugmap requires NumPy: python -m pip install numpy\n" + str(exc))

REFERENCE={'plane': 'Top-down after fixed arm roll +90 deg: plot (x,y) = (-world Y, world X), relative to node0; '
          'height omitted.',
 'reference_plane': 'Arm CAD profiles/FK goldens were extracted at roll=0 as (local Z, local X), then '
                    'reinterpreted after the fixed +90 deg arm roll.',
 'fixed_arm_roll_deg': 90.0,
 'excluded': ['go2', 'plate'],
 'gripper_model': {'kind': 'URDF-full-gripper-top-view',
                   'source': 'payload/data/models/assemblies/zed-mini/arm.urdf',
                   'included': ['gripper_base', 'gripper_claw_left', 'gripper_claw_right'],
                   'outline_m': [[0.0355, 0.0335],
                                 [0.03546, 0.0335],
                                 [0.03546, 0.089],
                                 [-0.01454, 0.089],
                                 [-0.01454, 0.0335],
                                 [-0.025, 0.0335],
                                 [-0.025, -0.0],
                                 [0.0355, -0.0]],
                   'forward_tip_m': [0.01046, 0.089],
                   'frozen_pose': 'URDF gripper zero opening; opening translates the jaws out of this '
                                  'top-view plane.',
                   'projection_method': 'URDF joint origin/rpy and mesh visual origin; base uses an outer '
                                        'rectangle, jaws use convex mesh projections.',
                   'cad_reference': '062222.stp inspected separately; runtime mounting follows the ZED Mini '
                                    'URDF.'},
 'body_model': {'kind': 'housing-footprint-rectangle',
                'width_m': 0.13,
                'depth_m': 0.233,
                'source_axes': 'housing CAD -Y (plot right), X (plot forward)',
                'mount_offset_m': 0.028,
                'note': 'Housing footprint rectangle plus the rolled CAD wedge connecting it to node0; omit '
                        'plate.'},
 'profiles': {'node': [[0.0, 0.05],
                       [-0.035, 0.038627811],
                       [-0.035, 0.011372189],
                       [0.0, 0.0],
                       [0.035, 0.011372189],
                       [0.035, 0.038627811]],
              'node_end': [[0.035, 0.05],
                           [-0.035, 0.05],
                           [-0.035, 0.011372189],
                           [0.0, 0.0],
                           [0.035, 0.011372189]],
              'gripper_base': [[0.0355, 0.0335], [-0.025, 0.0335], [-0.025, -0.0], [0.0355, -0.0]],
              'wedge': [[0.042, -0.013647],
                        [0.0, 0.0],
                        [-0.042, -0.013647],
                        [-0.042, -0.028],
                        [0.042, -0.028]],
              'housing': [[-0.065, -0.261], [0.065, -0.261], [0.065, -0.028], [-0.065, -0.028]],
              'gripper_claw_left': [[-0.01454, 0.089],
                                    [-0.01454, 0.029],
                                    [-0.00954, 0.021],
                                    [-0.00354, 0.019],
                                    [0.02446, 0.019],
                                    [0.03046, 0.021],
                                    [0.03546, 0.029],
                                    [0.03546, 0.089]],
              'gripper_claw_right': [[-0.01454, 0.089],
                                     [-0.01454, 0.029],
                                     [-0.00954, 0.021],
                                     [-0.00354, 0.019],
                                     [0.02446, 0.019],
                                     [0.03046, 0.021],
                                     [0.03546, 0.029],
                                     [0.03546, 0.089]],
              'camera': [[-0.055386236, 0.018031],
                         [-0.055386236, 0.005321],
                         [-0.055378714, 0.005254244],
                         [-0.055356526, 0.005190835],
                         [-0.055320786, 0.005133953],
                         [-0.055273283, 0.005086451],
                         [-0.055216401, 0.005050709],
                         [-0.055152992, 0.005028522],
                         [-0.055086236, 0.005021],
                         [-0.029062001, 0.005021],
                         [-0.028995244, 0.005028522],
                         [-0.028931835, 0.005050709],
                         [-0.028874953, 0.005086451],
                         [-0.028827451, 0.005133953],
                         [-0.028791709, 0.005190835],
                         [-0.028769522, 0.005254244],
                         [-0.028762, 0.005321],
                         [-0.028762, 0.018031],
                         [-0.032562, 0.035271],
                         [-0.051562, 0.035271]]},
 'sources': {'assets/node/node_mesh.obj': '05a412f156ed8c5261d41fa31c9e8ff36d7cc9512139c8f47ab6e81d658c669a',
             'assets/node_end/node_end_mesh.obj': 'b2362cfa1872f04ce988bc2b688f08dc1826c4c7b1a9d126d58d0dc3b228ce65',
             'assets/housing/housing_mesh.obj': '72e74010fdb32f0ebe1655ac46dbfb30e44e7820f2aed5fd35afe9785a5153f6',
             'assets/wedge/wedge_mesh.obj': 'b466e42154e2ae890b41094e5738412187fbcf352fb634706418286f7f09889d',
             'blueprint.json': '0d661a7d14965b6c563001a0457b301640305a53a3a1a2915a4c772ca7a43773',
             'payload/data/models/arm/default.json': '02a440f7ad8a0037d18c8cfd306d32af081fdb645ee3fb90250bdc5f9d6bd13a',
             'payload/config/robot/default.yaml': '9ea504a19c2579c368f1ef11a39eb82b5a1e4a274d9bfb8f7f4381ecc1987e68',
             'payload/config/pilot/config.yaml': 'c1e4e24b267d8bded0a4608000a60418cf93b37010c306a02fae8eb55834c295',
             '062222.stp': 'f7fa3a255afdbe21f3a58dd018452603e0bee8c47540e58e63c240ea17f224a4',
             'assets/gripper_base/gripper_base_mesh.obj': '597581d382eb8f9df2783949f4c9c18c468227cc8162679ec55fa523abf63668',
             'assets/gripper_claw_left/gripper_claw_left_mesh.obj': 'b07092ed3187a12cab9bd068fdaa442b85e3671458c373a28b45e0b22ed1febb',
             'assets/gripper_claw_right/gripper_claw_right_mesh.obj': '11401fbae8eaf572da9f33ca1626debb5c498d79119fdad7ae5196e4eced940c',
             'assets/camera/camera_mesh.obj': '92cb94b57cd5d91369b5ee6b78d963fc0f7a995e7089a2433b7a41bd577a5eb6',
             'arm.urdf': '1cfdfdfe180170da6457253acf680b71e31ab3104cfc21a54c37e050adaf2005'},
 'golden_fk': [{'section_turns_deg': [0, 0],
                'points': {'node0': [0.0, 0.0],
                           'node1': [0.0, 0.05],
                           'node2': [0.0, 0.1],
                           'node3': [0.0, 0.15],
                           'node4': [0.0, 0.19999999999999998],
                           'node5': [0.0, 0.24999999999999997],
                           'node6': [0.0, 0.29999999999999993],
                           'node7': [0.0, 0.3499999999999999],
                           'node8': [0.0, 0.3999999999999999],
                           'node9': [0.0, 0.4499999999999999],
                           'gripper_base': [0.0, 0.4999999999999999],
                           'arm_tip': [0.0, 0.4999999999999999]}},
               {'section_turns_deg': [90, -45],
                'points': {'node0': [0.0, 0.0],
                           'node1': [-0.015450849718747384, 0.047552825814757685],
                           'node2': [-0.04484011233337104, 0.08800367553350506],
                           'node3': [-0.08529096205211842, 0.1173929381481287],
                           'node4': [-0.13284378786687612, 0.1328437878668761],
                           'node5': [-0.1828437878668761, 0.1328437878668761],
                           'node6': [-0.232228204896633, 0.14066551111888764],
                           'node7': [-0.2797810307113907, 0.15611636083763503],
                           'node8': [-0.3243313569208091, 0.17881588582461236],
                           'node9': [-0.3647822066395565, 0.20820514843923602],
                           'gripper_base': [-0.4001375456988839, 0.24356048749856343],
                           'arm_tip': [-0.40013754569888393, 0.24356048749856343]}},
               {'section_turns_deg': [-120, 150],
                'points': {'node0': [0.0, 0.0],
                           'node1': [0.020336832153790008, 0.045677272882130054],
                           'node2': [0.05749407342765975, 0.07913380320007299],
                           'node3': [0.10504689924241742, 0.09458465291882037],
                           'node4': [0.15477299401083114, 0.0893582297554377],
                           'node5': [0.1980742642000531, 0.06435822975543769],
                           'node6': [0.24807426420005316, 0.06435822975543769],
                           'node7': [0.2913755343892751, 0.0893582297554377],
                           'node8': [0.3163755343892751, 0.13265949994465967],
                           'node9': [0.3163755343892751, 0.18265949994465971],
                           'gripper_base': [0.2913755343892751, 0.2259607701338817],
                           'arm_tip': [0.2913755343892751, 0.2259607701338817]}},
               {'section_turns_deg': [180, -180],
                'points': {'node0': [0.0, 0.0],
                           'node1': [-0.029389262614623657, 0.04045084971874738],
                           'node2': [-0.07694208842938133, 0.05590169943749475],
                           'node3': [-0.12449491424413897, 0.04045084971874739],
                           'node4': [-0.15388417685876263, 5.204170427930421e-17],
                           'node5': [-0.15388417685876263, -0.049999999999999906],
                           'node6': [-0.18327343947338626, -0.09045084971874723],
                           'node7': [-0.23082626528814387, -0.10590169943749457],
                           'node8': [-0.2783790911029015, -0.09045084971874721],
                           'node9': [-0.3077683537175251, -0.04999999999999989],
                           'gripper_base': [-0.3077683537175251, 3.122502256758253e-17],
                           'arm_tip': [-0.3077683537175251, 4.5102810375396984e-17]}}],
 'protected_parts': ['camera', 'housing', 'wedge'],
 'camera_model': {'name': 'ZED Mini',
                  'link': 'camera',
                  'source': 'URDF visual mesh via j_gripper_base_camera',
                  'contact_forbidden': True,
                  'default_clearance_m': 0.001,
                  'note': 'Included as a protected obstacle even though this URDF link has no collision '
                          'element.'}}


class InputError(ValueError):
    pass


def rotate(points, angle):
    c,s = math.cos(angle),math.sin(angle)
    return np.asarray(points,dtype=float) @ np.array([[c,s],[-s,c]])


def cross(a,b):
    a,b = np.asarray(a),np.asarray(b)
    return a[...,0]*b[...,1]-a[...,1]*b[...,0]


def friction_score(points, inward_normals):
    """Infimum of the common mu allowing two-contact PLANAR force closure.

    mu == threshold is not force closure. In particular threshold zero still
    needs positive mu. A contact chord outside the compressive half-space
    cannot be rescued by increasing friction. Coincident contacts are invalid.
    """
    p,n = np.asarray(points,dtype=float),np.asarray(inward_normals,dtype=float)
    if (p.shape!=(2,2) or n.shape!=(2,2) or not np.isfinite(p).all()
            or not np.isfinite(n).all()):
        raise InputError("Two finite 2-D contacts/normals are required.")
    norm = np.linalg.norm(n,axis=1)
    chord = p[1]-p[0];length = float(np.linalg.norm(chord))
    if min(norm)<1e-12 or length<1e-10:
        raise InputError("Contact normals must be nonzero and contact points distinct.")
    n = n/norm[:,None]
    directions = np.array([chord,-chord])/length
    cosine = np.clip(np.sum(n*directions,axis=1),-1,1)
    angles = np.arccos(cosine)
    worst = float(max(angles))
    nominal_value = math.tan(worst) if worst<math.pi/2-1e-12 else math.inf
    return dict(mu_critical=nominal_value,
        angles_deg=np.degrees(angles).tolist(),worst_angle_deg=math.degrees(worst),
        contact_separation_m=length)


def point_segment_distance(points, start, end):
    p = np.asarray(points,dtype=float)
    a,b = np.asarray(start,dtype=float),np.asarray(end,dtype=float)
    delta = b-a
    fraction = np.clip(np.sum((p-a)*delta,axis=-1)/np.maximum(np.sum(delta*delta,axis=-1),1e-30),0,1)
    return np.linalg.norm(p-a-fraction[...,None]*delta,axis=-1)


def inside_polygon(points, vertices):
    p = np.atleast_2d(np.asarray(points,dtype=float))
    a,b = vertices,np.roll(vertices,-1,axis=0)
    px,py = p[:,None,0],p[:,None,1]
    denominator = b[:,1]-a[:,1]
    denominator = np.where(np.abs(denominator)>1e-20,denominator,1.)
    xhit = a[:,0]+(py-a[:,1])*(b[:,0]-a[:,0])/denominator
    hits = ((a[:,1]>py)!=(b[:,1]>py)) & (px<xhit)
    return np.logical_xor.reduce(hits,axis=1)


def nearest_boundary(points, vertices):
    p = np.atleast_2d(np.asarray(points,dtype=float))
    a,b = vertices,np.roll(vertices,-1,axis=0)
    delta = b-a
    fraction = np.clip(np.sum((p[:,None]-a)*delta,axis=2)/np.sum(delta*delta,axis=1),0,1)
    closest = a[None]+fraction[:,:,None]*delta
    squared = np.sum((p[:,None]-closest)**2,axis=2)
    index = np.argmin(squared,axis=1)
    return np.sqrt(squared[np.arange(len(p)),index]),closest[np.arange(len(p)),index],index


def signed_distances(points, vertices):
    distance,*_ = nearest_boundary(points,vertices)
    return np.where(inside_polygon(points,vertices),-distance,distance)


@dataclass(frozen=True)
class Geometry:
    """CAD dimensions; joint commands are one angle per five-node group."""
    pitch_m: float = .05
    nodes_per_section: int = 5
    half_thickness_m: float = .035
    bevel_m: float = .011372189
    joint_limit_deg: float = 36.

    @property
    def length1_m(self):return self.pitch_m*self.nodes_per_section

    @property
    def length2_m(self):return self.pitch_m*self.nodes_per_section

    @property
    def turn_limit_deg(self):return self.joint_limit_deg*self.nodes_per_section

    def checked(self):
        if type(self.nodes_per_section) is not int or self.nodes_per_section!=5:
            raise InputError("This arm has two groups of five nodes.")
        if any(not isinstance(v,(int,float)) or not math.isfinite(v) for v in asdict(self).values()):
            raise InputError("Geometry values must be finite numbers.")
        if not (.005<=self.pitch_m<=.2 and .001<=self.half_thickness_m<=.1
                and 0<self.bevel_m<self.pitch_m/2):
            raise InputError("Invalid node pitch/thickness/bevel.")
        if not 0<self.joint_limit_deg<=36:
            raise InputError("Physical node bend is bounded by +/-36 degrees.")
        return self


def node_profile(g,last=False):
    p,h,b=g.pitch_m,g.half_thickness_m,g.bevel_m
    if last:
        return np.array([[h,p],[-h,p],[-h,b],[0,0],[h,b]])
    return np.array([[0,p],[-h,p-b],[-h,b],[0,0],[h,b],[h,p-b]])


def gripper_profile(g=None):
    """Visible union outline of the full gripper in the URDF terminal frame."""
    return np.asarray(REFERENCE["gripper_model"]["outline_m"],dtype=float)


GRIPPER_PART_NAMES=("gripper_base","gripper_claw_left","gripper_claw_right")


PART_NAMES=tuple("node"+str(i) for i in range(10))+GRIPPER_PART_NAMES+("camera",)


CAMERA_INDEX=PART_NAMES.index("camera")


BODY_NAMES=("housing","wedge")


CONTACT_GROUPS=dict(seg1=PART_NAMES[:5],seg2=PART_NAMES[5:10],gripper_base=("gripper_base",),
                    gripper_claw=("gripper_claw_left","gripper_claw_right"))


ALLOWED_CONTACT_PART_NAMES=frozenset(itertools.chain.from_iterable(CONTACT_GROUPS.values()))


PROTECTED_PART_NAMES=("camera",)+BODY_NAMES


def local_profiles(g):
    return [node_profile(g,i==9) for i in range(10)]+[
        np.asarray(REFERENCE["profiles"][name],dtype=float) for name in GRIPPER_PART_NAMES+("camera",)]


def fk_frames(turns,g=Geometry()):
    """Rotate each physical joint BEFORE translating its 50 mm node."""
    q=np.asarray(turns,dtype=float)
    if q.shape!=(2,) or not np.isfinite(q).all():
        raise InputError("Two finite total section turns are required.")
    n=g.nodes_per_section
    angles=np.r_[np.arange(1,n+1)*q[0]/n,q[0]+np.arange(1,n+1)*q[1]/n]
    delta=g.pitch_m*np.column_stack((-np.sin(angles),np.cos(angles)))
    hinges=np.vstack((np.zeros(2),np.cumsum(delta,axis=0)))
    return hinges,np.r_[angles,angles[-1]]


def robot_polygons(turns,g=Geometry()):
    hinges,angles=fk_frames(turns,g)
    surfaces=[rotate(profile,angles[min(i,10)])+hinges[min(i,10)] for i,profile in enumerate(local_profiles(g))]
    bodies=[np.asarray(REFERENCE["profiles"][name],dtype=float) for name in BODY_NAMES]
    return surfaces,bodies,hinges,angles


def polygon_normals(poly):
    edges=np.roll(poly,-1,axis=0)-poly
    norms=np.linalg.norm(edges,axis=1)
    return np.column_stack((edges[:,1],-edges[:,0]))/norms[:,None]


def convex_gap(a,b):
    """Positive = separated, zero = touch, negative = interpenetration."""
    lower=np.maximum(np.min(b,axis=0)-np.max(a,axis=0),np.min(a,axis=0)-np.max(b,axis=0))
    if np.max(lower)>0:return float(np.max(lower))
    axes=np.vstack((polygon_normals(a),polygon_normals(b)))
    pa=a@axes.T;pb=b@axes.T
    gap=np.maximum(np.min(pb,axis=0)-np.max(pa,axis=0),
                   np.min(pa,axis=0)-np.max(pb,axis=0))
    return float(np.max(gap))


def self_gaps(surfaces,bodies):
    values=[]
    for i,a in enumerate(surfaces):
        for j in range(i+1,len(surfaces)):
            # Node9 and the gripper share one rigid frame; fixed mounting pieces
            # overlap in projection. This is fixed internal assembly geometry.
            if i>=9 and j>=10:continue
            values.append(convex_gap(a,surfaces[j]))
        for b in bodies:values.append(convex_gap(a,b))
    return np.asarray(values)


def circle_gap(polygon,radius,centre=(0.,0.)):
    return float(signed_distances(np.asarray([centre]),polygon)[0])-radius


VERSION = "0.5.0"
OBJECT_SHAPE = "circle"
RADIUS = 0.05
REACH = 0.5
CLEARANCE = 0.001
TOL = 0.00005
MAX_ATTEMPTS = 64
MAX_CANDIDATES = 16
MAX_NFEV = 80
HOUSING = np.asarray(REFERENCE['profiles']['housing'])
BODY_CENTER = (HOUSING.min(axis=0) + HOUSING.max(axis=0)) / 2
G = Geometry()
PROFILES = local_profiles(G)
PAIR_IDS = np.array(list(itertools.combinations(range(CAMERA_INDEX), 2)),dtype=int)


class Cancelled(Exception):
    pass


@dataclass(frozen=True)
class Config:
    dx_mm: float = 20.
    dy_mm: float = 20.
    dtheta_deg: float = 5.
    mu_allow: float = .5

    def checked(self):
        for name, value in asdict(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.dtheta_deg > 360:
            raise ValueError("Angle step must be <= 360 degrees")
        if min(self.dx_mm,self.dy_mm) < .01:
            raise ValueError("Position steps must be at least 0.01 mm")
        return self


def theta_values(config, shape=OBJECT_SHAPE):
    if shape == "circle":
        return (0.,)
    return tuple(-180+i*config.dtheta_deg for i in range(math.ceil(360/config.dtheta_deg)))


def grid_axes(config):
    config.checked()
    # Integer-indexed lattice, anchored at the fixed body center.
    nx = math.floor(REACH * 1000 / config.dx_mm)
    ny = math.floor(REACH * 1000 / config.dy_mm)
    return range(-nx, 1), range(ny+1)


def grid_points(config):
    for ix in grid_axes(config)[0]:
        for iy in grid_axes(config)[1]:
            u, v = ix*config.dx_mm, iy*config.dy_mm
            if u*u + v*v <= (REACH*1000)**2 + 1e-8:
                for theta in theta_values(config):
                    yield (u,v,theta)


def grid_size(config):
    # Count analytically by rows without allocating the whole map.
    config.checked()
    count = 0
    for iy in grid_axes(config)[1]:
        y = iy*config.dy_mm
        xmax = math.sqrt(max(0.,(REACH*1000)**2-y*y))
        count += math.floor((xmax+1e-10)/config.dx_mm)+1
    return count * len(theta_values(config))


def local_center(u_mm, v_mm, theta_deg=0.):
    return BODY_CENTER + np.array([u_mm,v_mm])/1000


def exclusion(center, radius=None):
    radius = RADIUS if radius is None else radius
    gap = circle_gap(HOUSING, radius, center)
    if gap > REACH:
        return 'outside_interest'
    if gap <= CLEARANCE:
        return 'body_clearance'
    if circle_gap(np.asarray(REFERENCE['profiles']['wedge']), radius, center) <= CLEARANCE:
        return 'wedge_clearance'
    return None


def polygon_at(part, hinges, angles):
    k = min(part, 10)
    return rotate(PROFILES[part], angles[k]) + hinges[k]


def inspect_pose(q, center, mu_allow, radius=None):
    radius = RADIUS if radius is None else radius
    if not np.isfinite(q).all() or np.max(np.abs(q)) > math.pi+1e-12:
        return None
    surfaces, bodies, _, _ = robot_polygons(q, G)
    gaps = np.array([circle_gap(p, radius, center) for p in surfaces])
    if gaps.min() < -TOL or gaps[CAMERA_INDEX] <= CLEARANCE:
        return None
    if any(circle_gap(p, radius, center) <= CLEARANCE for p in bodies):
        return None
    if self_gaps(surfaces, bodies).min() < -TOL:
        return None
    contacts = np.flatnonzero(np.abs(gaps[:CAMERA_INDEX]) <= TOL)
    best = None
    for i, j in itertools.combinations(contacts, 2):
        points = np.array([nearest_boundary([center], surfaces[k])[1][0] for k in (i,j)])
        normals = center - points
        if np.linalg.norm(points[0]-points[1]) < 1e-8:
            continue
        score = friction_score(points, normals)
        mu = score['mu_critical']
        if not math.isfinite(mu) or mu >= mu_allow:
            continue
        if best is None or mu < best['mu_min']:
            best = dict(seg1_deg=float(np.degrees(q[0])), seg2_deg=float(np.degrees(q[1])),
                        mu_min=mu, contact_parts=[PART_NAMES[i], PART_NAMES[j]],
                        contacts_node0_m=points.tolist(),
                        contact_gaps_mm=(gaps[[i,j]]*1000).tolist())
    return best


BEND_COEFF=np.array([[j/5,0] if j<=5 else [1,(j-5)/5] for j in range(1,11)]+[[1,1]])
MAX_VERTICES=max(map(len,PROFILES))
EDGE_START=np.zeros((len(PROFILES),MAX_VERTICES,2));EDGE_END=EDGE_START.copy();EDGE_MASK=np.zeros(EDGE_START.shape[:2],bool)
for i,p in enumerate(PROFILES):
    EDGE_START[i,:len(p)]=p;EDGE_END[i,:len(p)]=np.roll(p,-1,axis=0);EDGE_MASK[i,:len(p)]=True

def batched_contacts(q,parts,center,radius=None):
    """Signed disk gaps, analytic angle Jacobians and nearest boundary points.

    q: (N,2) total section bends, parts: (K,) or (N,K) polygon indices.
    Outputs have shapes (N,K), (N,K,2), (N,K,2). Padded edges are masked.
    The nearest-point derivative uses the envelope theorem: along an edge,
    displacement of the minimizing edge parameter is tangent and contributes
    zero to the distance derivative. At feature switches the derivative is
    piecewise defined, as in the original closest-boundary residual.
    """
    radius=RADIUS if radius is None else radius
    q=np.atleast_2d(q);n=len(q);parts=np.broadcast_to(parts,(n,np.asarray(parts).shape[-1]));frames=np.minimum(parts,10)
    angles=q@BEND_COEFF.T;c=np.cos(angles);s=np.sin(angles)
    delta=.05*np.stack((-s[:,:10],c[:,:10]),axis=-1)
    hinges=np.concatenate((np.zeros((n,1,2)),np.cumsum(delta,axis=1)),axis=1)
    dh=.05*np.stack((-c[:,:10],-s[:,:10]),axis=-1)[...,None]*BEND_COEFF[None,:10,None,:]
    hj=np.concatenate((np.zeros((n,1,2,2)),np.cumsum(dh,axis=1)),axis=1)
    index=np.arange(n)[:,None];cc=c[index,frames];ss=s[index,frames];origin=hinges[index,frames]
    aa=EDGE_START[parts];bb=EDGE_END[parts]
    def rot(p):
        return np.stack((p[...,0]*cc[...,None]-p[...,1]*ss[...,None],p[...,0]*ss[...,None]+p[...,1]*cc[...,None]),axis=-1)
    a=rot(aa)+origin[...,None,:];b=rot(bb)+origin[...,None,:];d=b-a
    frac=np.clip(np.sum((center-a)*d,axis=-1)/np.maximum(np.sum(d*d,axis=-1),1e-30),0,1)
    close=a+frac[...,None]*d
    sq=np.sum((close-center)**2,axis=-1);sq=np.where(EDGE_MASK[parts],sq,np.inf)
    edge=np.argmin(sq,axis=-1);take=edge[...,None,None]
    p=np.take_along_axis(close,take,axis=2).squeeze(2)
    t=np.take_along_axis(frac,edge[...,None],axis=2).squeeze(2)
    # Same odd/even crossing sign as the original polygon routine.
    dy=b[...,1]-a[...,1];den=np.where(np.abs(dy)>1e-20,dy,1.)
    xhit=a[...,0]+(center[1]-a[...,1])*(b[...,0]-a[...,0])/den
    hits=((a[...,1]>center[1])!=(b[...,1]>center[1]))&(center[0]<xhit)&EDGE_MASK[parts]
    inside=np.logical_xor.reduce(hits,axis=-1);sign=np.where(inside,-1.,1.)
    dist=np.sqrt(np.min(sq,axis=-1));gap=sign*dist-radius
    local=np.take_along_axis(aa,take,axis=2).squeeze(2)+t[...,None]*np.take_along_axis(bb-aa,take,axis=2).squeeze(2)
    rotated=np.stack((local[...,0]*cc-local[...,1]*ss,local[...,0]*ss+local[...,1]*cc),axis=-1)
    deriv=np.stack((-rotated[...,1],rotated[...,0]),axis=-1)[...,None]*BEND_COEFF[frames][...,None,:]+hj[index,frames]
    jac=np.einsum('nki,nkij->nkj',(p-center)/np.maximum(dist[...,None],1e-30)*sign[...,None],deriv)
    return gap,jac,p

def solve_placement(center,mu_allow,stop=None,warm=None,radius=None):
    """Solve all 64 deterministic starts together with bounded damped Gauss-Newton.

    Each start keeps its own contact pair, damping and accepted iterate. A trial
    is accepted only when its squared contact-gap residual decreases. Group
    bounds and a 0.35 rad trust-step cap apply to each start independently.
    Final candidates pass the unchanged original collision/friction validator.
    Unlike serial early termination, attempts reports all starts in the batch;
    at most MAX_CANDIDATES distinct valid poses are retained in seed order.
    """
    radius=RADIUS if radius is None else radius
    stop=stop or threading.Event()
    if stop.is_set():raise Cancelled()
    starts=[np.radians(np.array(p)*5) for p in [(-12,-33),(-12,-28),(12,33),(24,-36),(-24,36),(-6,-30),(6,30)]]
    lattice=[np.radians([a,b]) for a in np.linspace(-171,171,8) for b in np.linspace(-171,171,8)]
    starts += [lattice[k] for k in np.linspace(0,len(lattice)-1,MAX_ATTEMPTS-len(starts),dtype=int)]
    if warm is not None:starts=[np.radians([warm['seg1_deg'],warm['seg2_deg']])]+starts[:-1]
    q=np.array(starts);n=len(q);ix=np.arange(n)
    allgap,alljac,points=batched_contacts(q,np.arange(CAMERA_INDEX),center,radius)
    norm=center-points;length=np.linalg.norm(norm,axis=-1);norm/=np.maximum(length[...,None],1e-30)
    i,j=PAIR_IDS.T;chord=points[:,j]-points[:,i];cl=np.linalg.norm(chord,axis=-1);direction=chord/np.maximum(cl[...,None],1e-30)
    cosine=np.clip(np.minimum(np.sum(norm[:,i]*direction,axis=-1),np.sum(-norm[:,j]*direction,axis=-1)),-1,1)
    mu=np.where(cosine>0,np.sqrt(np.maximum(0,1-cosine*cosine))/np.maximum(cosine,1e-30),10.)
    priority=np.abs(allgap[:,i])+np.abs(allgap[:,j])+.02*np.minimum(mu,10.)
    priority[(cl<1e-8)|(length[:,i]<1e-10)|(length[:,j]<1e-10)]=np.inf
    selected=np.argsort(priority,axis=1,kind='stable')[ix,ix%4];pairs=PAIR_IDS[selected]
    if warm is not None:pairs[0]=[PART_NAMES.index(name) for name in warm['contact_parts']]
    # Preserve exact valid seeds before fitting; vector screen avoids full checks on impossible seeds.
    originals={}
    possible=np.sort(np.abs(allgap),axis=1)[:,1]<=TOL
    for k in np.flatnonzero(possible):
        sol=inspect_pose(q[k],center,mu_allow,radius)
        if sol is not None:originals[k]=sol
    r=allgap[ix[:,None],pairs]/.05;jac=alljac[ix[:,None],pairs]/.05
    damping=np.full(n,.001);active=np.ones(n,bool)
    for k in originals:active[k]=False
    for step in range(MAX_NFEV-1):
        if stop.is_set():raise Cancelled()
        active &= np.max(np.abs(r),axis=1)>.00000001
        if not active.any():break
        H=np.einsum('nki,nkj->nij',jac,jac);g=np.einsum('nki,nk->ni',jac,r)
        aa=H[:,0,0]+damping*np.maximum(H[:,0,0],1e-4);dd=H[:,1,1]+damping*np.maximum(H[:,1,1],1e-4);bb=H[:,0,1]
        det=np.maximum(aa*dd-bb*bb,1e-30)
        delta=np.stack(((-dd*g[:,0]+bb*g[:,1])/det,(bb*g[:,0]-aa*g[:,1])/det),axis=-1)
        delta*=np.minimum(1,.35/np.maximum(np.linalg.norm(delta,axis=1),1e-30))[:,None]
        trial=np.clip(q+delta,-np.pi,np.pi)
        rr,jj,_=batched_contacts(trial,pairs,center,radius);rr/=.05;jj/=.05
        accepted=(np.sum(rr*rr,axis=1)<np.sum(r*r,axis=1))&active
        q[accepted]=trial[accepted];r[accepted]=rr[accepted];jac[accepted]=jj[accepted]
        damping=np.where(accepted,np.maximum(damping/3,1e-9),np.minimum(damping*5,1e12))
        active &= damping<1e12
    candidates=[]
    for k in range(n):
        if stop.is_set():raise Cancelled()
        sol=originals.get(k)
        if sol is None and np.max(np.abs(r[k]))*.05<=TOL:
            sol=inspect_pose(q[k],center,mu_allow,radius)
        if sol is not None and not any(max(abs(sol[key]-s[key]) for key in ('seg1_deg','seg2_deg'))<.1 for s in candidates):
            candidates.append(sol)
        if len(candidates)>=MAX_CANDIDATES:break
    best=min(candidates,key=lambda s:s['mu_min']) if candidates else None
    return dict(status='found' if best else 'not_found',attempts=n,candidate_count=len(candidates),numerical_failures=int(np.sum(~np.isfinite(r).all(axis=1))),
    termination='candidate_limit' if len(candidates)>=MAX_CANDIDATES else 'attempt_limit',seg1_deg=None,seg2_deg=None,mu_min=None,**({'best':best} if best else {}))


class JsonWriter:
    """Stream one valid JSON document to a private partial file; finalize atomically.

    Cooperative stop finishes valid JSON with status=cancelled. A hard process
    kill can leave an unfinished .partial file; automatic resume is not claimed.
    """
    def __init__(self, path, metadata):
        self.path = Path(path)
        self.partial = Path(str(path)+'.partial')
        if self.path.exists():
            raise FileExistsError(f'Output already exists: {self.path}')
        self.file = self.partial.open('x', encoding='utf-8')
        self.first = True
        self.file.write('{"metadata":'+json.dumps(metadata,allow_nan=False)+',"results":[\n')

    def append(self, row):
        if not self.first:
            self.file.write(',\n')
        self.first = False
        self.file.write(json.dumps(row,allow_nan=False,separators=(',',':')))
        self.file.flush()

    def finish(self, summary):
        self.file.write('\n],"summary":'+json.dumps(summary,allow_nan=False)+'}\n')
        self.file.flush()
        os.fsync(self.file.fileno())
        self.file.close()
        # Hard-link publication refuses to overwrite an existing destination.
        os.link(self.partial, self.path)
        self.partial.unlink()


def available_workers():
    """Respect CPU affinity and the container's CPU quota; leave one CPU free."""
    count = len(os.sched_getaffinity(0)) if hasattr(os, 'sched_getaffinity') else (os.cpu_count() or 1)
    try:
        quota, period = Path('/sys/fs/cgroup/cpu.max').read_text().split()
        if quota != 'max':
            count = min(count, max(1, math.ceil(int(quota)/int(period))))
    except (OSError, ValueError):
        pass
    return max(1, min(32, count-1 if count > 2 else count))


_WORKER_STOP = None


def worker_init(stop):
    global _WORKER_STOP
    _WORKER_STOP = stop


def solve_batch(batch, mu_allow, radius=None):
    """Fixed-size batches retain neighboring warm starts regardless of CPU count."""
    rows = []
    warm = None
    for item in batch:
        pose, aliases = item[:2]
        if len(item) == 3 and item[2] is not None:
            saved = item[2]
            warm = saved if saved["status"] == "found" else None
            continue
        if _WORKER_STOP.is_set():
            break
        try:
            row = solve_placement(local_center(*pose), mu_allow, _WORKER_STOP, warm, radius)
        except Cancelled:
            break
        best = row.pop('best',None)
        if best:
            row.update(best)
        warm = best
        rows.append((aliases,row))
    return rows


def map_metadata(config,workers,diameter_mm=100.):
    total=grid_size(config)
    return dict(schema='hugmap', schema_version=1, version=VERSION,
        created_utc=datetime.now(timezone.utc).isoformat(), config=asdict(config),
        object=dict(shape=OBJECT_SHAPE,diameter_mm=diameter_mm,position='varies per result'),
        coordinates=dict(position_unit='mm',angle_unit='degree',
            origin='fixed housing rectangle center; x=0 on roll axis; not physical Go2 base',
            position='object center (x_mm,y_mm) in body frame; +y forward, +x right',
            orientation='object theta_deg relative to fixed body; circle uses 0 only',
            angle_output='total bend per five-node group; per-node angle = group/5',
            linear='fixed at neutral',node0_from_body_center_mm=(-BODY_CENTER*1000).tolist()),
        constraints=dict(object_center_semicircle_radius_mm=REACH*1000,forward_half_plane="y >= 0",interest_region="left_forward_quadrant",
            lateral_half_plane="x <= 0",
            clearance_mm=CLEARANCE*1000,
            contact_tolerance_mm=TOL*1000,group_bend_limit_deg=180,
            max_attempts=MAX_ATTEMPTS,max_candidates=MAX_CANDIDATES,max_nfev=MAX_NFEV),
        execution=dict(workers=workers,batch_size=4,solver='batched-damped-gauss-newton',
            jacobian='analytic',starts_per_batch=MAX_ATTEMPTS,
            result_order='completion order; use coordinate keys, not array indices',
            contact_coordinates='node0-local meters; add node0_from_body_center_mm to plot in body frame'),
        grid_total=total,geometry=REFERENCE,
        symmetry=dict(kind='circle_roll_reflection',computed_roll_deg=90,
            reflected_roll_deg=-90,position_rule='(x,y) -> (-x,y)',
            angle_rule='seg1 and seg2 unchanged',mu_rule='unchanged',
            scope='top-view circle model; 3D height and approach collisions not evaluated'),
        semantics='Lowest mu among discovered eligible candidates, not certified global minimum. '
                  'not_found does not mean impossible. Static contacts, not approach/holding proof.')


def run_map(config, output, stop=None, progress=None, workers=None):
    config.checked()
    if OBJECT_SHAPE != 'circle':
        raise ValueError('Only circle contact solving is implemented')
    stop = stop or threading.Event()
    workers = available_workers() if workers is None else workers
    if not isinstance(workers,int) or workers < 1:
        raise ValueError('workers must be a positive integer')
    total = grid_size(config)
    metadata = map_metadata(config,workers)
    writer = JsonWriter(output,metadata)
    counts = Counter()
    start = time.monotonic()
    status = 'complete'
    error = None
    visited = 0
    solved = 0
    last_report = 0.

    def emit(poses,row):
        nonlocal visited,last_report
        for u,v,theta in poses:
            writer.append(dict(row,x_mm=u,y_mm=v,theta_deg=theta))
            visited += 1
            counts[row['status']] += 1
        now = time.monotonic()
        if progress and (now-last_report >= .2 or visited == total):
            progress(dict(visited=visited,total=total,counts=dict(counts),
                          elapsed_s=now-start,current=[u,v,theta],unique_solves=solved,workers=workers))
            last_report = now

    def jobs():
        batch = []
        for p in grid_points(config):
            if stop.is_set():
                return
            reason = exclusion(local_center(*p))
            if reason:
                emit([p],dict(status='excluded',reason=reason,attempts=0,candidate_count=0,
                              seg1_deg=None,seg2_deg=None,mu_min=None))
                continue
            batch.append((p,[p]))
            if len(batch) == 4:
                yield batch
                batch = []
        if batch and not stop.is_set():
            yield batch

    # Spawn avoids forking the Tk GUI or an already initialized numerical runtime.
    # Bound BLAS threads in children to prevent workers x BLAS-thread oversubscription.
    context = multiprocessing.get_context('spawn')
    worker_stop = context.Event()
    env_keys = ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS')
    old_env = {key:os.environ.get(key) for key in env_keys}
    for key in env_keys:
        os.environ[key] = '1'
    pool = None
    pending = set()
    try:
        pool = ProcessPoolExecutor(max_workers=workers,mp_context=context,
                                   initializer=worker_init,initargs=(worker_stop,))
        generator = iter(jobs())
        exhausted = False
        while pending or not exhausted:
            if stop.is_set():
                status = 'cancelled'
                worker_stop.set()
                exhausted = True
            while not exhausted and len(pending) < 2*workers:
                batch = next(generator,None)
                if batch is None:
                    exhausted = True
                    break
                pending.add(pool.submit(solve_batch,batch,config.mu_allow))
            if not pending:
                continue
            finished,pending = wait(pending,timeout=.1,return_when=FIRST_COMPLETED)
            for future in finished:
                for aliases,row in future.result():
                    solved += 1
                    emit(aliases,row)
        if stop.is_set():
            status = 'cancelled'
    except (Cancelled, KeyboardInterrupt):
        status = 'cancelled'
        worker_stop.set()
    except Exception as exc:
        status = 'failed'
        error = f'{type(exc).__name__}: {exc}'
        worker_stop.set()
    finally:
        if pool:
            pool.shutdown(wait=True,cancel_futures=True)
        for key,value in old_env.items():
            if value is None:
                os.environ.pop(key,None)
            else:
                os.environ[key] = value
    summary = dict(status=status,visited=visited,total=total,counts=dict(counts),
                   unique_solves=solved,workers=workers,
                   elapsed_s=time.monotonic()-start,error=error)
    writer.finish(summary)
    return summary


def diameter_values(minimum=50.,maximum=250.,step=5.):
    values=(minimum,maximum,step)
    if any(isinstance(v,bool) or not math.isfinite(v) or v<=0 for v in values):
        raise ValueError('Diameter bounds and step must be finite positive numbers')
    if maximum<minimum:
        raise ValueError('Maximum diameter must be >= minimum diameter')
    intervals=(maximum-minimum)/step
    if abs(intervals-round(intervals))>1e-8:
        raise ValueError('Diameter range must be divisible by the diameter step')
    if intervals>10000:
        raise ValueError('At most 10001 diameters per table')
    return [round(minimum+i*step,10) for i in range(round(intervals)+1)]


class TableCheckpoint:
    """One atomic JSON snapshot, with an OS-released writer lock for crash recovery."""
    def __init__(self,path,resume):
        self.path=Path(path)
        self.resume=resume
        self.published=resume
        self.lock=None

    def __enter__(self):
        self.lock=open(str(self.path)+'.lock','a+b')
        try:
            if os.name=='nt':
                import msvcrt
                self.lock.seek(0,os.SEEK_END)
                if self.lock.tell()==0:
                    self.lock.write(b'0');self.lock.flush()
                self.lock.seek(0);msvcrt.locking(self.lock.fileno(),msvcrt.LK_NBLCK,1)
            else:
                import fcntl
                fcntl.flock(self.lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        except OSError as exc:
            self.lock.close();self.lock=None
            raise ValueError('Another process is writing this table') from exc
        if self.path.exists() and not self.resume:
            self.__exit__(None,None,None)
            raise FileExistsError(f'Output already exists: {self.path}; use Resume table')
        if self.resume and not self.path.is_file():
            self.__exit__(None,None,None)
            raise FileNotFoundError(self.path)
        return self

    def save(self,document):
        temporary=None
        try:
            with tempfile.NamedTemporaryFile(mode='w',encoding='utf-8',dir=self.path.parent,
                    prefix='.'+self.path.name+'.',suffix='.tmp',delete=False) as stream:
                temporary=Path(stream.name)
                json.dump(document,stream,allow_nan=False,separators=(',',':'))
                stream.write('\n');stream.flush();os.fsync(stream.fileno())
            if self.published:
                os.replace(temporary,self.path)
            else:
                os.link(temporary,self.path)
                temporary.unlink()
                self.published=True
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()

    def __exit__(self,*_):
        if self.lock is not None:
            if os.name=='nt':
                import msvcrt
                self.lock.seek(0);msvcrt.locking(self.lock.fileno(),msvcrt.LK_UNLCK,1)
            else:
                import fcntl
                fcntl.flock(self.lock.fileno(),fcntl.LOCK_UN)
            self.lock.close();self.lock=None
        # Keep the empty lock inode stable across successive writers. No map data
        # lives here, and OS locks are automatically released on process death.


def table_map_document(document,entry):
    metadata=dict(document['metadata'])
    metadata.update(schema='hugmap',schema_version=1,
                    object=dict(shape='circle',diameter_mm=entry['diameter_mm'],position='varies per result'))
    summary=dict(entry.get('summary',{}),status=entry.get('status','pending'))
    return dict(metadata=metadata,results=entry['results'],summary=summary)


def validate_table(document,for_resume=False):
    if not isinstance(document,dict) or not isinstance(document.get('metadata'),dict):
        raise ValueError('Expected a Hugmap table JSON object')
    metadata=document['metadata']
    if metadata.get('schema')!='hugmap-table' or metadata.get('schema_version')!=2:
        raise ValueError('Expected Hugmap table schema version 2')
    if not isinstance(metadata.get('object'),dict) or metadata['object'].get('shape')!='circle':
        raise ValueError('Expected a circle diameter table')
    if not isinstance(metadata.get('config'),dict):
        raise ValueError('Missing table configuration')
    try:
        config=Config(**metadata['config']).checked()
    except TypeError as exc:
        raise ValueError('Invalid table configuration') from exc
    plan=metadata.get('diameters_mm')
    entries=document.get('maps')
    if not isinstance(plan,list) or not plan or not isinstance(entries,list) or len(plan)!=len(entries):
        raise ValueError('Table diameter plan and maps do not match')
    if any(isinstance(d,bool) or not isinstance(d,(int,float)) or not math.isfinite(d) or d<=0 for d in plan):
        raise ValueError('Diameters must be finite positive numbers')
    if plan!=sorted(set(plan)) or [e.get('diameter_mm') for e in entries if isinstance(e,dict)]!=plan:
        raise ValueError('Diameter maps must match the sorted unique diameter plan')
    if not isinstance(document.get('summary',{}),dict):
        raise ValueError('Invalid table summary')
    if for_resume:
        expected=map_metadata(config,1)
        for key in ('geometry','constraints','coordinates','symmetry'):
            if metadata.get(key)!=expected[key]:
                raise ValueError(f'Table {key} is incompatible with this solver')
        if metadata.get('solver_revision')!='batch-gn-v1':
            raise ValueError('Cannot resume a table from a different solver revision')
        if metadata.get('object',{}).get('shape')!='circle':
            raise ValueError('Only circle tables can be resumed')
    for entry in entries:
        if entry.get('status') not in ('pending','running','complete','cancelled','failed'):
            raise ValueError('Unknown diameter-map status')
        data=SavedMapData(table_map_document(document,entry))
        for row in entry['results']:
            if (row['theta_deg']!=0 or row['y_mm']<0 or
                    (metadata.get('constraints',{}).get('interest_region')=='left_forward_quadrant' and row['x_mm']>0) or
                    row['x_mm']**2+row['y_mm']**2>(REACH*1000)**2+1e-8):
                raise ValueError('Saved pose is outside the circle-table grid')
        if for_resume and entry['status']=='complete' and data.count!=grid_size(config):
            raise ValueError('Completed diameter map is missing grid poses')
    return config


def run_table(config,output,minimum=50.,maximum=250.,step=5.,stop=None,progress=None,
              workers=None,resume=False):
    """Generate/resume all diameters in one JSON; reuse one process pool."""
    stop=stop or threading.Event()
    workers=available_workers() if workers is None else workers
    if not isinstance(workers,int) or workers<1:
        raise ValueError('workers must be a positive integer')
    with TableCheckpoint(output,resume) as checkpoint:
        if resume:
            with open(output,encoding='utf-8') as stream:
                document=json.load(stream)
            config=validate_table(document,for_resume=True)
        else:
            config.checked()
            diameters=diameter_values(minimum,maximum,step)
            metadata=map_metadata(config,workers)
            metadata.update(schema='hugmap-table',schema_version=2,solver_revision='batch-gn-v1',
                object=dict(shape='circle'),diameters_mm=diameters,
                diameter_range=dict(min_mm=minimum,max_mm=maximum,step_mm=step))
            document=dict(metadata=metadata,maps=[dict(diameter_mm=d,status='pending',results=[],summary={})
                                               for d in diameters],summary={})
        document['metadata']['execution']['workers']=workers
        per_map=grid_size(config);total=per_map*len(document['maps'])
        previous_elapsed=float(document.get('summary',{}).get('elapsed_s',0))
        if not math.isfinite(previous_elapsed) or previous_elapsed<0:
            raise ValueError('Invalid accumulated elapsed time')
        start=time.monotonic();last_save=start;last_report=0.;status='running';error=None
        counts=Counter();saved={};map_counts={};visited=0;computed=0
        for entry in document['maps']:
            d=entry['diameter_mm']
            saved[d]={(r['x_mm'],r['y_mm'],r['theta_deg']):r for r in entry['results']}
            map_counts[d]=Counter(r['status'] for r in entry['results'])
            counts.update(map_counts[d]);visited+=len(entry['results'])
            entry['status']='complete' if len(entry['results'])==per_map else ('pending' if not entry['results'] else 'running')
        entries={e['diameter_mm']:e for e in document['maps']}
        resumed_visited=visited

        def summary():
            for entry in document['maps']:
                d=entry['diameter_mm'];entry['summary']=dict(visited=len(entry['results']),total=per_map,counts=dict(map_counts[d]))
            document['summary']=dict(status=status,visited=visited,total=total,counts=dict(counts),
                completed_maps=sum(e['status']=='complete' for e in document['maps']),total_maps=len(entries),
                resumed_visited=resumed_visited,computed_this_run=computed,workers=workers,
                elapsed_s=previous_elapsed+time.monotonic()-start,error=error)
            return document['summary']

        def emit(d,poses,row):
            nonlocal visited,computed,last_save,last_report
            entry=entries[d]
            for p in poses:
                key=tuple(p)
                if key in saved[d]:
                    raise ValueError('Duplicate result produced during resume')
                record=dict(row,x_mm=p[0],y_mm=p[1],theta_deg=p[2])
                saved[d][key]=record;entry['results'].append(record)
                counts[row['status']]+=1;map_counts[d][row['status']]+=1;visited+=1
                if row['status']!='excluded':computed+=1
            entry['status']='complete' if len(entry['results'])==per_map else 'running'
            now=time.monotonic()
            if progress and (now-last_report>=.2 or visited==total):
                progress(dict(visited=visited,total=total,counts=dict(counts),current=list(p),
                    diameter_mm=d,completed_maps=sum(e['status']=='complete' for e in document['maps']),
                    total_maps=len(entries),elapsed_s=now-start,workers=workers))
                last_report=now
            if now-last_save>=10 or entry['status']=='complete':
                checkpoint.save(dict(document,summary=summary()));last_save=now

        def jobs():
            for entry in document['maps']:
                d=entry['diameter_mm'];batch=[]
                if entry['status']=='complete':continue
                for p in grid_points(config):
                    if stop.is_set():return
                    cached=saved[d].get(tuple(p))
                    reason=exclusion(local_center(*p),d/2000)
                    if reason:
                        if cached is None:
                            emit(d,[p],dict(status='excluded',reason=reason,attempts=0,candidate_count=0,
                                           seg1_deg=None,seg2_deg=None,mu_min=None))
                        continue
                    batch.append((p,[p],cached))
                    if len(batch)==4:
                        if any(item[2] is None for item in batch):yield d,batch
                        batch=[]
                if batch and any(item[2] is None for item in batch) and not stop.is_set():
                    yield d,batch

        checkpoint.save(dict(document,summary=summary()))
        context=multiprocessing.get_context('spawn');worker_stop=context.Event()
        env_keys=('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS')
        old_env={k:os.environ.get(k) for k in env_keys}
        for k in env_keys:os.environ[k]='1'
        pool=None;pending={}
        try:
            pool=ProcessPoolExecutor(max_workers=workers,mp_context=context,initializer=worker_init,initargs=(worker_stop,))
            iterator=iter(jobs());exhausted=False
            while pending or not exhausted:
                if stop.is_set():
                    status='cancelled';worker_stop.set();exhausted=True
                while not exhausted and len(pending)<2*workers:
                    job=next(iterator,None)
                    if job is None:
                        exhausted=True;break
                    d,batch=job
                    pending[pool.submit(solve_batch,batch,config.mu_allow,d/2000)]=d
                if not pending:continue
                finished,_=wait(pending,timeout=.1,return_when=FIRST_COMPLETED)
                for future in finished:
                    d=pending.pop(future)
                    for poses,row in future.result():emit(d,poses,row)
            status='cancelled' if stop.is_set() else 'complete'
        except KeyboardInterrupt:
            status='cancelled';worker_stop.set()
        except Exception as exc:
            status='failed';error=f'{type(exc).__name__}: {exc}';worker_stop.set()
        finally:
            if pool:pool.shutdown(wait=True,cancel_futures=True)
            for key,value in old_env.items():
                if value is None:os.environ.pop(key,None)
                else:os.environ[key]=value
        for entry in document['maps']:
            if entry['status']=='running':entry['status']=status if status!='complete' else 'failed'
        if status=='complete' and visited!=total:
            status='failed';error='Incomplete table after worker completion'
        checkpoint.save(dict(document,summary=summary()))
        return document['summary']


HEATMAP_FIELDS = {'seg1': 'seg1_deg', 'seg2': 'seg2_deg', 'mu_min': 'mu_min'}


class SavedMapData:
    """Validated sparse (x,y,theta) map; missing cells are never interpolated."""
    def __init__(self, document):
        if not isinstance(document,dict):
            raise ValueError('Expected a Hugmap JSON object')
        metadata = document.get('metadata',{})
        if not isinstance(metadata,dict):
            raise ValueError('metadata must be an object')
        if metadata.get('schema') != 'hugmap' or metadata.get('schema_version') != 1:
            raise ValueError('Expected Hugmap JSON schema version 1')
        config = metadata.get('config',{})
        if not isinstance(config,dict):
            raise ValueError('metadata.config must be an object')
        constraints = metadata.get('constraints',{})
        if not isinstance(constraints,dict):
            raise ValueError('metadata.constraints must be an object')
        self.dx = self.number(config.get('dx_mm'),'dx_mm',positive=True)
        self.dy = self.number(config.get('dy_mm'),'dy_mm',positive=True)
        self.radius = self.number(constraints.get(
            'object_center_semicircle_radius_mm',500),'radius',positive=True)
        self.metadata = metadata
        self.summary = document.get('summary',{})
        if not isinstance(self.summary,dict):
            raise ValueError('summary must be an object')
        self.slices = {}
        rows = document.get('results')
        if not isinstance(rows,list):
            raise ValueError('results must be a list')
        self.ranges = {}
        values = {field:[] for field in HEATMAP_FIELDS}
        for row in rows:
            if not isinstance(row,dict):
                raise ValueError('Each result must be an object')
            x,y,t = [self.number(row.get(key),key) for key in ('x_mm','y_mm','theta_deg')]
            ix,iy = round(x/self.dx),round(y/self.dy)
            if abs(x/self.dx-ix)>1e-6 or abs(y/self.dy-iy)>1e-6:
                raise ValueError('Coordinates do not match the declared grid resolution')
            if row.get('status') not in ('found','not_found','excluded'):
                raise ValueError('Unknown result status')
            layer = self.slices.setdefault(t,{})
            if (ix,iy) in layer:
                raise ValueError(f'Duplicate pose: ({x}, {y}, {t})')
            layer[ix,iy] = row
            if row['status'] == 'found':
                for field,key in HEATMAP_FIELDS.items():
                    value = self.number(row.get(key),key)
                    if field == 'mu_min' and value < 0:
                        raise ValueError('mu_min cannot be negative')
                    values[field].append(value)
        for field,items in values.items():
            if not items:
                self.ranges[field] = None
            elif field.startswith('seg'):
                bound = max(abs(min(items)),abs(max(items)))
                self.ranges[field] = (-bound,bound)
            else:
                self.ranges[field] = (min(items),max(items))
        self.thetas = sorted(self.slices)
        self.count = len(rows)
        self.bounds = (-self.radius-self.dx/2,self.radius+self.dx/2,
                       -self.dy/2,self.radius+self.dy/2)
        if rows:
            self.bounds = (min(self.bounds[0],min(r['x_mm'] for r in rows)-self.dx/2),
                           max(self.bounds[1],max(r['x_mm'] for r in rows)+self.dx/2),
                           min(self.bounds[2],min(r['y_mm'] for r in rows)-self.dy/2),
                           max(self.bounds[3],max(r['y_mm'] for r in rows)+self.dy/2))

    @staticmethod
    def number(value,name,positive=False):
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
            raise ValueError(f'{name} must be a finite number')
        if positive and value<=0:
            raise ValueError(f'{name} must be positive')
        return float(value)

    def at(self,x,y,theta):
        ix = math.floor(x/self.dx+.5)
        iy = math.floor(y/self.dy+.5)
        return self.slices.get(theta,{}).get((ix,iy))

def gui():
    try:
        import tkinter as tk
        from tkinter import ttk, filedialog, messagebox
    except ImportError:
        raise SystemExit('GUI requires Tkinter (Ubuntu: python3-tk); CLI --table is also available.')
    root=tk.Tk();root.title('Hugmap - Circle rainbow table');root.geometry('820x620')
    frame=ttk.Frame(root,padding=18);frame.pack(fill='both',expand=True)
    ttk.Label(frame,text='Hugmap circle table',font=('',20,'bold')).pack(anchor='w')
    ttk.Label(frame,text='One JSON file for all diameters | Fixed body | Left-forward quadrant: 500 mm | Roll: +90 deg\n'
              f'64 starts / 16 candidates | CPU workers: {available_workers()} | Circle ignores theta step',
              justify='left').pack(anchor='w',pady=8)
    fields=ttk.Frame(frame);fields.pack(fill='x')
    defaults=dict(asdict(Config()),diameter_min_mm=50.,diameter_max_mm=250.,diameter_step_mm=5.)
    entries={}
    labels=[('diameter_min_mm','Minimum diameter (mm)'),('diameter_max_mm','Maximum diameter (mm)'),
            ('diameter_step_mm','Diameter step (mm)'),('dx_mm','X step (mm)'),('dy_mm','Y step (mm)'),
            ('dtheta_deg','Theta step (deg; ignored for circle)'),('mu_allow','Available friction coefficient (mu)')]
    for i,(key,label) in enumerate(labels):
        ttk.Label(fields,text=label).grid(row=i,column=0,sticky='w',pady=3)
        variable=tk.StringVar(value=str(defaults[key]))
        entry=ttk.Entry(fields,textvariable=variable);entry.grid(row=i,column=1,padx=12)
        entries[key]=(variable,entry)
    estimate=tk.StringVar();ttk.Label(frame,textvariable=estimate).pack(anchor='w',pady=8)
    state=tk.StringVar(value='Create a new table, or resume an existing table JSON.')
    ttk.Label(frame,textvariable=state,wraplength=770).pack(anchor='w',pady=8)
    bar=ttk.Progressbar(frame,maximum=1);bar.pack(fill='x',pady=6)
    actions=ttk.Frame(frame);actions.pack(fill='x')
    events=queue.Queue(maxsize=1);done=queue.Queue();stop=threading.Event()
    running=False;closing=False

    def settings():
        values={k:float(v.get()) for k,(v,e) in entries.items()}
        cfg=Config(**{k:values[k] for k in asdict(Config())}).checked()
        bounds=[values[k] for k in ('diameter_min_mm','diameter_max_mm','diameter_step_mm')]
        diameter_values(*bounds)
        return cfg,bounds

    def update_estimate(*_):
        try:
            cfg,bounds=settings();n=len(diameter_values(*bounds));per_map=grid_size(cfg)
            estimate.set(f'{n:,} diameters x {per_map:,} poses = {n*per_map:,} total poses before filtering')
        except (ValueError,OverflowError):
            estimate.set('Enter valid positive values; diameter range must be divisible by its step.')
    for v,e in entries.values():v.trace_add('write',update_estimate)
    update_estimate()

    def report(p):
        try:events.put_nowait(p)
        except queue.Full:pass

    def start(resume=False):
        nonlocal running
        if resume:
            path=filedialog.askopenfilename(filetypes=[('Hugmap table JSON','*.json')])
            if not path:return
            try:
                with open(path,encoding='utf-8') as stream:document=json.load(stream)
                cfg=validate_table(document,for_resume=True)
                plan=document['metadata']['diameters_mm'];range_meta=document['metadata'].get('diameter_range',{})
                bounds=[plan[0],plan[-1],range_meta.get('step_mm',plan[1]-plan[0] if len(plan)>1 else 5.)]
                for key,value in dict(asdict(cfg),diameter_min_mm=bounds[0],diameter_max_mm=bounds[1],diameter_step_mm=bounds[2]).items():
                    entries[key][0].set(str(value))
            except (OSError,ValueError,TypeError,KeyError) as exc:
                messagebox.showerror('Cannot resume table',str(exc));return
        else:
            try:cfg,bounds=settings()
            except (ValueError,OverflowError) as exc:
                messagebox.showerror('Invalid input',str(exc));return
            path=filedialog.asksaveasfilename(defaultextension='.json',
                initialfile='hugmap-circle-table-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.json',
                filetypes=[('JSON','*.json')])
            if not path:return
        running=True;stop.clear();bar['value']=0
        start_button.config(state='disabled');resume_button.config(state='disabled');cancel_button.config(state='normal')
        for v,e in entries.values():e.config(state='disabled')
        state.set('Running. Checkpoints are saved automatically; Stop finalizes the same JSON.')
        def worker():
            try:
                result=run_table(cfg,path,*bounds,stop=stop,progress=report,resume=resume)
                done.put((result,path,None))
            except Exception as exc:done.put((None,path,str(exc)))
        threading.Thread(target=worker,daemon=True).start()

    def cancel():
        stop.set();state.set('Stopping workers and saving the table checkpoint...')

    def close():
        nonlocal closing
        if running:closing=True;cancel()
        else:root.destroy()

    def poll():
        nonlocal running
        try:
            p=events.get_nowait();bar['value']=p['visited']/p['total']
            state.set(f"{p['visited']:,}/{p['total']:,} | Diameter: {p['diameter_mm']:g} mm | "
                      f"Maps: {p['completed_maps']}/{p['total_maps']} | Found: {p['counts'].get('found',0):,} | {p['elapsed_s']:.0f}s")
        except queue.Empty:pass
        try:
            result,path,error=done.get_nowait();running=False
            start_button.config(state='normal');resume_button.config(state='normal');cancel_button.config(state='disabled')
            for v,e in entries.values():e.config(state='normal')
            if error:state.set('Error: '+error)
            else:
                bar['value']=result['visited']/result['total']
                state.set(f"{result['status']} | Maps: {result['completed_maps']}/{result['total_maps']} | "
                          f"Saved {result['visited']:,} poses\n{path}"+(f"\n{result['error']}" if result['error'] else ''))
            if closing:root.destroy();return
        except queue.Empty:pass
        root.after(150,poll)

    start_button=ttk.Button(actions,text='New table',command=start);start_button.pack(side='left')
    resume_button=ttk.Button(actions,text='Resume table',command=lambda:start(True));resume_button.pack(side='left',padx=8)
    cancel_button=ttk.Button(actions,text='Stop and save',command=cancel,state='disabled');cancel_button.pack(side='left')
    root.protocol('WM_DELETE_WINDOW',close);root.after(150,poll);root.mainloop()


def main():
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--run',action='store_true',help='legacy single 100 mm map')
    mode.add_argument('--table',action='store_true',help='generate a circle diameter table')
    mode.add_argument('--resume',type=Path,help='resume an existing table using its saved settings')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--dx-mm',type=float,default=20)
    parser.add_argument('--dy-mm',type=float,default=20)
    parser.add_argument('--dtheta-deg',type=float,default=5,help='object angle step; skipped for circle')
    parser.add_argument('--mu',type=float,default=.5,help='available coefficient; require mu_min < mu')
    parser.add_argument('--workers',type=int,default=None,help='CPU processes (default: automatic)')
    parser.add_argument('--diameter-min-mm',type=float,default=50)
    parser.add_argument('--diameter-max-mm',type=float,default=250)
    parser.add_argument('--diameter-step-mm',type=float,default=5)
    args=parser.parse_args()
    if not (args.run or args.table or args.resume):
        gui(); return
    if args.resume and args.output is not None:
        parser.error('--resume updates its own JSON; do not provide --output')
    if args.output is None and not args.resume:
        parser.error('--run/--table requires --output')
    cfg=Config(args.dx_mm,args.dy_mm,args.dtheta_deg,args.mu)
    try:
        cfg.checked()
        if not args.resume:
            multiplier=len(diameter_values(args.diameter_min_mm,args.diameter_max_mm,args.diameter_step_mm)) if args.table else 1
            print(f'{grid_size(cfg)*multiplier:,} grid poses before exclusion',flush=True)
        last=[0.]
        def progress(p):
            now=time.monotonic()
            if now-last[0]>2:
                print(f"{p['visited']}/{p['total']} {p['counts']}",flush=True); last[0]=now
        if args.table or args.resume:
            summary=run_table(cfg,args.resume or args.output,args.diameter_min_mm,args.diameter_max_mm,
                args.diameter_step_mm,progress=progress,workers=args.workers,resume=bool(args.resume))
        else:
            summary=run_map(cfg,args.output,progress=progress,workers=args.workers)
        print(json.dumps(summary,ensure_ascii=False))
        if summary['status']=='failed':
            raise SystemExit(1)
        if summary['status']=='cancelled':
            raise SystemExit(130)
    except (ValueError,OSError) as exc:
        parser.exit(1,str(exc)+'\n')


if __name__ == '__main__':
    main()
