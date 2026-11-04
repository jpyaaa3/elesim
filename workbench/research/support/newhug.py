#!/usr/bin/env python3
"""Independent planar newhug calculator using the repository's physical arm CAD.

Run:
    python3 newhug.py
    python3 newhug.py --describe-arm
    python3 newhug.py --solve --shape circle --width-mm 160 --budget 8 --output result.json
    python3 newhug.py --solve --record-needles --iterations 256 --mu .5 --output map.json
    python3 newhug.py --self-test

Python 3.10+, NumPy and SciPy; no EleSim, ROS, Genesis or CAD files are needed.
The ten 50 mm rigid nodes are controlled in two groups of five. The kinematics
and terminal mounting follow the ZED Mini arm URDF. Arm roll is fixed at +90
degrees: plot x is right (-world Y), plot y is forward (+world X), height omitted.
The full gripper includes its base and both jaws, projecting 89 mm beyond the
last-node tip. The base is rectangular; jaws retain their projected outlines.
The ZED Mini follows its URDF fixed joint and is a no-contact obstacle with
configurable clearance (default 1 mm). It cannot become a grasp contact.
Housing and wedge are also protected no-contact obstacles, with their own
clearance (default 1 mm). Only seg1, seg2, gripper base and claws may contact.
The housing uses a 130 x 233 mm footprint rectangle and the wedge retains its
84 x 28 mm outline. These are embedded frozen profiles; runtime needs no URDF,
mesh or STEP files. Motor limits remain the Robot-configured +/-36 deg per node;
the generated URDF's zero bend-limit placeholders do not set calculator limits.
Search combines robot-face support contacts with inverse contact fitting;
vertex contact normals come from valid support planes instead of only a few
sampled directions. Every candidate uses the same geometry and camera checks.
Both poses and needles require minimum mu < the configured available mu.
Needle collection records every eligible solution before top-result truncation.
Its iteration limit defaults to 256; an optional
wall-time limit and cancellation preserve partial results. Circle maps use
analytic disk/face tangencies. Gathering clips a consensus radial band and
rotates each remaining rail section onto the same polar ray, retaining its own
radius. It does not interpolate holes or test the GO2 body, legs or plate.

Score: critical common Coulomb friction coefficient of two planar point
contacts. Actual mu must be STRICTLY greater. Actuator capacity, compliance,
approach paths and actual holding are not certified. Placement is free in
this standalone plane; it is not an EleSim motion command.

Reference: Nguyen's planar antipodal condition, MLS section 5.4.2.
https://modernrobotics.northwestern.edu/nu-gm-book-resource/12-2-3-force-closure/
"""
from __future__ import annotations

import argparse
from collections import Counter
import copy
from dataclasses import asdict, dataclass
from functools import lru_cache
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import itertools
import json
import math
from pathlib import Path
import secrets
import threading
import time
import traceback
import webbrowser
import warnings
import subprocess
try:
    import numpy as np
    from scipy.optimize import least_squares, minimize
    from scipy.spatial import cKDTree
except ImportError as exc:
    raise SystemExit("newhug needs NumPy and SciPy in the selected Python environment: "+str(exc))

VERSION="0.6.1"
TAU=2*math.pi
MODEL_NOTE=("URDF ZED Mini arm, 10 rigid 50 mm nodes, two five-node groups, top-down at +90 degree roll. "
    "Full gripper with separately colored base and jaws. ZED Mini, housing and wedge are protected no-contact obstacles. "
    "Housing rectangle and CAD wedge. Free planar placement, ideal two-contact forces. "
    "Motor capacity, 3-D, approach, compression and actual holding are unverified.")

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


class SearchStopped(Exception):
    pass


def rotate(points, angle):
    c,s = math.cos(angle),math.sin(angle)
    return np.asarray(points,dtype=float) @ np.array([[c,s],[-s,c]])


def cross(a,b):
    a,b = np.asarray(a),np.asarray(b)
    return a[...,0]*b[...,1]-a[...,1]*b[...,0]


def wrap(angle):
    return (float(angle)+math.pi)%TAU-math.pi


def line_angle(angle):
    return (float(angle)+math.pi/2)%math.pi-math.pi/2


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


def finite_score(score):
    return {key:(value if not isinstance(value,float) or math.isfinite(value) else None)
            for key,value in score.items()}


def closure_at_mu(score, mu):
    if not math.isfinite(mu) or mu<0:
        raise InputError("Friction coefficient must be finite and nonnegative.")
    threshold = score["mu_critical"]
    return bool(threshold is not None and math.isfinite(threshold) and mu>threshold)


@dataclass(frozen=True)
class SearchSettings:
    budget_s: float = 8.
    pair_limit: int = 96
    max_solves: int = 1200
    max_evaluations: int = 70
    results: int = 10
    operating_mu: float = .5
    contact_samples: int = 32
    tolerance_m: float = .00005
    camera_clearance_m: float = .001
    body_clearance_m: float = .001

    def checked(self):
        if not math.isfinite(self.budget_s) or not .05<=self.budget_s<=120:
            raise InputError("Search time must be between 0.05 and 120 seconds.")
        if not math.isfinite(self.operating_mu) or not 0<=self.operating_mu<=1000:
            raise InputError("Friction cutoff must be in [0,1000].")
        if any(type(getattr(self,k)) is not int or not 1<=getattr(self,k)<=5000
               for k in ("pair_limit","max_solves","max_evaluations","results","contact_samples")):
            raise InputError("Search counts must be positive integers, at most 5000.")
        if self.results>50 or self.contact_samples>128 or self.pair_limit>512:
            raise InputError("At most 50 results, 128 boundary samples and 512 contact pairs.")
        if not math.isfinite(self.camera_clearance_m) or not 0<=self.camera_clearance_m<=.05:
            raise InputError("Camera clearance must be between 0 and 50 mm; touching is always forbidden.")
        if not math.isfinite(self.body_clearance_m) or not 0<=self.body_clearance_m<=.05:
            raise InputError("Body clearance must be between 0 and 50 mm; touching is always forbidden.")
        if not math.isfinite(self.tolerance_m) or not 1e-7<=self.tolerance_m<=.001:
            raise InputError("Geometry tolerance must be between 0.0001 and 1 mm.")
        return self


@dataclass(frozen=True)
class NeedleSettings:
    """Offline map options; all solutions below the global mu cutoff are recorded."""
    iterations: int = 256
    seed: int = 0
    time_limit_s: float = 0.  # zero: finish iterations or stop explicitly
    linear_min_m: float = -260/1500
    linear_max_m: float = 0.
    neutral_hinge_forward_m: float = .343  # GO2 base -> plate .315 + wedge .028

    def checked(self):
        if type(self.iterations) is not int or not 1<=self.iterations<=5000:
            raise InputError("Map iterations must be an integer in [1,5000].")
        if type(self.seed) is not int or not 0<=self.seed<2**32:
            raise InputError("Map seed must be an integer in [0,2**32).")
        if not math.isfinite(self.time_limit_s) or not 0<=self.time_limit_s<=3600:
            raise InputError("Map time limit must be 0 (off) or up to 3600 seconds.")
        if not all(math.isfinite(v) for v in (self.linear_min_m,self.linear_max_m,self.neutral_hinge_forward_m)):
            raise InputError("Rail parameters must be finite.")
        if not -1<=self.linear_min_m<self.linear_max_m<=0 or not 0<=self.neutral_hinge_forward_m<=1:
            raise InputError("Rail requires -1 <= min < max <= 0 and hinge offset in [0,1] metres.")
        return self


def radial_interval(segment,centre=(0.,0.)):
    a,b=np.asarray(segment,dtype=float)-np.asarray(centre,dtype=float)
    d=b-a;t=float(np.clip(-np.dot(a,d)/np.dot(d,d),0,1))
    return [float(np.linalg.norm(a+t*d)),float(max(np.linalg.norm(a),np.linalg.norm(b)))]


def make_needle(candidate,settings,identity,*,operating_mu=.5):
    settings.checked()
    if not candidate["geometry"]["valid"] or not closure_at_mu(candidate["friction"],operating_mu):
        return None
    hinge=np.asarray(candidate["base_xy_m"],dtype=float)
    heading=float(candidate["heading_rad"]);forward=np.array([math.cos(heading),math.sin(heading)])
    ends=[hinge-(settings.neutral_hinge_forward_m+linear)*forward
          for linear in (settings.linear_min_m,settings.linear_max_m)]
    keys=("turns_rad","node_bends_deg","contact_mode","contact_surfaces","contacts_m",
          "inward_normals","friction","geometry")
    row=copy.deepcopy({key:candidate[key] for key in keys})
    row.update(id=identity,hinge_xy_m=hinge.tolist(),heading_rad=heading,
               heading_deg=math.degrees(heading),segment_xy_m=np.asarray(ends).tolist(),
               linear_limits_m=[settings.linear_min_m,settings.linear_max_m],
               neutral_hinge_forward_m=settings.neutral_hinge_forward_m,
               radial_interval_m=radial_interval(ends),actual_hold_verified=None,
               approach_verified=None,go2_body_collision_verified=None)
    return row


def rotate_needle(needle,angle,centre=(0.,0.)):
    row=copy.deepcopy(needle);centre=np.asarray(centre,dtype=float)
    for key in ("hinge_xy_m","segment_xy_m","contacts_m"):
        row[key]=(rotate(np.asarray(needle[key])-centre,angle)+centre).tolist()
    row["inward_normals"]=rotate(needle["inward_normals"],angle).tolist()
    row["heading_rad"]=wrap(needle["heading_rad"]+angle)
    row["heading_deg"]=math.degrees(row["heading_rad"])
    return row


def segment_circle_intersections(segment,centre,radius):
    if not math.isfinite(radius) or radius<=0:raise InputError("Gather radius must be positive and finite.")
    a,b=np.asarray(segment,dtype=float);centre=np.asarray(centre,dtype=float)
    d=b-a;length2=float(d@d)
    if length2<=1e-20:raise InputError("Needle must have a nonzero finite rail segment.")
    t=float((centre-a)@d/length2);foot=a+t*d
    delta=radius*radius-float((foot-centre)@(foot-centre))
    eps=1e-12*max(radius*radius,length2,1e-6)
    if delta < -eps:return []
    dt=0. if abs(delta)<=eps else math.sqrt(max(0.,delta)/length2)
    roots=[]
    for value in (t-dt,t+dt):
        if -1e-10<=value<=1+1e-10:
            value=float(np.clip(value,0,1))
            if not roots or abs(roots[-1][0]-value)>1e-10:roots.append((value,a+value*d))
    return roots


def suggest_gather_band(intervals,support_fraction=.8):
    """Connected radial band supported by at least a fraction of peak overlap.

    Outlying disconnected modes are kept in the source map, not stretched into
    the chosen band. Coverage counts are sample support, not grasp success rates.
    """
    if not math.isfinite(support_fraction) or not 0<support_fraction<=1:
        raise InputError("Band support fraction must be in (0,1].")
    events={}
    for low,high in intervals:
        events.setdefault(float(low),[0,0])[0]+=1
        events.setdefault(float(high),[0,0])[1]+=1
    positions=sorted(events);active=0;profile=[]
    for i,position in enumerate(positions):
        starts,ends=events[position];active+=starts-ends
        if i+1<len(positions):
            profile.append([position,positions[i+1],active])
    peak=max((p[2] for p in profile),default=0)
    required=max(1,math.ceil(peak*support_fraction-1e-12));components=[]
    for low,high,count in profile:
        if count<required:continue
        if components and abs(components[-1][1]-low)<1e-12:
            components[-1][1]=high;components[-1][2]=max(components[-1][2],count)
            components[-1][3]+=(high-low)*count
        else:components.append([low,high,count,(high-low)*count])
    best=max(components,key=lambda p:(p[2],p[3],p[1]-p[0]),default=None)
    return dict(band_m=best[:2] if best else None,support_fraction=support_fraction,
                peak_support=peak,minimum_support=required,coverage_profile=profile)


def clip_segment_to_band(segment,centre,band):
    low,high=band;cuts=[0.,1.]
    for radius in (low,high):
        if radius>0:cuts.extend(t for t,_ in segment_circle_intersections(segment,centre,radius))
    cuts=sorted(set(round(t,13) for t in cuts));a,b=np.asarray(segment);centre=np.asarray(centre)
    pieces=[]
    for first,last in zip(cuts,cuts[1:]):
        distance=np.linalg.norm(a+(first+last)/2*(b-a)-centre)
        if low-1e-12<=distance<=high+1e-12 and last-first>1e-12:
            if pieces and abs(pieces[-1][1]-first)<1e-12:pieces[-1]=(pieces[-1][0],last)
            else:pieces.append((first,last))
    return pieces


def gather_needles(needle_map,*,band_m=None,support_fraction=.8,anchor_angle_deg=-90.):
    target=needle_map["target"]
    if target["kind"]!="circle":raise InputError("Gathering requires an explicit exact circle target.")
    if not math.isfinite(anchor_angle_deg):raise InputError("Anchor angle must be finite.")
    centre=np.asarray(target["centre_xy_m"],dtype=float)
    limit=needle_map.get("filter",{}).get("maximum",SearchSettings().operating_mu)
    eligible=[n for n in needle_map["needles"] if closure_at_mu(n["friction"],limit)]
    consensus=suggest_gather_band([n["radial_interval_m"] for n in eligible],support_fraction)
    automatic=band_m is None
    if automatic:band_m=consensus["band_m"]
    if band_m is not None and (len(band_m)!=2 or not all(math.isfinite(v) for v in band_m) or not 0<=band_m[0]<band_m[1]):
        raise InputError("Gather band requires finite 0 <= inner < outer radii.")
    angle=math.radians(anchor_angle_deg)
    direction=np.array([math.cos(angle),math.sin(angle)])
    rows={};intersecting=0;intersections=0;duplicates=0
    for needle in eligible:
        pieces=clip_segment_to_band(needle["segment_xy_m"],centre,band_m) if band_m is not None else []
        if pieces:intersecting+=1
        for branch,(first,last) in enumerate(pieces):
            intersections+=1
            fraction=(first+last)/2
            a,b=np.asarray(needle["segment_xy_m"]);point=a+fraction*(b-a)
            radius=float(np.linalg.norm(point-centre));anchor=centre+radius*direction
            rotation=wrap(angle-math.atan2(*(point-centre)[::-1]))
            row=rotate_needle(needle,rotation,centre)
            linear=(1-fraction)*needle["linear_limits_m"][0]+fraction*needle["linear_limits_m"][1]
            key=tuple(np.round(np.r_[row["turns_rad"],math.cos(row["heading_rad"]),math.sin(row["heading_rad"]),linear],8))
            key+=tuple((c["part"],c["edge"],round(c["fraction"],7)) for c in row["contact_surfaces"])
            if key in rows:
                rows[key]["symmetry_source_count"]+=1;duplicates+=1;continue
            row.update(id=f'{needle["id"]}:{branch}',source_id=needle["id"],rotation_rad=rotation,
                       segment_fraction=fraction,selected_linear_m=linear,parked_xy_m=anchor.tolist(),
                       anchor_radius_m=radius,band_segment_fraction=[first,last],
                       band_segment_xy_m=(rotate(np.array([a+first*(b-a),a+last*(b-a)])-centre,rotation)+centre).tolist(),
                       band_linear_limits_m=[(1-t)*needle["linear_limits_m"][0]+t*needle["linear_limits_m"][1] for t in (first,last)],
                       symmetry_source_count=1)
            rows[key]=row
    return dict(band_m=list(band_m) if band_m is not None else None,automatic_band=automatic,
                consensus=consensus,anchor_angle_deg=anchor_angle_deg,
                band_anchor_xy_m=[(centre+r*direction).tolist() for r in band_m] if band_m is not None else [],
                input_needles=len(eligible),intersecting_needles=intersecting,
                outside_band=len(eligible)-intersecting,
                friction_cutoff_excluded=len(needle_map["needles"])-len(eligible),
                intersection_count=intersections,duplicate_rotations=duplicates,
                needles=list(rows.values()),interpolated=False)


def collect_needle_map(vertices,geometry=None,settings=SearchSettings(),*,
                       needle_settings=NeedleSettings(),circle_radius_m=None,cancelled=None,progress=None):
    geometry=Geometry() if geometry is None else geometry
    needle_settings.checked();needles=[]
    def record(candidate):
        row=make_needle(candidate,needle_settings,f'n{len(needles)+1}',operating_mu=settings.operating_mu)
        if row is not None:needles.append(row)
    def update(value):
        if progress:progress(dict(value,recorded_needles=len(needles),iterations_requested=needle_settings.iterations))
    result=solve(vertices,geometry,settings,cancelled=cancelled,progress=update,on_candidate=record,
                 circle_radius_m=circle_radius_m,iteration_limit=needle_settings.iterations,
                 time_limit_s=needle_settings.time_limit_s,random_seed=needle_settings.seed)
    result["needle_map"]=dict(schema_version=3,settings=asdict(needle_settings),
        filter=dict(field="mu_critical",operator="<",maximum=settings.operating_mu,
                    maximum_inclusive=False,finite_only=True),
        target=dict(kind="circle" if circle_radius_m is not None else "polygon",centre_xy_m=[0.,0.],radius_m=circle_radius_m),
        coordinates="Object-centred plot XY; needle endpoints are GO2 base positions at linear min/max.",
        model_scope=dict(excluded=["go2_body","go2_legs","plate"],arm_roll_deg=90.,
                         neutral_hinge_forward_m=needle_settings.neutral_hinge_forward_m,
                         allowed_contact_groups=list(CONTACT_GROUPS),protected_parts=list(PROTECTED_PART_NAMES),
                         body_clearance_m=settings.body_clearance_m),
        iterations_requested=needle_settings.iterations,iterations_completed=result["search"]["attempts"],
        needles=needles)
    if circle_radius_m is not None:result["needle_map"]["gathered"]=gather_needles(result["needle_map"])
    return result


def filter_result_at_mu(result,mu,*,gather_options=None):
    """Refilter already discovered solutions; never recompute or invent a pose."""
    SearchSettings(operating_mu=mu).checked()
    value=copy.deepcopy(result)
    value["search"].setdefault("search_mu_limit",result["settings"]["operating_mu"])
    value["settings"]["operating_mu"]=mu
    value["candidates"]=[c for c in value["candidates"] if closure_at_mu(c["friction"],mu)]
    value["search"]["returned_candidates"]=len(value["candidates"])
    value["search"]["active_mu_cutoff"]=mu
    value["search"]["additional_search_needed"]=mu>value["search"]["search_mu_limit"]
    if value["search"]["status"]!="cancelled":
        value["search"]["status"]="candidates_found" if value["candidates"] else "none_below_mu_cutoff"
    if "needle_map" in value:
        m=value["needle_map"];m["filter"]["maximum"]=mu
        m["needles"]=[n for n in m["needles"] if closure_at_mu(n["friction"],mu)]
        if m["target"]["kind"]=="circle":
            old=gather_options or m.get("gathered",{})
            m["gathered"]=gather_needles(m,band_m=None if old.get("automatic_band",True) else old.get("band_m"),
                support_fraction=old.get("consensus",{}).get("support_fraction",.8),
                anchor_angle_deg=old.get("anchor_angle_deg",-90.))
    return value


def point_segment_distance(points, start, end):
    p = np.asarray(points,dtype=float)
    a,b = np.asarray(start,dtype=float),np.asarray(end,dtype=float)
    delta = b-a
    fraction = np.clip(np.sum((p-a)*delta,axis=-1)/np.maximum(np.sum(delta*delta,axis=-1),1e-30),0,1)
    return np.linalg.norm(p-a-fraction[...,None]*delta,axis=-1)


def segment_distances(a,b,c,d):
    """Broadcastable exact distance between closed planar segments."""
    a,b,c,d = map(np.asarray,(a,b,c,d))
    distance = np.minimum.reduce([point_segment_distance(a,c,d),point_segment_distance(b,c,d),
                                 point_segment_distance(c,a,b),point_segment_distance(d,a,b)])
    ab,cd = b-a,d-c
    denominator = cross(ab,cd)
    valid = np.abs(denominator)>1e-15
    safe = np.where(valid,denominator,1.)
    t,u = cross(c-a,cd)/safe,cross(c-a,ab)/safe
    intersects = valid & (t>=-1e-12) & (t<=1+1e-12) & (u>=-1e-12) & (u<=1+1e-12)
    return np.where(intersects,0.,distance)


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


def polygon_area_centroid(vertices):
    following = np.roll(vertices,-1,axis=0)
    weight = cross(vertices,following)
    area = float(np.sum(weight)/2)
    center = np.sum((vertices+following)*weight[:,None],axis=0)/(6*area)
    return area,center


def clean_polygon(vertices):
    p = np.asarray(vertices,dtype=float)
    if p.ndim!=2 or p.shape[1]!=2 or not 3<=len(p)<=128 or not np.isfinite(p).all():
        raise InputError("A polygon needs 3..128 finite [x,y] points.")
    if np.max(np.abs(p))>10:
        raise InputError("Target coordinates are metres, within +/-10 m.")
    if np.linalg.norm(p[0]-p[-1])<1e-10:
        p=p[:-1]
    if len(p)<3 or np.any(np.linalg.norm(np.roll(p,-1,axis=0)-p,axis=1)<1e-8):
        raise InputError("Repeated vertices / zero-length edges are not allowed.")
    n=len(p)
    i,j=np.triu_indices(n,1)
    keep=(j!=i+1)&~((i==0)&(j==n-1))
    if np.any(segment_distances(p[i[keep]],p[(i[keep]+1)%n],p[j[keep]],p[(j[keep]+1)%n])<1e-10):
        raise InputError("Target polygon self-intersects or has touching non-neighbour edges.")
    area,center=polygon_area_centroid(p)
    if abs(area)<1e-9:
        raise InputError("Target area is too small.")
    if area<0:
        p=p[::-1]
    return p-center


def template(name,width_m,height_m):
    if not all(math.isfinite(v) and .005<=v<=1 for v in (width_m,height_m)):
        raise InputError("Target width/height must be between 5 and 1000 mm.")
    angle=np.linspace(0,TAU,64,endpoint=False)
    if name=="circle":
        p=width_m/2*np.column_stack((np.cos(angle),np.sin(angle)))
    elif name=="ellipse":
        p=np.column_stack((width_m/2*np.cos(angle),height_m/2*np.sin(angle)))
    elif name=="rectangle":
        p=np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*[width_m/2,height_m/2]
    elif name=="triangle":
        p=np.array([[-.5,-1/3],[.5,-1/3],[0,2/3]])*[width_m,height_m]
    elif name=="jar":
        right=np.array([[.35,-.5],[.46,-.35],[.5,-.075],[.385,.2],
                        [.208,.325],[.208,.45],[.27,.45],[.27,.5]])
        p=np.vstack((right,np.column_stack((-right[::-1,0],right[::-1,1]))))*[width_m,height_m]
    else:
        raise InputError("Unknown template: "+str(name))
    return clean_polygon(p)


@dataclass(frozen=True)
class Feature:
    point: np.ndarray
    outward: np.ndarray
    tangent: float
    u: float


def boundary_features(vertices,count):
    edges=np.roll(vertices,-1,axis=0)-vertices
    lengths=np.linalg.norm(edges,axis=1)
    cumulative=np.r_[0,np.cumsum(lengths)]
    result=[]
    for distance in np.linspace(0,cumulative[-1],count,endpoint=False):
        i=min(int(np.searchsorted(cumulative,distance,side="right")-1),len(edges)-1)
        fraction=(distance-cumulative[i])/lengths[i]
        tangent=edges[i]/lengths[i]
        result.append(Feature(vertices[i]+fraction*edges[i],np.array([tangent[1],-tangent[0]]),
                              math.atan2(tangent[1],tangent[0]),distance/cumulative[-1]))
    for i,p in enumerate(vertices):
        before=edges[i-1]/lengths[i-1];after=edges[i]/lengths[i]
        delta=wrap(math.atan2(after[1],after[0])-math.atan2(before[1],before[0]))
        # Reflex vertices cannot carry a circular/capsule contact in their interior.
        if delta<=1e-8:continue
        steps=max(2,int(math.ceil(delta/math.radians(20))))
        for angle in np.linspace(math.atan2(before[1],before[0]),
                                 math.atan2(before[1],before[0])+delta,steps+1):
            result.append(Feature(p.copy(),np.array([math.sin(angle),-math.cos(angle)]),
                                  float(angle),cumulative[i]/cumulative[-1]))
    unique={}
    for f in result:
        unique[tuple(np.round(np.r_[f.point,f.outward],8))]=f
    return list(unique.values())


def select_pairs(features,settings,minimum_separation):
    p=np.array([f.point for f in features]);n=-np.array([f.outward for f in features])
    chord=p[None]-p[:,None]
    length=np.linalg.norm(chord,axis=2)
    direction=chord/np.maximum(length[:,:,None],1e-20)
    c1=np.sum(n[:,None]*direction,axis=2)
    c2=np.sum(n[None]*(-direction),axis=2)
    worst=np.arccos(np.clip(np.minimum(c1,c2),-1,1))
    valid=(length>=minimum_separation)&(worst<math.pi/2-1e-8)
    i,j=np.nonzero(valid)
    score=np.tan(worst[i,j])
    order=np.argsort(score,kind="stable")
    selected=[];used=set();buckets=set()
    # First keep good, spatially distinct pairs; reserve a quarter for exploration.
    for k in order:
        first,second=features[int(i[k])],features[int(j[k])]
        key=(int(first.u*24),int(second.u*24),
             round(first.tangent/math.radians(30)),round(second.tangent/math.radians(30)))
        if key in buckets:continue
        buckets.add(key);used.add(int(k))
        selected.append((first,second,float(score[k])))
        if len(selected)>=max(1,3*settings.pair_limit//4):break
    room=settings.pair_limit-len(selected)
    if room and len(order):
        available=[int(k) for k in order if int(k) not in used]
        for position in np.linspace(0,len(available)-1,min(room,len(available)),dtype=int):
            k=available[int(position)]
            selected.append((features[int(i[k])],features[int(j[k])],float(score[k])))
    selected.sort(key=lambda p:p[2])
    return selected



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


def contact_part_allowed(part):
    return bool(isinstance(part,(int,np.integer)) and 0<=part<len(PART_NAMES)
                and PART_NAMES[part] in ALLOWED_CONTACT_PART_NAMES)


def local_profiles(g):
    return [node_profile(g,i==9) for i in range(10)]+[
        np.asarray(REFERENCE["profiles"][name],dtype=float) for name in GRIPPER_PART_NAMES+("camera",)]


def describe_arm(g=Geometry()):
    g.checked();tool=gripper_profile();low,high=tool.min(0),tool.max(0)
    return dict(profile="zed-mini-arm-URDF-full-gripper-protected-camera",geometry=asdict(g),
        dimensions=dict(nodes=10,nodes_per_section=5,node_pitch_mm=g.pitch_m*1000,
            section_lengths_mm=[g.length1_m*1000,g.length2_m*1000],
            bending_plane_thickness_mm=2*g.half_thickness_m*1000,
            gripper_forward_mm=float(high[1]*1000),gripper_rear_overlap_mm=max(0.,float(-low[1]*1000)),
            gripper_lateral_bounds_mm=(np.array([low[0],high[0]])*1000).tolist(),
            gripper_span_mm=((high-low)*1000).tolist(),
            node_joint_limit_deg=g.joint_limit_deg,section_total_limit_deg=g.turn_limit_deg,
            straight_bend_origin_to_gripper_front_mm=(g.length1_m+g.length2_m+high[1])*1000,
            fixed_arm_roll_deg=REFERENCE["fixed_arm_roll_deg"],wedge_width_mm=84.,wedge_forward_mm=28.,
            body_width_mm=REFERENCE["body_model"]["width_m"]*1000,
            body_depth_mm=REFERENCE["body_model"]["depth_m"]*1000,
            body_front_to_bend_origin_mm=REFERENCE["body_model"]["mount_offset_m"]*1000),
        section_angle_convention="turns_rad/deg are TOTAL turns. Each of five node joints receives total/5.",
        reference_match=asdict(g)==asdict(Geometry()),
        local_profiles_m={name:p.tolist() for name,p in zip(PART_NAMES,local_profiles(g))},
        gripper_part_names=list(GRIPPER_PART_NAMES),protected_part_names=list(PROTECTED_PART_NAMES),
        allowed_contact_groups={name:list(parts) for name,parts in CONTACT_GROUPS.items()},
        allowed_contact_part_names=[name for name in PART_NAMES if name in ALLOWED_CONTACT_PART_NAMES],
        gripper_outline_m=tool.tolist(),camera_model=REFERENCE["camera_model"],
        collision_profiles_m={name:p.tolist() for name,p in zip(PART_NAMES,local_profiles(g))},
        fixed_body_profiles_m={name:REFERENCE["profiles"][name] for name in BODY_NAMES},
        body_model=REFERENCE["body_model"],gripper_model=REFERENCE["gripper_model"],
        projection_note="Top-down at fixed +90 degree arm roll, URDF joint/visual transforms. "
                        "Base and both jaws included; ZED Mini, housing and wedge are protected no-contact obstacles. "
                        "Finger opening is out of this plane; camera and body clearances default to 1 mm.",
        excluded_parts=REFERENCE["excluded"],source_sha256=REFERENCE["sources"],
        plane=REFERENCE["plane"],reference_plane=REFERENCE["reference_plane"])


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


def convex_parts(vertices):
    """Ear clipping keeps a concave target's empty notches empty."""
    p=np.asarray(vertices)
    while len(p)>3:
        edge1=p-np.roll(p,1,axis=0);edge2=np.roll(p,-1,axis=0)-p
        collinear=(np.abs(cross(edge1,edge2))<1e-13)&(np.sum(edge1*edge2,axis=1)>0)
        if not collinear.any():break
        p=p[~collinear]
    turns=cross(p-np.roll(p,1,axis=0),np.roll(p,-1,axis=0)-p)
    if np.min(turns)>=-1e-12:return (p,)
    ids=list(range(len(p)));triangles=[]
    for _ in range(len(p)**2):
        if len(ids)==3:
            triangles.append(p[ids]);return tuple(triangles)
        clipped=False
        for k,i in enumerate(ids):
            j,h=ids[k-1],ids[(k+1)%len(ids)]
            a,b,c=p[[j,i,h]]
            if cross(b-a,c-b)<=1e-13:continue
            other=[v for v in ids if v not in (j,i,h)]
            v=p[other]
            inside=(cross(b-a,v-a)>=-1e-12)&(cross(c-b,v-b)>=-1e-12)&(cross(a-c,v-c)>=-1e-12)
            if inside.any():continue
            triangles.append(p[[j,i,h]]);ids.pop(k);clipped=True;break
        if not clipped:break
    raise InputError("Cannot triangulate the target polygon.")


def target_gap(poly,parts):
    return min(convex_gap(poly,part) for part in parts)


def protected_target_gap(polygon,vertices,target_parts,*,circle_radius_m=None):
    """Signed penetration/touch; Euclidean boundary distance when separated."""
    if circle_radius_m is not None:return circle_gap(polygon,circle_radius_m)
    gap=target_gap(polygon,target_parts)
    if gap<=1e-10:return min(gap,0.)
    return float(np.min(segment_distances(polygon[:,None],np.roll(polygon,-1,axis=0)[:,None],
        vertices[None],np.roll(vertices,-1,axis=0)[None])))


def protected_camera_gap(camera,vertices,target_parts):
    return protected_target_gap(camera,vertices,target_parts)


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


@dataclass(frozen=True)
class Face:
    part: int
    edge: int
    first: np.ndarray
    second: np.ndarray
    normal: np.ndarray
    tangent: float


@lru_cache(maxsize=2)
def robot_faces(g):
    result=[];profiles=local_profiles(g)[:CAMERA_INDEX]
    for i,poly in enumerate(profiles[:10]):
        for edge,(a,b,n) in enumerate(zip(poly,np.roll(poly,-1,axis=0),polygon_normals(poly))):
            result.append(Face(i,edge,a,b,n,math.atan2(*(b-a)[::-1])))
    outline=gripper_profile()
    # Select only the exposed union boundary, trimming partly hidden rectangle
    # edges. Internal mounting seams can never become candidate contacts.
    for first,last,n in zip(outline,np.roll(outline,-1,axis=0),polygon_normals(outline)):
        delta=last-first;cuts=[0.,1.]
        for p in np.vstack(profiles[10:]):
            t=float(np.dot(p-first,delta)/np.dot(delta,delta))
            if 1e-9<t<1-1e-9 and np.linalg.norm(p-first-t*delta)<1e-9:cuts.append(t)
        cuts=sorted(set(round(t,12) for t in cuts))
        for low,high in zip(cuts,cuts[1:]):
            a,b=first+low*delta,first+high*delta;found=False
            for i in range(10,len(profiles)):
                poly=profiles[i]
                for edge,(c,d,normal) in enumerate(zip(poly,np.roll(poly,-1,axis=0),polygon_normals(poly))):
                    if np.dot(n,normal)>1-1e-9 and np.max(point_segment_distance([a,b],c,d))<1e-9:
                        result.append(Face(i,edge,a,b,n,math.atan2(*(b-a)[::-1])));found=True;break
                if found:break
            if not found:raise InputError("URDF gripper outline does not match its collision pieces.")
    return tuple(f for f in result if contact_part_allowed(f.part))


def placed_face(face,hinges,angles,fraction):
    angle=float(angles[min(face.part,10)])
    point=hinges[min(face.part,10)]+rotate(face.first+fraction*(face.second-face.first),angle)
    normal=rotate(face.normal,angle)
    return point,normal,face.tangent+angle


def contact_frame(values,first,second,g):
    hinges,angles=fk_frames(values[:2],g)
    p1,n1,t1=placed_face(first,hinges,angles,float(values[2]))
    p2,n2,t2=placed_face(second,hinges,angles,float(values[3]))
    return p1,p2,n1,n2,t1,t2


def intrinsic_signature(p1,p2,n1,n2,scale):
    delta=p2-p1
    return np.stack((np.sum(delta*n1,axis=-1)/scale,cross(n1,delta)/scale,
                     np.sum(n1*n2,axis=-1),cross(n1,n2)),axis=-1)


@lru_cache(maxsize=1)
def contact_atlas(g):
    """Small in-memory seed atlas. No files, training, or result cache."""
    began=time.monotonic();faces=robot_faces(g)
    first_ids=np.array([i for i,f in enumerate(faces) if f.part<9],dtype=int)
    second_ids=np.array([i for i,f in enumerate(faces) if f.part>=1],dtype=int)
    rows=[];parameters=[];identities=[];configs=0
    limit=math.radians(g.turn_limit_deg)
    for q1,q2 in itertools.product(np.linspace(-limit,limit,11),repeat=2):
        surfaces,bodies,hinges,angles=robot_polygons((q1,q2),g)
        if np.min(self_gaps(surfaces,bodies))<-.00002:continue
        configs+=1
        points=np.array([placed_face(f,hinges,angles,.5)[0] for f in faces])
        normals=np.array([rotate(f.normal,angles[min(f.part,10)]) for f in faces])
        p1=points[first_ids,None];p2=points[None,second_ids]
        n1=normals[first_ids,None];n2=normals[None,second_ids]
        delta=p2-p1;length=np.linalg.norm(delta,axis=2)
        mask=(length>.005)&(np.sum(n1*delta,axis=2)>1e-8)&(np.sum(-n2*delta,axis=2)>1e-8)
        mask &= (np.array([faces[k].part for k in first_ids])[:,None]
                 < np.array([faces[k].part for k in second_ids])[None,:])
        i,j=np.nonzero(mask)
        if not len(i):continue
        sig=intrinsic_signature(p1[i,0],p2[0,j],n1[i,0],n2[0,j],g.length1_m)
        rows.append(sig);parameters.extend([(q1,q2)]*len(i))
        identities.extend(zip(first_ids[i].tolist(),second_ids[j].tolist()))
    if not rows:raise InputError("No collision-free seed configurations for this geometry.")
    table=np.vstack(rows)
    return dict(tree=cKDTree(table),parameters=np.asarray(parameters),faces=np.asarray(identities,dtype=int),
                rows=len(table),configurations=configs,build_s=time.monotonic()-began)


def circle_gap(polygon,radius,centre=(0.,0.)):
    return float(signed_distances(np.asarray([centre]),polygon)[0])-radius


def validate_configuration(vertices,target_parts,turns,rotation,translation,face1,face2,a,b,features,g,settings,
                           *,circle_radius_m=None):
    g.checked();settings.checked()
    surfaces,bodies,hinges,angles=robot_polygons(turns,g)
    node_bends_deg=np.degrees(np.asarray(turns,dtype=float))/g.nodes_per_section
    joint_headroom=float(g.joint_limit_deg-np.max(np.abs(node_bends_deg)))
    # Ignore degree/radian conversion round-off, never geometric tolerances.
    if abs(joint_headroom)<=1e-9:joint_headroom=0.
    # Keep the target fixed in its own coordinates for all exact polygon checks.
    world_surfaces=[rotate(p-translation,-rotation) for p in surfaces]
    world_bodies=[rotate(p-translation,-rotation) for p in bodies]
    gap=(lambda p:circle_gap(p,circle_radius_m)) if circle_radius_m is not None else (lambda p:target_gap(p,target_parts))
    gaps=np.array([gap(p) for p in world_surfaces])
    body_gaps=np.array([gap(p) for p in world_bodies])
    intrinsic=self_gaps(surfaces,bodies)
    reasons=[]
    if joint_headroom<0.:reasons.append("joint_limit_exceeded")
    if gaps.min()<-settings.tolerance_m:reasons.append("target_penetration")
    if body_gaps.min()<-settings.tolerance_m:reasons.append("body_target_collision")
    if intrinsic.min()<-settings.tolerance_m:reasons.append("robot_self_collision")
    camera_gap=(float(gaps[CAMERA_INDEX]) if circle_radius_m is not None else
                protected_camera_gap(world_surfaces[CAMERA_INDEX],vertices,target_parts))
    if camera_gap<=settings.camera_clearance_m+1e-9:reasons.append("camera_clearance")
    if camera_gap<=1e-9:reasons.append("camera_contact")
    protected_bodies=np.array([protected_target_gap(p,vertices,target_parts,circle_radius_m=circle_radius_m) for p in world_bodies])
    if protected_bodies.min()<=settings.body_clearance_m+1e-9:reasons.append("body_clearance")
    if protected_bodies.min()<=1e-9:reasons.append("body_contact")
    if not all(contact_part_allowed(f.part) for f in (face1,face2)):
        reasons.append("forbidden_contact_surface");errors=align=[None,None]
    else:
        p1,n1,t1=placed_face(face1,hinges,angles,a)
        p2,n2,t2=placed_face(face2,hinges,angles,b)
        expected=np.array([rotate(f.point,rotation)+translation for f in features])
        errors=np.linalg.norm(np.array([p1,p2])-expected,axis=1)
        if errors.max()>2*settings.tolerance_m:reasons.append("intended_contact_missing")
        target_in=rotate(-np.array([f.outward for f in features]),rotation)
        align=np.sum(np.array([n1,n2])*target_in,axis=1)
        if np.min(align)<1-1e-6:reasons.append("contact_normal_mismatch")
        errors=errors.tolist();align=align.tolist()
    if circle_radius_m is not None and any(abs(np.linalg.norm(f.point)-circle_radius_m)>1e-8 for f in features):
        reasons.append("intended_contact_not_on_circle")
    metrics=dict(valid=not reasons,reasons=reasons,minimum_object_gap_m=float(gaps.min()),
        camera_target_clearance_m=camera_gap,required_camera_clearance_m=settings.camera_clearance_m,
        body_target_clearance_m=float(protected_bodies.min()),
        body_target_clearances_m={name:float(gap) for name,gap in zip(BODY_NAMES,protected_bodies)},
        required_body_clearance_m=settings.body_clearance_m,minimum_self_gap_m=float(intrinsic.min()),
        contact_errors_m=errors,normal_alignment=align,
        geometry_tolerance_m=settings.tolerance_m,
        minimum_bend_headroom_deg=joint_headroom*g.nodes_per_section,
        minimum_joint_headroom_deg=joint_headroom)
    return metrics,world_surfaces,world_bodies,hinges


def support_contact_seeds(vertices,g,settings,stopped,counts,max_seeds=32):
    """Build exact polygon contacts from robot face normals and support vertices.

    Target points/normals come from support planes, including the full normal
    cone at a vertex. Every seed still passes the usual collision/camera check.
    """
    faces=robot_faces(g);parts=convex_parts(vertices)
    starts=np.array([f.first for f in faces]);ends=np.array([f.second for f in faces])
    local_normals=np.array([f.normal for f in faces]);part=np.array([f.part for f in faces])
    frame=np.minimum(part,10)
    ia,ib=np.triu_indices(len(faces),1)
    allowed=(part[ia]<9)&(part[ia]<part[ib])
    ia,ib=ia[allowed],ib[allowed]
    extent=np.ptp(vertices,axis=0);area,_=polygon_area_centroid(vertices)
    radii=sorted(set(round(float(r),8) for r in (min(extent)/2,max(extent)/2,math.sqrt(area/math.pi))))
    bends=[];seen=set()
    for radius in radii:
        angle=math.degrees(2*math.atan(g.pitch_m/(2*(radius+g.half_thickness_m))))
        for magnitude in (round(angle),angle+3,angle,angle-3,angle+.5,angle-.5,angle+1,angle-1):
            if not 0<magnitude<=g.joint_limit_deg:continue
            for sign in (-1,1):
                bend=round(sign*magnitude,8)
                if bend not in seen:seen.add(bend);bends.append(bend)
    for bend in (-g.joint_limit_deg,g.joint_limit_deg):
        if bend not in seen:bends.append(bend)
    first_bends=[0.]
    for bend in np.linspace(g.joint_limit_deg/6,g.joint_limit_deg,6):
        first_bends.extend((-float(bend),float(bend)))
    length2=np.sum((ends-starts)**2,axis=1)
    emitted=0
    # Interleave proximal bend and distal curvature instead of spending all
    # seeds at one curvature. The same budget can cover distinct grasp families.
    for diagonal in range(len(bends)+len(first_bends)-1):
        for second_index in range(len(bends)):
            first_index=diagonal-second_index
            if not 0<=first_index<len(first_bends):continue
            second_bend,first_bend=bends[second_index],first_bends[first_index]
            if stopped() or emitted>=max_seeds:return
            counts["support_seed_poses"]+=1
            q=np.radians([first_bend,second_bend])*g.nodes_per_section
            surfaces,bodies,hinges,angles=robot_polygons(q,g)
            if self_gaps(surfaces,bodies).min()<-settings.tolerance_m:continue
            a=angles[frame];c,s=np.cos(a),np.sin(a)
            def placed(p):
                return np.column_stack((p[:,0]*c-p[:,1]*s,p[:,0]*s+p[:,1]*c))
            world_a=placed(starts)+hinges[frame];world_b=placed(ends)+hinges[frame]
            normal=placed(local_normals);edge=world_b-world_a
            n1,n2=normal[ia],normal[ib];det=cross(n1,n2)
            nonparallel=np.abs(det)>1e-6;safe=np.where(nonparallel,det,1.)
            per_pose=0
            for rotation in np.linspace(0,TAU,12,endpoint=False):
                if stopped():return
                target=rotate(vertices,rotation);target_parts=[rotate(p,rotation) for p in parts]
                support_ids=np.argmin(normal@target.T,axis=1);support=target[support_ids]
                rhs=np.sum(normal*(world_a-support),axis=1);r1,r2=rhs[ia],rhs[ib]
                translations=np.column_stack(((r1*n2[:,1]-n1[:,1]*r2)/safe,
                    (n1[:,0]*r2-r1*n2[:,0])/safe))
                p1,p2=support[ia]+translations,support[ib]+translations
                f1=np.sum((p1-world_a[ia])*edge[ia],axis=1)/length2[ia]
                f2=np.sum((p2-world_a[ib])*edge[ib],axis=1)/length2[ib]
                chord=p2-p1;distance=np.linalg.norm(chord,axis=1)
                direction=chord/np.maximum(distance[:,None],1e-20)
                cosine=np.minimum(np.sum(n1*direction,axis=1),np.sum(-n2*direction,axis=1))
                valid=nonparallel&(f1>=-1e-10)&(f1<=1+1e-10)&(f2>=-1e-10)&(f2<=1+1e-10)
                valid &= (distance>max(.001,.04*np.linalg.norm(extent)))&(cosine>1e-8)
                rows=np.flatnonzero(valid)
                if not len(rows):continue
                translations_valid=translations[rows];keep=np.ones(len(rows),dtype=bool)
                # Batched SAT is the same convex separation test as convex_gap.
                # Concave targets are checked against every triangulated part.
                for poly in surfaces+bodies:
                    pa_normals=polygon_normals(poly)
                    for target_part in target_parts:
                        axes=np.vstack((pa_normals,polygon_normals(target_part)))
                        pa,pb=poly@axes.T,target_part@axes.T;offset=translations_valid@axes.T
                        gap=np.maximum(pb.min(0)+offset-pa.max(0),
                                       pa.min(0)-pb.max(0)-offset).max(1)
                        keep &= gap>=-settings.tolerance_m
                    if not keep.any():break
                rows=rows[keep]
                if not len(rows):continue
                score=np.tan(np.arccos(np.clip(cosine[rows],-1,1)))
                for row in rows[np.argsort(score,kind="stable")]:
                    if stopped():return
                    translation=translations[row]
                    camera_gap=protected_camera_gap(surfaces[CAMERA_INDEX]-translation,target,target_parts)
                    if camera_gap<=settings.camera_clearance_m+1e-9:
                        counts["support_seed_camera_rejections"]+=1;continue
                    body_gap=min(protected_target_gap(p-translation,target,target_parts) for p in bodies)
                    if body_gap<=settings.body_clearance_m+1e-9:
                        counts["support_seed_body_rejections"]+=1;continue
                    first,second=int(ia[row]),int(ib[row])
                    out=rotate(-normal[[first,second]],-rotation)
                    target_features=tuple(Feature(vertices[support_ids[k]],out[i],
                        math.atan2(out[i,0],-out[i,1]),float(support_ids[k]/len(vertices)))
                        for i,k in enumerate((first,second)))
                    initial=np.r_[q,np.clip([f1[row],f2[row]],0,1)]
                    counts["support_seeds"]+=1;emitted+=1;per_pose+=1
                    yield (*target_features,faces[first],faces[second],initial,"support")
                    if emitted>=max_seeds or per_pose>=2:break
                if emitted>=max_seeds or per_pose>=2:break


def exact_circle_seeds(radius,g,settings,stopped,counts,iterations,random_seed):
    """Analytic disk/face tangencies, varied joint poses; no polygon symmetry assumption."""
    rng=np.random.default_rng(random_seed);faces=robot_faces(g)
    starts=np.array([f.first for f in faces]);ends=np.array([f.second for f in faces])
    normals=np.array([f.normal for f in faces]);parts=np.array([f.part for f in faces]);frame=np.minimum(parts,10)
    ia,ib=np.triu_indices(len(faces),1);keep=(parts[ia]<9)&(parts[ia]<parts[ib]);ia,ib=ia[keep],ib[keep]
    preferred=[(-12.,-33.),(-12.,-28.),(12.,33.),(24.,-36.),(-24.,36.),(-6.,-30.),(6.,30.)]
    for pose in range(max(2048,iterations*16)):
        if stopped():return
        counts["circle_pose_samples"]+=1
        bends=np.array(preferred[pose]) if pose<len(preferred) else rng.uniform(-g.joint_limit_deg,g.joint_limit_deg,2)
        if np.max(np.abs(bends))>g.joint_limit_deg:continue
        q=np.radians(bends)*g.nodes_per_section
        surfaces,bodies,hinges,angles=robot_polygons(q,g)
        if self_gaps(surfaces,bodies).min()<-settings.tolerance_m:continue
        a=angles[frame];c,s=np.cos(a),np.sin(a)
        def placed(p):return np.column_stack((c*p[:,0]-s*p[:,1],s*p[:,0]+c*p[:,1]))
        first=placed(starts)+hinges[frame];last=placed(ends)+hinges[frame];normal=placed(normals)
        na,nb=normal[ia],normal[ib];det=cross(na,nb);rhs=np.sum(normal*first,axis=1)+radius
        safe=np.where(abs(det)>1e-8,det,1.)
        centres=np.column_stack(((rhs[ia]*nb[:,1]-na[:,1]*rhs[ib])/safe,
                                 (na[:,0]*rhs[ib]-rhs[ia]*nb[:,0])/safe))
        feet_a=centres-radius*na;feet_b=centres-radius*nb
        da,db=last[ia]-first[ia],last[ib]-first[ib]
        fa=np.sum((feet_a-first[ia])*da,axis=1)/np.sum(da*da,axis=1)
        fb=np.sum((feet_b-first[ib])*db,axis=1)/np.sum(db*db,axis=1)
        # Distinct radial contacts have a finite threshold.
        dot=np.clip(np.sum(na*nb,axis=1),-1,1)
        valid=(abs(det)>1e-8)&(fa>=0)&(fa<=1)&(fb>=0)&(fb<=1)
        valid &= np.arccos(dot)>1e-8
        rows=np.flatnonzero(valid)
        if not len(rows):continue
        centres_kept=centres[rows];valid=np.ones(len(rows),dtype=bool)
        for part,poly in enumerate(surfaces+bodies):
            gaps=signed_distances(centres_kept,poly)-radius
            if part==CAMERA_INDEX:valid &= gaps>settings.camera_clearance_m+1e-9
            elif part>=len(surfaces):valid &= gaps>settings.body_clearance_m+1e-9
            else:valid &= gaps>=-settings.tolerance_m
            if not valid.any():break
        rows=rows[valid]
        if not len(rows):continue
        # Keep low-friction contacts represented, then explore other contact
        # modes. Uniform sampling alone wastes the cutoff budget on near-coincident contacts.
        dot_rows=dot[rows]
        required=np.sqrt(np.maximum(0.,(1+dot_rows)/np.maximum(1-dot_rows,1e-20)))
        ordered=rows[np.argsort(required,kind="stable")]
        selected_rows=np.r_[ordered[:2],rng.permutation(ordered[2:])[:2]]
        for row in selected_rows:
            if stopped():return
            rotation=float(rng.uniform(-math.pi,math.pi))
            features=[]
            for n in normal[[ia[row],ib[row]]]:
                point=rotate(-radius*n,-rotation);out=point/radius
                features.append(Feature(point,out,math.atan2(out[0],-out[1]),0.))
            counts["circle_analytic_seeds"]+=1
            yield (*features,faces[ia[row]],faces[ib[row]],np.r_[q,fa[row],fb[row]],"support")


def solve(vertices,geometry=Geometry(),settings=SearchSettings(),*,cancelled=None,progress=None,
          on_candidate=None,circle_radius_m=None,iteration_limit=None,time_limit_s=None,random_seed=0):
    geometry.checked();settings.checked();vertices=clean_polygon(vertices)
    if circle_radius_m is not None and (not math.isfinite(circle_radius_m) or not .0025<=circle_radius_m<=.5):
        raise InputError("Exact circle radius must be between 2.5 and 500 mm.")
    began=time.monotonic();seconds=settings.budget_s if time_limit_s is None else time_limit_s
    deadline=began+seconds if seconds else math.inf
    max_solves=settings.max_solves if iteration_limit is None else iteration_limit
    def stopped():return time.monotonic()>=deadline or bool(cancelled and cancelled())
    features=boundary_features(vertices,settings.contact_samples)
    extent=float(np.linalg.norm(np.ptp(vertices,axis=0)))
    pairs=select_pairs(features,settings,max(.001,.04*extent))
    parts=convex_parts(vertices);faces=robot_faces(geometry)
    counts=Counter();rejections=Counter();pool={};attempts=0;roots=0
    atlas=None;atlas_elapsed=0.;tasks=[]
    limit=math.radians(geometry.turn_limit_deg)
    low=np.array([-limit,-limit,0.,0.]);high=np.array([limit,limit,1.,1.])
    forward_deadline=min(deadline,began+min(3.,settings.budget_s*.35))
    def forward_stopped():return stopped() or time.monotonic()>=forward_deadline
    def inverse_tasks():
        nonlocal atlas,atlas_elapsed,tasks
        if stopped() or not pairs:return
        if progress:progress(dict(stage="기구 접촉 후보 준비",attempts=attempts,roots=roots,
            candidates=len(pool),elapsed_s=time.monotonic()-began))
        before=time.monotonic();atlas=contact_atlas(geometry);atlas_elapsed=time.monotonic()-before
        groups={}
        for pair_index,(f1,f2,_) in enumerate(pairs):
            if stopped():return
            signature=intrinsic_signature(f1.point,f2.point,-f1.outward,-f2.outward,geometry.length1_m)
            groups.setdefault(tuple(np.round(signature,10)),[]).append(pair_index)
            _,indices=atlas["tree"].query(signature,k=min(72,atlas["rows"]))
            chosen=[];seen=set()
            for row in np.atleast_1d(indices):
                a,b=atlas["faces"][row];q=atlas["parameters"][row]
                key=(int(a),int(b),round(math.degrees(q[0])/30),round(math.degrees(q[1])/30))
                if key in seen:continue
                seen.add(key);chosen.append(int(row))
                if len(chosen)>=12:break
            tasks.append(chosen)
        counts["distinct_contact_geometries"]=len(groups)
        # Preserve every target pair, while trying distinct geometries first.
        pair_order=[members[rank] for rank in range(max(map(len,groups.values())))
                    for members in groups.values() if rank<len(members)]
        for rank in range(12):
            for pair_index in pair_order:
                if stopped():return
                if rank>=len(tasks[pair_index]):continue
                row=tasks[pair_index][rank];first,other,_=pairs[pair_index]
                ia,ib=map(int,atlas["faces"][row]);initial=np.r_[atlas["parameters"][row],.5,.5]
                yield first,other,faces[ia],faces[ib],initial,"inverse"
    if progress and not stopped():
        progress(dict(stage="접촉 후보 직접 구성",attempts=0,roots=0,candidates=0,elapsed_s=0))
    seeds=(exact_circle_seeds(circle_radius_m,geometry,settings,stopped,counts,max_solves,random_seed)
           if circle_radius_m is not None else support_contact_seeds(vertices,geometry,settings,forward_stopped,counts))
    tasks_iter=seeds if circle_radius_m is not None else itertools.chain(seeds,inverse_tasks())
    for first,other,fa,fb,initial,method in itertools.islice(tasks_iter,max_solves):
        if stopped() or attempts>=max_solves:break
        _,_,_,_,t1,t2=contact_frame(initial,fa,fb,geometry)
        delta=wrap(other.tangent-first.tangent)
        winding=round((t2-t1-delta)/TAU)
        def frame(values):
            p1,p2,n1,n2,t1,t2=contact_frame(values,fa,fb,geometry)
            rotation=wrap(t1-first.tangent+math.pi)
            translation=p1-rotate(first.point,rotation)
            return p1,p2,rotation,translation,t1,t2
        def residual(values):
            if stopped():raise SearchStopped()
            p1,p2,rotation,_,t1,t2=frame(values)
            positional=(p2-p1-rotate(other.point-first.point,rotation))/geometry.length1_m
            return np.r_[positional,t2-t1-delta-winding*TAU]
        attempts+=1
        if progress and attempts%4==0:
            progress(dict(stage="접촉·관통 검사",attempts=attempts,roots=roots,candidates=len(pool),
                best_mu=min((c["friction"]["mu_critical"] for c in pool.values()),default=None),
                elapsed_s=time.monotonic()-began))
        try:
            if method=="support":
                values=initial.copy();error=residual(values)
            else:
                counts["inverse_fits"]+=1
                fit=least_squares(residual,np.clip(initial,low+1e-9,high-1e-9),bounds=(low,high),
                    max_nfev=settings.max_evaluations,ftol=1e-9,xtol=1e-9,gtol=1e-9)
                values=fit.x;error=fit.fun
        except SearchStopped:break
        if np.max(np.abs(error))>1e-6:continue
        roots+=1
        _,_,rotation,translation,_,_=frame(values)
        metrics,world_surfaces,world_bodies,hinges=validate_configuration(
            vertices,parts,values[:2],rotation,translation,fa,fb,values[2],values[3],
            (first,other),geometry,settings,circle_radius_m=circle_radius_m)
        if not metrics["valid"] and not stopped() and circle_radius_m is None:
            def collision_cost(v):
                if stopped():raise SearchStopped()
                _,_,r,tr,_,_=frame(v)
                surfaces,bodies,_,_=robot_polygons(v[:2],geometry)
                world=[rotate(p-tr,-r) for p in surfaces+bodies]
                gaps=np.array([target_gap(p,parts) for p in world])
                self_gap=self_gaps(surfaces,bodies)
                camera_gap=protected_camera_gap(world[CAMERA_INDEX],vertices,parts)
                protected=[min(0.,camera_gap-settings.camera_clearance_m-1e-9)]
                protected.extend(min(0.,protected_target_gap(p,vertices,parts)-settings.body_clearance_m-1e-9)
                                 for p in world[len(surfaces):])
                violation=np.minimum(np.r_[gaps,self_gap,protected]/geometry.length1_m,0)
                return float(violation@violation)+1e-9*float(v[:2]@v[:2])
            # Refine only promising near-contact roots. Severe collisions are
            # better handled by another atlas seed instead of exhausting the budget.
            if metrics["minimum_object_gap_m"]>-.025 and metrics["minimum_self_gap_m"]>-.01:
                counts["collision_refinements"]+=1
                try:
                    with warnings.catch_warnings():
                        warnings.filterwarnings("ignore",message="Values in x were outside bounds during a minimize step, clipping to bounds",
                            category=RuntimeWarning,module=r"scipy\.optimize\._slsqp_py")
                        refinement=minimize(collision_cost,values,method="SLSQP",bounds=list(zip(low,high)),
                            constraints=[dict(type="eq",fun=residual)],options=dict(maxiter=20,ftol=1e-10))
                    error=float(np.max(np.abs(residual(refinement.x))))
                except SearchStopped:break
                if error<=1e-6:
                    values=refinement.x;_,_,rotation,translation,_,_=frame(values)
                    metrics,world_surfaces,world_bodies,hinges=validate_configuration(
                        vertices,parts,values[:2],rotation,translation,fa,fb,values[2],values[3],
                        (first,other),geometry,settings)
        if not metrics["valid"]:
            rejections.update(metrics["reasons"]);continue
        score=friction_score([first.point,other.point],[-first.outward,-other.outward])
        if not closure_at_mu(score,settings.operating_mu):
            rejections["friction_cutoff"]+=1;continue
        base=rotate(-translation,-rotation);heading=wrap(math.pi/2-rotation)
        signature=tuple(np.round(np.r_[base*1000,math.degrees(heading),np.degrees(values[:2])],1))
        if signature in pool:continue
        world_hinges=rotate(hinges-translation,-rotation)
        gripper_end=hinges[-1]+rotate(REFERENCE["gripper_model"]["forward_tip_m"],sum(values[:2]))
        tip=rotate(gripper_end-translation,-rotation)
        surface_data=[dict(part=PART_NAMES[i],section=("first" if i<5 else "second" if i<10
                            else "camera" if i==CAMERA_INDEX else "gripper"),
                           points_m=p.tolist()) for i,p in enumerate(world_surfaces)]
        surface_data += [dict(part=name,section="body",points_m=p.tolist()) for name,p in zip(BODY_NAMES,world_bodies)]
        pool[signature]=dict(turns_rad=values[:2].tolist(),turns_deg=np.degrees(values[:2]).tolist(),
            node_bends_deg=(np.degrees(values[:2])/5).tolist(),base_xy_m=base.tolist(),heading_rad=heading,
            heading_deg=math.degrees(heading),contact_mode="gripper" if 10<=fb.part<CAMERA_INDEX else "node",
            contact_surfaces=[dict(part=PART_NAMES[f.part],edge=f.edge,fraction=float(v),local_segment_m=[f.first.tolist(),f.second.tolist()])
                              for f,v in ((fa,values[2]),(fb,values[3]))],
            contacts_m=[first.point.tolist(),other.point.tolist()],
            inward_normals=[(-first.outward).tolist(),(-other.outward).tolist()],
            friction=finite_score(score),geometry=metrics,surface_polygons_m=surface_data,
            paths_m=dict(first=world_hinges[:6].tolist(),second=world_hinges[5:].tolist(),
                         gripper=[world_hinges[-1].tolist(),tip.tolist()]),
            planar_force_closure_at_operating_mu=closure_at_mu(score,settings.operating_mu),
            actual_hold_verified=None,actuator_force_margin=None,approach_verified=None)
        if on_candidate:on_candidate(pool[signature])
        if progress:
            progress(dict(stage="후보 평가",attempts=attempts,roots=roots,candidates=len(pool),
                best_mu=min(c["friction"]["mu_critical"] for c in pool.values()),elapsed_s=time.monotonic()-began))
    candidates=sorted(pool.values(),key=lambda c:(round(c["friction"]["mu_critical"],7),
        -c["geometry"]["minimum_joint_headroom_deg"],-c["geometry"]["body_target_clearance_m"]))
    was_cancelled=bool(cancelled and cancelled())
    return dict(schema_version=5,calculator="newhug",version=VERSION,model_note=MODEL_NOTE,
        target_vertices_m=vertices.tolist(),fixed_geometry=asdict(geometry),arm_model=describe_arm(geometry),
        settings=asdict(settings),search=dict(status="cancelled" if was_cancelled else
            "candidates_found" if candidates else "not_found_within_budget",
            search_mu_limit=settings.operating_mu,
            elapsed_s=time.monotonic()-began,attempts=attempts,roots=roots,distinct_candidates=len(candidates),
            returned_candidates=min(len(candidates),settings.results),contact_features=len(features),
            contact_pairs=len(pairs),counts=dict(counts),rejections=dict(rejections),
            atlas_prepare_s=atlas_elapsed,atlas_rows=atlas["rows"] if atlas else 0,
            atlas_configurations=atlas["configurations"] if atlas else 0,
            deadline_reached=time.monotonic()>=deadline,attempt_limit_reached=attempts>=max_solves,
            iteration_limit=max_solves,exact_circle_radius_m=circle_radius_m,
            exhaustive=False),candidates=candidates[:settings.results])

def build_request(data):
    if not isinstance(data,dict):raise InputError("Request must be an object.")
    allowed_geometry=set(Geometry.__dataclass_fields__)
    raw_geometry=data.get("geometry",{})
    if not isinstance(raw_geometry,dict) or set(raw_geometry)-allowed_geometry:
        raise InputError("Unknown geometry field.")
    geometry=Geometry(**raw_geometry).checked()
    allowed_settings=set(SearchSettings.__dataclass_fields__)
    raw_settings=data.get("settings",{})
    if not isinstance(raw_settings,dict) or set(raw_settings)-allowed_settings:
        raise InputError("Unknown search field.")
    settings=SearchSettings(**raw_settings).checked()
    if data.get("shape","circle")=="custom":
        raw=data.get("vertices_mm")
        vertices=clean_polygon(np.asarray(raw,dtype=float)/1000)
    else:
        vertices=template(data.get("shape","circle"),float(data.get("width_mm",160))/1000,
                          float(data.get("height_mm",160))/1000)
    return vertices,geometry,settings


def build_needle_settings(data):
    raw=data.get("needle_settings")
    if raw is None:return None
    if not isinstance(raw,dict) or set(raw)-set(NeedleSettings.__dataclass_fields__):
        raise InputError("Unknown needle-map field.")
    return NeedleSettings(**raw).checked()



PAGE = r"""<!doctype html>
<html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>newhug · 최소 마찰계수 계산기</title>
<style>
:root{color-scheme:light;--ink:#182a3b;--muted:#57697b;--line:#d7e0e8;--blue:#28689c}
*{box-sizing:border-box}body{margin:0;background:#edf2f6;color:var(--ink);font:14px/1.55 system-ui,sans-serif}
header{padding:16px 24px;background:#142d45;color:#fff;display:flex;align-items:center;justify-content:space-between;gap:16px}
h1{font-size:22px;margin:0}h2{font-size:15px;margin:0 0 12px}.subtitle{color:#c5d6e7;font-size:12px}
main{max-width:1800px;margin:auto;padding:18px;display:grid;grid-template-columns:320px minmax(400px,1fr);gap:18px;align-items:start}
.card{border:1px solid var(--line);background:#fff;border-radius:10px;padding:16px;margin-bottom:14px}
.fields{display:grid;grid-template-columns:1fr 1fr;gap:8px 12px}label{display:block;font-size:12px;color:var(--muted)}
input,select,textarea{width:100%;padding:7px 9px;border:1px solid #b8c6d3;border-radius:5px;background:#fff;color:var(--ink);font:inherit}
input:focus,select:focus,textarea:focus{outline:2px solid #87b9df;outline-offset:1px}textarea{font:12px monospace;min-height:95px}
button{border:1px solid #a9bac9;border-radius:6px;padding:9px 12px;background:#f3f6fa;color:var(--ink);font:inherit;cursor:pointer}
button.primary{background:#215e8e;color:#fff;border-color:#215e8e;font-weight:600}
button:disabled,input:disabled,select:disabled{opacity:.5;cursor:default}
.buttons{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}.buttons>button{flex:1}
.note{color:var(--muted);font-size:12px;margin:9px 0 0}details summary{cursor:pointer;font-size:13px;font-weight:600;margin-bottom:12px}
.workspace{min-width:0}.viewport{position:relative;background:#fff;border:1px solid var(--line);border-radius:10px;overflow:hidden}
svg#plot{display:block;width:100%;height:clamp(440px,64vh,850px);touch-action:none}
.legend{padding:9px 16px;border-top:1px solid var(--line);display:flex;gap:16px;flex-wrap:wrap;color:var(--muted);font-size:12px}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:5px}.blue{background:#2676b6}.orange{background:#e48b30}.red{background:#d44a53}.wedge{background:#8b7797}.gripperBase{background:#b78660}.camera{background:#369d99}
.scorebar{display:flex;gap:24px;align-items:flex-start;flex-wrap:wrap}.score{font-size:34px;font-weight:700;letter-spacing:-1px}.unit{font-size:12px;color:var(--muted)}
.good{color:#187344}.bad{color:#b43f3f}.pill{display:inline-block;border-radius:5px;padding:3px 8px;background:#e8eef4;font-size:12px}
#status{white-space:pre-line;font-size:13px}#status.error{color:#ad3232}.progress{height:4px;background:#dde5ec;border-radius:3px;margin:10px 0}
#bar{height:100%;width:0;background:#3b81b7;transition:width .25s}
.candidates{display:flex;gap:9px;overflow-x:auto;padding:2px 0 8px}
.candidate{min-width:180px;text-align:left;padding:10px 13px;background:#fff;border:1px solid var(--line)}
.candidate.selected{border:2px solid #2972a8;padding:9px 12px;background:#eef6fc}.candidate small{display:block;color:var(--muted)}
.metrics{display:grid;grid-template-columns:repeat(4,minmax(100px,1fr));gap:12px;margin-top:14px}.metric strong{display:block;font-size:18px}.metric span{font-size:12px;color:var(--muted)}
.empty{padding:20px;color:var(--muted)}footer{font-size:12px;color:var(--muted);padding:0 18px 20px;max-width:1800px;margin:auto}
@media(max-width:1000px){main{grid-template-columns:280px minmax(320px,1fr)}.metrics{grid-template-columns:1fr 1fr}}
@media(max-width:740px){main{display:block;padding:10px}.controls{display:grid;grid-template-columns:1fr 1fr;gap:10px}.controls>.card{margin:0}.controls>.card:last-child{grid-column:1/-1;margin-bottom:12px}.workspace{margin-top:12px}header{padding:12px}.scorebar{gap:16px}svg#plot{height:450px}}
</style>
<header><div><h1>newhug <span style="font-weight:400;font-size:15px">최소 마찰계수 계산기</span></h1>
<div class="subtitle">탑뷰 · 팔 롤 90° 고정 · 실제 10절점 팔 + 그리퍼 전체 + ZED Mini · 낮은 μ가 유리</div></div><span class="subtitle">독립 실행 / EleSim 연결 없음</span></header>
<main>
<aside class="controls">
<section class="card"><h2>물체</h2>
<label>형상<select id="shape"><option value="circle">원형</option><option value="ellipse">타원</option><option value="rectangle">사각형</option><option value="triangle">삼각형</option><option value="jar">Jar 단면</option><option value="custom">직접 입력 / 그리기</option></select></label>
<div class="fields" style="margin-top:10px"><label>폭 / 지름 (mm)<input id="width" type="number" min="5" max="1000" value="160"></label>
<label>높이 (mm)<input id="height" type="number" min="5" max="1000" value="160"></label></div>
<div id="custom" hidden style="margin-top:10px"><label>꼭짓점 [x,y] 목록 (mm)<textarea id="vertices">[[-80,-80],[80,-80],[80,80],[-80,80]]</textarea></label>
<div class="buttons"><button id="draw">화면에서 그리기</button><button id="undo">마지막 점 지우기</button></div>
<p class="note">그리기 중 화면 클릭으로 점을 추가한다. 꼭짓점은 순서대로, 최대 128개.</p></div>
</section>
<section class="card"><h2>평가</h2><div class="fields">
<label>허용 마찰계수 μ · 컷오프<input id="mu" type="number" min="0" max="1000" step=".05" value=".5"></label>
<label>계산 시간 (초)<input id="budget" type="number" min=".1" max="120" step="1" value="8"></label>
<label>접촉쌍 예산<input id="pairs" type="number" min="8" max="512" step="8" value="96"></label>
<label>카메라 보호 간격 (mm)<input id="cameraGap" type="number" min="0" max="50" step=".5" value="1"></label>
<label>몸체·웨지 보호 간격 (mm)<input id="bodyGap" type="number" min="0" max="50" step=".5" value="1"></label>
</div><p class="note">최소 필요 μ가 설정 μ보다 작은 해만 팔 자세·바늘 맵에 남긴다. 같은 값은 force closure 경계여서 제외한다. 오차 가산 없이 계산된 μ를 사용한다.</p></section>
<section class="card"><h2>바늘 맵</h2><div class="fields">
<label>수집 iteration<input id="iterations" type="number" min="1" max="5000" step="1" value="256"></label>
<label>탐색 seed<input id="mapSeed" type="number" min="0" max="4294967295" step="1" value="0"></label>
<label>수집 시간 상한 (초, 0=없음)<input id="mapTime" type="number" min="0" max="3600" value="0"></label>
</div><p class="note"><strong>최소 필요 μ &lt; 허용 μ</strong>인 유효 해를 전부 기록한다. 낮은 μ의 해도 모두 남긴다. iteration은 접촉 후보 평가 횟수다. 일반 후보 계산의 시간 예산과 별개이며, 중단하면 부분 맵을 남긴다.</p>
<div class="buttons"><button id="collectMap" class="primary">바늘 맵 수집</button></div>
<p class="note">리니어 −173.33~0 mm (1.5°/mm, 260°). 바늘은 GO2 기준점의 주차 선분이며 팔 첫 관절과의 중립 전방 간격은 343 mm다. GO2 본체·다리·플레이트 충돌은 이 모델에 포함되지 않는다.</p>
<hr style="border:0;border-top:1px solid var(--line);margin:14px 0"><div class="fields">
<label>band 안쪽 반경 (mm)<input id="bandInner" class="gatherControl" type="number" min="0" step="1" placeholder="비우면 자동"></label>
<label>band 바깥쪽 반경 (mm)<input id="bandOuter" class="gatherControl" type="number" min="1" step="1" placeholder="비우면 자동"></label>
<label>band 합의 강도 (%)<input id="bandSupport" class="gatherControl" type="number" min="1" max="100" value="80"></label>
<label>기준점 둘레각 (°)<input id="gatherAngle" class="gatherControl" type="number" step="1" value="-90"></label>
</div><div class="buttons"><button id="gather" disabled>원형 바늘 끌어모으기</button><button id="autoBand" disabled>자동 band</button></div>
<p class="note">최대 겹침 수의 설정 비율 이상이 지지하는 연속 반경대를 자동 선택한다 (기본 80%). 각 바늘의 거리를 보존해 같은 둘레각으로 모으고, band 밖 부분은 잘라 표시한다. 대칭 중복만 합치며 빈틈은 보간하지 않는다.</p>
<div id="mapStatus" class="note">아직 기록된 맵이 없습니다.</div>
</section>
<section class="card"><h2>실제 팔 · 그리퍼 손까지</h2>
<div class="note" style="font-size:13px;margin:0">
노드 <strong>50 mm × 10개 (5 + 5)</strong><br>
구간 길이 <strong>250 + 250 mm</strong><br>
굽힘 평면 두께 <strong>70 mm</strong><br>
그리퍼 전방 돌출 <strong>89 mm</strong><br>
그리퍼 전체 외곽 범위 <strong>60.5 × 89 mm</strong><br>
카메라 <strong>ZED Mini · 접촉 금지</strong><br>
접촉 허용 <strong>seg1 · seg2 · 그리퍼 베이스 · 클로우</strong><br>
몸체·웨지 <strong>접촉 금지 · 기본 1 mm 보호</strong><br>
웨지 <strong>84 × 28 mm CAD 외곽</strong><br>
절점당 한계 <strong>±36°</strong> · 구간 총각 ±180°<br>
몸체 <strong>130 × 233 mm 직사각형</strong></div>
<p class="note">롤 90°로 눕힌 팔을 위에서 본다. 팔은 CAD 외곽과 실제 관절 순서를 사용한다. 몸체는 하우징의 좌우·앞뒤 폭으로 만든 직사각형 하나다. 웨지를 복원하고, ZED Mini URDF의 베이스·양쪽 집게·카메라 장착 좌표를 사용한다. 베이스는 직사각형이다. 집게 개폐는 이 평면 밖의 운동이므로 여기서 최적화하지 않는다. 카메라는 접촉 후보에서 제외하고 보호 간격을 검사한다. 플레이트는 제외한다.</p>
<div class="buttons"><button id="previewArm">물체/후보 보기</button></div>
<details style="margin-top:12px"><summary>절점 각도 미리보기</summary>
<label>1구간 절점 각도 q₃ (°)<input id="preview1" type="range" min="-36" max="36" value="0" step="1"></label>
<div id="previewValue1" class="note">0° / 구간 총각 0°</div>
<label style="margin-top:9px">2구간 절점 각도 q₄ (°)<input id="preview2" type="range" min="-36" max="36" value="0" step="1"></label>
<div id="previewValue2" class="note">0° / 구간 총각 0°</div>
<p class="note">같은 구간의 5개 관절에 같은 각도를 적용한다. 이 조작은 기구 확인용이며 접촉·충돌의 성공 판정이 아니다.</p></details>
<div class="buttons"><button id="run" class="primary">후보 계산</button><button id="cancel" disabled>중단</button></div>
<div class="progress"><div id="bar"></div></div><div id="status">형상을 선택하고 후보 계산을 누르세요.</div>
</section>
</aside>
<div class="workspace">
<section class="card"><div class="scorebar"><div><div class="unit">최소 필요 마찰계수</div><div class="score" id="critical">—</div></div>
<div><div class="unit">허용 마찰계수 · 컷오프</div><div class="score" id="muLimit">0.500</div></div>
<div style="padding-top:5px"><div id="pass" class="pill">후보 미선택</div><p class="note">실제 μ가 임계값보다 <strong>커야</strong> 한다.<br>임계값 0도 무마찰 파지 성공을 뜻하지 않는다.</p></div></div>
<div class="metrics" id="metrics"></div></section>
<div class="buttons" style="margin:0 0 10px"><button id="armView">팔 / 접촉</button><button id="mapView" disabled>바늘 맵</button><button id="gatheredView" disabled>끌어모은 바늘</button></div>
<div class="viewport"><svg id="plot" viewBox="-.35 -.35 .7 .7" role="img" aria-label="물체, 고정 길이 팔, 접촉점과 마찰원뿔"></svg>
<div class="legend"><span><i class="dot blue"></i>구간 1</span><span><i class="dot orange"></i>구간 2</span><span><i class="dot gripperBase"></i>그리퍼 베이스</span><span><i class="dot red"></i>집게</span><span><i class="dot camera"></i>ZED Mini · 접촉 금지</span><span><i class="dot wedge"></i>웨지</span><span>실선 화살표: 물체 안쪽 법선</span><span>색 원뿔: 허용 μ</span></div></div>
<section class="card" style="margin-top:14px"><div style="display:flex;align-items:center;justify-content:space-between;gap:12px"><h2>계산된 후보</h2>
<div class="buttons" style="margin:0"><button id="saveJson" disabled>JSON 저장</button><button id="saveSvg" disabled>SVG 저장</button></div></div>
<div id="candidates" class="candidates"><div class="empty">관통·몸통 간섭·팔 자체 충돌을 검사한 정적 후보를 여기에 표시합니다.</div></div>
</section>
</div></main>
<footer>롤 90°로 눕힌 팔의 탑뷰, 평면 강체·두 점접촉 계산이다. 팔 배치는 수평면에서 자유롭게 선택한다.
실제 로봇의 도달·접근 경로, 모터 힘, 3D 회전, 물체 압축 및 유지 성공은 별도 검증 대상이다.
미발견은 제한된 탐색의 결과이며 물리적 불가능의 증명이 아니다.</footer>
<script>
'use strict';
const TOKEN="__TOKEN__",$=id=>document.getElementById(id);
const MODEL=__MODEL_JSON__;
let previewMode=true,result=null,selected=0,running=false,job=null,drawMode=false,draft=[],started=0;
let viewMode='arm',mapSelected=-1,collecting=false,cutoffRevision=0,cutoffPending=false;
const fmt=(v,n=3)=>v==null||!Number.isFinite(v)?'∞':Number(v).toFixed(n);
function center(p){let sum=0,cx=0,cy=0;for(let i=0;i<p.length;i++){let a=p[i],b=p[(i+1)%p.length],z=a[0]*b[1]-b[0]*a[1];sum+=z;cx+=(a[0]+b[0])*z;cy+=(a[1]+b[1])*z}
 if(Math.abs(sum)<1e-12)return p;return p.map(a=>[a[0]-cx/(3*sum),a[1]-cy/(3*sum)])}
function preview(){let w=Number($('width').value)/1000,h=Number($('height').value)/1000,name=$('shape').value,p=[];
 if(name==='custom'){p=(drawMode?draft:JSON.parse($('vertices').value)).map(a=>[Number(a[0])/1000,Number(a[1])/1000]);return drawMode?p:center(p)}
 if(name==='circle'||name==='ellipse'){for(let i=0;i<64;i++){let t=2*Math.PI*i/64;p.push([w/2*Math.cos(t),(name==='circle'?w:h)/2*Math.sin(t)])}}
 if(name==='rectangle')p=[[-w/2,-h/2],[w/2,-h/2],[w/2,h/2],[-w/2,h/2]];
 if(name==='triangle')p=[[-w/2,-h/3],[w/2,-h/3],[0,2*h/3]];
 if(name==='jar'){let r=[[.35,-.5],[.46,-.35],[.5,-.075],[.385,.2],[.208,.325],[.208,.45],[.27,.45],[.27,.5]];
  p=r.concat(r.slice().reverse().map(a=>[-a[0],a[1]])).map(a=>[a[0]*w,a[1]*h])}return center(p)}
function previewSurfaces(){
 const g=MODEL.geometry,q=[Number($('preview1').value)*Math.PI/180,Number($('preview2').value)*Math.PI/180];
 let origin=[0,0],angle=0,shapes=[];
 const transform=(p,a,o)=>[p[0]*Math.cos(a)-p[1]*Math.sin(a)+o[0],p[0]*Math.sin(a)+p[1]*Math.cos(a)+o[1]];
 for(let i=0;i<10;i++){angle+=q[i<5?0:1];shapes.push({part:'node'+i,section:i<5?'first':'second',
  points_m:MODEL.local_profiles_m['node'+i].map(p=>transform(p,angle,origin))});
  origin=[origin[0]-g.pitch_m*Math.sin(angle),origin[1]+g.pitch_m*Math.cos(angle)]}
 for(const part of MODEL.gripper_part_names)shapes.push({part,section:'gripper',points_m:MODEL.local_profiles_m[part].map(p=>transform(p,angle,origin))});
 shapes.push({part:'camera',section:'camera',points_m:MODEL.local_profiles_m.camera.map(p=>transform(p,angle,origin))});
 for(const [part,points] of Object.entries(MODEL.fixed_body_profiles_m))shapes.push({part,section:'body',points_m:points});
 return shapes
}
function candidate(){return !previewMode&&result&&result.candidates[selected]}
function draw(){if(viewMode!=='arm'&&result&&result.needle_map){drawNeedles();return}let c=candidate(),v;
 try{v=previewMode?[]:result?result.target_vertices_m:preview()}catch(e){v=[]}
 if(!v.every(a=>Array.isArray(a)&&a.length===2&&a.every(Number.isFinite)))v=[];
 let surfaces=previewMode?previewSurfaces():c?c.surface_polygons_m:[];let all=v.slice();for(let s of surfaces)all=all.concat(s.points_m);
 let xmin=-.3,xmax=.3,ymin=-.3,ymax=.3;
 if(all.length&&!drawMode){xmin=Math.min(...all.map(a=>a[0]));xmax=Math.max(...all.map(a=>a[0]));ymin=Math.min(...all.map(a=>a[1]));ymax=Math.max(...all.map(a=>a[1]));
  let side=Math.max(xmax-xmin,ymax-ymin,.12),pad=.15*side,cx=(xmin+xmax)/2,cy=(ymin+ymax)/2;
  xmin=cx-side/2-pad;xmax=cx+side/2+pad;ymin=cy-side/2-pad;ymax=cy+side/2+pad}
 let span=Math.max(xmax-xmin,ymax-ymin),sw=span*.0025,font=span*.026;
 $('plot').setAttribute('viewBox',[xmin,-ymax,xmax-xmin,ymax-ymin].join(' '));
 const pts=p=>p.map(a=>a.join(',')).join(' ');
 const poly=(p,stroke,fill='none',width=sw,close=false,dash='')=>'<'+(close?'polygon':'polyline')+' points="'+pts(p)+'" fill="'+fill+'" stroke="'+stroke+'" stroke-width="'+width+'" stroke-linecap="round" stroke-linejoin="round" '+dash+'/>';
 let parts=['<defs><marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" orient="auto"><path d="M0 0 L10 5 L0 10z" fill="#1f4b36"/></marker></defs><g transform="scale(1,-1)">'];
 if(v.length)parts.push(poly(v,'#60788e','#e6edf3',sw,true));
 if(drawMode){parts.push(poly(v,'#276a9d','none',sw,false));for(let p of v)parts.push('<circle cx="'+p[0]+'" cy="'+p[1]+'" r="'+span*.007+'" fill="#276a9d"/>')}
 const colors={first:['#22659c','#8eb9dc'],second:['#be6b1d','#f2bd8a'],gripper:['#ac3540','#df969c'],gripper_base:['#8d6241','#d5b49a'],camera:['#156e6b','#a5d5d0'],body:['#61758a','#c4d0dc'],wedge:['#786181','#d6c9df']};
 const layer=s=>s.section==='body'?0:s.part==='camera'?3:s.part==='gripper_base'?2:1;
 for(let s of surfaces.slice().sort((a,b)=>layer(a)-layer(b))){
  const key=s.part==='wedge'?'wedge':s.part==='gripper_base'?'gripper_base':s.section;
  let [stroke,fill]=colors[key];
  if(MODEL.protected_part_names.includes(s.part)){let gap=s.part==='camera'?(result?result.settings.camera_clearance_m:Number($('cameraGap').value)/1000):(result?result.settings.body_clearance_m:Number($('bodyGap').value)/1000);
   if(gap>0)parts.push(poly(s.points_m,'#52b5af','none',2*gap,true))}
  parts.push(poly(s.points_m,stroke,fill,sw,true))}
 if(c){let mu=Math.max(0,Number($('mu').value)),alpha=Math.atan(mu),len=span*.14;
  parts.push(poly(c.contacts_m,'#8361b2','none',sw,false,'stroke-dasharray="'+span*.012+' '+span*.008+'"'));
  for(let i=0;i<2;i++){let p=c.contacts_m[i],n=c.inward_normals[i],angle=Math.atan2(n[1],n[0]);
   let good=mu>c.friction.mu_critical,color=good?'#2f9565':'#b85a55';
   let cone=[p,[p[0]+len*Math.cos(angle-alpha),p[1]+len*Math.sin(angle-alpha)],[p[0]+len*Math.cos(angle+alpha),p[1]+len*Math.sin(angle+alpha)]];
   parts.push(poly(cone,color,good?'#8bcda34a':'#e6a7a54a',sw*.7,true));
   parts.push('<line x1="'+p[0]+'" y1="'+p[1]+'" x2="'+(p[0]+n[0]*len*.9)+'" y2="'+(p[1]+n[1]*len*.9)+'" stroke="#1f4b36" stroke-width="'+sw+'" marker-end="url(#arrow)"/>');
   parts.push('<circle cx="'+p[0]+'" cy="'+p[1]+'" r="'+span*.005+'" fill="#152c40"/>')}
 }
 parts.push('</g>');
 for(const s of surfaces)if(s.part==='camera'){
  let x=Math.min(...s.points_m.map(p=>p[0])),y=Math.max(...s.points_m.map(p=>p[1]));
  parts.push('<text x="'+x+'" y="'+(-y-span*.012)+'" font-family="system-ui,sans-serif" font-size="'+font*.7+'" fill="#156e6b">ZED Mini</text>')}
 if(c)for(let i=0;i<2;i++){let p=c.contacts_m[i];parts.push('<text x="'+(p[0]+span*.014)+'" y="'+(-p[1]-span*.014)+'" font-family="system-ui,sans-serif" font-size="'+font+'" fill="#152c40">P'+(i+1)+'</text>')}
 let bar=Math.pow(10,Math.floor(Math.log10(span*.2))),sx=xmin+span*.05,sy=-ymin-span*.045;
 parts.push('<line x1="'+sx+'" y1="'+sy+'" x2="'+(sx+bar)+'" y2="'+sy+'" stroke="#536d82" stroke-width="'+sw+'"/><text x="'+sx+'" y="'+(sy-span*.018)+'" fill="#536d82" font-family="system-ui,sans-serif" font-size="'+font*.8+'">'+fmt(bar*1000,0)+' mm</text>');
 $('plot').innerHTML=parts.join('');$('previewArm').textContent=previewMode?'물체/후보 보기':'실제 기구 보기';
 $('muLimit').textContent=fmt(Number($('mu').value));
 if(c){$('critical').textContent=fmt(c.friction.mu_critical);
  let good=Number($('mu').value)>c.friction.mu_critical;
  $('pass').textContent=good?'설정 μ에서 평면 force closure':'설정 μ가 충분하지 않음';$('pass').className='pill '+(good?'good':'bad');
  let metrics=[['절점당 각도',c.node_bends_deg.map(v=>fmt(v,1)+'°').join(' / ')],
   ['구간 총 굽힘',c.turns_deg.map(v=>fmt(v,1)+'°').join(' / ')],
   ['최소 절점 여유',fmt(c.geometry.minimum_joint_headroom_deg,1)+'°'],
   ['몸통 간격',fmt(c.geometry.body_target_clearance_m*1000,2)+' mm'],
   ['카메라 간격',fmt(c.geometry.camera_target_clearance_m*1000,2)+' mm']];
  $('metrics').innerHTML=metrics.map(a=>'<div class="metric"><span>'+a[0]+'</span><strong>'+a[1]+'</strong></div>').join('');
 }else{$('critical').textContent='—';$('pass').textContent=previewMode?'실제 치수 기구 미리보기':'후보 미선택';$('pass').className='pill';$('metrics').innerHTML=''}
}
function renderCandidates(){let box=$('candidates');box.innerHTML='';
 if(!result||!result.candidates.length){let e=document.createElement('div');e.className='empty';e.textContent=result?'이번 예산에서 후보 미발견. 기구적으로 불가능하다는 뜻은 아닙니다.':'계산된 후보가 여기에 표시됩니다.';box.appendChild(e)}
 else result.candidates.forEach((c,i)=>{let b=document.createElement('button');b.className='candidate'+(i===selected?' selected':'');
  b.innerHTML='<strong>#'+(i+1)+' · 최소 μ '+fmt(c.friction.mu_critical)+'</strong><small>'+({'gripper':'그리퍼 전체','node':'팔 절점'}[c.contact_mode])+'</small><small>'+'절점 '+c.node_bends_deg.map(v=>fmt(v,1)+'°').join(' / ')+'</small>';
  b.onclick=()=>{previewMode=false;viewMode='arm';selected=i;renderCandidates();draw()};box.appendChild(b)});
 $('saveJson').disabled=!result||cutoffPending;$('saveSvg').disabled=!result||cutoffPending;renderMapControls();draw()
}
function setRunning(value){running=value;$('run').disabled=value;$('cancel').disabled=!value;
 for(let el of document.querySelectorAll('.controls input,.controls select,.controls textarea'))el.disabled=value;
 if(!value)$('height').disabled=$('shape').value==='circle';
 $('previewArm').disabled=value;$('draw').disabled=value;$('undo').disabled=value;$('collectMap').disabled=value;renderMapControls()}
async function api(path,data){let r=await fetch(path,{method:data===undefined?'GET':'POST',headers:{'Content-Type':'application/json','X-Newhug-Token':TOKEN},
 body:data===undefined?undefined:JSON.stringify(data)});let value=await r.json();if(!r.ok)throw Error(value.error||r.statusText);return value}
function request(){let val=id=>Number($(id).value),mm=id=>val(id)/1000;
 return {shape:$('shape').value,width_mm:val('width'),height_mm:val('height'),vertices_mm:$('shape').value==='custom'?JSON.parse($('vertices').value):undefined,
 geometry:{},
 settings:{budget_s:val('budget'),pair_limit:val('pairs'),operating_mu:val('mu'),camera_clearance_m:val('cameraGap')/1000,body_clearance_m:val('bodyGap')/1000}}}
async function runSearch(map){try{if(drawMode){drawMode=false;$('draw').textContent='화면에서 그리기'}
 cutoffRevision++;cutoffPending=false;collecting=map;viewMode=map?'needles':'arm';mapSelected=-1;
 let data=request();if(map)data.needle_settings={iterations:Number($('iterations').value),
  seed:Number($('mapSeed').value),time_limit_s:Number($('mapTime').value)};
 previewMode=false;setRunning(true);result=null;renderCandidates();$('status').className='';$('status').textContent='접촉 후보를 계산하는 중…';started=Date.now();
 job=(await api('/api/solve',data)).job;poll()}catch(e){setRunning(false);$('status').className='error';$('status').textContent=e.message}};
$('run').onclick=()=>runSearch(false);$('collectMap').onclick=()=>runSearch(true);
async function poll(){try{let s=await api('/api/status');if(s.job!==job)return;
 if(s.running){let p=s.progress||{},elapsed=(Date.now()-started)/1000;$('bar').style.width=Math.min(98,100*(collecting?(p.attempts||0)/Number($('iterations').value):elapsed/Number($('budget').value)))+'%';
  $('status').textContent=elapsed.toFixed(1)+'초 · 시도 '+(p.attempts||0)+' · 후보 '+(p.candidates||0)+(collecting?' · 기록 '+(p.recorded_needles||0):'')+(p.best_mu!=null?'\n현재 최저 μ: '+fmt(p.best_mu):'');
  setTimeout(poll,350);return}
 setRunning(false);$('bar').style.width='100%';
 if(s.error)throw Error(s.error);result=s.result;selected=0;
 if(result.needle_map&&result.needle_map.gathered)showBand(result.needle_map.gathered);
 renderCandidates();
 let q=result.search;$('status').textContent=(q.status==='cancelled'?'중단됨 · 부분 결과':q.returned_candidates?'계산 완료':'이번 예산에서 미발견')+
 '\n'+q.elapsed_s.toFixed(2)+'초 · 시도 '+q.attempts+' · 후보 '+q.distinct_candidates+(collecting?' · 바늘 '+result.needle_map.needles.length+' / 요청 '+result.needle_map.iterations_requested+' iterations':'');
 }catch(e){setRunning(false);$('status').className='error';$('status').textContent=e.message}}
$('cancel').onclick=async()=>{try{await api('/api/cancel',{});$('status').textContent='계산 중단 요청…'}catch(e){$('status').textContent=e.message}};
function changed(){if(running)return;cutoffRevision++;cutoffPending=false;previewMode=false;result=null;job=null;selected=0;viewMode='arm';mapSelected=-1;renderCandidates()}
$('shape').onchange=()=>{$('custom').hidden=$('shape').value!=='custom';$('height').disabled=$('shape').value==='circle';drawMode=false;changed()};
for(let el of document.querySelectorAll('.controls input,.controls textarea,.controls select'))if(!['shape','mu','preview1','preview2'].includes(el.id)&&!el.classList.contains('gatherControl'))el.onchange=changed;
$('mu').oninput=applyCutoff;
$('draw').onclick=()=>{drawMode=!drawMode;if(drawMode){draft=[];$('vertices').value='[]'}$('draw').textContent=drawMode?'그리기 완료':'화면에서 그리기';changed()};
$('undo').onclick=()=>{if(drawMode){draft.pop();$('vertices').value=JSON.stringify(draft);draw()}else{try{let p=JSON.parse($('vertices').value);p.pop();$('vertices').value=JSON.stringify(p);changed()}catch(e){$('status').textContent=e.message}}};
$('plot').onclick=e=>{if(viewMode!=='arm'){let line=e.target.closest('[data-needle]');if(line){mapSelected=Number(line.dataset.needle);draw()}return}if(!drawMode||running||draft.length>=128)return;let p=new DOMPoint(e.clientX,e.clientY).matrixTransform($('plot').getScreenCTM().inverse());
 draft.push([Math.round(p.x*1000),Math.round(-p.y*1000)]);$('vertices').value=JSON.stringify(draft);draw()};
function download(name,text,type){let url=URL.createObjectURL(new Blob([text],{type})),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)}
$('saveJson').onclick=()=>download('newhug-result.json',JSON.stringify({...result,presentation:{selected_candidate:selected,operating_mu:Number($('mu').value)}},null,2),'application/json');
$('saveSvg').onclick=()=>{let copy=$('plot').cloneNode(true);copy.setAttribute('xmlns','http://www.w3.org/2000/svg');copy.setAttribute('width','1000');copy.setAttribute('height','800');
 download('newhug-candidate.svg',new XMLSerializer().serializeToString(copy),'image/svg+xml')};
$('previewArm').onclick=()=>{viewMode='arm';previewMode=!previewMode;draw()};
for(const id of ['preview1','preview2'])$(id).oninput=()=>{
 previewMode=true;viewMode='arm';let index=id==='preview1'?1:2,value=Number($(id).value);
 $('previewValue'+index).textContent=value+'° / 구간 총각 '+(5*value)+'°';draw()};
function renderMapControls(){let m=result&&result.needle_map,g=m&&m.gathered;
 $('mapView').disabled=running||!m;$('gatheredView').disabled=running||!g;$('gather').disabled=running||cutoffPending||!m||m.target.kind!=='circle';
 $('autoBand').disabled=$('gather').disabled;
 $('mapStatus').textContent=m?'기록 '+m.needles.length+'개 · '+m.iterations_completed+'/'+m.iterations_requested+' iterations'+
  (g?' · band에 남은 바늘 '+g.intersecting_needles+'/'+g.input_needles+' · 제외 '+g.outside_band+' · 대칭 중복 '+g.duplicate_rotations+
  (g.band_m?' · band '+g.band_m.map(x=>fmt(x*1000,1)).join('~')+' mm':''):''):'아직 기록된 맵이 없습니다.';
}
function needleSurfaces(n){let q=n.turns_rad.map(v=>v/5),origin=n.hinge_xy_m.slice(),angle=n.heading_rad-Math.PI/2,baseAngle=angle,shapes=[];
 const turn=(p,a,o)=>[p[0]*Math.cos(a)-p[1]*Math.sin(a)+o[0],p[0]*Math.sin(a)+p[1]*Math.cos(a)+o[1]];
 for(let i=0;i<10;i++){angle+=q[i<5?0:1];shapes.push({part:'node'+i,points_m:MODEL.local_profiles_m['node'+i].map(p=>turn(p,angle,origin))});
  origin=[origin[0]-MODEL.geometry.pitch_m*Math.sin(angle),origin[1]+MODEL.geometry.pitch_m*Math.cos(angle)]}
 for(let part of [...MODEL.gripper_part_names,'camera'])shapes.push({part,points_m:MODEL.local_profiles_m[part].map(p=>turn(p,angle,origin))});
 for(let [part,points] of Object.entries(MODEL.fixed_body_profiles_m))shapes.push({part,points_m:points.map(p=>turn(p,baseAngle,n.hinge_xy_m))});return shapes;
}
function drawNeedles(){let m=result.needle_map,g=viewMode==='gathered'?m.gathered:null,rows=g?g.needles:m.needles,v=result.target_vertices_m;
 let chosen=rows[mapSelected],surfaces=chosen?needleSurfaces(chosen):[],points=v.slice();
 for(let n of rows)points.push(...(g?n.band_segment_xy_m:n.segment_xy_m));for(let s of surfaces)points.push(...s.points_m);if(g)points.push(...g.band_anchor_xy_m);
 let xmin=Math.min(...points.map(p=>p[0])),xmax=Math.max(...points.map(p=>p[0])),ymin=Math.min(...points.map(p=>p[1])),ymax=Math.max(...points.map(p=>p[1]));
 let span=Math.max(xmax-xmin,ymax-ymin,.15)*1.2,cx=(xmin+xmax)/2,cy=(ymin+ymax)/2,sw=span*.002;
 $('plot').setAttribute('viewBox',[cx-span/2,-cy-span/2,span,span].join(' '));
 let parts=['<defs><marker id="needleArrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M0 0 L10 5 L0 10z" fill="#477ba8"/></marker></defs><g transform="scale(1,-1)">'];
 if(m.target.kind==='circle')parts.push('<circle cx="0" cy="0" r="'+m.target.radius_m+'" fill="#e6edf3" stroke="#60788e" stroke-width="'+sw+'"/>');
 else parts.push('<polygon points="'+v.map(p=>p.join(',')).join(' ')+'" fill="#e6edf3" stroke="#60788e" stroke-width="'+sw+'"/>');
 if(g&&g.band_m)for(let radius of g.band_m)parts.push('<circle cx="0" cy="0" r="'+radius+'" fill="none" stroke="#bccbd7" stroke-dasharray="'+sw*4+' '+sw*4+'" stroke-width="'+sw*.5+'"/>');
 for(let s of surfaces.slice().sort((a,b)=>(a.part==='camera'?3:a.part==='gripper_base'?2:1)-(b.part==='camera'?3:b.part==='gripper_base'?2:1))){let color=s.part==='camera'?'#369d99':s.part==='gripper_base'?'#b78660':s.part.startsWith('gripper')?'#df969c':'#adb9c4';
  if(MODEL.protected_part_names.includes(s.part)){let gap=s.part==='camera'?result.settings.camera_clearance_m:result.settings.body_clearance_m;
   if(gap>0)parts.push('<polygon points="'+s.points_m.map(p=>p.join(',')).join(' ')+'" fill="none" stroke="#52b5af" stroke-width="'+2*gap+'"/>')}
  parts.push('<polygon points="'+s.points_m.map(p=>p.join(',')).join(' ')+'" fill="'+color+'" fill-opacity=".55" stroke="#687b8c" stroke-width="'+sw*.6+'"/>')}
 rows.forEach((n,i)=>{let segment=g?n.band_segment_xy_m:n.segment_xy_m,a=segment[1],b=segment[0],color=i===mapSelected?'#bd3c48':'#477ba8';
  parts.push('<line data-needle="'+i+'" x1="'+a[0]+'" y1="'+a[1]+'" x2="'+b[0]+'" y2="'+b[1]+'" stroke="'+color+'" stroke-opacity=".65" stroke-width="'+sw*(i===mapSelected?2.5:1)+'" marker-end="url(#needleArrow)" style="cursor:pointer"><title>'+n.id+' · μ '+fmt(n.friction[m.filter.field])+' · heading '+fmt(n.heading_deg,1)+'°</title></line>')});
 if(g&&g.band_anchor_xy_m.length){let [a,b]=g.band_anchor_xy_m;parts.push('<line x1="'+a[0]+'" y1="'+a[1]+'" x2="'+b[0]+'" y2="'+b[1]+'" stroke="#bd3c48" stroke-width="'+sw*2+'"/>')}
 parts.push('</g>');parts.push('<text x="'+(cx-span*.46)+'" y="'+(-cy-span*.44)+'" font-size="'+span*.025+'" fill="#40576b">'+(g?'끌어모은 바늘':'바늘 맵')+' · '+rows.length+'개 · 클릭하면 팔 자세 표시</text>');
 $('plot').innerHTML=parts.join('');$('critical').textContent=chosen?fmt(chosen.friction.mu_critical):'—';$('muLimit').textContent=fmt(Number($('mu').value));
 $('pass').className='pill';$('pass').textContent=g?'반경 band 안 · 회전 정렬된 정적 해':'리니어로 보상 가능한 주차 선분';
 $('metrics').textContent=chosen?'heading '+fmt(chosen.heading_deg,1)+'° · 절점 '+chosen.node_bends_deg.map(x=>fmt(x,2)+'°').join(' / ')+(chosen.selected_linear_m!=null?' · 리니어 '+fmt(chosen.selected_linear_m*1000,2)+' mm':''):'';
}
$('armView').onclick=()=>{viewMode='arm';draw()};$('mapView').onclick=()=>{viewMode='needles';mapSelected=-1;draw()};$('gatheredView').onclick=()=>{viewMode='gathered';mapSelected=-1;draw()};
function showBand(g){if(g.band_m){$('bandInner').value=(g.band_m[0]*1000).toFixed(6);$('bandOuter').value=(g.band_m[1]*1000).toFixed(6)}}
async function applyCutoff(){let revision=++cutoffRevision,mu=Number($('mu').value),currentJob=job;
 if(running)return;if(!result){draw();return}
 cutoffPending=true;const valid=Number.isFinite(mu)&&mu>=0&&mu<=1000;
 const allowed=c=>valid&&Number.isFinite(c.friction.mu_critical)&&c.friction.mu_critical<mu;
 result.candidates=result.candidates.filter(allowed);selected=0;mapSelected=-1;
 if(result.needle_map){let m=result.needle_map;m.needles=m.needles.filter(allowed);if(m.gathered)m.gathered.needles=m.gathered.needles.filter(allowed)}
 renderCandidates();
 if(!valid){$('status').className='error';$('status').textContent='허용 μ는 0~1000 범위여야 합니다.';return}
 try{let value=await api('/api/cutoff',{job:currentJob,operating_mu:mu,revision});
  if(revision!==cutoffRevision||currentJob!==job)return;
  result=value;cutoffPending=false;if(result.needle_map&&result.needle_map.gathered)showBand(result.needle_map.gathered);
  renderCandidates();$('status').className='';$('status').textContent='μ 컷오프 적용 · 팔 자세 '+result.candidates.length+(result.needle_map?' · 바늘 '+result.needle_map.needles.length:'')+
   (result.search.additional_search_needed?'\n탐색 당시 μ보다 큰 범위의 추가 해는 다시 계산하세요.':'');
 }catch(e){if(revision!==cutoffRevision||currentJob!==job)return;$('status').className='error';$('status').textContent=e.message}
}
async function gatherMap(automatic){try{let lo=$('bandInner').value,hi=$('bandOuter').value;
 if(!automatic&&((lo==='')!==(hi==='')))throw Error('band의 안쪽·바깥쪽을 함께 입력하거나 모두 비우세요.');
 let band=automatic||lo===''?null:[Number(lo)/1000,Number(hi)/1000];
 result.needle_map.gathered=await api('/api/gather',{job,band_m:band,support_fraction:Number($('bandSupport').value)/100,anchor_angle_deg:Number($('gatherAngle').value)});
 showBand(result.needle_map.gathered);
 viewMode='gathered';mapSelected=-1;renderMapControls();draw();
 }catch(e){$('status').className='error';$('status').textContent=e.message}};
$('gather').onclick=()=>gatherMap(false);$('autoBand').onclick=()=>gatherMap(true);
$('height').disabled=true;renderMapControls();draw();
</script></html>"""


class Application:
    def __init__(self):
        self.lock=threading.RLock()
        self.cancel_event=threading.Event()
        self.running=False
        self.job=None
        self.progress={}
        self.result=None
        self.search_result=None
        self.cutoff_revision=0
        self.error=None
        self.worker=None

    def start(self,vertices,geometry,settings,needle_settings=None,circle_radius_m=None):
        with self.lock:
            if self.running:raise InputError("A search is already running; stop it first.")
            self.cancel_event=threading.Event()
            self.running=True;self.job=secrets.token_hex(8)
            self.progress={};self.result=None;self.error=None
            self.search_result=None;self.cutoff_revision=0
            job=self.job
            def update(value):
                with self.lock:self.progress=value
            def work():
                try:
                    if needle_settings is not None:
                        value=collect_needle_map(vertices,geometry,settings,needle_settings=needle_settings,
                                                circle_radius_m=circle_radius_m,cancelled=self.cancel_event.is_set,progress=update)
                    else:value=solve(vertices,geometry,settings,cancelled=self.cancel_event.is_set,progress=update)
                    with self.lock:self.result=self.search_result=value
                except Exception as exc:
                    traceback.print_exc()
                    with self.lock:self.error=str(exc)
                finally:
                    with self.lock:self.running=False
            self.worker=threading.Thread(target=work,name="newhug-solver",daemon=True)
            self.worker.start()
            return job

    def snapshot(self):
        with self.lock:
            return dict(job=self.job,running=self.running,progress=self.progress,
                        result=self.result,error=self.error)

    def gather(self,data):
        with self.lock:
            if self.running or data.get("job")!=self.job:raise InputError("Gather requires the completed current job.")
            if not self.result or "needle_map" not in self.result:raise InputError("Collect a needle map first.")
            value=gather_needles(self.result["needle_map"],band_m=data.get("band_m"),support_fraction=data.get("support_fraction",.8),
                                 anchor_angle_deg=data.get("anchor_angle_deg",-90.))
            self.result["needle_map"]["gathered"]=value
            return value

    def cutoff(self,data):
        with self.lock:
            if self.running or data.get("job")!=self.job or self.search_result is None:
                raise InputError("Cutoff requires the completed current job.")
            revision=data.get("revision",self.cutoff_revision+1)
            if type(revision) is not int or revision<=self.cutoff_revision:
                raise InputError("Stale cutoff revision.")
            options=self.result.get("needle_map",{}).get("gathered")
            value=filter_result_at_mu(self.search_result,data["operating_mu"],gather_options=options)
            self.cutoff_revision=revision;self.result=value
            return value


def serve(port=0,open_browser=True):
    app=Application();token=secrets.token_hex(24)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*_):
            pass

        def reply(self,status,value,content_type="application/json; charset=utf-8"):
            body=value.encode() if isinstance(value,str) else json.dumps(value,allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type",content_type);self.send_header("Content-Length",str(len(body)))
            self.send_header("Cache-Control","no-store")
            self.send_header("X-Content-Type-Options","nosniff")
            self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass

        def authorized(self):
            if not secrets.compare_digest(self.headers.get("X-Newhug-Token",""),token):
                self.reply(403,dict(error="Open the calculator page from this server."))
                return False
            return True

        def do_GET(self):
            if self.path=="/":
                self.reply(200,browser_document(token),"text/html; charset=utf-8")
            elif self.path=="/api/status":
                if self.authorized():self.reply(200,app.snapshot())
            elif self.path=="/favicon.ico":
                self.send_response(204);self.end_headers()
            else:self.reply(404,dict(error="Not found."))

        def do_POST(self):
            if not self.authorized():return
            try:
                size=int(self.headers.get("Content-Length","0"))
                if not 0<size<=65536:raise InputError("Request must be at most 64 KiB.")
                data=json.loads(self.rfile.read(size))
                if not isinstance(data,dict):raise InputError("Request must be an object.")
                if self.path=="/api/solve":
                    vertices,geometry,settings=build_request(data)
                    needle_settings=build_needle_settings(data)
                    radius=float(data.get("width_mm",160))/2000 if data.get("shape","circle")=="circle" else None
                    self.reply(202,dict(job=app.start(vertices,geometry,settings,needle_settings,radius)))
                elif self.path=="/api/gather":
                    self.reply(200,app.gather(data))
                elif self.path=="/api/cutoff":
                    self.reply(200,app.cutoff(data))
                elif self.path=="/api/cancel":
                    app.cancel_event.set();self.reply(200,dict(cancel_requested=True))
                else:self.reply(404,dict(error="Not found."))
            except (ValueError,TypeError,KeyError,OverflowError) as exc:
                self.reply(400,dict(error=str(exc)))
    server=ThreadingHTTPServer(("127.0.0.1",port),Handler)
    url="http://127.0.0.1:"+str(server.server_port)+"/"
    print("newhug "+VERSION+" — "+url,flush=True)
    print("Open this URL in your browser. Ctrl+C stops the standalone calculator.",flush=True)
    if open_browser:
        threading.Thread(target=lambda:open_calculator_browser(url),daemon=True).start()
    try:server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:pass
    finally:
        app.cancel_event.set();server.server_close()
        if app.worker:app.worker.join(timeout=3)


def save_new(path,text):
    with Path(path).open("x",encoding="utf-8") as file:file.write(text)


def main():
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--solve",action="store_true",help="Run a headless static search instead of opening the browser UI.")
    parser.add_argument("--self-test",action="store_true",help="Run built-in mathematical/geometry regression tests.")
    parser.add_argument("--no-browser",action="store_true")
    parser.add_argument("--port",type=int,default=0,help="Local UI port; 0 selects a free loopback port.")
    parser.add_argument("--shape",choices=("circle","ellipse","rectangle","triangle","jar"),default="circle")
    parser.add_argument("--width-mm",type=float,default=160.)
    parser.add_argument("--height-mm",type=float,default=160.)
    parser.add_argument("--polygon",type=Path,help="Custom JSON array of [x,y] vertices, in mm.")
    parser.add_argument("--budget",type=float,default=8.)
    parser.add_argument("--pair-limit",type=int,default=96)
    parser.add_argument("--camera-clearance-mm",type=float,default=1.,help="Minimum ZED Mini gap in mm; touching is always forbidden.")
    parser.add_argument("--body-clearance-mm",type=float,default=1.,help="Minimum housing/wedge gap in mm; touching is always forbidden.")
    parser.add_argument("--mu",type=float,default=.5,help="Global cutoff: keep only solutions whose minimum required mu is strictly below this value.")
    parser.add_argument("--record-needles",action="store_true",help="Record all eligible static solutions, not just the top preview candidates.")
    parser.add_argument("--iterations",type=int,default=256,help="Contact-candidate evaluation limit for map collection (default: 256).")
    parser.add_argument("--seed",type=int,default=0,help="Map exploration seed; same seed repeats the exploration sequence.")
    parser.add_argument("--map-time-limit",type=float,default=0.,help="Optional map wall-time limit; 0 means no time limit.")
    parser.add_argument("--gather-band-mm",type=float,nargs=2,metavar=("INNER","OUTER"),help="Manual radial band; omitted selects a consensus band.")
    parser.add_argument("--gather-support",type=float,default=.8,help="Automatic band needs this fraction of peak radial overlap (default .8).")
    parser.add_argument("--gather-angle-deg",type=float,default=-90.,help="Object-centred polar angle of the common parking point.")
    parser.add_argument("--needle-svg",type=Path,help="Write the scattered needle map to a NEW SVG file.")
    parser.add_argument("--gathered-svg",type=Path,help="Write the circle's gathered needles to a NEW SVG file.")
    parser.add_argument("--geometry",type=Path,help="JSON object of node CAD parameters in metres/degrees; terminal is fixed from URDF.")
    parser.add_argument("--output",type=Path,help="Write JSON to a NEW file; otherwise print it.")
    parser.add_argument("--svg",type=Path,help="Also write a standalone preview to a NEW SVG file.")
    parser.add_argument("--describe-arm",action="store_true",help="Print embedded physical CAD dimensions and provenance.")
    args=parser.parse_args()
    if args.self_test:
        return self_test()
    if args.describe_arm:
        print(json.dumps(describe_arm(),indent=2));return 0
    if not args.solve:
        if not 0<=args.port<=65535:parser.error("port must be 0..65535")
        serve(args.port,not args.no_browser);return 0
    try:
        geometry=Geometry(**json.loads(args.geometry.read_text())) if args.geometry else Geometry()
        target=(clean_polygon(np.asarray(json.loads(args.polygon.read_text()),dtype=float)/1000)
                if args.polygon else template(args.shape,args.width_mm/1000,args.height_mm/1000))
        settings=SearchSettings(budget_s=args.budget,pair_limit=args.pair_limit,
                                operating_mu=args.mu,
                                camera_clearance_m=args.camera_clearance_mm/1000,body_clearance_m=args.body_clearance_mm/1000)
        if args.record_needles:
            options=NeedleSettings(iterations=args.iterations,
                                   seed=args.seed,time_limit_s=args.map_time_limit).checked()
            radius=args.width_mm/2000 if not args.polygon and args.shape=="circle" else None
            if radius is None and (args.gathered_svg or args.gather_band_mm is not None):
                raise InputError("Gathering requires a circle template, not a custom polygon.")
            result=collect_needle_map(target,geometry,settings,needle_settings=options,circle_radius_m=radius)
            if radius is not None:
                result["needle_map"]["gathered"]=gather_needles(result["needle_map"],
                    band_m=[x/1000 for x in args.gather_band_mm] if args.gather_band_mm is not None else None,
                    support_fraction=args.gather_support,
                    anchor_angle_deg=args.gather_angle_deg)
            if args.needle_svg:save_new(args.needle_svg,needle_svg_document(result))
            if args.gathered_svg:save_new(args.gathered_svg,needle_svg_document(result,gathered=True))
        else:
            if args.needle_svg or args.gathered_svg or args.gather_band_mm is not None:
                raise InputError("Needle/gather exports require --record-needles.")
            result=solve(target,geometry,settings)
        text=json.dumps(result,indent=2,allow_nan=False)+"\n"
        if args.svg:save_new(args.svg,svg_document(result))
        if args.output:
            save_new(args.output,text)
            print(json.dumps(dict(output=str(args.output),**result["search"])),flush=True)
        else:print(text,end="")
        return 0 if (result["needle_map"]["needles"] if args.record_needles else result["candidates"]) else 2
    except (InputError,ValueError,TypeError,OSError) as exc:
        parser.error(str(exc))





def needle_svg_document(result,*,gathered=False):
    """Standalone map SVG. Lines remain discrete: no filled capture-region claim."""
    result=filter_result_at_mu(result,result["settings"]["operating_mu"])
    m=result["needle_map"];g=m.get("gathered") if gathered else None
    if gathered and g is None:raise InputError("Gathering requires a circle target.")
    rows=g["needles"] if g is not None else m["needles"]
    vertices=np.asarray(result["target_vertices_m"]);points=[*vertices]
    for n in rows:points.extend(n["band_segment_xy_m"] if g is not None else n["segment_xy_m"])
    if g is not None:points.extend(g["band_anchor_xy_m"])
    points=np.asarray(points);low=points.min(0);high=points.max(0);centre=(low+high)/2
    span=max(float(max(high-low)),.15)*1.3;sw=span*.0018
    chunks=['<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="1000" viewBox="0 0 1100 1000"><rect width="1100" height="1000" fill="white"/>',
            f'<svg x="0" y="140" width="1100" height="850" viewBox="{centre[0]-span/2} {-centre[1]-span/2} {span} {span}">',
            '<rect x="-100" y="-100" width="200" height="200" fill="white"/>',
            '<defs><marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="5" markerHeight="5" orient="auto"><path d="M0 0 L10 5 L0 10z" fill="#477ba8"/></marker></defs>',
            '<g transform="scale(1,-1)">']
    if m["target"]["kind"]=="circle":
        chunks.append(f'<circle cx="0" cy="0" r="{m["target"]["radius_m"]}" fill="#e6edf3" stroke="#60788e" stroke-width="{sw}"/>')
    else:
        coords=' '.join(f'{p[0]},{p[1]}' for p in vertices)
        chunks.append(f'<polygon points="{coords}" fill="#e6edf3" stroke="#60788e" stroke-width="{sw}"/>')
    if g is not None and g["band_m"] is not None:
        for radius in g["band_m"]:
            chunks.append(f'<circle cx="0" cy="0" r="{radius}" fill="none" stroke="#bccbd7" stroke-dasharray="{sw*4} {sw*4}" stroke-width="{sw*.5}"/>')
    for n in rows:
        segment=n["band_segment_xy_m"] if g is not None else n["segment_xy_m"]
        a,b=segment[1],segment[0]
        chunks.append(f'<line x1="{a[0]}" y1="{a[1]}" x2="{b[0]}" y2="{b[1]}" stroke="#477ba8" stroke-opacity=".65" stroke-width="{sw}" marker-end="url(#arrow)"><title>{n["id"]}: required mu {n["friction"][m["filter"]["field"]]:.5f}, heading {n["heading_deg"]:.3f} deg</title></line>')
    if g is not None and g["band_anchor_xy_m"]:
        a,b=g["band_anchor_xy_m"];chunks.append(f'<line x1="{a[0]}" y1="{a[1]}" x2="{b[0]}" y2="{b[1]}" stroke="#bd3c48" stroke-width="{sw*2}"/>')
    chunks.append('</g></svg>')
    title=f'{"Gathered" if gathered else "Scattered"} needles: {len(rows)} | required mu &lt; {m["filter"]["maximum"]}'
    labels=[title,f'{m["iterations_completed"]}/{m["iterations_requested"]} candidate evaluations | GO2 base rail segments | static planar arm model',
            'GO2 body/legs/plate, approach and holding are unverified; gaps are not interpolated.']
    if g is not None:labels.append(f'Band {g["band_m"][0]*1000:.3f}..{g["band_m"][1]*1000:.3f} mm | outside band: {g["outside_band"]} | symmetry duplicates: {g["duplicate_rotations"]}' if g["band_m"] is not None else 'No recorded needles at this filter.')
    for i,label in enumerate(labels):
        chunks.append(f'<text x="28" y="{32+i*26}" font-family="sans-serif" font-size="{22 if i==0 else 17}" fill="#40576b">{label}</text>')
    return ''.join(chunks)+'</svg>'


def svg_document(result,index=0,mu=None):
    """Pixel-coordinate SVG export of the same physical CAD polygons as the UI."""
    vertices=np.asarray(result["target_vertices_m"])
    candidate=result["candidates"][index] if result["candidates"] else None
    mu=result["settings"]["operating_mu"] if mu is None else float(mu)
    if not math.isfinite(mu) or mu<0:raise InputError("SVG friction must be nonnegative.")
    if candidate and not closure_at_mu(candidate["friction"],mu):candidate=None
    surfaces=candidate["surface_polygons_m"] if candidate else []
    points=np.vstack([vertices]+[np.asarray(p["points_m"]) for p in surfaces])
    low,high=np.min(points,axis=0),np.max(points,axis=0)
    span=max(float(max(high-low)),.12);center=(low+high)/2;scale=820/span
    def xy(p):
        a=np.asarray(p)
        return np.column_stack((500+(a[:,0]-center[0])*scale,530-(a[:,1]-center[1])*scale))
    def polygon(p,stroke,fill,width=1.5,opacity=1):
        text=" ".join(f"{x:.5f},{y:.5f}" for x,y in xy(p))
        return f'<polygon points="{text}" stroke="{stroke}" fill="{fill}" fill-opacity="{opacity}" stroke-width="{width}" stroke-linejoin="round"/>'
    chunks=['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="1000" viewBox="0 0 1000 1000">',
            '<rect width="1000" height="1000" fill="white"/>',polygon(vertices,"#60788e","#e6edf3")]
    colors=dict(first=("#22659c","#8eb9dc"),second=("#be6b1d","#f2bd8a"),
                gripper=("#ac3540","#df969c"),gripper_base=("#8d6241","#d5b49a"),camera=("#156e6b","#a5d5d0"),body=("#61758a","#c4d0dc"),wedge=("#786181","#d6c9df"))
    for p in sorted(surfaces,key=lambda v:0 if v["section"]=="body" else 3 if v["part"]=="camera" else 2 if v["part"]=="gripper_base" else 1):
        key="wedge" if p["part"]=="wedge" else "gripper_base" if p["part"]=="gripper_base" else p["section"]
        stroke,fill=colors[key]
        if p["part"] in PROTECTED_PART_NAMES:
            clearance=result["settings"]["camera_clearance_m" if p["part"]=="camera" else "body_clearance_m"]
            if clearance>0:chunks.append(polygon(p["points_m"],"#52b5af","none",width=2*clearance*scale))
        chunks.append(polygon(p["points_m"],stroke,fill))
        if p["part"]=="camera":
            q=xy(p["points_m"]);x,y=float(q[:,0].min()),float(q[:,1].min())
            chunks.append(f'<text x="{x}" y="{y-7}" font-size="13" font-family="sans-serif" fill="#156e6b">ZED Mini</text>')
    if candidate:
        alpha=math.atan(mu);length=.12*span
        for i,(point,normal) in enumerate(zip(candidate["contacts_m"],candidate["inward_normals"])):
            point,normal=np.array(point),np.array(normal);angle=math.atan2(normal[1],normal[0])
            cone=np.vstack((point,point+length*np.array([math.cos(angle-alpha),math.sin(angle-alpha)]),
                            point+length*np.array([math.cos(angle+alpha),math.sin(angle+alpha)])))
            chunks.append(polygon(cone,"#348b65","#83bb91",opacity=.35))
            start,end=xy([point,point+.9*length*normal])
            chunks.append(f'<line x1="{start[0]}" y1="{start[1]}" x2="{end[0]}" y2="{end[1]}" stroke="#173c2c" stroke-width="2"/>')
            x,y=start
            chunks.append(f'<circle cx="{x}" cy="{y}" r="4" fill="#173044"/><text x="{x+8}" y="{y-8}" font-family="sans-serif" font-size="16">P{i+1}</text>')
    label=(f'Required mu* {candidate["friction"]["mu_critical"]:.4f} | cutoff {mu:.4f}' if candidate else
           "Not found within bounded search")
    chunks += ['<text x="28" y="34" font-size="22" font-family="sans-serif" fill="#183047">newhug | URDF arm + full gripper + protected ZED Mini</text>',
        f'<text x="28" y="65" font-size="18" font-family="sans-serif" fill="#183047">{label}</text>',
        '<text x="28" y="980" font-size="13" font-family="sans-serif" fill="#52697c">Top view / arm roll 90 deg / housing rectangle. Motor force and actual holding remain unverified.</text></svg>']
    return "".join(chunks)


def browser_document(token):
    return PAGE.replace("__TOKEN__",token).replace("__MODEL_JSON__",json.dumps(describe_arm(),allow_nan=False))

def open_calculator_browser(url):
    """Use the Windows default browser when running under WSL without xdg-open."""
    try:
        release=Path("/proc/sys/kernel/osrelease")
        if release.exists() and "microsoft" in release.read_text().lower():
            explorer=Path("/mnt/c/Windows/explorer.exe")
            if explorer.exists():
                subprocess.Popen([str(explorer),url],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                return
        if not webbrowser.open(url):
            print("Automatic browser opening unavailable; open the printed URL manually.",flush=True)
    except OSError:
        print("Open the printed URL manually in your browser.",flush=True)



def self_test():
    import unittest
    from scipy.optimize import linprog
    import xml.etree.ElementTree as ET

    class FrictionTests(unittest.TestCase):
        points=np.array([[-.1,0.],[.1,0.]])
        normals=np.array([[1.,0.],[-1.,0.]])

        def test_opposed_contacts_have_zero_threshold_but_need_positive_mu(self):
            score=friction_score(self.points,self.normals)
            self.assertEqual(score["mu_critical"],0.)
            self.assertFalse(closure_at_mu(score,0.))
            self.assertTrue(closure_at_mu(score,.0001))

        def test_sixty_degree_contact_sets_sqrt_three_threshold(self):
            n=[[math.cos(math.pi/6),math.sin(math.pi/6)],
               [-math.cos(math.pi/3),math.sin(math.pi/3)]]
            self.assertAlmostEqual(friction_score(self.points,n)["mu_critical"],math.sqrt(3))

        def test_exact_threshold_is_not_interior_force_closure(self):
            n=rotate(self.normals,.4)
            score=friction_score(self.points,n)
            self.assertFalse(closure_at_mu(score,score["mu_critical"]))
            self.assertTrue(closure_at_mu(score,score["mu_critical"]+1e-5))

        def test_unilateral_half_space_cannot_be_rescued_by_friction(self):
            for n in ([[0,1],[0,1]],[[-1,0],[1,0]]):
                score=friction_score(self.points,n)
                self.assertTrue(math.isinf(score["mu_critical"]))
                self.assertFalse(closure_at_mu(score,100000.))
                self.assertIsNone(finite_score(score)["mu_critical"])

        def test_near_ninety_degrees_grows_without_bound(self):
            value=friction_score(self.points,rotate(self.normals,math.radians(89)))["mu_critical"]
            self.assertAlmostEqual(value,math.tan(math.radians(89)))
            self.assertGreater(value,57.)

        def test_finite_high_threshold_is_cut_off_by_available_mu(self):
            score=friction_score(self.points,rotate(self.normals,math.radians(89)))
            self.assertFalse(closure_at_mu(score,1.))
            self.assertTrue(closure_at_mu(score,60.))

        def test_score_contains_no_added_uncertainty(self):
            score=friction_score(self.points,rotate(self.normals,math.radians(30)))
            self.assertAlmostEqual(score["mu_critical"],math.tan(math.radians(30)))
            self.assertNotIn("mu_robust",score)

        def test_translation_rotation_scale_and_contact_order_do_not_change_score(self):
            normals=rotate(self.normals,.3)
            expected=friction_score(self.points,normals)["mu_critical"]
            p=rotate(self.points*1000,1.2)+[50,-70]
            n=rotate(normals,1.2)
            self.assertAlmostEqual(friction_score(p,n)["mu_critical"],expected)
            self.assertAlmostEqual(friction_score(p[::-1],n[::-1])["mu_critical"],expected)

        def test_invalid_contacts_and_normals_fail_explicitly(self):
            for p,n in (([[0,0],[0,0]],self.normals),(self.points,[[0,0],[1,0]]),
                        ([[0,0]],self.normals),([[0,0],[float("nan"),0]],self.normals)):
                with self.subTest(p=p,n=n),self.assertRaises(InputError):
                    friction_score(p,n)

        def test_threshold_agrees_with_independent_wrench_linear_programs(self):
            rng=np.random.default_rng(7)
            for _ in range(25):
                a,b=rng.uniform(-1.1,1.1,2)
                normals=np.array([[math.cos(a),math.sin(a)],[-math.cos(b),math.sin(b)]])
                critical=friction_score(self.points,normals)["mu_critical"]
                for mu,expected in ((max(0.,critical-.02),False),(critical+.02,True)):
                    tangent=np.column_stack((-normals[:,1],normals[:,0]))
                    forces=np.stack((normals+mu*tangent,normals-mu*tangent),axis=1).reshape(4,2)
                    points=np.repeat(self.points,2,axis=0)
                    matrix=np.vstack((forces.T,cross(points,forces)/.1))
                    equal=np.zeros((4,5));equal[:3,:4]=matrix;equal[3,:4]=1
                    result=linprog([0,0,0,0,-1],A_ub=np.column_stack((-np.eye(4),np.ones(4))),
                        b_ub=np.zeros(4),A_eq=equal,b_eq=[0,0,0,1],bounds=[(0,None)]*5,method="highs")
                    closure=bool(np.linalg.matrix_rank(matrix)==3 and result.success and result.x[-1]>1e-8)
                    self.assertEqual(closure,expected)


    class GeometryTests(unittest.TestCase):
        def test_planar_fk_matches_the_independent_assembly_fk_golden_data(self):
            for fixture in REFERENCE["golden_fk"]:
                q=np.radians(fixture["section_turns_deg"])
                hinges,angles=fk_frames(q)
                for i in range(10):
                    np.testing.assert_allclose(hinges[i],fixture["points"]["node"+str(i)],atol=2e-10)
                np.testing.assert_allclose(hinges[-1],fixture["points"]["arm_tip"],atol=2e-10)
                np.testing.assert_allclose(hinges[-1],fixture["points"]["gripper_base"],atol=2e-10)
                self.assertAlmostEqual(float(angles[-1]),sum(q))

        def test_neutral_arm_has_ten_fifty_mm_nodes_and_the_full_urdf_gripper(self):
            g=Geometry();surfaces,bodies,hinges,_=robot_polygons([0.,0.],g)
            np.testing.assert_allclose(np.linalg.norm(np.diff(hinges,axis=0),axis=1),.05,atol=1e-12)
            np.testing.assert_allclose(hinges[-1],[0,.5],atol=1e-12)
            self.assertAlmostEqual(float(surfaces[12][:,1].max()),.589)
            self.assertEqual(len(surfaces),14)
            self.assertEqual(len(bodies),2)
            np.testing.assert_allclose(np.ptp(bodies[1],axis=0),[.084,.028],atol=1e-12)
            self.assertTrue(any(np.linalg.norm(v)<1e-12 for v in bodies[1]))
            self.assertEqual(bodies[0].shape,(4,2))
            np.testing.assert_allclose(np.ptp(bodies[0],axis=0),[.130,.233],atol=1e-12)
            np.testing.assert_allclose(bodies[0].max(axis=0),[.0635,-.028],atol=1e-12)

        def test_bending_plane_dimensions_are_not_the_out_of_plane_widths(self):
            p=node_profile(Geometry());tool=gripper_profile(Geometry())
            self.assertEqual(tool.shape,(8,2))
            self.assertAlmostEqual(float(np.ptp(p[:,0])),.070)
            self.assertAlmostEqual(float(np.ptp(tool[:,0])),.0605)
            self.assertAlmostEqual(float(tool[:,0].min()),-.025)
            self.assertAlmostEqual(float(tool[:,0].max()),.0355)
            self.assertAlmostEqual(float(tool[:,1].max()),.089)

        def test_each_node_turn_is_one_fifth_of_the_group_total(self):
            g=Geometry();hinges,angles=fk_frames(np.radians([180,-180]),g)
            np.testing.assert_allclose(np.degrees(angles[:5]),[36,72,108,144,180],atol=1e-10)
            np.testing.assert_allclose(np.degrees(angles[5:10]),[144,108,72,36,0],atol=1e-10)
            self.assertEqual(g.turn_limit_deg,180)

        def test_terminal_node_is_square_fronted_and_both_fingers_are_included(self):
            ordinary,last=node_profile(Geometry()),node_profile(Geometry(),last=True)
            self.assertEqual(int(np.sum(np.isclose(ordinary[:,1],.05))),1)
            self.assertEqual(int(np.sum(np.isclose(last[:,1],.05))),2)
            info=describe_arm()
            self.assertEqual(len(info["local_profiles_m"]),14)
            self.assertEqual(len(info["collision_profiles_m"]),14)
            self.assertIn("gripper_claw_left",info["collision_profiles_m"])
            self.assertIn("gripper_claw_right",info["collision_profiles_m"])
            self.assertNotIn("gripper_claw_left",info["excluded_parts"])
            self.assertIn("camera",info["protected_part_names"])

        def test_embedded_profiles_match_the_declared_arm_model(self):
            g=Geometry()
            for i,poly in enumerate(local_profiles(g)):
                key="node" if i<9 else "node_end" if i==9 else PART_NAMES[i]
                expected=np.array(REFERENCE["profiles"][key])
                distance=np.linalg.norm(poly[:,None]-expected[None],axis=2)
                self.assertLess(float(np.max(np.min(distance,axis=0))),1e-10)
                self.assertLess(float(np.max(np.min(distance,axis=1))),1e-10)

        def test_physical_joint_and_geometry_limits_are_validated(self):
            for g in (Geometry(joint_limit_deg=40),Geometry(nodes_per_section=4),
                      Geometry(pitch_m=0),Geometry(half_thickness_m=0)):
                with self.assertRaises(InputError):g.checked()
            for settings in (SearchSettings(budget_s=float("nan")),SearchSettings(pair_limit=0)):
                with self.assertRaises(InputError):settings.checked()

        def test_touching_polygons_are_not_mistaken_for_interpenetration(self):
            p=np.array([[0,0],[1,0],[1,1],[0,1]],dtype=float)
            self.assertAlmostEqual(convex_gap(p,p+[1,0]),0)
            self.assertLess(convex_gap(p,p+[.5,0]),-.49)
            self.assertGreater(convex_gap(p,p+[2,0]),.99)

        def test_crossing_polygons_are_detected_without_inside_vertices(self):
            a=np.array([[-1,-.1],[1,-.1],[1,.1],[-1,.1]])
            b=np.array([[-.1,-1],[.1,-1],[.1,1],[-.1,1]])
            self.assertFalse(inside_polygon(a,b).any())
            self.assertFalse(inside_polygon(b,a).any())
            self.assertLess(convex_gap(a,b),0)

        def test_concave_notch_is_not_filled_by_a_convex_hull(self):
            p=np.array([[-.2,-.2],[.2,-.2],[.2,.2],[.1,.2],[.1,-.1],[-.1,-.1],[-.1,.2],[-.2,.2]])
            square=np.array([[-.04,-.04],[.04,-.04],[.04,.04],[-.04,.04]])
            self.assertGreater(target_gap(square,convex_parts(p)),.05)

        def test_ear_clipping_preserves_target_area(self):
            p=template("jar",.14,.18)
            expected,_=polygon_area_centroid(p)
            area=sum(polygon_area_centroid(t)[0] for t in convex_parts(p))
            self.assertAlmostEqual(area,expected,places=12)

        def test_neutral_meshes_do_not_self_penetrate(self):
            surfaces,bodies,_,_=robot_polygons([0,0])
            self.assertGreaterEqual(float(self_gaps(surfaces,bodies).min()),-1e-10)

        def test_full_loop_detects_real_body_or_link_collision(self):
            surfaces,bodies,_,_=robot_polygons([math.pi,math.pi])
            self.assertLess(float(self_gaps(surfaces,bodies).min()),-.001)

        def test_contact_normal_follows_the_real_node_face(self):
            g=Geometry();faces=robot_faces(g)
            fa=next(f for f in faces if f.part==3 and f.edge==1)
            fb=next(f for f in faces if f.part==10 and f.edge==0)
            values=np.r_[np.radians([100,-40]),.2,.7]
            p1,p2,n1,n2,t1,t2=contact_frame(values,fa,fb,g)
            self.assertAlmostEqual(float(np.dot(n1,[math.cos(t1),math.sin(t1)])),0)
            self.assertAlmostEqual(float(np.dot(n2,[math.cos(t2),math.sin(t2)])),0)
            self.assertAlmostEqual(float(np.linalg.norm(n1)),1)

        def test_body_collision_is_classified_with_housing_rectangle(self):
            g=Geometry();settings=SearchSettings()
            vertices=np.array([[-.01,-.11],[.01,-.11],[.01,-.09],[-.01,-.09]])
            features=(Feature(vertices[0],np.array([0,-1]),0,0),
                      Feature(vertices[1],np.array([0,-1]),0,.5))
            faces=robot_faces(g)
            metrics,*_=validate_configuration(vertices,convex_parts(vertices),[0,0],0,np.zeros(2),
                faces[0],faces[-1],.5,.5,features,g,settings)
            self.assertIn("body_target_collision",metrics["reasons"])

        def test_cancelled_search_does_not_build_or_claim_a_grasp(self):
            r=solve(template("circle",.16,.16),cancelled=lambda:True)
            self.assertEqual(r["search"]["status"],"cancelled")
            self.assertEqual(r["search"]["attempts"],0)
            self.assertEqual(r["search"]["atlas_rows"],0)
            self.assertFalse(r["search"]["exhaustive"])

        def test_custom_target_and_svg_stay_standalone(self):
            vertices,g,s=build_request(dict(shape="custom",vertices_mm=[[-50,-50],[50,-50],[0,50]]))
            self.assertAlmostEqual(float(np.ptp(vertices[:,0])),.1)
            result=dict(target_vertices_m=vertices.tolist(),candidates=[],fixed_geometry=asdict(g),settings=asdict(s))
            self.assertTrue(ET.fromstring(svg_document(result)).tag.endswith("svg"))
            self.assertIn("protected ZED Mini",svg_document(result))
            with self.assertRaises(InputError):build_request(dict(geometry=dict(unknown=1)))

        def test_self_intersection_is_rejected(self):
            with self.assertRaises(InputError):clean_polygon([[0,0],[1,1],[0,1],[1,0]])

        def test_gripper_contacts_use_only_the_exposed_union_boundary(self):
            outline=gripper_profile();faces=[f for f in robot_faces(Geometry()) if 10<=f.part<CAMERA_INDEX]
            self.assertGreaterEqual(len(faces),len(outline))
            for f in faces:
                for point in (f.first,(f.first+f.second)/2,f.second):
                    self.assertLess(float(nearest_boundary(np.array([point]),outline)[0][0]),1e-9)
            self.assertFalse(any(f.part==CAMERA_INDEX for f in robot_faces(Geometry())))

        def test_camera_touch_and_clearance_are_separate_from_grasp_contacts(self):
            surfaces,_,_,_=robot_polygons([0,0]);camera=surfaces[CAMERA_INDEX]
            right=float(camera[:,0].max());y=.510
            touching=np.array([[right,y],[right+.002,y],[right+.002,y+.002],[right,y+.002]])
            self.assertLessEqual(protected_camera_gap(camera,touching,convex_parts(touching)),1e-9)
            separated=touching+[.0015,0]
            self.assertGreater(protected_camera_gap(camera,separated,convex_parts(separated)),.001)
            features=(Feature(touching[0],np.array([-1,0]),-math.pi/2,0),
                      Feature(touching[1],np.array([1,0]),math.pi/2,.5))
            faces=robot_faces(Geometry())
            metrics,*_=validate_configuration(touching,convex_parts(touching),[0,0],0,np.zeros(2),
                faces[0],faces[-1],.5,.5,features,Geometry(),SearchSettings())
            self.assertIn("camera_contact",metrics["reasons"])
            self.assertIn("camera_clearance",metrics["reasons"])
            zero_gap,*_=validate_configuration(touching,convex_parts(touching),[0,0],0,np.zeros(2),
                faces[0],faces[-1],.5,.5,features,Geometry(),SearchSettings(camera_clearance_m=0))
            self.assertIn("camera_contact",zero_gap["reasons"])
            self.assertFalse(zero_gap["valid"])
            with self.assertRaises(InputError):SearchSettings(camera_clearance_m=-.001).checked()

        def test_contact_pair_exploration_reaches_high_friction_layouts(self):
            vertices=template("circle",.1,.1);settings=SearchSettings()
            pairs=select_pairs(boundary_features(vertices,settings.contact_samples),settings,.005)
            self.assertEqual(len(pairs),settings.pair_limit)
            self.assertGreater(max(score for _,_,score in pairs),3.)
            self.assertLess(min(score for _,_,score in pairs),.1)

        def test_100_mm_circle_finds_safe_force_closure(self):
            settings=SearchSettings(budget_s=8.,max_solves=8)
            vertices=template("circle",.1,.1);result=solve(vertices,settings=settings)
            self.assertTrue(result["candidates"],result["search"])
            self.assertTrue(any(c["planar_force_closure_at_operating_mu"] for c in result["candidates"]))
            for candidate in result["candidates"]:
                metrics=candidate["geometry"]
                self.assertTrue(metrics["valid"])
                self.assertGreater(metrics["camera_target_clearance_m"],settings.camera_clearance_m)
                self.assertGreaterEqual(metrics["minimum_object_gap_m"],-settings.tolerance_m)
                for point,inward in zip(candidate["contacts_m"],candidate["inward_normals"]):
                    self.assertGreaterEqual(float(np.min((vertices-point)@inward)),-1e-9)
                self.assertFalse(any(s["part"]=="camera" for s in candidate["contact_surfaces"]))

        def test_160_mm_circle_keeps_a_safe_force_closure_candidate(self):
            settings=SearchSettings(budget_s=8.,max_solves=32)
            result=solve(template("circle",.16,.16),settings=settings)
            self.assertTrue(result["candidates"],result["search"])
            good=[c for c in result["candidates"] if c["planar_force_closure_at_operating_mu"]]
            self.assertTrue(good,result["search"])
            self.assertTrue(all(c["geometry"]["valid"] for c in good))
            self.assertTrue(all(c["geometry"]["camera_target_clearance_m"]>settings.camera_clearance_m
                                for c in good))

        def _validate_known_100mm_pose(self,**overrides):
            # Fixed feasible certificate, independent of the candidate search:
            # node bends -12 / -33 deg, node2 and the left jaw in contact.
            vertices=template("circle",.1,.1);faces=robot_faces(Geometry())
            first=next(f for f in faces if f.part==2 and f.edge==4)
            second=next(f for f in faces if f.part==11 and f.edge==6)
            features=(Feature(vertices[26],np.array([-math.cos(math.radians(36)),
                math.sin(math.radians(36))]),math.radians(-126),26/64),
                Feature(vertices[56],np.array([math.sqrt(.5),-math.sqrt(.5)]),math.pi/4,56/64))
            args=dict(vertices=vertices,target_parts=convex_parts(vertices),
                turns=np.radians([-60.,-165.]),rotation=0.,
                translation=np.array([.10769081870104287,.05596375074339059]),
                face1=first,face2=second,a=.02402688712693026,b=.2884931891286458,
                features=features,g=Geometry(),settings=SearchSettings())
            args.update(overrides)
            return validate_configuration(**args)[0]

        def test_final_validator_rejects_33_degree_pose_under_32_degree_limit(self):
            original=self._validate_known_100mm_pose()
            self.assertTrue(original["valid"],original)
            boundary=self._validate_known_100mm_pose(g=Geometry(joint_limit_deg=33.))
            self.assertTrue(boundary["valid"],boundary)
            self.assertAlmostEqual(boundary["minimum_joint_headroom_deg"],0.)
            rejected=self._validate_known_100mm_pose(g=Geometry(joint_limit_deg=32.))
            self.assertFalse(rejected["valid"],rejected)
            self.assertIn("joint_limit_exceeded",rejected["reasons"])
            self.assertAlmostEqual(rejected["minimum_joint_headroom_deg"],-1.)

        def test_final_validator_checks_both_section_limits_in_both_directions(self):
            for section in range(2):
                for sign in (-1,1):
                    for offset in (-1e-6,0.,1e-12,1e-6):
                        with self.subTest(section=section,sign=sign,offset=offset):
                            bends=np.zeros(2);bends[section]=sign*(32.+offset)
                            result=self._validate_known_100mm_pose(
                                turns=np.radians(bends*5),g=Geometry(joint_limit_deg=32.))
                            self.assertEqual("joint_limit_exceeded" in result["reasons"],offset>1e-9)
                            if offset==0.:
                                self.assertEqual(result["minimum_joint_headroom_deg"],0.)

        def test_final_validator_checks_model_and_settings_itself(self):
            for model in (Geometry(joint_limit_deg=40.),Geometry(nodes_per_section=4)):
                with self.subTest(model=model):
                    with self.assertRaises(InputError):self._validate_known_100mm_pose(g=model)
            for settings in (SearchSettings(camera_clearance_m=-.001),SearchSettings(tolerance_m=.002)):
                with self.subTest(settings=settings):
                    with self.assertRaises(InputError):self._validate_known_100mm_pose(settings=settings)

        def test_browser_receives_real_profiles_without_filesystem_access(self):
            page=browser_document("test")
            self.assertNotIn("__MODEL_JSON__",page)
            self.assertIn("gripper_claw_left",page)
            self.assertIn("ZED Mini",page)
            self.assertIn("89 mm",page)

    suite=unittest.TestSuite()
    suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(FrictionTests))
    suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(GeometryTests))
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__=="__main__":
    raise SystemExit(main())
