"""Audit hug criteria with the existing gripper and section lengths fixed.

The original solver remains unchanged. Its variable-length results and gates
are recorded separately from a bounded fixed-length search. All certificates
here concern an explicit planar, zero-thickness, two-point contact model.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, dataclass
import hashlib
import importlib.util
import itertools
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
from scipy.optimize import least_squares, linprog


ROOT = Path(__file__).resolve().parents[3]
DEFAULT_REFERENCE = ROOT/'payload/runtime/docker/pilot/app/elesim_pilot/pick/hug_reference.py'


@dataclass(frozen=True)
class Settings:
    length_m: float = .25
    body_length_m: float = .0875
    body_width_m: float = .07
    turn_limit_rad: float = math.pi
    budget_s: float = 8.
    legacy_budget_s: float = 2.
    pair_limit: int = 32
    max_solves: int = 512
    max_evaluations: int = 60
    correction_deg: float = 5.

    def checked(self):
        positive = ('length_m','body_length_m','body_width_m','turn_limit_rad',
                    'budget_s','legacy_budget_s','correction_deg')
        if any(not math.isfinite(getattr(self,k)) or getattr(self,k) <= 0 for k in positive):
            raise ValueError('geometry, budgets and correction limits must be finite and positive')
        if self.turn_limit_rad > math.pi:
            raise ValueError('this reference model supports at most pi per section')
        if any(type(getattr(self,k)) is not int or getattr(self,k) < 1
               for k in ('pair_limit','max_solves','max_evaluations')):
            raise ValueError('search limits must be positive integers')
        return self


def load_reference(path):
    """Load only an explicitly trusted solver source; never start its GUI."""
    path = Path(path).resolve(strict=True)
    name = '_elesim_hug_diagnostic_reference'
    spec = importlib.util.spec_from_file_location(name,path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def case_vertices(name):
    angle = np.linspace(0,2*math.pi,64,endpoint=False)
    if name == 'circle-120':
        return .06*np.column_stack((np.cos(angle),np.sin(angle)))
    if name == 'circle-160':
        return .08*np.column_stack((np.cos(angle),np.sin(angle)))
    if name == 'circle-200':
        return .10*np.column_stack((np.cos(angle),np.sin(angle)))
    if name == 'circle-240':
        return .12*np.column_stack((np.cos(angle),np.sin(angle)))
    if name == 'ellipse':
        return np.column_stack((.09*np.cos(angle),.13*np.sin(angle)))
    if name == 'rectangle':
        return np.array([[-.09,-.12],[.09,-.12],[.09,.12],[-.09,.12]])
    if name == 'triangle':
        angle = np.linspace(0,2*math.pi,3,endpoint=False)+math.pi/2
        return .15*np.column_stack((np.cos(angle),np.sin(angle)))
    if name == 'jar-outline':
        right = np.array([[.045,-.10],[.060,-.07],[.065,-.015],[.050,.04],
                          [.027,.065],[.027,.09],[.035,.09],[.035,.10]])
        return 1.4*np.vstack((right,np.column_stack((-right[::-1,0],right[::-1,1]))))
    raise ValueError('unknown case: '+name)


CASES = ('circle-120','circle-160','circle-200','circle-240','ellipse','rectangle','triangle','jar-outline')


def endpoint_witness(ref, settings=None):
    """A constructed no-stick circle fixture, independent of timed search.

    Include both contacts as polygon vertices to avoid polygonization gaps.
    The second section is saturated: existence alone does not imply headroom.
    """
    from shapely.geometry import Polygon
    settings = settings or Settings()
    turns = np.radians([120,180]);a = .8
    p1,p2,t1,_ = ref._contact_geometry(*turns,a,1.,'arc2',settings.length_m)
    normal = np.array([-math.sin(t1),math.cos(t1)])
    delta = p2-p1;radius = float(delta@delta/(2*delta@normal));center = p1+radius*normal
    contact_angles = [math.atan2(*(p-center)[::-1]) for p in (p1,p2)]
    angles = np.unique(np.r_[np.linspace(-math.pi,math.pi,256,endpoint=False),contact_angles])
    vertices = center+radius*np.column_stack((np.cos(angles),np.sin(angles)))
    centroid = np.asarray(Polygon(vertices).centroid.coords[0])
    target = ref.make_target(vertices,sdf_resolution=128)
    first_normal,second_normal = (p1-center)/radius,(p2-center)/radius
    second_tangent = math.atan2(second_normal[0],-second_normal[1])
    first = SimpleNamespace(point=p1-centroid,tangent=t1,normal=first_normal,u=0.)
    second = SimpleNamespace(point=p2-centroid,tangent=second_tangent,normal=second_normal,u=.5)
    pose = make_pose(ref,turns,first,second,a,'tip',0.,settings)
    return target,pose,first,second


def wrench_matrix(points, inward_normals, mu):
    points, normals = np.asarray(points,dtype=float), np.asarray(inward_normals,dtype=float)
    if (points.ndim != 2 or points.shape[1] != 2 or normals.shape != points.shape
            or not np.isfinite(points).all() or not np.isfinite(normals).all()
            or not math.isfinite(mu) or mu < 0):
        raise ValueError('finite planar contacts and nonnegative friction are required')
    norm = np.linalg.norm(normals,axis=1)
    if np.any(norm < 1e-12):
        raise ValueError('contact normal is zero')
    normals = normals/norm[:,None]
    tangents = np.column_stack((-normals[:,1],normals[:,0]))
    forces = np.stack((normals+mu*tangents,normals-mu*tangents),axis=1)
    # A positive length scale changes neither positive span nor equilibrium.
    scale = max(float(np.linalg.norm(points,axis=1).max()),1e-9)
    torque = (points[:,None,0]*forces[:,:,1]-points[:,None,1]*forces[:,:,0])/scale
    return np.column_stack((forces.reshape(-1,2),torque.reshape(-1))).T


def planar_force_closure(points, inward_normals, mu):
    matrix = wrench_matrix(points,inward_normals,mu)
    count = matrix.shape[1]
    rank = int(np.linalg.matrix_rank(matrix,tol=1e-9))
    if rank < 3:
        return dict(closure=False,rank=rank,positive_span_margin=0.)
    # Maximize epsilon with F k=0, sum(k)=1, k_i >= epsilon >= 0.
    equal = np.zeros((4,count+1));equal[:3,:count] = matrix;equal[3,:count] = 1.
    upper = np.column_stack((-np.eye(count),np.ones(count)))
    result = linprog(np.r_[np.zeros(count),-1.],A_ub=upper,b_ub=np.zeros(count),
        A_eq=equal,b_eq=[0.,0.,0.,1.],bounds=[(0,None)]*(count+1),method='highs')
    margin = float(result.x[-1]) if result.success else 0.
    return dict(closure=bool(margin > 1e-8),rank=rank,positive_span_margin=margin)


def planar_support_capacity(points, inward_normals, mu, direction=(0.,1.)):
    """Task force with zero moment and ONE assumed normal-force unit/contact.

    This is a contact-geometry proxy, not a calibrated motor/load capacity.
    The area centroid is the assumed torque origin, not a measured mass COM.
    """
    matrix = wrench_matrix(points,inward_normals,mu)
    direction = np.asarray(direction,dtype=float)
    if direction.shape != (2,) or not np.isfinite(direction).all() or np.linalg.norm(direction) < 1e-12:
        raise ValueError('task direction must be finite and nonzero')
    direction /= np.linalg.norm(direction)
    count = matrix.shape[1]
    equal = np.column_stack((matrix,-np.r_[direction,0.]))
    upper = np.zeros((len(points),count+1))
    for i in range(len(points)):
        upper[i,2*i:2*i+2] = 1.
    result = linprog(np.r_[np.zeros(count),-1.],A_ub=upper,b_ub=np.ones(len(points)),
        A_eq=equal,b_eq=np.zeros(3),bounds=[(0,None)]*(count+1),method='highs')
    return float(result.x[-1]) if result.success else None


def angle_margins(turns, limit=math.pi):
    values = np.asarray(turns,dtype=float)
    return dict(lower_deg=np.degrees(values+limit).tolist(),
        upper_deg=np.degrees(limit-values).tolist(),
        minimum_deg=float(np.degrees(np.min(limit-np.abs(values)))))


def legacy_forward(projection, tolerance):
    return bool(projection >= -tolerance)


def outcome(found, budget_exhausted):
    return ('candidates_found' if found else 'not_found_within_budget') + (
        '_budget_exhausted' if budget_exhausted else '')


def contact_geometry(ref, turns, a, second, mode, terminal, length):
    physical_mode = 'stick' if mode == 'stick' or (mode == 'tip' and terminal > 0) else 'arc2'
    if mode == 'tip':
        second = terminal if terminal > 0 else 1.
    return (*ref._contact_geometry(*turns,a,second,physical_mode,length),physical_mode,second)


def fixed_residual(ref, values, first, second_feature, a, mode, terminal, settings, winding,
                   orientation=0):
    turns = values[:2]
    second = float(values[2]) if mode != 'tip' else 1.
    p1,p2,t1,t2,_,_ = contact_geometry(ref,turns,a,second,mode,terminal,settings.length_m)
    rotated = ref.rotate(second_feature.point-first.point,t1-first.tangent+orientation*math.pi)
    positional = (p2-p1-rotated)/settings.length_m
    if mode == 'tip':
        return positional
    delta = ref.wrap_line(second_feature.tangent-first.tangent)
    angular = t2-t1-delta-winding*math.pi
    return np.r_[positional,angular]


def make_pose(ref, values, first, second_feature, a, mode, terminal, settings, orientation=0):
    turns = list(map(float,values[:2]))
    second = float(values[2]) if mode != 'tip' else 1.
    p1,p2,t1,t2,physical_mode,second = contact_geometry(
        ref,turns,a,second,mode,terminal,settings.length_m)
    rotation = ref.wrap_pi(t1-first.tangent+orientation*math.pi)
    translation = p1-ref.rotate(first.point,rotation)
    return dict(turns=turns,a=float(a),second=second,mode=physical_mode,
        search_modes=[mode],terminal_ratio=float(terminal),length_m=settings.length_m,
        rotation=float(rotation),translation=translation.tolist(),
        contact_points_local=[first.point.tolist(),second_feature.point.tolist()],
        inward_normals_local=[(-first.normal).tolist(),(-second_feature.normal).tolist()],
        tangent1=float(first.tangent),tangent2=float(second_feature.tangent),
        u1=float(first.u),u2=float(second_feature.u),
        endpoint_tangency_residual=float(abs(math.sin(t2-first.tangent-rotation
            -ref.wrap_line(second_feature.tangent-first.tangent)))))


def reference_candidate(ref, pose):
    first,second = pose['contact_points_local']
    return ref.Candidate(pose['mode'],*pose['turns'],pose['a'],pose['second'],pose['length_m'],
        pose['rotation'],np.array(pose['translation']),np.array(first),np.array(second),
        pose['tangent1'],pose['tangent2'],pose['u1'],pose['u2'],0)


def assess_geometry(ref, target, pose, settings, *, polygon=None, contact_tolerance=None):
    from shapely.geometry import LineString, Point, Polygon
    from shapely.ops import substring
    characteristic = target.major_axis
    tol = 1e-5*characteristic
    contact_tolerance = 2*tol if contact_tolerance is None else contact_tolerance
    if polygon is None:
        polygon = Polygon(ref.rotate(target.vertices,pose['rotation'])+pose['translation'])
    t1,t2 = pose['turns'];a,second = pose['a'],pose['second']
    gripper = ref.build_gripper(t1,t2,pose['terminal_ratio'],length=settings.length_m,
        samples=(ref._adaptive_samples(t1,tol,settings.length_m),
                 ref._adaptive_samples(t2,tol,settings.length_m)),with_region=False,
        extra1=(a,),extra2=(second,) if pose['mode']=='arc2' else (),
        extra_stick=(second,) if pose['mode']=='stick' else ())
    p1,p2,tangent1,_,_,_ = contact_geometry(ref,pose['turns'],a,second,pose['mode'],
        pose['terminal_ratio'],settings.length_m)
    line1,line2 = LineString(gripper.arc1),LineString(gripper.side2)
    chain = LineString(np.vstack((gripper.arc1,gripper.side2[1:])))
    inner = polygon.buffer(-tol)
    half = settings.body_width_m/2
    body = Polygon([[-half,0],[half,0],[half,-settings.body_length_m],[-half,-settings.body_length_m]])
    reasons = []
    if not inner.is_empty and (line1.intersects(inner) or line2.intersects(inner)):
        reasons.append('target_penetration_beyond_tolerance')
    if body.intersects(inner):
        reasons.append('body_target_collision')
    if not chain.is_simple:
        reasons.append('arm_self_intersection')
    if chain.intersects(body.buffer(-tol)):
        reasons.append('arm_body_collision')
    contacts = [float(polygon.boundary.distance(Point(*p))) for p in (p1,p2)]
    if any(distance > contact_tolerance for distance in contacts):
        reasons.append('intended_contact_missing')
    distance1 = a*line1.length
    distance2 = (second*LineString(gripper.arc2).length if pose['mode']=='arc2'
                 else LineString(gripper.arc2).length+second*settings.length_m)
    part1,part2 = substring(line1,distance1,line1.length),substring(line2,0,distance2)
    coordinates = np.vstack((np.asarray(part1.coords),np.asarray(part2.coords)[1:]))
    capture = Polygon(coordinates) if len(coordinates)>=3 else Polygon()
    captured = bool(capture.is_valid and capture.area > 1e-6*characteristic**2
                    and capture.contains(polygon.centroid))
    depth = float(capture.boundary.distance(polygon.centroid)) if captured else 0.
    if not captured:
        reasons.append('area_centroid_outside_capture_proxy')
    elif depth < ref.Settings().centroid_margin_relative*characteristic:
        reasons.append('area_centroid_margin_below_reference_threshold')
    forward = float((p2-p1)@np.array([math.cos(tangent1),math.sin(tangent1)]))
    return dict(geometric_proxy_pass=not reasons,reasons=reasons,centroid_captured=captured,
        centroid_margin_m=depth,capture_score=depth/(depth+characteristic),
        forward_projection_m=forward,legacy_forward_pass=legacy_forward(forward,tol),
        contact_gaps_m=contacts,body_target_clearance_m=float(body.distance(polygon)),
        opening_chord_m=float(np.linalg.norm(gripper.tip-gripper.e1)),
        section_angle_margins=angle_margins(pose['turns'],settings.turn_limit_rad),
        hold_verified=None,motor_load_margin=None,compression_margin=None)


def contact_proxies(pose):
    points,normals = pose['contact_points_local'],pose['inward_normals_local']
    angle = -pose['rotation']
    direction = [-math.sin(angle),math.cos(angle)]
    return dict(assumptions='Two planar point contacts, chosen feature normals, Coulomb friction; '
        'one ideal normal-force unit per contact for a fixed base-plane +Y task proxy; '
        'this axis is not a verified gravity direction; area centroid as torque origin. '
        'No actuator coupling, compliance or measured mass/force model.',
        by_friction=[dict(mu=mu,**planar_force_closure(points,normals,mu),
            task_support_per_assumed_normal_unit=planar_support_capacity(points,normals,mu,direction))
            for mu in (0.,.3,.6)])


def legacy_run(ref, target, terminal, settings):
    stats = {}
    config = ref.Settings(n=terminal,time_limit=settings.legacy_budget_s,solution_count=3,
        boundary_levels=(32,64),pair_limits=(64,128),sdf_resolution=128,
        contact_grid=5,turn_samples=33,refinement_levels=1)
    began = time.monotonic()
    try:
        solutions = ref.solve_target(target,config,progress=lambda row:stats.update(row))
        error = ''
    except ref.DesignError as exc:
        solutions,error = (),str(exc)
    return dict(scope='Unmodified variable-length solver; not the fixed-length comparison.',
        elapsed_s=time.monotonic()-began,error=error,stats=stats,
        solutions=[dict(length_m=float(s.length),turns=list(map(float,(s.gripper.turn1,s.gripper.turn2))),
            mode=s.mode,score=float(s.score),length_error_m=float(s.length-settings.length_m)) for s in solutions])


class SearchDeadline(Exception):
    pass


def finite_capture_pairs(pairs, minimum_margin):
    """Discard chords too close to the centroid to meet the capture margin.

    Targets are centered at their area centroid. The virtual chord is part of
    the capture boundary, so this is a necessary, not sufficient, condition.
    """
    if not pairs:
        return []
    first = np.array([p.first.point for p in pairs])
    chord = np.array([p.second.point-p.first.point for p in pairs])
    fraction = np.clip(-np.einsum('ij,ij->i',first,chord)/np.einsum('ij,ij->i',chord,chord),0,1)
    distance = np.linalg.norm(first+fraction[:,None]*chord,axis=1)
    return [p for p,keep in zip(pairs,distance >= minimum_margin) if keep]


def fixed_search(ref, target, terminal, settings):
    settings.checked()
    began = time.monotonic(); deadline = began+settings.budget_s
    features = ref.boundary_features(target,32,cone_step=30.)
    config = ref.Settings(n=terminal)
    raw_pairs = ref._contact_pairs(features,config,target)
    admissible = finite_capture_pairs(raw_pairs,config.centroid_margin_relative*target.major_axis)
    pairs = ref._select_pairs(admissible,settings.pair_limit)
    modes = ('tip','arc2','stick') if terminal > 0 else ('tip','arc2')
    seeds = np.radians([[120,180],[-120,-180],[135,135],[-135,-135],[90,180],[-90,-180]])
    solves = 0;counts = Counter();rejections = Counter();pool = {};budget_exhausted = False
    # Rotate seeds and both tangent orientations across pairs from the start.
    # The first material contact a is optimized continuously, never by scaling L.
    variants = list(itertools.product(range(len(seeds)),(0,1),(.8,.4)))
    tasks = ((pair,*variants[(round_index+5*pair_index)%len(variants)],mode)
        for round_index in range(len(variants)) for pair_index,pair in enumerate(pairs) for mode in modes)
    for pair,seed_index,orientation,a,mode in tasks:
        if solves >= settings.max_solves or time.monotonic() >= deadline:
            budget_exhausted = True;break
        initial = np.clip(seeds[seed_index],-settings.turn_limit_rad+1e-7,settings.turn_limit_rad-1e-7)
        if mode != 'tip':
            second = .75 if mode=='arc2' else .6*terminal
            initial = np.r_[initial,second]
        initial = np.r_[initial,a]
        delta = ref.wrap_line(pair.second.tangent-pair.first.tangent)
        total = (1-a)*initial[0]+(initial[2]*initial[1] if mode=='arc2' else initial[1])
        winding = int(round((total-delta)/math.pi))
        low = [-settings.turn_limit_rad]*2;high = [settings.turn_limit_rad]*2
        if mode != 'tip':
            low.append(0. if mode=='stick' else .03);high.append(terminal if mode=='stick' else 1.)
        low.append(.03);high.append(.97)
        def residual(values):
            if time.monotonic() >= deadline:
                raise SearchDeadline()
            return fixed_residual(ref,values[:-1],pair.first,pair.second,values[-1],mode,
                terminal,settings,winding,orientation)
        solves += 1;counts[mode+'_attempts'] += 1
        try:
            fit = least_squares(residual,initial,bounds=(low,high),max_nfev=settings.max_evaluations,
                ftol=1e-9,xtol=1e-9,gtol=1e-9)
        except SearchDeadline:
            budget_exhausted = True;break
        if float(np.max(np.abs(fit.fun))) > 1e-6:
            counts['contact_equations_not_solved'] += 1;continue
        counts[mode+'_roots'] += 1
        pose = make_pose(ref,fit.x[:-1],pair.first,pair.second,fit.x[-1],mode,
            terminal,settings,orientation)
        metrics = assess_geometry(ref,target,pose,settings)
        if not metrics['geometric_proxy_pass']:
            rejections.update(metrics['reasons']);continue
        counts[mode+'_geometric_roots'] += 1
        signature = tuple(np.round(np.r_[np.degrees(pose['turns']),np.degrees(pose['rotation']),
            np.asarray(pose['translation'])*1000],1))
        if signature in pool:
            pool[signature]['search_modes'] = sorted(set(pool[signature]['search_modes']+[mode]))
            continue
        original = ref.evaluate_exact(target,reference_candidate(ref,pose),config)
        pose.update(metrics=metrics,legacy_gate=dict(pass_=bool(original.feasible),
            violations=list(original.violations),score=float(original.score)))
        pool[signature] = pose
    candidates = sorted(pool.values(),key=lambda p:(
        p['metrics']['section_angle_margins']['minimum_deg'],p['metrics']['capture_score']),reverse=True)
    return dict(status=outcome(bool(candidates),budget_exhausted),elapsed_s=time.monotonic()-began,
        budget_exhausted=budget_exhausted,search_exhaustive=False,solves=solves,
        pairs_before_centroid_filter=len(raw_pairs),pairs_after_centroid_filter=len(admissible),
        pair_count=len(pairs),counts=dict(counts),rejections=dict(rejections),
        pair_contacts=[dict(u1=p.first.u,u2=p.second.u,priority=p.priority) for p in pairs],
        candidate_count=len(candidates),legacy_gate_pass_count=sum(p['legacy_gate']['pass_'] for p in candidates),
        forward_rejected_count=sum(not p['metrics']['legacy_forward_pass'] for p in candidates),
        endpoint_non_tangent_count=sum('tip' in p['search_modes'] and p['endpoint_tangency_residual']>1e-3 for p in candidates),
        candidates=candidates)


def local_recovery(ref,target,pose,settings):
    from shapely.affinity import rotate,translate
    from shapely.geometry import Point,Polygon
    polygon = Polygon(ref.rotate(target.vertices,pose['rotation'])+pose['translation'])
    q0 = np.array(pose['turns']);window = math.radians(settings.correction_deg)
    low = np.maximum(-settings.turn_limit_rad,q0-window)
    high = np.minimum(settings.turn_limit_rad,q0+window)
    cases = [(name,dx,dy,angle) for name,dx,dy,angle in
        (('+x',.001,0,0),('-x',-.001,0,0),('+y',0,.001,0),('-y',0,-.001,0),
         ('+rotation',0,0,1),('-rotation',0,0,-1))]
    rows = []
    for name,dx,dy,angle in cases:
        changed = translate(rotate(polygon,angle,origin='centroid'),xoff=dx,yoff=dy)
        def residual(q):
            p1,p2,*_ = contact_geometry(ref,q,pose['a'],pose['second'],pose['mode'],
                pose['terminal_ratio'],settings.length_m)
            def signed(p):
                point = Point(*p);distance = changed.boundary.distance(point)
                return (-distance if changed.contains(point) else distance)/settings.length_m
            return [signed(p1),signed(p2)]
        fit = least_squares(residual,q0,bounds=(low,high),max_nfev=60,ftol=1e-10,xtol=1e-10,gtol=1e-10)
        corrected = dict(pose,turns=fit.x.tolist())
        metrics = assess_geometry(ref,target,corrected,settings,polygon=changed)
        rows.append(dict(perturbation=name,recovered_geometrically=metrics['geometric_proxy_pass'],
            corrections_deg=np.degrees(fit.x-q0).tolist(),reasons=metrics['reasons']))
    return dict(scope='Six sampled pose errors; fixed base, same material contact locations; two bend axes '
        f'limited to +/-{settings.correction_deg:g} deg corrections. A failed local solve is not proof of impossibility.',
        successes=sum(r['recovered_geometrically'] for r in rows),trials=len(rows),rows=rows,
        compression_margin=None,motor_load_margin=None,hold_verified=None)


def write_json(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def run_study(ref,reference_path,output,case_names,terminal,settings):
    settings.checked()
    if not math.isfinite(terminal) or not 0 < terminal <= 1:
        raise ValueError('the attached gripper ratio must lie in (0,1]')
    output.mkdir(parents=True,exist_ok=False)
    report = dict(schema_version=1,complete=False,scope='Offline fixed-length planar geometry/contact diagnostic; '
        'no ROS, Genesis, hardware, training or motion.',settings=asdict(settings),
        gripper=dict(present=True,ratio=terminal,fixed=True,
            model='Existing hug.py straight terminal approximation; not a CAD/contact model of the jaws.',
            scope='Gripper geometry is fixed input, never a design variable; no ID5 jaw-closing action.'),
        reference=dict(path=str(reference_path),sha256=hashlib.sha256(reference_path.read_bytes()).hexdigest()),
        limitations=['Zero-thickness constant-curvature sections and simple 2-D polygon targets.',
            'Sparse/compressible objects are not represented by this rigid geometry model.',
            'Search and local recovery are bounded, not exhaustive; no-solution is not a mechanical impossibility proof.',
            'Force proxies assume two independent point contacts; no actuator, ID3/ID4 current, friction or material calibration.',
            'Original centroid/forward gates are recorded unchanged; no runtime criterion is relaxed.'],rows=[])
    write_json(output/'report.json',report)
    for name in case_names:
        target = ref.make_target(case_vertices(name),sdf_resolution=128)
        row = dict(case=name,terminal_ratio=terminal,section_length_m=settings.length_m,
            target_vertices=target.vertices.tolist(),target_major_axis_m=float(target.major_axis),
            target_sha256=hashlib.sha256(np.asarray(target.vertices,dtype='<f8').tobytes()).hexdigest(),
            legacy=legacy_run(ref,target,terminal,settings))
        fixed = row['fixed'] = fixed_search(ref,target,terminal,settings)
        if fixed['candidates']:
            best = fixed['candidates'][0]
            best['contact_proxies'] = contact_proxies(best)
            best['local_recovery'] = local_recovery(ref,target,best,settings)
        report['rows'].append(row)
        write_json(output/'report.json',report)
        print(json.dumps({k:v for k,v in dict(case=name,n=terminal,
            candidates=fixed['candidate_count'],legacy_pass=fixed['legacy_gate_pass_count'],
            forward_rejected=fixed['forward_rejected_count'],endpoint_non_tangent=fixed['endpoint_non_tangent_count'],
            search_s=fixed['elapsed_s']).items()}),flush=True)
    report['complete'] = True
    write_json(output/'report.json',report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference',type=Path,default=DEFAULT_REFERENCE)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--cases',nargs='+',choices=CASES,default=list(CASES))
    parser.add_argument('--gripper-ratio',type=float,default=.155,
        help='Fixed existing reference-model gripper geometry; not a design sweep.')
    parser.add_argument('--budget',type=float,default=8.)
    parser.add_argument('--legacy-budget',type=float,default=2.)
    parser.add_argument('--pair-limit',type=int,default=32)
    parser.add_argument('--max-solves',type=int,default=512)
    args = parser.parse_args()
    settings = Settings(budget_s=args.budget,legacy_budget_s=args.legacy_budget,
        pair_limit=args.pair_limit,max_solves=args.max_solves).checked()
    reference_path = args.reference.resolve(strict=True)
    run_study(load_reference(reference_path),reference_path,args.output,args.cases,args.gripper_ratio,settings)


if __name__ == '__main__':
    main()
