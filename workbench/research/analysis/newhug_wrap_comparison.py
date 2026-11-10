"""Compare an explicitly selected standalone newhug with the fixed Wrap task.

This is an endpoint geometry experiment, not a policy or Genesis rollout.
The target is an exact circle. Two exposed robot faces are tangent to it;
the remaining DOF is sampled, with the lateral placement equation solved by
Brent's method. Base yaw/lateral motion are forbidden. No runtime is modified.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, dataclass
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation
import yaml


ROOT = Path(__file__).resolve().parents[3]
URDF = ROOT / 'payload/data/models/assemblies/zed-mini/robot.urdf'


def find_key(data, name):
    if isinstance(data, dict):
        if name in data:
            return data[name]
        for value in data.values():
            try:
                return find_key(value, name)
            except KeyError:
                pass
    raise KeyError(name)


@dataclass(frozen=True)
class Task:
    radius_m: float
    object_xyz_m: tuple
    neutral_hinge_xyz_m: tuple
    linear_limits_m: tuple
    coupled_weight: float
    coupled_cap_rad: float
    node_limit_deg: float
    camera_margin_m: float = .001
    tolerance_m: float = .00005

    @property
    def centre_x(self):
        return -(self.object_xyz_m[1] - self.neutral_hinge_xyz_m[1])

    @property
    def centre_y_at_zero(self):
        return self.object_xyz_m[0] - self.neutral_hinge_xyz_m[0]

    def centre(self, linear):
        return np.array([self.centre_x, self.centre_y_at_zero-linear])

    def linear_at(self, centre):
        return self.centre_y_at_zero-float(centre[1])


def load_task():
    wrap = yaml.safe_load((ROOT/'payload/config/pilot/wrap_grasp.yaml').read_text())
    pilot = yaml.safe_load((ROOT/'payload/config/pilot/config.yaml').read_text())
    model = json.loads((ROOT/'payload/data/models/arm/default.json').read_text())
    manifest = json.loads((ROOT/'payload/data/policies/wrap-grasp/interface.json').read_text())
    tree = ET.parse(URDF).getroot()
    origins = {j.find('child').get('link'): np.fromstring(j.find('origin').get('xyz', '0 0 0'), sep=' ')
               for j in tree.findall('joint') if j.find('origin') is not None}
    hinge = origins['plate']+origins['housing']+origins['wedge']+origins['node0']
    hinge[2] += float(find_key(pilot['go2'] if 'go2' in pilot else pilot, 'spawn_height'))
    cap = manifest['waypoint']['coupled_curl_cap']
    return Task(float(wrap['object']['radius_m']), tuple(wrap['object']['xyz_m']), tuple(hinge),
                (find_key(model, 'linear_min_m'), find_key(model, 'linear_max_m')),
                cap['theta1_weight'], cap['cap_rad'],
                math.degrees(manifest['waypoint']['limits']['theta_rad'][1]))


def load_solver(path, roll_sign):
    """Load trusted source without its UI; reflect only rolled rigid profiles."""
    name = f'_newhug_wrap_{"positive" if roll_sign > 0 else "negative"}'
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    if roll_sign < 0:
        for key in (*module.GRIPPER_PART_NAMES, 'camera', 'wedge'):
            points = np.asarray(module.REFERENCE['profiles'][key]) * [-1, 1]
            module.REFERENCE['profiles'][key] = points[::-1].tolist()
        tool = module.REFERENCE['gripper_model']
        tool['outline_m'] = (np.asarray(tool['outline_m'])*[-1, 1])[::-1].tolist()
        tool['forward_tip_m'][0] *= -1
        module.REFERENCE['fixed_arm_roll_deg'] = -90.
    return module


def circle_gap(module, centre, radius, polygon):
    """Exact signed circle/polygon clearance, including a contained centre."""
    return float(module.signed_distances(np.asarray([centre]), polygon)[0])-radius


def tangent_centres(starts, ends, normals, pair_a, pair_b, radius):
    """Intersect two face support lines and return tangent-foot fractions."""
    na, nb = normals[pair_a], normals[pair_b]
    rhs = np.einsum('ij,ij->i', normals, starts)+radius
    det = na[:, 0]*nb[:, 1]-na[:, 1]*nb[:, 0]
    with np.errstate(divide='ignore', invalid='ignore'):
        centres = np.column_stack(((rhs[pair_a]*nb[:, 1]-na[:, 1]*rhs[pair_b])/det,
                                   (na[:, 0]*rhs[pair_b]-rhs[pair_a]*nb[:, 0])/det))
        da, db = ends[pair_a]-starts[pair_a], ends[pair_b]-starts[pair_b]
        fa = np.einsum('ij,ij->i', centres-radius*na-starts[pair_a], da)/np.sum(da*da, axis=1)
        fb = np.einsum('ij,ij->i', centres-radius*nb-starts[pair_b], db)/np.sum(db*db, axis=1)
    return centres, fa, fb, det


class FixedTaskSearch:
    def __init__(self, module, task, roll_sign):
        self.module, self.task, self.roll_sign = module, task, roll_sign
        self.geometry = module.Geometry()
        self.faces = module.robot_faces(self.geometry)
        self.parts = np.array([min(f.part, 10) for f in self.faces])
        self.starts = np.array([f.first for f in self.faces])
        self.ends = np.array([f.second for f in self.faces])
        self.normals = np.array([f.normal for f in self.faces])
        pairs = [(i, j) for i, a in enumerate(self.faces) for j, b in enumerate(self.faces)
                 if a.part < b.part and a.part < 9]
        self.ia, self.ib = np.array(pairs).T
        self.counts = Counter()
        self.candidates = []
        self.seen = set()

    def frames(self, q1, q2):
        hinges, angles = self.module.fk_frames(np.radians([q1, q2])*5, self.geometry)
        angle = angles[self.parts]
        c, s = np.cos(angle), np.sin(angle)

        def turn(points):
            return np.column_stack((c*points[:, 0]-s*points[:, 1], s*points[:, 0]+c*points[:, 1]))

        return turn(self.starts)+hinges[self.parts], turn(self.ends)+hinges[self.parts], turn(self.normals)

    def pose(self, q1, q2, centre, ia=None, ib=None):
        task, m = self.task, self.module
        reasons = []
        linear = task.linear_at(centre)
        command = self.roll_sign*np.radians([q1, q2])
        curl = abs(task.coupled_weight*command[0]+command[1])
        if max(abs(q1), abs(q2)) > task.node_limit_deg+1e-8:
            reasons.append('node_limit')
        if curl > task.coupled_cap_rad+1e-8:
            reasons.append('coupled_cap')
        if not task.linear_limits_m[0]-1e-9 <= linear <= task.linear_limits_m[1]+1e-9:
            reasons.append('linear_limit')
        if abs(float(centre[0])-task.centre_x) > 1e-7:
            reasons.append('lateral_base_motion')
        surfaces, bodies, hinges, angles = m.robot_polygons(np.radians([q1, q2])*5, self.geometry)
        gaps = [circle_gap(m, centre, task.radius_m, poly) for poly in surfaces+bodies]
        if min(gaps) < -task.tolerance_m:
            reasons.append('object_penetration')
        camera = gaps[m.CAMERA_INDEX]
        if camera <= task.camera_margin_m+1e-10:
            reasons.append('camera_clearance')
        self_gap = None
        # Expensive self checks are only needed after the exact object check.
        if not reasons:
            self_gap = float(min(m.self_gaps(surfaces, bodies)))
            if self_gap < -task.tolerance_m:
                reasons.append('self_collision')
        contacts = []
        score = None
        if ia is not None:
            starts, ends, normals = self.frames(q1, q2)
            points = centre-task.radius_m*normals[[ia, ib]]
            score = m.finite_score(m.friction_score(points, normals[[ia, ib]]))
            for k, point in zip((ia, ib), points):
                f = self.faces[k]
                d = ends[k]-starts[k]
                fraction = float(np.dot(point-starts[k], d)/np.dot(d, d))
                error = float(np.linalg.norm(starts[k]+fraction*d-point))
                if not -1e-8 <= fraction <= 1+1e-8 or error > 1e-7:
                    reasons.append('contact_foot_outside_face')
                contacts.append(dict(part=m.PART_NAMES[f.part], edge=f.edge, fraction=fraction,
                                     point_local_m=point.tolist(), inward_normal=normals[k].tolist()))
        return dict(valid=not reasons, reasons=reasons, roll_deg=90*self.roll_sign,
                    node_bends_deg=np.degrees(command).tolist(), effective_node_bends_deg=[q1, q2],
                    linear_m=linear, centre_local_m=np.asarray(centre).tolist(),
                    coupled_curl_deg=math.degrees(curl), friction=score,
                    min_object_gap_m=float(min(gaps)), camera_clearance_m=camera,
                    min_self_gap_m=self_gap, contacts=contacts,
                    hinges_local_m=hinges.tolist(),
                    polygons=[dict(part=name, points_m=p.tolist()) for name, p in
                              zip((*m.PART_NAMES, *m.BODY_NAMES), surfaces+bodies)])

    def scan(self, q2_values, q1_step=2.):
        task = self.task
        q1_values = np.linspace(-task.node_limit_deg, task.node_limit_deg,
                                round(2*task.node_limit_deg/q1_step)+1)
        lower_y = task.centre_y_at_zero-task.linear_limits_m[1]
        upper_y = task.centre_y_at_zero-task.linear_limits_m[0]
        for q2 in q2_values:
            rows = []
            for q1 in q1_values:
                starts, ends, normals = self.frames(q1, q2)
                rows.append(tangent_centres(starts, ends, normals, self.ia, self.ib, task.radius_m))
            for k, (left, right) in enumerate(zip(rows, rows[1:])):
                ca, fa, ga, da = left
                cb, fb, gb, db = right
                ra, rb = ca[:, 0]-task.centre_x, cb[:, 0]-task.centre_x
                with np.errstate(invalid='ignore', divide='ignore'):
                    alpha = -ra/(rb-ra)
                    cy = ca[:, 1]+alpha*(cb[:, 1]-ca[:, 1])
                    f = fa+alpha*(fb-fa)
                    g = ga+alpha*(gb-ga)
                    q1_est = q1_values[k]+alpha*(q1_values[k+1]-q1_values[k])
                with np.errstate(invalid='ignore', over='ignore'):
                    crosses = (ra*rb <= 0)
                mask = (np.isfinite(ra) & np.isfinite(rb) & crosses & (da*db > 0) &
                        (abs(da) > 1e-6) & (abs(db) > 1e-6) &
                        (cy > lower_y-.005) & (cy < upper_y+.005) &
                        (f > -.15) & (f < 1.15) & (g > -.15) & (g < 1.15) &
                        (abs(task.coupled_weight*q1_est+q2) < math.degrees(task.coupled_cap_rad)+3))
                for pair in np.flatnonzero(mask):
                    ia, ib = int(self.ia[pair]), int(self.ib[pair])

                    def values(q1):
                        starts, ends, normals = self.frames(q1, q2)
                        return tangent_centres(starts, ends, normals, np.array([ia]),
                                               np.array([ib]), task.radius_m)[0][0]

                    self.counts['root_attempts'] += 1
                    try:
                        q1 = brentq(lambda v: values(v)[0]-task.centre_x,
                                    q1_values[k], q1_values[k+1], xtol=1e-10)
                    except ValueError:
                        self.counts['root_failure'] += 1
                        continue
                    key = (round(q1, 6), round(float(q2), 6), ia, ib)
                    if key in self.seen:
                        continue
                    self.seen.add(key)
                    result = self.pose(q1, float(q2), values(q1), ia, ib)
                    if result['valid'] and result['friction']['mu_critical'] is not None:
                        self.candidates.append(result)
                        self.counts['valid'] += 1
                    else:
                        self.counts.update(result['reasons'])
            self.counts['q2_slices'] += 1

    def run(self):
        started = time.monotonic()
        self.scan(np.arange(-36., 36.01, 1.))
        if self.candidates:
            best = min(self.candidates, key=lambda c: c['friction']['mu_critical'])
            q2 = best['effective_node_bends_deg'][1]
            self.scan(np.linspace(max(-36, q2-1), min(36, q2+1), 41), q1_step=1.)
        self.candidates.sort(key=lambda c: c['friction']['mu_critical'])
        return dict(roll_deg=self.roll_sign*90, elapsed_s=time.monotonic()-started,
                    counts=dict(self.counts), best=self.candidates[0] if self.candidates else None,
                    candidates=self.candidates[:10])


def urdf_check(module, candidate, task):
    """Independent 3-D joint transforms, including actual rolled camera mesh."""
    tree = ET.parse(URDF).getroot()
    transforms = {'base': np.eye(4)}
    transforms['base'][2, 3] = task.neutral_hinge_xyz_m[2]-.087-.155
    commands = {'j_plate_housing': candidate['linear_m'],
                'j_housing_wedge': math.radians(candidate['roll_deg'])}
    for i in range(10):
        name = 'j_wedge_node0' if i == 0 else f'j_node{i-1}_node{i}'
        commands[name] = math.radians(candidate['node_bends_deg'][i//5])
    pending = list(tree.findall('joint'))
    while pending:
        advance = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in transforms:
                continue
            origin = joint.find('origin')
            transform = np.eye(4)
            if origin is not None:
                transform[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
                transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            motion = np.eye(4)
            axis = joint.find('axis')
            q = commands.get(joint.get('name'), 0.)
            if axis is not None:
                axis_v = np.fromstring(axis.get('xyz'), sep=' ')
                if joint.get('type') == 'prismatic':
                    motion[:3, 3] = q*axis_v
                elif joint.get('type') in ('continuous', 'revolute'):
                    motion[:3, :3] = Rotation.from_rotvec(q*axis_v).as_matrix()
            transforms[joint.find('child').get('link')] = transforms[parent]@transform@motion
            pending.remove(joint)
            advance = True
        if not advance:
            raise ValueError('disconnected URDF')
    hinge = transforms['node0'][:3, 3]
    expected = np.asarray(candidate['hinges_local_m'])
    actual = np.array([transforms[f'node{i}'][:3, 3] for i in range(10)])-hinge
    projected = np.column_stack((-actual[:, 1], actual[:, 0]))
    errors = np.linalg.norm(projected-expected[:10], axis=1)
    camera = tree.find("link[@name='camera']/visual")
    mesh = URDF.parent/camera.find('geometry/mesh').get('filename')
    vertices = np.array([list(map(float, line.split()[1:4])) for line in mesh.read_text().splitlines()
                         if line.startswith('v ')])
    visual = np.eye(4)
    origin = camera.find('origin')
    if origin is not None:
        visual[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        visual[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
    world = np.c_[vertices, np.ones(len(vertices))]@(transforms['camera']@visual).T
    camera2d = np.column_stack((-(world[:, 1]-hinge[1]), world[:, 0]-hinge[0]))
    profile = np.array(next(p['points_m'] for p in candidate['polygons'] if p['part'] == 'camera'))
    # Compare hull support functions, stronger than just comparing AABBs.
    angle = np.linspace(0, 2*math.pi, 360, endpoint=False)
    directions = np.column_stack((np.cos(angle), np.sin(angle)))
    error = np.max(abs(np.max(camera2d@directions.T, axis=0)-np.max(profile@directions.T, axis=0)))
    return dict(max_hinge_fk_error_m=float(max(errors)), camera_hull_support_error_m=float(error),
                arm_plane_z_m=float(hinge[2]), source='independent robot.urdf 3-D FK + camera OBJ')


def svg_report(report):
    task = Task(**report['task'])
    panels = [(f"{'Default' if row['roll_deg'] > 0 else 'Reflected'} geometry: roll {row['roll_deg']:+.0f} deg",
               row['best']) for row in report['runs']]
    width, height, panel_w = 1200, 810, 600
    text = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
            '<rect width="100%" height="100%" fill="#111827"/>',
            '<style>text{font-family:DejaVu Sans,sans-serif;fill:#e5e7eb;font-size:16px} .small{font-size:13px;fill:#b9c4d5}</style>',
            '<text x="24" y="32" font-size="23">100 mm Wrap task / fixed base, rail only / planar endpoint geometry</text>']
    colors = {'housing':'#526077', 'wedge':'#a78bfa', 'gripper_base':'#a27044',
              'gripper_claw_left':'#fb7185', 'gripper_claw_right':'#fb7185', 'camera':'#2dd4bf'}
    object_plot = np.array([-task.object_xyz_m[1], task.object_xyz_m[0]])
    allpts = [object_plot+[-task.radius_m, -task.radius_m],
              object_plot+[task.radius_m, task.radius_m]]
    for _, c in panels:
        if c:
            base = np.array([-task.neutral_hinge_xyz_m[1], task.neutral_hinge_xyz_m[0]+c['linear_m']])
            allpts.extend(np.array(p['points_m'])+base for p in c['polygons'])
    allpts = np.vstack(allpts)
    lo, hi = allpts.min(0)-.02, allpts.max(0)+.02
    scale = min(550/(hi[0]-lo[0]), 520/(hi[1]-lo[1]))
    for col, (title, c) in enumerate(panels):
        x0 = col*panel_w+25
        text.append(f'<text x="{x0}" y="65">{title}</text>')
        if c is None:
            text.append(f'<text x="{x0}" y="110">No valid candidate found.</text>')
            continue

        def xy(p):
            return np.array([x0+(p[0]-lo[0])*scale, 90+(hi[1]-p[1])*scale])

        def points(poly):
            return ' '.join(f'{x:.2f},{y:.2f}' for x, y in map(xy, poly))

        base = np.array([-task.neutral_hinge_xyz_m[1], task.neutral_hinge_xyz_m[0]+c['linear_m']])
        obj = np.array([-task.object_xyz_m[1], task.object_xyz_m[0]])
        ox, oy = xy(obj)
        text.append(f'<circle cx="{ox}" cy="{oy}" r="{task.radius_m*scale}" fill="#fbbf2444" stroke="#fbbf24" stroke-width="2"/>')
        for p in sorted(c['polygons'], key=lambda p: {'housing':0, 'wedge':0, 'gripper_base':2, 'camera':3}.get(p['part'], 1)):
            color = ('#60a5fa' if int(p['part'][4:]) < 5 else '#c084fc') if p['part'].startswith('node') else colors[p['part']]
            text.append(f'<polygon points="{points(np.array(p["points_m"])+base)}" fill="{color}" fill-opacity=".88" stroke="#111827" stroke-width="1.2"/>')
        text.append(f'<polyline points="{points(np.array(c["hinges_local_m"])+base)}" fill="none" stroke="#ffffff88" stroke-width="1.5"/>')
        for contact in c['contacts']:
            x, y = xy(np.array(contact['point_local_m'])+base)
            text.append(f'<circle cx="{x}" cy="{y}" r="4.5" fill="#fef08a" stroke="#111827"/>')
        q1, q2 = c['node_bends_deg']
        lines = [f'node theta1/theta2 = {q1:+.2f} / {q2:+.2f} deg',
                 f'linear = {c["linear_m"]*1000:.2f} mm; base yaw = 0 deg',
                 f'camera gap = {c["camera_clearance_m"]*1000:.2f} mm',
                 f'coupled curl = {c["coupled_curl_deg"]:.2f} / {math.degrees(task.coupled_cap_rad):.2f} deg']
        if c['friction']:
            lines.append(f'mu infimum = {c["friction"]["mu_critical"]:.4f}')
            lines.append(' + '.join(contact['part'] for contact in c['contacts']))
        lines.append('VALID static endpoint' if c['valid'] else 'REJECTED: '+', '.join(c['reasons']))
        for row, line in enumerate(lines):
            text.append(f'<text class="small" x="{x0}" y="{652+row*19}">{line}</text>')
    text.extend(['<text class="small" x="24" y="781">Blue: proximal / Purple: distal / Brown: base / Red: claws / Teal: protected ZED Mini. Yellow dots: contacts.</text>',
                 '<text class="small" x="24" y="800">Roll -90 reflection is in the experiment runner only. This is not a fresh RL rollout or an approach/lift test.</text>', '</svg>'])
    return '\n'.join(text)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--solver', required=True, type=Path, help='explicit trusted standalone newhug.py')
    parser.add_argument('--output', required=True, type=Path, help='fresh evidence directory')
    args = parser.parse_args()
    source = args.solver.resolve(strict=True)
    args.output.mkdir(parents=True, exist_ok=False)
    task = load_task()
    runs = []
    checks = []
    config_reference = None
    for sign in (1, -1):
        module = load_solver(source, sign)
        search = FixedTaskSearch(module, task, sign)
        result = search.run()
        runs.append(result)
        if result['best']:
            checks.append(dict(roll_deg=90*sign, **urdf_check(module, result['best'], task)))
        if sign == 1:
            # This nominal example is a source comment, not a trained rollout.
            config_reference = search.pose(18., 36., task.centre(-.077))
        print(json.dumps(dict(roll_deg=90*sign, elapsed_s=result['elapsed_s'], counts=result['counts'],
                              best=None if result['best'] is None else {k: result['best'][k] for k in
                              ('node_bends_deg','linear_m','friction','camera_clearance_m','contacts')})), flush=True)
    wrong_roll = None
    if runs[1]['best']:
        c = runs[1]['best']
        positive = FixedTaskSearch(load_solver(source, 1), task, 1)
        wrong_roll = positive.pose(*c['effective_node_bends_deg'], c['centre_local_m'])
    report = dict(schema=1, scope='static_endpoint_not_Genesis_or_policy_rollout', task=asdict(task),
                  solver_sha256=hashlib.sha256(source.read_bytes()).hexdigest(), solver_path=str(source),
                  runs=runs, config_reference=config_reference, independent_urdf_checks=checks,
                  same_c_shape_in_default_positive_roll=wrong_roll,
                  search_complete=False,
                  limitations=['finite q2 sampling; no impossibility/global optimality certificate',
                               '2-D rigid profiles; no motor torque, approach, deformation, gravity/lift test',
                               'configuration nominal command is not measured RL policy behavior'])
    sources = [URDF, ROOT/'payload/config/pilot/wrap_grasp.yaml', ROOT/'payload/config/pilot/config.yaml',
               ROOT/'payload/data/models/arm/default.json', ROOT/'payload/data/policies/wrap-grasp/interface.json']
    report['sources_sha256'] = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in sources}
    (args.output/'comparison.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    (args.output/'comparison.svg').write_text(svg_report(report))
    print(json.dumps(dict(output=str(args.output), independent_urdf_checks=checks)), flush=True)


if __name__ == '__main__':
    main()
