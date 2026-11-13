"""CPU geometry and command boundaries shared by the isolated V2 workers."""
from __future__ import annotations

import copy
import hashlib
import itertools
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def save(path, data):
    path = Path(path)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def restore_mesh_urdf(source_model, assembly, output):
    """Retain the calibrated four-axis tree and restore original collision meshes."""
    root = ET.parse(source_model / 'arm.urdf').getroot()
    original = ET.parse(assembly / 'arm.urdf').getroot()
    hashes = {}
    for link in root.findall('link'):
        source = original.find(f"link[@name='{link.get('name')}']")
        if source is None:
            continue
        for tag in ('visual', 'collision'):
            if link.get('name') == 'plate' and tag == 'collision':
                continue
            geometries = source.findall(tag)
            if link.get('name') == 'camera' and tag == 'collision' and not geometries:
                # The Hug task explicitly enables camera visual geometry as
                # collision geometry in materialize_robot(). Match that rule.
                geometries = []
                for visual in source.findall('visual'):
                    geometry = ET.Element('collision')
                    for child in ('origin', 'geometry'):
                        item = visual.find(child)
                        if item is not None:
                            geometry.append(copy.deepcopy(item))
                    geometries.append(geometry)
            for geometry in geometries:
                geometry = copy.deepcopy(geometry)
                for mesh in geometry.findall('geometry/mesh'):
                    path = (assembly / mesh.get('filename')).resolve()
                    hashes[str(path)] = sha(path)
                    mesh.set('filename', str(path))
                link.append(geometry)
    ET.indent(root)
    ET.ElementTree(root).write(output, encoding='unicode', xml_declaration=True)
    return hashes


def urdf_poses(path, joint_names, q):
    """Independent FK for every fixed, prismatic, revolute and direct mimic link."""
    root = ET.parse(path).getroot()
    joints = list(root.findall('joint'))
    children = {j.find('child').get('link') for j in joints}
    roots = {link.get('name') for link in root.findall('link')} - children
    if len(roots) != 1:
        raise ValueError('Expected a single rooted URDF')
    poses = {roots.pop(): np.eye(4)}
    values = dict(zip(joint_names, q))
    while joints:
        progressed = False
        for joint in joints[:]:
            parent = joint.find('parent').get('link')
            if parent not in poses:
                continue
            origin = joint.find('origin')
            t = np.eye(4)
            if origin is not None:
                t[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                t[:3, :3] = Rotation.from_euler(
                    'xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
            value = values.get(joint.get('name'), 0.)
            mimic = joint.find('mimic')
            if mimic is not None:
                value = (values[mimic.get('joint')] * float(mimic.get('multiplier', '1'))
                         + float(mimic.get('offset', '0')))
            motion = np.eye(4)
            kind = joint.get('type')
            if kind != 'fixed':
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                if kind == 'prismatic':
                    motion[:3, 3] = axis * value
                elif kind in ('revolute', 'continuous'):
                    motion[:3, :3] = Rotation.from_rotvec(axis * value).as_matrix()
                else:
                    raise ValueError(f'Unsupported joint: {kind}')
            poses[joint.find('child').get('link')] = poses[parent] @ t @ motion
            joints.remove(joint)
            progressed = True
        if not progressed:
            raise ValueError('Unresolved URDF tree')
    return poses


def mesh_cylinder_scene(context, output, contact_band=0.):
    """Cylinder mesh and floor in the measured plate frame; no stacked boxes."""
    import trimesh
    radius = context['diameter_mm'] / 2000.
    # Circumscribe, rather than inscribe, the true circle with a 128-side mesh.
    n = 128
    mesh = trimesh.creation.cylinder(radius=radius / np.cos(np.pi / n),
                                    height=context['height_m'], sections=n)
    mesh.export(output)
    base = Rotation.from_quat(np.roll(context['plate_quaternion_wxyz'], -1))
    obj = Rotation.from_quat(np.roll(context['object_quaternion_wxyz'], -1))
    center = base.inv().apply(np.asarray(context['object_position']) - context['plate_position'])
    quat = np.roll((base.inv() * obj).as_quat(), 1)
    floor_center = base.inv().apply(np.array([0., 0., -.025]) - context['plate_position'])
    floor_quat = np.roll(base.inv().as_quat(), 1)
    scene = dict(
        mesh={'cylinder': dict(file_path=str(Path(output).resolve()),
                               pose=[*center.tolist(), *quat.tolist()])},
        cuboid={'floor': dict(dims=[4., 4., .05],
                              pose=[*floor_center.tolist(), *floor_quat.tolist()])})
    if contact_band:
        if not 0 < contact_band < min(radius, context['height_m']/2):
            raise ValueError('Contact band must fit inside the cylinder')
        core_path = Path(output).with_name(Path(output).stem+'-contact-core.obj')
        core = trimesh.creation.cylinder(radius=(radius-contact_band)/np.cos(np.pi/n),
                                        height=context['height_m']-2*contact_band, sections=n)
        core.export(core_path)
        scene['mesh']['cylinder_contact_core'] = dict(file_path=str(core_path.resolve()),
                                                     pose=scene['mesh']['cylinder']['pose'].copy())
    return scene


def normalize_commands(positions, source_dt, start, home, lower, upper, rate, dt=.01,
                       retiming='segment'):
    """Preserve the polyline; stretch offending segments to enforce command slew."""
    p = np.asarray(positions, dtype=float).copy()
    start, home, rate = map(np.asarray, (start, home, rate))
    if p.ndim != 2 or p.shape[1] != 4 or len(p) < 2 or not np.isfinite(p).all():
        raise ValueError('Expected finite Nx4 commands')
    if not np.isfinite(source_dt) or source_dt <= 0 or np.any(rate <= 0):
        raise ValueError('Invalid trajectory timing/rate')
    if not np.allclose(p[0], start, atol=1e-5, rtol=0) or not np.allclose(p[-1], home, atol=1e-5, rtol=0):
        raise ValueError('Planner endpoints differ from the exact task endpoints')
    original = p.copy()
    p[0], p[-1] = start, home
    # Preserve the actual lock: monotone initial 90 physical degrees, then
    # reversals are allowed. Only float32-scale jitter is normalized.
    roll_travel = 0.
    for i in range(1, len(p)):
        delta = p[i, 1] - p[i-1, 1]
        if roll_travel < np.pi / 2 - 1e-8 and delta < 0:
            p[i, 1] = p[i-1, 1]
        roll_travel += abs(p[i, 1] - p[i-1, 1])
    if np.max(np.abs(p[:, 1] - original[:, 1])) > 5e-7:
        raise ValueError('Substantive initial roll reversal; no automatic repair')
    if np.any(p < np.asarray(lower) - 1e-7) or np.any(p > np.asarray(upper) + 1e-7):
        raise ValueError('Command exceeds task bounds')
    required_dt = np.max(np.abs(np.diff(p, axis=0)) / (rate / dt), axis=1)
    if retiming == 'uniform':
        # Preserve the native trajectory's time profile. No phases, added
        # waypoints or segment-specific slowdown in the full-planner mode.
        segment_dt = np.full(len(p)-1, max(source_dt, float(required_dt.max())))
    elif retiming == 'segment':
        segment_dt = np.maximum(source_dt, required_dt)
    else:
        raise ValueError(f'Unknown retiming policy: {retiming}')
    duration = float(segment_dt.sum())
    if duration > 19.:
        raise ValueError('No time for the one-second Home verification')
    times = np.arange(round(20 / dt) + 1) * dt
    path_times = np.r_[0., np.cumsum(segment_dt)]
    commands = np.stack([np.interp(times, path_times, p[:, j]) for j in range(4)], axis=-1)
    if np.any(np.abs(np.diff(commands, axis=0)) > rate + 1e-7):
        raise ValueError('Resampling exceeded command slew')
    return commands, dict(raw_dt_s=source_dt, retiming=retiming,
                          retimed_segment_dt_min_s=float(segment_dt.min()),
                          retimed_segment_dt_max_s=float(segment_dt.max()),
                          movement_duration_s=duration,
                          max_normalization=np.max(np.abs(p - original), axis=0).tolist())


def small_release_offsets(start, lower, upper, limit=256):
    """Nearest bounded pose offsets; object pose and original admission stay fixed."""
    start = np.asarray(start)
    bend = np.deg2rad([0., -.1, .1, -.5, .5, -1., 1., -2., 2.])
    grid = itertools.product([0., -.001, -.003, -.005, -.01, -.02],
                             np.deg2rad([0., .5, 1., 3., 5., 10.]), bend, bend)
    poses = np.clip(start + np.asarray(list(grid)), lower, upper)
    poses = np.unique(poses, axis=0)
    span = np.array([.02, np.deg2rad(10), np.deg2rad(2), np.deg2rad(2)])
    order = np.argsort(np.linalg.norm((poses-start)/span, axis=1), kind='stable')
    return poses[order[:limit]]


def contact_release_offsets(start, home, lower, upper):
    """Candidate ends of an executed release prefix, without moving the object.

    Positive bends open the admitted C-shape. The full original motor budget,
    direction lock and physical contact rules still apply to each whole path.
    """
    start, home = np.asarray(start), np.asarray(home)
    target = np.r_[home[:2], np.maximum(start[2:], 0.)]
    fractions = np.asarray(list(itertools.product(np.linspace(0., 1., 5), repeat=4)))
    return np.unique(np.clip(start+fractions*(target-start), lower, upper), axis=0)


def diverse_release_starts(poses, start, limit=8):
    """Retain nearby and well-separated feasible release alternatives."""
    poses, start = np.asarray(poses), np.asarray(start)
    if not len(poses):
        return poses
    scale = np.maximum(np.ptp(poses, axis=0), 1e-9)
    normalized = (poses-start)/scale
    selected = [int(np.argmin(np.linalg.norm(normalized, axis=1)))]
    distance = np.full(len(poses), np.inf)
    for _ in range(min(limit, len(poses))-1):
        distance = np.minimum(distance, np.linalg.norm(normalized-normalized[selected[-1]], axis=1))
        distance[selected] = -1.
        selected.append(int(np.argmax(distance)))
    return poses[selected]


def budget_search_bounds(start, home, lower, upper, contract):
    """Necessary bounds from triangle inequalities, not new task constraints.

    Any intermediate point q needs at least |q-start|+|home-q| command
    travel. For the shared bend budget, reserve the other bend's minimum
    endpoint travel before bounding each axis.
    """
    start, home = np.asarray(start), np.asarray(home)
    scale = np.asarray(contract['sim_to_motor_abs_scale'])
    limits = np.asarray(contract['limits_motor_deg']) + 1e-6
    minimum = np.abs(home-start)*scale
    allowances = np.array([limits[0], limits[1], limits[2]-minimum[3], limits[2]-minimum[2]])
    if np.any(allowances < minimum-1e-8):
        raise ValueError('Endpoint travel exceeds the task budget')
    reach = allowances/scale
    lo = np.maximum(lower, (start+home-reach)/2)
    hi = np.minimum(upper, (start+home+reach)/2)
    return lo, hi


def command_motor_travel(commands, contract):
    travel = np.abs(np.diff(commands, axis=0)).sum(axis=0) * np.asarray(contract['sim_to_motor_abs_scale'])
    groups = np.array([travel[0], travel[1], travel[2]+travel[3]])
    return groups


def task_velocity_limits(urdf_maximum, command_rate, dt=.01):
    """Convert the original per-tick, already-per-node task rates exactly once."""
    original, rate = np.asarray(urdf_maximum, float), np.asarray(command_rate, float)
    if (original.shape != (4,) or rate.shape != (4,) or not np.isfinite(dt) or dt <= 0
            or not np.isfinite(original).all() or not np.isfinite(rate).all()
            or np.any(original <= 0) or np.any(rate <= 0)):
        raise ValueError('Expected four positive finite joint speeds and command rates')
    return np.minimum(original, rate/dt)


def native_endpoint_bounds(start, home, lower, upper):
    """Solver-only float32 endpoint slack; original command checks stay exact.

    Cubic interpolation can move a fixed endpoint by a few float32 ULPs. The
    output adapter still checks original task bounds and normalizes at most
    5e-7 rad of initial Roll noise, without accepting a real direction change.
    """
    start, home, lower, upper = map(lambda x: np.asarray(x, float), (start, home, lower, upper))
    tolerance = np.array([5e-8, 5e-7, 5e-7, 5e-7])
    low_touch = np.isclose(start, lower, rtol=0, atol=1e-10) | np.isclose(home, lower, rtol=0, atol=1e-10)
    high_touch = np.isclose(start, upper, rtol=0, atol=1e-10) | np.isclose(home, upper, rtol=0, atol=1e-10)
    return lower-np.where(low_touch, tolerance, 0), upper+np.where(high_touch, tolerance, 0)
