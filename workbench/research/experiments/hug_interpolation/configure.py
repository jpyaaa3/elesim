"""Offline, reproducible experiment configuration and task-local robot assets."""
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
BUNDLE = ROOT / 'payload/data/models/assemblies/zed-mini'
VERSIONS = {'torch': '2.12.1', 'genesis-world': '1.4.5', 'tensordict': '0.14.2', 'rsl-rl-lib': '5.4.0'}
DEFAULTS = dict(dt=.01, solver_substeps=4, max_collision_pairs=256, max_contacts=512, action_ticks=10, approach_steps=30,
                hold_steps=10, action_delta_rad=.03, friction=.5, floor_friction=.01,
                mass_kg=.30, height_m=1.1, spawn_height_m=.32,
                goal_tolerance_rad=.06, hold_displacement_m=.01,
                max_displacement_m=.08, max_tilt_deg=15., max_penetration_m=.005,
                hold_contact_fraction=.8, kick_velocity_m_s=.10,
                kp=200., kv=20., torque_limit=100., max_joint_spread_rad=.05, start_bend_rad=.30, seed=42)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def code_digest():
    digest = hashlib.sha256()
    paths = sorted(Path(__file__).parent.glob('*.py'))
    paths += sorted((ROOT/'payload/runtime/docker/sim/app/elesim_sim/rl').glob('*.py'))
    paths += [ROOT/'workbench/research/experiments/wrap_grasp/scene.py',
              ROOT/'workbench/research/experiments/wrap_grasp/envs/contacts.py',
              ROOT/'workbench/research/experiments/wrap_grasp/configs/loader.py',
              ROOT/'workbench/research/experiments/wrap_grasp/configs/default.yaml',
              ROOT/'workbench/research/experiments/wrap_grasp/app_config.py',
              ROOT/'payload/config/sim/config.yaml']
    for path in paths:
        digest.update(str(path.relative_to(ROOT)).encode()); digest.update(path.read_bytes())
    return digest.hexdigest()


def rotation(axis, angle):
    axis = np.array(axis, dtype=float); axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3)+math.sin(angle)*skew+(1-math.cos(angle))*(skew@skew)


def fk(root, link, positions=None):
    """URDF link frame with explicit movable coordinates, robot root at origin."""
    positions = positions or {}
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    chain = []
    while link in joints:
        joint = joints[link]; chain.append(joint); link = joint.find('parent').get('link')
    result = np.eye(4)
    for joint in reversed(chain):
        origin = joint.find('origin')
        xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
        rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
        step = np.eye(4)
        step[:3, :3] = rotation([0, 0, 1], rpy[2])@rotation([0, 1, 0], rpy[1])@rotation([1, 0, 0], rpy[0])
        step[:3, 3] = xyz
        result = result@step
        axis_element = joint.find('axis')
        axis = np.fromstring(axis_element.get('xyz'), sep=' ') if axis_element is not None else np.array([1., 0, 0])
        q = positions.get(joint.get('name'), 0.)
        motion = np.eye(4)
        if joint.get('type') in ('revolute', 'continuous'):
            motion[:3, :3] = rotation(axis, q)
        elif joint.get('type') == 'prismatic':
            motion[:3, 3] = axis*q
        result = result@motion
    return result


def asset_digest(bundle=BUNDLE):
    root = ET.parse(bundle/'robot.urdf').getroot()
    files = {bundle/'robot.urdf'}
    for mesh in root.iter('mesh'):
        files.add((bundle/mesh.get('filename')).resolve())
    digest = hashlib.sha256()
    for path in sorted(files):
        digest.update(str(path.relative_to(bundle)).encode()); digest.update(path.read_bytes())
    return digest.hexdigest()


def materialize_robot(output, bundle=BUNDLE):
    """Copy only the URDF; mesh references become absolute, original is untouched."""
    root = ET.parse(bundle/'robot.urdf').getroot()
    bends = [j.get('name') for j in root.findall('joint') if j.get('type') == 'revolute' and
             (j.get('name') == 'j_wedge_node0' or j.get('name', '').startswith('j_node'))]
    if len(bends) != 9+1:
        raise ValueError(f'Expected ten bending joints, found {bends}')
    for joint in root.findall('joint'):
        name = joint.get('name')
        if name in ('j_gripper_base_claw_left', 'j_gripper_base_claw_right'):
            joint.set('type', 'fixed')
            for tag in ('axis', 'limit', 'dynamics'):
                element = joint.find(tag)
                if element is not None:
                    joint.remove(element)
        bound = math.pi/5 if name in bends else math.pi/2 if name == 'j_housing_wedge' else None
        if bound is not None:
            limit = joint.find('limit')
            limit.set('lower', str(-bound)); limit.set('upper', str(bound))
    camera = root.find("link[@name='camera']")
    if camera is None:
        raise ValueError('Missing protected camera link')
    if camera.find('collision') is None:
        for visual in camera.findall('visual'):
            collision = ET.SubElement(camera, 'collision')
            for tag in ('origin', 'geometry'):
                element = visual.find(tag)
                if element is not None:
                    collision.append(copy.deepcopy(element))
    for mesh in root.iter('mesh'):
        mesh.set('filename', str((bundle/mesh.get('filename')).resolve()))
    path = Path(output); path.parent.mkdir(parents=True, exist_ok=True)
    body = ET.tostring(root, encoding='unicode')
    if path.exists() and path.read_text() != body:
        raise ValueError('Existing generated robot differs; use a new output directory')
    path.write_text(body)
    return path


def validate_settings(settings):
    if set(settings) != set(DEFAULTS):
        raise ValueError('Unknown/missing settings')
    for key, value in settings.items():
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value < 0:
            raise ValueError(f'Invalid setting {key}')
    for key in ('solver_substeps', 'max_collision_pairs', 'max_contacts', 'action_ticks', 'approach_steps', 'hold_steps', 'seed'):
        if type(settings[key]) is not int:
            raise ValueError(f'{key} must be integer')
    for key in set(DEFAULTS)-{'seed', 'floor_friction', 'kick_velocity_m_s'}:
        if settings[key] <= 0:
            raise ValueError(f'{key} must be positive')
    if not .01 <= settings['floor_friction'] <= settings['friction'] <= 5:
        raise ValueError('Require 0.01 <= floor friction <= arm-contact friction <= 5')
    if settings['start_bend_rad'] > math.pi/5:
        raise ValueError('start_bend_rad exceeds physical limit')
    if settings['hold_contact_fraction'] > 1:
        raise ValueError('hold_contact_fraction must be <=1')


def case_digest(data):
    body = {key: data[key] for key in ('source_sha256', 'cases', 'triangles')}
    return hashlib.sha256(json.dumps(body, sort_keys=True, allow_nan=False).encode()).hexdigest()


def configure(pilot, overrides=None):
    data = json.loads(Path(pilot).read_text())
    if data.get('schema') != 'elesim-hug-pilot' or data.get('schema_version') != 1:
        raise ValueError('Unsupported pilot')
    settings = dict(DEFAULTS, **(overrides or {})); validate_settings(settings)
    root = ET.parse(BUNDLE/'robot.urdf').getroot()
    frame = fk(root, 'node0')
    if not np.allclose(frame[:3, :3], np.eye(3), atol=1e-8):
        raise ValueError('Unexpected mount orientation; map frame must be recalibrated')
    offset = data['source_coordinates']['node0_from_body_center_mm']
    if not np.allclose(offset, [0, 144.5], atol=1e-6):
        raise ValueError('Unexpected map mounting offset')
    arm_z = float(frame[2, 3])+settings['spawn_height_m']
    if not .05 < arm_z < settings['height_m']-.05:
        raise ValueError('Cylinder does not span the arm plane with 50mm margin')
    seen = set()
    for case in data['cases']:
        goal = np.asarray(case['goal_command'], dtype=float)
        if (case['case_id'] in seen or case['split'] not in ('train', 'validation', 'test')
                or case['band'] not in (1, 2) or case['roll_deg'] != (-90 if case['band'] == 1 else 90)
                or goal.shape != (4,) or not np.isfinite(goal).all()
                or goal[0] != 0 or abs(goal[1]-math.radians(case['roll_deg'])) > 1e-8
                or np.any(abs(goal[2:]) > math.pi/5+1e-8) or goal[3] > 0
                or not 0 <= case['mu_min'] < settings['friction']):
            raise ValueError('Invalid case or insufficient simulation friction')
        seen.add(case['case_id'])
        case['start_command'] = [0., math.radians(case['roll_deg']),
                                 -case['roll_deg']/90*settings['start_bend_rad'], 0.]
        x, y = case['xy_mm']
        case['object_xyz_m'] = [float(frame[0, 3])+(y-offset[1])/1000,
                                float(frame[1, 3])-(x-offset[0])/1000,
                                settings['height_m']/2]
    data.update(schema='elesim-hug-experiment', settings=settings,
                simulation_context=dict(model_sha256=asset_digest(), code_sha256=code_digest(),
                    case_set_sha256=case_digest(data),
                    settings=settings, stack_versions=VERSIONS, frame='map +x=-world Y, +y=world X; origin from URDF node0 minus map offset',
                    robot_asset_policy='task-copy: +/-36deg bend, +/-90deg roll; jaws fixed at map zero; camera visual collision',
                    criterion='two-role-contact + goal + kick/hold; not a universal force-closure proof'),
                readiness='configured-not-physics-tested')
    return data


def load_experiment(path):
    data = json.loads(Path(path).read_text())
    if data.get('schema') != 'elesim-hug-experiment' or data.get('schema_version') != 1:
        raise ValueError('Unsupported experiment')
    validate_settings(data['settings'])
    ctx = data['simulation_context']
    if ctx['case_set_sha256'] != case_digest(data):
        raise ValueError('Case set changed after configuration')
    if ctx['settings'] != data['settings'] or ctx['model_sha256'] != asset_digest() or ctx['code_sha256'] != code_digest():
        raise ValueError('Model/code/settings changed: configure a new experiment and use a new run directory')
    return data


def write_json(path, data):
    body = json.dumps(data, indent=2, allow_nan=False)
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as stream:
        stream.write(body+'\n')
