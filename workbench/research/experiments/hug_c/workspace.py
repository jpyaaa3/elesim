"""Soft horizontal workspace preference from the actual arm geometry.

The fixed envelope covers Home, the geometric goal and the initial object.
Distance outside it is a compact-motion preference, never a collision test or
proof that an approach is possible inside the envelope. Height stays free.
"""
from itertools import product
import xml.etree.ElementTree as ET
import numpy as np


LINK_NAMES = tuple(f'node{i}' for i in range(10)) + (
    'gripper_base', 'gripper_claw_left', 'gripper_claw_right')


def hull_2d(points):
    points = sorted(set(map(tuple, np.round(np.asarray(points, float),12))))
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    def half(values):
        result = []
        for point in values:
            while len(result) >= 2 and cross(result[-2], result[-1], point) <= 1e-12:
                result.pop()
            result.append(point)
        return result
    if len(points) < 3:
        raise ValueError('Workspace envelope requires a nondegenerate polygon')
    result = np.asarray(half(points)[:-1]+half(points[::-1])[:-1])
    if len(result) < 3:
        raise ValueError('Workspace envelope requires a nondegenerate polygon')
    return result


def marker_geometry():
    """Bounding-box corners of each link's collision mesh, in its local frame."""
    from .configure import BUNDLE
    from workbench.research.experiments.hug_interpolation.configure import rotation
    root = ET.parse(BUNDLE/'robot.urdf').getroot()
    points, owners = [], []
    cache = {}
    for name in LINK_NAMES:
        link = root.find(f"link[@name='{name}']")
        collisions = link.findall('collision')
        if not collisions:
            raise ValueError(f'Missing workspace collision geometry for {name}')
        for collision in collisions:
            mesh = collision.find('geometry/mesh')
            if mesh is None:
                raise ValueError('Workspace requires the model collision meshes')
            path = BUNDLE/mesh.get('filename')
            if path not in cache:
                cache[path] = np.asarray([list(map(float, line.split()[1:4]))
                    for line in path.read_text().splitlines() if line.startswith('v ')])
            vertices = cache[path]*np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            lower, upper = vertices.min(0), vertices.max(0)
            corners = np.array(list(product(*zip(lower, upper))))
            origin = collision.find('origin')
            xyz = np.zeros(3) if origin is None else np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            rpy = np.zeros(3) if origin is None else np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
            rotate = rotation([0,0,1], rpy[2])@rotation([0,1,0], rpy[1])@rotation([1,0,0], rpy[0])
            points.extend(corners@rotate.T+xyz)
            owners.extend([name]*len(corners))
    return tuple(owners), np.asarray(points)


def command_points(command, owners, markers, spawn_height):
    from .configure import BUNDLE, fk
    root = ET.parse(BUNDLE/'robot.urdf').getroot()
    # Resolve actual joint names from the model; no alternative angle convention.
    linear = next(j.get('name') for j in root.findall('joint') if j.find('child').get('link') == 'housing')
    roll = next(j.get('name') for j in root.findall('joint') if j.find('child').get('link') == 'wedge')
    positions = {linear:float(command[0]), roll:float(command[1])}
    for i in range(10):
        joint = next(j for j in root.findall('joint') if j.find('child').get('link') == f'node{i}')
        positions[joint.get('name')] = float(command[2 if i < 5 else 3])
    transforms = {name:fk(root, name, positions) for name in set(owners)}
    result = np.array([transforms[name][:3,:3]@point+transforms[name][:3,3]
                       for name, point in zip(owners, markers)])
    result[:,2] += spawn_height
    return result


def envelope(case, settings, owners, markers):
    start = command_points(case['start_command'], owners, markers, settings['spawn_height_m'])
    goal = command_points(case['goal_command'], owners, markers, settings['spawn_height_m'])
    angles = np.linspace(0, 2*np.pi, 32, endpoint=False)
    circle = np.asarray(case['object_xyz_m'])[:2] + case['diameter_mm']/2000*np.column_stack((np.cos(angles),np.sin(angles)))
    return hull_2d(np.vstack((start[:,:2], goal[:,:2], circle)))


def transform_markers(positions, quaternions, markers, xp=np):
    """Selected link poses and corners: (..., markers, 3), quaternion wxyz."""
    point = markers + xp.zeros_like(positions)
    axis = quaternions[...,1:]
    if xp is np:
        twice = 2*np.cross(axis, point)
        return positions+point+quaternions[...,:1]*twice+np.cross(axis, twice)
    twice = 2*xp.cross(axis, point, dim=-1)
    return positions+point+quaternions[...,:1]*twice+xp.cross(axis, twice, dim=-1)


def outside_distance(points, polygon, xp=np):
    """Exact point-to-convex-polygon XY distance, including vertex regions."""
    if xp is np:
        following = np.roll(polygon, -1, axis=0)
    else:
        following = xp.roll(polygon, -1, dims=0)
    edge = following-polygon
    relative = points[...,:2][...,None,:]-polygon
    parameter = xp.clip((relative*edge).sum(-1)/(edge*edge).sum(-1), 0, 1)
    offset = relative-parameter[...,None]*edge
    squared = (offset*offset).sum(-1)
    cross = edge[:,0]*relative[...,1]-edge[:,1]*relative[...,0]
    if xp is np:
        distance = np.sqrt(np.min(squared, axis=-1))
        inside = np.all(cross >= -1e-9, axis=-1)
    else:
        distance = xp.sqrt(squared.min(dim=-1).values)
        inside = (cross >= -1e-9).all(dim=-1)
    return xp.where(inside, xp.zeros_like(distance), distance)


def workspace_excess(points, polygon, settings, xp=np):
    distance = outside_distance(points, polygon, xp)
    maximum = np.max(distance, axis=-1) if xp is np else distance.max(dim=-1).values
    return xp.clip(maximum-settings['workspace_margin_m'], 0, float('inf'))


def workspace_fraction(excess, settings, xp=np):
    return xp.clip(excess/settings['workspace_scale_m'], 0, 1)**2
