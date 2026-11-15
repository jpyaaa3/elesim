"""Build physical arm variants once; runtime never patches its URDF."""
from __future__ import annotations

import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from elesim_protocol import SimMappingConfig

PHYSICS_FILES = {
    'arm': 'arm.physics.urdf', 'robot': 'robot.physics.urdf',
    'arm_fixed_open': 'arm.physics-open.urdf', 'robot_fixed_open': 'robot.physics-open.urdf',
}


def export_physics_urdf(source, output, *, fixed_open=False, mapping=None):
    mapping = mapping or SimMappingConfig()
    tree = ET.parse(source)
    root = tree.getroot()
    bends = [j for j in root.findall('joint') if j.get('type') == 'revolute'
             and (j.get('name') == 'j_wedge_node0' or j.get('name', '').startswith('j_node'))]
    if len(bends) != 10:
        raise ValueError('physical arm requires ten named bending joints')
    bounds = {'j_plate_housing': (mapping.linear_q_min_m, mapping.linear_q_max_m),
              'j_housing_wedge': (mapping.roll_q_min_rad, mapping.roll_q_max_rad)}
    for i, joint in enumerate(bends):
        bounds[joint.get('name')] = ((mapping.seg1_q_min_rad, mapping.seg1_q_max_rad)
                                    if i < 5 else (mapping.seg2_q_min_rad, mapping.seg2_q_max_rad))
    for joint in root.findall('joint'):
        name = joint.get('name')
        if name in bounds:
            lo, hi = bounds[name]
            if not math.isfinite(lo + hi) or lo >= hi:
                raise ValueError('invalid physical joint bounds')
            limit = joint.find('limit')
            if limit is None:
                raise ValueError('physical controlled joint has no limit: ' + name)
            limit.set('lower', format(lo, '.17g')); limit.set('upper', format(hi, '.17g'))
        if fixed_open and name in ('j_gripper_base_claw_left', 'j_gripper_base_claw_right'):
            joint.set('type', 'fixed')
            for tag in ('axis', 'limit', 'dynamics'):
                element = joint.find(tag)
                if element is not None: joint.remove(element)
    camera = root.find("link[@name='camera']")
    if camera is None:
        raise ValueError('physical model has no protected camera link')
    if camera.find('collision') is None:
        for visual in camera.findall('visual'):
            collision = ET.SubElement(camera, 'collision')
            for tag in ('origin', 'geometry'):
                item = visual.find(tag)
                if item is not None: collision.append(copy.deepcopy(item))
    Path(output).write_text(ET.tostring(root, encoding='unicode') + '\n')


def build_physics_models(bundle):
    bundle = Path(bundle)
    for key, filename in PHYSICS_FILES.items():
        source = bundle / ('arm.urdf' if key.startswith('arm') else 'robot.urdf')
        export_physics_urdf(source, bundle / filename, fixed_open=key.endswith('fixed_open'))
    m = SimMappingConfig()
    return dict(schema_version=1, entrypoints=dict(PHYSICS_FILES), camera_collision=True,
                command_convention='linear m, roll rad, per-node bends rad',
                lower=[m.linear_q_min_m, m.roll_q_min_rad, m.seg1_q_min_rad, m.seg2_q_min_rad],
                upper=[m.linear_q_max_m, m.roll_q_max_rad, m.seg1_q_max_rad, m.seg2_q_max_rad])
