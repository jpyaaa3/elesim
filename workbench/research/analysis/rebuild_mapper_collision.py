"""Bake a conservative Home→Extend Arm envelope into a standalone mapper.

Final-pose contact geometry stays independent of the preparation filter. Each
URDF collision part uses a convex XY hull of its sampled trajectory, enlarged
by a bound on between-sample curvature and 1 mm clearance. A blocked envelope
can be conservative; a clear one does not certify subsequent roll/closing/lift.
The object is a stationary vertical cylinder spanning the robot's heights.
"""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull

from workbench.research.experiments.hug_interpolation.configure import BUNDLE, ROOT, asset_digest, fk, rotation
from workbench.research.experiments.wrap_grasp.app_config import load_mechanism
from workbench.research.experiments.wrap_grasp.configs.loader import load_config


def hull(points):
    points = np.unique(np.asarray(points), axis=0)
    return points[ConvexHull(points).vertices]


def origin_transform(element):
    result = np.eye(4)
    if element is not None:
        rpy = np.fromstring(element.get('rpy', '0 0 0'), sep=' ')
        result[:3, :3] = rotation([0, 0, 1], rpy[2]) @ rotation([0, 1, 0], rpy[1]) @ rotation([1, 0, 0], rpy[0])
        result[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
    return result


def collision_parts(root, bundle):
    """Match URDF collisions; promote camera visual as the RL model does."""
    parts = []
    meshes = {}
    angles = (np.arange(32)+.5)*2*math.pi/32
    for link in root.findall('link'):
        elements = link.findall('collision')
        if link.get('name') == 'camera' and not elements:
            elements = link.findall('visual')
        for index, element in enumerate(elements):
            geom = element.find('geometry')[0]
            transform = origin_transform(element.find('origin'))
            sphere = None
            if geom.tag == 'mesh':
                path = (bundle / geom.get('filename')).resolve()
                if path not in meshes:
                    if path.suffix != '.obj':
                        raise ValueError(f'Unsupported collision mesh {path}')
                    vertices = np.array([list(map(float, row.split()[1:4])) for row in path.read_text().splitlines() if row.startswith('v ')])
                    meshes[path] = hull(vertices)
                vertices = meshes[path]*np.fromstring(geom.get('scale', '1 1 1'), sep=' ')
            elif geom.tag == 'box':
                size = np.fromstring(geom.get('size'), sep=' ')/2
                vertices = np.array(list(itertools.product((-1, 1), repeat=3)))*size
            elif geom.tag == 'cylinder':
                # Circumscribed polygon contains the entire circular cross-section.
                radius = float(geom.get('radius'))/math.cos(math.pi/32)
                vertices = np.array([[radius*math.cos(a), radius*math.sin(a), z]
                                     for a in angles for z in (-float(geom.get('length'))/2, float(geom.get('length'))/2)])
            elif geom.tag == 'sphere':
                sphere = float(geom.get('radius'))
                vertices = np.zeros((1, 3))
            else:
                raise ValueError(f'Unsupported collision geometry {geom.tag}')
            vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
            parts.append(dict(name=link.get('name'), shape_index=index, vertices=vertices,
                              sphere=sphere))
    return parts


def preparation_envelope(*, max_step_deg=.5):
    mechanism = load_mechanism()
    config = load_config()
    root = ET.parse(BUNDLE/'robot.urdf').getroot()
    parts = collision_parts(root, BUNDLE)
    home = np.array(mechanism.presets['home'])
    extend = np.array(mechanism.presets['extend_arm'])
    if home[1] != 0 or extend[1] != 0 or home[0] == extend[0]:
        raise ValueError('Recheck curvature bound for a changed preparation primitive')
    count = max(1, int(math.ceil(np.max(np.abs(np.degrees(home[2:]-extend[2:])))*5/max_step_deg)))
    housing = next(p for p in parts if p['name'] == 'housing')
    frame = fk(root, 'housing')
    vertices = housing['vertices'] @ frame[:3, :3].T + frame[:3, 3]
    body_center = (vertices.min(0)+vertices.max(0))/2
    if not np.allclose((fk(root, 'node0')[:3, 3]-body_center)[:2], [.1445, 0], atol=1e-8):
        raise ValueError('Mapper frame changed; rebuild analytic mounting constants too')
    moving = {f'node{i}' for i in range(10)} | {'housing', 'wedge', 'gripper_base', 'gripper_claw_left', 'gripper_claw_right', 'camera'}
    leg_positions = {f'{leg}_{joint}_joint': angle for leg in ('FL', 'FR', 'RL', 'RR')
                     for joint, angle in zip(('hip', 'thigh', 'calf'), config.scene.go2.leg_pose_rad)}
    def positions(command):
        values = dict(leg_positions, j_plate_housing=command[0], j_housing_wedge=command[1])
        values.update({name: command[2 if i < 5 else 3] for i, name in enumerate(config.arm.bend_joints)})
        return values
    def project(part, command):
        frame = fk(root, part['name'], positions(command))
        world = part['vertices'] @ frame[:3, :3].T + frame[:3, 3]
        if part['sphere'] is not None:
            angles = (np.arange(32)+.5)*2*math.pi/32
            radius = part['sphere']/math.cos(math.pi/32)
            world = world[0]+np.c_[radius*np.cos(angles), radius*np.sin(angles), np.zeros(32)]
        return hull(np.c_[-(world[:, 1]-body_center[1]), world[:, 0]-body_center[0]]*1000)
    # A rotated vector's second derivative is bounded by length*(sum |dq|)^2.
    # Summing all fixed chain lengths is deliberately conservative. Linear is
    # the first moving axis, so its affine translation adds no curvature.
    joint_by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
    def chain_radius(part):
        length = float(np.linalg.norm(part['vertices'], axis=1).max())
        link = part['name']
        while link in joint_by_child:
            joint = joint_by_child[link]
            element = joint.find('origin')
            if element is not None:
                length += float(np.linalg.norm(np.fromstring(element.get('xyz', '0 0 0'), sep=' ')))
            link = joint.find('parent').get('link')
        return length
    chain_length = max(chain_radius(part) for part in parts if part['name'] in moving)
    angular_change = 5*float(np.abs(home[2:]-extend[2:]).sum())
    padding = chain_length*1000*angular_change**2/(8*count**2) + 1e-6
    samples = np.linspace(home, extend, count+1)
    result = []
    # Moving parts first: they usually decide rejection without testing legs.
    for part in sorted(parts, key=lambda p: (p['name'] not in moving, p['name'])):
        if part['name'] in moving:
            polygons = [project(part, command) for command in samples]
            row = dict(name=part['name'], poly=hull(np.vstack(polygons)).round(9).tolist(),
                       home=polygons[0].round(9).tolist(), extend=polygons[-1].round(9).tolist())
        else:
            row = dict(name=part['name'], poly=project(part, home).round(9).tolist())
        result.append(row)
    return dict(schema='elesim-mapper-preparation-envelope', schema_version=1,
                model_sha256=asset_digest(), home_command=home.tolist(), extend_command=extend.tolist(),
                command_units=['linear_m', 'roll_rad', 'q3_per_node_rad', 'q4_per_node_rad'],
                home_total_deg=(np.degrees(home[2:])*5).tolist(),
                frame='map x=-world Y, y=world X; nominal housing centre',
                body_center_world_m=body_center.tolist(), leg_pose_rad=list(config.scene.go2.leg_pose_rad),
                clearance_mm=1., sampling_padding_mm=padding, intervals=count,
                chain_radius_bound_m=chain_length, total_joint_delta_rad=angular_change,
                construction='Per-part convex XY hull over linear interpolation Home→Extend Arm; curvature-bound padding; circumscribed URDF primitives',
                scope='Stationary tall vertical cylinder; fixed Go2 and legs. Subsequent roll, closing and lift untested. Envelope rejection is conservative, not global impossibility.',
                parts=result)


VIEWER = '''// BEGIN PREPARATION ENVELOPE VIEWER
const preparationCanvas=document.createElement('canvas');preparationCanvas.width=canvas.width;preparationCanvas.height=canvas.height;
let preparationDiameter=null;
function drawPreparationEnvelope(D){
 if(!$('preparation').checked)return;
 if(preparationDiameter!==D){const p=preparationCanvas.getContext('2d');p.clearRect(0,0,canvas.width,canvas.height);p.fillStyle='#64748b';p.strokeStyle='#64748b';p.lineJoin='round';p.lineWidth=2*(D/2+PREPARATION_COLLISION.clearance_mm+PREPARATION_COLLISION.sampling_padding_mm)*scale;
  for(const part of PREPARATION_COLLISION.parts){p.beginPath();part.poly.forEach((v,i)=>{if(i)p.lineTo(X(v[0]),Y(v[1]));else p.moveTo(X(v[0]),Y(v[1]));});p.closePath();p.fill();p.stroke();}preparationDiameter=D;
 }
 ctx.save();ctx.beginPath();ctx.rect(X(0),Y(600),500*scale,600*scale);ctx.clip();ctx.globalAlpha=.12;ctx.drawImage(preparationCanvas,0,0);ctx.restore();
}
// END PREPARATION ENVELOPE VIEWER
'''

CLASSIFY = '''function classifyCandidates(m,q3,q4,D,roll){
 if(!m.candidates.length)return;const polys=collisionGeometry(q3,q4),self=selfCollision(polys);
 for(const c of m.candidates){c.finalCollision=self||finalObjectCollision(polys,c,D,roll);c.preparation=preparationReport(c,D);c.collision=c.finalCollision||c.preparation.blocked;}
}'''


def rebuild(html, data):
    runtime = (ROOT/'workbench/research/support/hugmap_preparation_collision.js').read_text()
    runtime = runtime.replace('__PREPARATION_DATA__', json.dumps(data, separators=(',', ':'), allow_nan=False))
    marker = '// BEGIN GENERATED PREPARATION COLLISION'
    if marker in html:
        return re.sub(re.escape(marker)+r'.*?// END GENERATED PREPARATION COLLISION', lambda _: runtime.rstrip(), html, count=1, flags=re.S)
    old = 'function objectCollision(polys,c,D,roll){'
    if html.count(old) != 1:
        raise ValueError('Unsupported static collision kernel')
    html = html.replace(old, 'function finalObjectCollision(polys,c,D,roll){', 1)
    start = html.index('function classifyCandidates(')
    end = html.index('\n', start)
    html = html[:start]+runtime+'\n'+CLASSIFY+html[end:]
    old_label = "${c.collision?'COLLISION / clearance violation':'no projected collision'}"
    if old_label not in html:
        raise ValueError('Missing candidate status display')
    html = html.replace(old_label, '${collisionLabel(c)}', 1)
    checkbox = '<label><input id="preparation" type="checkbox" checked>Home → Extend Arm envelope</label>'
    html = html.replace('<label><input id="overlay"', checkbox+'\n<label><input id="overlay"', 1)
    html = html.replace('function drawQPlot(', VIEWER+'\nfunction drawQPlot(', 1)
    html = html.replace('for(const r of current)point(r[2]', 'drawPreparationEnvelope(D);\nfor(const r of current)point(r[2]', 1)
    html = html.replace("$('overlay').oninput=draw;", "$('overlay').oninput=draw;\n$('preparation').oninput=draw;", 1)
    html = html.replace("$('overlay').disabled=qMode();", "$('overlay').disabled=qMode();$('preparation').disabled=qMode();", 1)
    html = html.replace('translucent gray: collision', 'translucent gray: final collision or preparation envelope overlap', 1)
    paragraph = '''<p>Colored samples pass both final-pose projected CAD checks and the Home → Extend Arm envelope filter; μ runs blue (0) to red (selected maximum). Translucent gray samples fail at least one check. The shaded XY region is the conservative preparation exclusion envelope, expanded by D/2, 1 mm clearance and the computed sampling bound. Its checkbox changes visibility only. Home is linear −120 mm, roll 0°, q3 −170°, q4 +170°; Extend Arm is linear −10 mm, roll 0°, q3/q4 0°. Actual URDF arm, camera, housing and fixed Go2 collision shapes are included in preparation. Final-pose arm contact allows 1 mm reconstruction error; preparation allows no overlap and requires positive clearance. Camera/housing/wedge retain 1 mm final clearance; self-collision retains 0.05 mm numerical tolerance. Gray saved points remain reference data. The purple 114 mm line is a reference only. q3/q4 are segment total bends. Roll after extension, closing, moving-object contact and lifting remain untested.</p>'''
    html, count = re.subn(r'<p>Gray rectangle:.*?</p>', lambda _: paragraph, html, count=1, flags=re.S)
    if count != 1:
        raise ValueError('Missing collision explanation')
    html = re.sub(r'<p><b>Limits:</b>.*?</p>', '<p><b>Limits:</b> The contact model is approximate. Preparation uses conservative per-part convex XY envelopes for a stationary tall vertical cylinder and fixed Go2 legs; an envelope overlap can reject a feasible alternative. Clear samples have not been certified for subsequent roll, grasping or lifting. Saved map reconstruction is not an approach test.</p>', html, count=1, flags=re.S)
    return html


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mapper', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args(argv)
    data = preparation_envelope()
    old = args.mapper.read_bytes()
    updated = rebuild(old.decode(), data).encode()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(updated)
    report = dict(source_path=str(args.mapper.resolve()), source_sha256=hashlib.sha256(old).hexdigest(),
                  output_sha256=hashlib.sha256(updated).hexdigest(),
                  envelope={k:v for k,v in data.items() if k != 'parts'},
                  part_count=len(data['parts']), polygon_vertices=sum(len(p['poly']) for p in data['parts']))
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
