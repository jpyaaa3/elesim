"""CPU render of actual exported collision spheres beside their source meshes."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from .v2_geometry import urdf_poses


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--assembly', type=Path, default=Path('payload/data/models/assemblies/zed-mini'))
    parser.add_argument('--pose', choices=('start', 'home'), default='start')
    args = parser.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    import trimesh

    metadata = json.loads((args.model / 'metadata.json').read_text())
    data = yaml.safe_load((args.model / 'robot.yml').read_text())
    kin = data.get('robot_cfg', data)['kinematics']
    plan = json.loads(args.plan.read_text())
    q = plan['cases'][0]['goal_command'] if args.pose == 'start' else plan['home']
    poses = urdf_poses(args.model / 'arm.urdf', metadata['joint_names'], q)
    original = ET.parse(args.assembly / 'arm.urdf').getroot()
    names = list(kin['collision_spheres'])
    fig = plt.figure(figsize=(16, 8), facecolor='#f3f5f8')
    vertices = []
    for panel in (1, 2):
        ax = fig.add_subplot(1, 2, panel, projection='3d')
        ax.set_facecolor('#f3f5f8')
        for index, name in enumerate(names):
            color = plt.cm.tab20(index % 20)
            if panel == 1:
                link = original.find(f"link[@name='{name}']")
                geometry = link.findall('collision')
                if name == 'camera' and not geometry:
                    geometry = link.findall('visual')
                for col in geometry:
                    node = col.find('geometry/mesh')
                    mesh = trimesh.load_mesh(args.assembly / node.get('filename'), process=False)
                    mesh.apply_scale(np.fromstring(node.get('scale', '1 1 1'), sep=' '))
                    origin = col.find('origin')
                    transform = np.eye(4)
                    if origin is not None:
                        transform[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
                        transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                    mesh.apply_transform(poses[name] @ transform)
                    vertices.extend(mesh.vertices)
                    ax.add_collection3d(Poly3DCollection(mesh.triangles, facecolor=color, edgecolor='none'))
            else:
                unit = trimesh.creation.icosphere(subdivisions=2)
                triangles = []
                for sphere in kin['collision_spheres'][name]:
                    center = poses[name][:3, :3] @ sphere['center'] + poses[name][:3, 3]
                    triangles.append(unit.triangles * sphere['radius'] + center)
                if triangles:
                    ax.add_collection3d(Poly3DCollection(np.concatenate(triangles), facecolor=color,
                                                        edgecolor=(.1, .15, .2, .12), linewidth=.08))
        ax.view_init(elev=32, azim=-55)
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        count = sum(map(len, kin['collision_spheres'].values()))
        ax.set_title('Original collision meshes (plate excluded)' if panel == 1
                     else f'V2 MorphIt inward shell ({count} spheres)', fontsize=15)
    points = np.asarray(vertices)
    center = (points.min(0) + points.max(0)) / 2
    radius = np.max(np.ptp(points, axis=0)) * .56
    for ax in fig.axes:
        ax.set_xlim(center[0]-radius, center[0]+radius)
        ax.set_ylim(center[1]-radius, center[1]+radius)
        ax.set_zlim(center[2]-radius, center[2]+radius)
        ax.set_box_aspect((1, 1, 1))
    fig.suptitle(f'cuRobo 0.9.0 + official MorphIt | {args.pose} pose | same scale', fontsize=18)
    fig.text(.5, .03, 'Surface-inset sampling; inward-tangent filter. Physics acceptance remains independent.', ha='center', fontsize=11)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160, bbox_inches='tight')
    plt.close(fig)


if __name__ == '__main__':
    main()
