"""Real asset topology checks; run where the existing mesh stack is available."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

trimesh = pytest.importorskip('trimesh')

from workbench.research.experiments.hug_reverse_curobo.v2_geometry import mesh_cylinder_scene, restore_mesh_urdf
from workbench.research.experiments.hug_reverse_curobo.v2_model import load_link_mesh

ROOT = Path(__file__).resolve().parents[4]


@pytest.mark.parametrize('name', ['housing', 'wedge', 'node0', 'node9', 'gripper_base',
                                 'gripper_claw_left', 'gripper_claw_right', 'camera'])
def test_normal_seams_are_closed_without_changing_collision_extents(tmp_path, name):
    model = ROOT / 'payload/data/models/planning/zed-mini'
    assembly = ROOT / 'payload/data/models/assemblies/zed-mini'
    path = tmp_path / 'arm.urdf'
    restore_mesh_urdf(model, assembly, path)
    link = ET.parse(path).getroot().find(f"link[@name='{name}']")
    mesh = load_link_mesh(link)
    assert mesh.is_watertight
    assert mesh.is_winding_consistent
    assert mesh.volume > 0
    # Compare independently transformed original vertex locations. Welding
    # topology must not inflate or shift the actual geometry.
    from scipy.spatial.transform import Rotation
    original = []
    for collision in link.findall('collision'):
        element = collision.find('geometry/mesh')
        shape = trimesh.load_mesh(element.get('filename'), process=False)
        points = shape.vertices * np.fromstring(element.get('scale', '1 1 1'), sep=' ')
        origin = collision.find('origin')
        if origin is not None:
            points = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).apply(points)
            points += np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
        original.extend(points)
    original = np.asarray(original)
    np.testing.assert_allclose(mesh.bounds, [original.min(0), original.max(0)], atol=1e-10)


def test_contact_core_preserves_full_obstacle_floor_and_measured_pose(tmp_path):
    context = dict(diameter_mm=200., height_m=.15,
                   plate_position=[.1, -.2, .6], plate_quaternion_wxyz=[.9238795325, 0, .3826834324, 0],
                   object_position=[.4, .05, .075], object_quaternion_wxyz=[1., 0, 0, 0])
    strict = mesh_cylinder_scene(context, tmp_path/'strict.obj')
    contact = mesh_cylinder_scene(context, tmp_path/'contact.obj', contact_band=.005)
    assert contact['cuboid'] == strict['cuboid']
    assert contact['mesh']['cylinder']['pose'] == strict['mesh']['cylinder']['pose']
    assert contact['mesh']['cylinder_contact_core']['pose'] == strict['mesh']['cylinder']['pose']
    full = trimesh.load_mesh(contact['mesh']['cylinder']['file_path'], process=False)
    previous = trimesh.load_mesh(strict['mesh']['cylinder']['file_path'], process=False)
    np.testing.assert_array_equal(full.vertices, previous.vertices)
    core = trimesh.load_mesh(contact['mesh']['cylinder_contact_core']['file_path'], process=False)
    assert core.is_watertight and core.is_winding_consistent
    np.testing.assert_allclose(core.extents[2], .14)
    np.testing.assert_allclose(core.extents[:2]/full.extents[:2], .95, atol=1e-7)
    np.testing.assert_allclose(core.centroid, [0., 0., 0.], atol=1e-8)


def test_contact_band_cannot_remove_the_cylinder(tmp_path):
    context = dict(diameter_mm=100., height_m=.15,
                   plate_position=[0, 0, 0], plate_quaternion_wxyz=[1, 0, 0, 0],
                   object_position=[0, 0, .075], object_quaternion_wxyz=[1, 0, 0, 0])
    with pytest.raises(ValueError, match='inside the cylinder'):
        mesh_cylinder_scene(context, tmp_path/'invalid.obj', contact_band=.05)
