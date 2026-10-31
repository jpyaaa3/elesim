"""A hollow jar must have the same closed, concave surface in every scene."""
from collections import Counter
import hashlib
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from elesim_sim.simulation.target_geometry import jar_mesh, jar_profile, target_morph


def test_jar_is_watertight_outward_and_keeps_its_neck_and_open_mouth():
    vertices, faces = jar_mesh(.06, .22)
    vertices, faces = np.array(vertices), np.array(faces)
    assert np.ptp(vertices, axis=0) == pytest.approx([.12, .12, .22])
    edges = Counter(tuple(sorted((int(a), int(b)))) for face in faces
                    for a, b in zip(face, np.roll(face, -1)))
    assert set(edges.values()) == {2}
    triangles = vertices[faces]
    areas = np.linalg.norm(np.cross(triangles[:, 1]-triangles[:, 0],
                                    triangles[:, 2]-triangles[:, 0]), axis=1)/2
    assert areas.min() > 1e-8
    volume = np.einsum('ij,ij->i', triangles[:, 0],
                      np.cross(triangles[:, 1], triangles[:, 2])).sum()/6
    assert 0 < volume < math.pi*.06**2*.22
    profile = np.array(jar_profile(.06, .22))
    assert profile[:, 0].max() == pytest.approx(.06)
    assert profile[profile[:, 1] > .07, 0].max() < .04
    # The mouth is a ring; no triangle caps its central hole.
    top = triangles[np.all(np.isclose(triangles[:, :, 2], .11), axis=1)]
    assert len(top) and np.linalg.norm(top[:, :, :2], axis=2).min() > .02


def test_physics_and_visual_jar_share_bytes_without_convexifying(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_CACHE_HOME', str(tmp_path))
    gs = NS(morphs=NS(Mesh=lambda **kwargs: kwargs))
    physics = target_morph(gs, 'jar', radius=.06, height=.22, pos=(0., 0., 1.),
                           fixed=True, collision=True)
    visual = target_morph(gs, 'jar', radius=.06, height=.22, pos=(0., 0., 1.),
                          fixed=True, collision=False)
    from pathlib import Path
    path = Path(physics['file'])
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    assert path == Path(visual['file'])
    assert physics['convexify'] is False and physics['decimate'] is False
    assert physics['file_meshes_are_zup'] is True and physics['align'] is False
    assert physics['watertighten'] is None
    assert visual['collision'] is False and physics['collision'] is True
    assert digest in path.name
    assert hashlib.sha256(path.read_bytes()).hexdigest() == digest


@pytest.mark.parametrize('radius,height', [(0., .22), (.06, float('nan')), (.06, -1.)])
def test_jar_rejects_invalid_dimensions(radius, height):
    with pytest.raises(ValueError):
        jar_mesh(radius, height)
