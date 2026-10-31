"""Shared perception-target geometry for physics and the two camera scenes.

The procedural jar is an original test fixture, not a ShapeNet mesh. Dimensions
are metres; its open mouth and inner wall remain concave in Genesis.
"""
from __future__ import annotations

import hashlib
import math
import os
from pathlib import Path
import tempfile


def jar_profile(radius, height):
    """Ordered r,z surface meridian, from outer bottom to inner bottom.

    The two endpoints lie on the rotation axis. Their connecting segment has
    zero surface area and is intentionally absent from the surface mesh.
    """
    if not all(math.isfinite(v) and v > 0 for v in (radius, height)):
        raise ValueError('jar radius and height must be finite and positive')
    profile = ((0.,-.5), (.72,-.5), (.80,-.47), (.84,-.42), (.78,-.34),
        (.88,-.22), (1.,-.08), (1.,.04), (.94,.16), (.78,.25), (.52,.32),
        (.44,.36), (.44,.43), (.56,.45), (.56,.5), (.40,.5), (.36,.46),
        (.36,.36), (.45,.31), (.70,.23), (.86,.14), (.92,.03), (.92,-.07),
        (.80,-.20), (.69,-.32), (.66,-.42), (0.,-.42))
    return tuple((float(r*radius), float(z*height)) for r,z in profile)


def jar_mesh(radius, height):
    """Return a watertight outward-wound lathe with a neck, lip and hollow body."""
    vertices, rings, faces = [], [], []
    sides = 96
    for radius, z in jar_profile(radius, height):
        ring = []
        for index in range(sides if radius else 1):
            angle = 2*math.pi*index/sides
            ring.append(len(vertices))
            vertices.append((radius*math.cos(angle), radius*math.sin(angle), z))
        rings.append(ring)
    for lower,upper in zip(rings, rings[1:]):
        for index in range(sides):
            after = (index+1)%sides
            if len(lower) == 1:
                faces.append((lower[0], upper[after], upper[index]))
            elif len(upper) == 1:
                faces.append((lower[index], lower[after], upper[0]))
            else:
                faces.extend(((lower[index], lower[after], upper[after]),
                              (lower[index], upper[after], upper[index])))
    return vertices, faces


def jar_mesh_path(radius, height):
    vertices, faces = jar_mesh(radius, height)
    text = '# EleSim procedural jar v1; metres, Z up, centred on bounding box\n'
    text += ''.join('v %.10f %.10f %.10f\n'%v for v in vertices)
    text += ''.join('f %d %d %d\n'%tuple(i+1 for i in face) for face in faces)
    data = text.encode('ascii')
    digest = hashlib.sha256(data).hexdigest()
    from elesim_sim.cache import _prepare_cache_dir
    root = Path(os.environ.get('XDG_CACHE_HOME', str(Path.home()/'.cache')))/'elesim-targets'
    _prepare_cache_dir(root, private=True)
    target = root/('jar-'+digest+'.obj')
    if target.exists():
        if target.is_symlink() or target.read_bytes() != data:
            raise ValueError('cached jar mesh differs from its content hash')
        return target
    with tempfile.NamedTemporaryFile(dir=root, prefix='.jar-', delete=False) as stream:
        temporary = Path(stream.name)
        try:
            stream.write(data)
            stream.flush()
            os.replace(temporary, target)
        finally:
            temporary.unlink(missing_ok=True)
    return target


def target_morph(gs, shape, *, radius, height, pos, fixed, collision):
    kwargs = dict(pos=pos, fixed=fixed, collision=collision)
    if shape == 'jar':
        morph = gs.morphs.Mesh
        kwargs.update(file=str(jar_mesh_path(radius, height)), convexify=False, decimate=False,
                      watertighten=None, file_meshes_are_zup=True, align=False)
    elif shape in {'sphere', 'cylinder'}:
        morph = gs.morphs.Cylinder if shape == 'cylinder' else gs.morphs.Sphere
        kwargs['radius'] = max(.01, float(radius))
        if shape == 'cylinder':
            kwargs['height'] = max(.01, float(height))
    else:
        raise ValueError('unsupported perception target shape: '+str(shape))
    try:
        return morph(**kwargs)
    except TypeError:
        kwargs.pop('collision', None)
        return morph(**kwargs)
