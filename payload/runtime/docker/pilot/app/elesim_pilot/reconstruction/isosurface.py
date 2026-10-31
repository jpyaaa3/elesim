"""Conforming zero-level extraction on a bounded, regular scalar grid."""
from __future__ import annotations

import numpy as np


def marching_tetrahedra(field, origin, resolution, *, max_triangles=100_000):
    """Use a face-compatible six-tetrahedron subdivision and shared edge IDs.

    A 1%-cell guard around zero prevents exact grid-node contacts and vanishing
    triangles. The caller supplies exterior boundary samples and validates the
    resulting mesh; no disconnected component is silently welded or discarded.
    """
    field = field.copy()
    epsilon = resolution*.01
    near = np.abs(field) < epsilon
    field[near] = np.where(field[near] < 0., -epsilon, epsilon)
    corners = ((0,0,0), (1,0,0), (0,1,0), (1,1,0),
               (0,0,1), (1,0,1), (0,1,1), (1,1,1))
    slices = [field[x:field.shape[0]-1+x, y:field.shape[1]-1+y,
                    z:field.shape[2]-1+z] for x,y,z in corners]
    inside = sum((values < 0).astype(np.uint8) for values in slices)
    active = np.column_stack(np.nonzero((inside > 0) & (inside < 8)))
    if not len(active):
        raise ValueError('surface field has no zero-level surface')
    corner_ids = np.stack([np.ravel_multi_index((active+corner).T, field.shape)
                           for corner in corners], axis=1)
    tetrahedra = ((0,1,3,7), (0,3,2,7), (0,2,6,7),
                  (0,6,4,7), (0,4,5,7), (0,5,1,7))
    edge_faces, outward, total = [], [], 0
    values = field.ravel()
    for tetrahedron in tetrahedra:
        ids = corner_ids[:, tetrahedron]
        masks = (values[ids] < 0) @ np.array([1,2,4,8], dtype=np.uint8)
        for mask in range(1,15):
            selected = ids[masks == mask]
            if not len(selected):
                continue
            negative = [i for i in range(4) if mask & (1 << i)]
            positive = [i for i in range(4) if i not in negative]
            if len(negative) == 1:
                faces = [[(negative[0], i) for i in positive]]
            elif len(positive) == 1:
                faces = [[(positive[0], i) for i in negative]]
            else:
                a,b = negative
                c,d = positive
                faces = [[(a,c), (a,d), (b,d)], [(a,c), (b,d), (b,c)]]
            total += len(selected)*len(faces)
            if total > max_triangles:
                raise ValueError('isosurface triangle budget exceeded; increase remesh_resolution_m')
            coordinates = np.stack(np.unravel_index(selected, field.shape), axis=-1)
            direction = coordinates[:,positive].mean(axis=1)-coordinates[:,negative].mean(axis=1)
            for face in faces:
                edge_faces.append(selected[:,face])
                outward.append(direction)
    endpoints, inverse = np.unique(np.sort(np.concatenate(edge_faces), axis=2).reshape(-1,2),
                                    axis=0, return_inverse=True)
    if len(endpoints) > 100_000:
        raise ValueError('isosurface vertex budget exceeded; increase remesh_resolution_m')
    a,b = endpoints.T
    start = np.column_stack(np.unravel_index(a, field.shape))
    end = np.column_stack(np.unravel_index(b, field.shape))
    weight = values[a]/(values[a]-values[b])
    vertices = origin+(start+weight[:,None]*(end-start))*resolution
    triangles = inverse.reshape(-1,3)
    va,vb,vc = vertices[triangles].transpose(1,0,2)
    flip = np.einsum('ij,ij->i', np.cross(vb-va,vc-va), np.concatenate(outward)) < 0
    triangles[flip] = triangles[flip][:,[0,2,1]]
    return vertices, triangles
