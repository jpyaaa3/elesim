"""Point-cloud surface reconstruction; native Poisson runs in an owned CPU child."""
from __future__ import annotations

import importlib.util
import math
import multiprocessing
import time

import numpy as np

from .contracts import Mesh


def supported_points(shape, minimum, radius):
    """Use the existing generated-neighbour policy without filtering observations."""
    observed, generated = shape.cloud.observed, shape.predicted
    retained = generated
    if minimum and len(generated):
        from scipy.spatial import cKDTree
        distances, _ = cKDTree(np.concatenate((observed, generated))).query(
            generated, k=minimum + 1)
        retained = generated[distances[:, -1] <= radius]
    return np.concatenate((observed, retained)), dict(
        observed_points=len(observed), generated_points=len(generated),
        meshed_generated_points=len(retained),
        rejected_generated_points=len(generated) - len(retained))


def topology(vertices, triangles):
    """Count actual mesh edges/components, rather than occupied-cell neighbours."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    edges = np.concatenate((triangles[:, [0, 1]], triangles[:, [1, 2]],
                            triangles[:, [2, 0]]))
    unique, inverse, counts = np.unique(np.sort(edges, axis=1), axis=0,
                                        return_inverse=True, return_counts=True)
    graph = coo_matrix((np.ones(len(unique)), (unique[:, 0], unique[:, 1])),
                       shape=(len(vertices), len(vertices)))
    order = np.argsort(inverse)
    adjacent = inverse[order[1:]] == inverse[order[:-1]]
    face_ids = np.tile(np.arange(len(triangles)), 3)
    a, b = face_ids[order[:-1][adjacent]], face_ids[order[1:][adjacent]]
    face_graph = coo_matrix((np.ones(len(a)), (a, b)),
                            shape=(len(triangles), len(triangles)))
    winding = np.bincount(inverse, weights=np.where(edges[:, 0] < edges[:, 1], 1, -1))
    return dict(boundary_edges=int(np.count_nonzero(counts == 1)),
                nonmanifold_edges=int(np.count_nonzero(counts > 2)),
                inconsistent_winding_edges=int(np.count_nonzero((counts == 2) & (winding != 0))),
                mesh_vertex_components=int(connected_components(graph, directed=False,
                                                                 return_labels=False)),
                mesh_edge_components=int(connected_components(face_graph, directed=False,
                                                               return_labels=False)))


def _checkpoint(cancelled):
    if cancelled():
        from .pipeline import PipelineCancelled
        raise PipelineCancelled('surface reconstruction cancelled')


def _separated_triangles(a, b):
    """Prove separation on triangle normals, edge crosses and in-plane axes.

    Open3D's intersection predicate can report near-coplanar, disjoint faces.
    Only a strictly positive projection gap beyond roundoff removes a native
    candidate. Contact and numerically uncertain pairs remain intersections.
    """
    coordinate_scale = np.maximum(np.abs(a).max(axis=(1,2)), np.abs(b).max(axis=(1,2)))
    center = (a.mean(axis=1)+b.mean(axis=1))/2
    a, b = a-center[:,None], b-center[:,None]
    ea, eb = np.roll(a, -1, axis=1)-a, np.roll(b, -1, axis=1)-b
    na, nb = np.cross(ea[:,0], ea[:,1]), np.cross(eb[:,0], eb[:,1])
    axes = np.concatenate((na[:,None], nb[:,None],
        np.cross(ea[:,:,None], eb[:,None,:]).reshape(-1,9,3),
        np.cross(na[:,None], ea), np.cross(nb[:,None], eb)), axis=1)
    scale = np.maximum(np.linalg.norm(ea,axis=2).max(axis=1),
                       np.linalg.norm(eb,axis=2).max(axis=1))
    length = np.linalg.norm(axes, axis=2)
    valid = length > (scale*scale*1e-12)[:,None]
    axes = axes/np.where(valid, length, 1.)[:,:,None]
    pa, pb = np.einsum('kvc,kac->kav', a, axes), np.einsum('kvc,kac->kav', b, axes)
    gap = np.maximum(pb.min(axis=2)-pa.max(axis=2), pa.min(axis=2)-pb.max(axis=2))
    tolerance = scale*1e-10 + 64*np.finfo(float).eps*coordinate_scale
    return np.any(valid & (gap > tolerance[:,None]), axis=1)


def _self_intersections(surface):
    candidates = np.asarray(surface.get_self_intersecting_triangles()).reshape(-1,2)
    vertices, triangles = np.asarray(surface.vertices), np.asarray(surface.triangles)
    confirmed = []
    for start in range(0, len(candidates), 1024):
        pairs = candidates[start:start+1024]
        separated = _separated_triangles(vertices[triangles[pairs[:,0]]],
                                          vertices[triangles[pairs[:,1]]])
        confirmed.extend(pairs[~separated])
    return np.asarray(confirmed, dtype=np.int64).reshape(-1,2), len(candidates)


def _clean_surface(surface, tolerance, minimum_area=0., sliver_tolerance=0.):
    """Weld numerical near-duplicates with a measured micrometre-scale bound."""
    original = np.asarray(surface.vertices).copy()
    shift = 0.
    if tolerance and len(original):
        from scipy.spatial import cKDTree
        surface.merge_close_vertices(tolerance)
        shift = float(cKDTree(np.asarray(surface.vertices)).query(original)[0].max())
        if shift > 2*tolerance+1e-12:
            raise ValueError('Poisson vertex welding exceeded its displacement bound')
    surface.remove_duplicated_vertices()
    surface.remove_duplicated_triangles()
    surface.remove_degenerate_triangles()
    faces = np.asarray(surface.triangles)
    vertices = np.asarray(surface.vertices)
    corners = vertices[faces]
    cross = np.linalg.norm(np.cross(corners[:,1]-corners[:,0], corners[:,2]-corners[:,0]), axis=1)
    longest = np.maximum.reduce([np.linalg.norm(corners[:,a]-corners[:,b], axis=1)
                                for a,b in ((0,1),(1,2),(2,0))])
    slivers = (cross <= 1e-12) | (cross < longest*sliver_tolerance)
    surface.remove_triangles_by_mask(slivers)
    faces = np.asarray(surface.triangles)
    _, inverse, counts = np.unique(np.sort(faces, axis=1), axis=0,
                                   return_inverse=True, return_counts=True)
    a, b, c = faces.T
    parity = ((a > b).astype(int)+(a > c)+(b > c)) % 2
    winding = np.bincount(inverse, weights=1-2*parity)
    # An identical pair with opposite winding contributes no oriented surface.
    # Removing both cancels a zero-thickness flap without capping a real hole.
    cancelled = ((counts == 2) & (winding == 0))[inverse]
    surface.remove_triangles_by_mask(cancelled)
    removed_components, removed_area = 0, 0.
    if minimum_area and len(surface.triangles):
        labels, _, areas = surface.cluster_connected_triangles()
        labels, areas = np.asarray(labels), np.asarray(areas)
        small = areas < minimum_area
        removed_components, removed_area = int(small.sum()), float(areas[small].sum())
        if np.all(small):
            raise ValueError('Poisson surface components fall below min_component_area_m2')
        surface.remove_triangles_by_mask(small[labels])
    surface.remove_unreferenced_vertices()
    return dict(weld_max_displacement_m=shift,
                cleanup_removed_vertices=len(original)-len(surface.vertices),
                removed_sliver_triangles=int(slivers.sum()),
                cancelled_opposite_triangles=int(cancelled.sum()),
                removed_small_components=removed_components,
                removed_component_area_m2=removed_area)


def _regularize_surface(surface, options):
    """Extract the zero level of a sampled Poisson-mesh signed distance field.

    Parity voting is an inferred solid-proxy assumption, not camera free-space
    evidence. Exterior padding is positive even when the source mesh is open.
    The resulting proxy must pass all the usual geometry checks before export.
    """
    import open3d as o3d
    from .isosurface import marching_tetrahedra
    step = options['remesh_resolution_m']
    points = np.asarray(surface.vertices)
    origin = (np.floor(points.min(axis=0)/step)-2)*step
    upper = (np.ceil(points.max(axis=0)/step)+2)*step
    dimensions = np.rint((upper-origin)/step).astype(np.int64)+1
    nodes = math.prod(map(int, dimensions))
    if nodes > options['max_grid_points']:
        raise ValueError('Poisson regularization grid budget exceeded; increase remesh_resolution_m')
    scene = o3d.t.geometry.RaycastingScene(nthreads=options['threads'])
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(surface))
    field = np.empty(nodes, dtype=float)
    for start in range(0, nodes, 65_536):
        stop = min(nodes, start+65_536)
        coordinates = np.column_stack(np.unravel_index(np.arange(start,stop), dimensions))
        query = o3d.core.Tensor(origin+coordinates*step, dtype=o3d.core.Dtype.Float32)
        field[start:stop] = scene.compute_signed_distance(query, nthreads=options['threads'], nsamples=3).numpy()
    field = field.reshape(tuple(dimensions))
    boundary = np.zeros(field.shape, dtype=bool)
    boundary[[0,-1],:,:] = True
    boundary[:,[0,-1],:] = True
    boundary[:,:,[0,-1]] = True
    corrections = int(np.sum(field[boundary] < 0))
    field[boundary] = np.abs(field[boundary])
    vertices, triangles = marching_tetrahedra(field, origin, step)
    result = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices),
                                      o3d.utility.Vector3iVector(triangles))
    return result, dict(remesh_grid_points=nodes, remesh_boundary_sign_corrections=corrections,
                        field_kind='poisson_mesh_signed_distance', field_sign='parity_vote_solid_proxy')


def _fit_poisson(points, options):
    """Build a native candidate. Callers must validate before exporting a scene."""
    import open3d as o3d
    center = points.mean(axis=0)
    local = points-center
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(local))
    neighbors = min(options['normal_neighbors'], len(points)-1)
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=options['normal_radius_m'], max_nn=neighbors))
    cloud.orient_normals_consistent_tangent_plane(neighbors,
        options['orientation_penalty'], options['orientation_cosine'])
    normals = np.asarray(cloud.normals)
    # Only flip the whole orientation. Per-point radial flipping would erase
    # correct opposite normals on cavity/concave surfaces.
    if np.sum(normals*local) < 0:
        normals *= -1
    surface, density = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        cloud, depth=options['depth'], scale=1.1, linear_fit=options['linear_fit'],
        n_threads=options['threads'])
    raw_vertices, raw_triangles = len(surface.vertices), len(surface.triangles)
    source_pairs, source_candidates = _self_intersections(surface)
    source_quality = dict(source_boundary_edges=len(surface.get_non_manifold_edges(False))
        -len(surface.get_non_manifold_edges(True)),
        source_nonmanifold_edges=len(surface.get_non_manifold_edges(True)),
        source_self_intersection_pairs=len(source_pairs), source_intersection_candidates=source_candidates)
    threshold = None
    if options['density_trim_quantile']:
        density = np.asarray(density)
        threshold = float(np.quantile(density, options['density_trim_quantile']))
        surface.remove_vertices_by_mask(density < threshold)
    regularization = {}
    if options['remesh_resolution_m']:
        surface, regularization = _regularize_surface(surface, options)
    cleaning = _clean_surface(surface, options['weld_tolerance_m'], options['min_component_area_m2'],
                              options['sliver_tolerance_m'])
    if len(surface.vertices) > 100_000 or len(surface.triangles) > 100_000:
        raise ValueError('Poisson mesh exceeds scene budget; reduce depth')
    return surface, center, dict(library_version=o3d.__version__, raw_vertices=raw_vertices,
        raw_triangles=raw_triangles, density_threshold=threshold, **source_quality,
        **regularization, **cleaning)


def _poisson(points, options):
    """Native computation, called only in the spawned child (or native tests)."""
    surface, center, diagnostics = _fit_poisson(points, options)
    nonmanifold_vertices = len(surface.get_non_manifold_vertices())
    pairs, candidates = _self_intersections(surface)
    intersection_pairs = len(pairs)
    if nonmanifold_vertices or intersection_pairs:
        raise ValueError(f'Poisson surface quality rejected: nonmanifold_vertices={nonmanifold_vertices}, '
                         f'self_intersection_pairs={intersection_pairs}; rescan or adjust normals')
    return (np.asarray(surface.vertices)+center, np.asarray(surface.triangles).astype(np.int64),
            dict(diagnostics, vertex_manifold=True, self_intersections=False,
                 intersection_candidates=candidates, intersection_check='native_plus_separating_axes'))


def _poisson_worker(connection, points, options):
    try:
        connection.send(('ok', _poisson(points, options)))
    except Exception as error:
        connection.send(('error', f'{type(error).__name__}: {error}'[:1000]))
    finally:
        connection.close()


class PoissonSurface:
    """Fit Poisson, then extract an inferred solid proxy from its distance field.

    The bounded field closes an open source; it does not establish unobserved
    cavity geometry. Resolution and component-area filtering are explicit and
    recorded. The native-only option can use density trimming instead. Native
    crashes, cancellation and deadlines terminate only the owned CPU child.
    """
    api_version, stage, version = 1, 'mesher', 'screened-poisson-surface-v2'
    supports_cancellation = True

    def __init__(self, *, options, assets):
        defaults = dict(depth=8, normal_neighbors=30, normal_radius_m=.012,
                        orientation_penalty=0., orientation_cosine=1.,
                        linear_fit=False, weld_tolerance_m=0., sliver_tolerance_m=.000001,
                        min_component_area_m2=.000025,
                        remesh_resolution_m=.004, max_grid_points=2_000_000,
                        density_trim_quantile=0., threads=1, timeout_s=60.,
                        min_generated_neighbors=0, generated_neighbor_radius_m=.01)
        if assets or set(options)-set(defaults):
            raise ValueError('invalid Poisson surface options/assets')
        self.options = {**defaults, **options}
        if type(self.options['linear_fit']) is not bool:
            raise ValueError('linear_fit must be boolean')
        for name, lower, upper in (('depth', 5, 9), ('normal_neighbors', 8, 100),
                ('threads', 1, 8), ('min_generated_neighbors', 0, 64),
                ('max_grid_points', 1, 4_000_000)):
            value = self.options[name]
            if type(value) is not int or not lower <= value <= upper:
                raise ValueError(f'{name} must be an integer in {lower}..{upper}')
        for name, lower, upper in (('normal_radius_m', .001, .2),
                ('orientation_penalty', 0., 100.), ('orientation_cosine', .1, 1.),
                ('weld_tolerance_m', 0., .0001),
                ('sliver_tolerance_m', 0., .0001),
                ('min_component_area_m2', 0., .001),
                ('remesh_resolution_m', 0., .02),
                ('generated_neighbor_radius_m', .0001, .2),
                ('density_trim_quantile', 0., .1), ('timeout_s', 1., 120.)):
            value = self.options[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not lower <= value <= upper:
                raise ValueError(f'{name} must be finite and in {lower}..{upper}')
        if 0 < self.options['remesh_resolution_m'] < .001:
            raise ValueError('remesh_resolution_m must be zero or at least .001 metres')
        if self.options['remesh_resolution_m'] and self.options['density_trim_quantile']:
            raise ValueError('density trimming must be disabled for signed-distance regularization')
        self._diagnostics = {}

    def prepare(self, *, cancelled=lambda: False):
        _checkpoint(cancelled)
        # Do not import Open3D's native libraries in Pilot's parent process.
        if importlib.util.find_spec('open3d') is None:
            raise RuntimeError('Poisson mesher requires the Pilot meshing extra (Open3D); install it before scanning')

    def mesh(self, shape, task, *, cancelled=lambda: False):
        _checkpoint(cancelled)
        self.prepare()
        points, support = supported_points(shape, self.options['min_generated_neighbors'],
                                           self.options['generated_neighbor_radius_m'])
        points = np.unique(points, axis=0)
        if len(points) < 8 or np.linalg.matrix_rank(points-points.mean(axis=0)) < 3:
            raise ValueError('Poisson requires a spatial surface cloud; collect more distinct views')
        self._diagnostics = dict(support, **self.options, unique_input_points=len(points),
            surface_method='screened_poisson', normals='estimated_consistent_tangent_plane',
            free_space_observed=False, geometry_complete_claimed=False)
        _checkpoint(cancelled)
        context = multiprocessing.get_context('spawn')
        parent, child = context.Pipe(duplex=False)
        process = context.Process(target=_poisson_worker, args=(child, points, self.options), daemon=True)
        try:
            process.start()
            child.close()
            deadline = time.monotonic()+self.options['timeout_s']
            while True:
                _checkpoint(cancelled)
                if time.monotonic() >= deadline:
                    raise TimeoutError('Poisson surface reconstruction exceeded its deadline')
                if parent.poll(.05):
                    try:
                        status, payload = parent.recv()
                    except EOFError as error:
                        process.join(.1)
                        raise RuntimeError(f'Poisson worker exited without a result (exit {process.exitcode})') from error
                    if status != 'ok':
                        raise RuntimeError('Poisson surface reconstruction failed: '+payload)
                    vertices, triangles, diagnostics = payload
                    break
                if not process.is_alive():
                    raise RuntimeError(f'Poisson worker exited without a result (exit {process.exitcode})')
        finally:
            parent.close()
            child.close()
            if process.pid is not None:
                process.join(.1)
                if process.is_alive():
                    process.terminate()
                    process.join(2.)
                if process.is_alive():
                    process.kill()
                    process.join(2.)
                process.close()
        _checkpoint(cancelled)
        mesh = Mesh(shape.cloud.object_id, task.frame_id, vertices, triangles)
        metrics = topology(mesh.vertices, mesh.triangles)
        self._diagnostics.update(diagnostics, **metrics,
                                 vertices=len(mesh.vertices), triangles=len(mesh.triangles))
        if metrics['nonmanifold_edges'] or metrics['inconsistent_winding_edges']:
            raise ValueError(f'Poisson surface produced invalid edge topology: '
                f'nonmanifold_edges={metrics["nonmanifold_edges"]}, '
                f'inconsistent_winding_edges={metrics["inconsistent_winding_edges"]}')
        return mesh

    def diagnostics(self):
        return dict(self._diagnostics)
