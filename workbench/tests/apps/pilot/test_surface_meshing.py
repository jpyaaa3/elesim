import importlib.util
import multiprocessing
import os
import time

import numpy as np
import pytest

from elesim_pilot.reconstruction import meshing
from elesim_pilot.reconstruction.artifacts import export_result
from elesim_pilot.reconstruction.contracts import CompletedObject, ObjectCloud, SceneObject, Task
from elesim_pilot.reconstruction.meshing import PoissonSurface, supported_points, topology
from elesim_pilot.reconstruction.isosurface import marching_tetrahedra
from elesim_pilot.reconstruction.models import load_model
from elesim_pilot.reconstruction.pipeline import PipelineCancelled, Result, call_model
from elesim_pilot.reconstruction.providers import VoxelSurface
from elesim_protocol.scene_artifact import SceneArtifact


def shape(observed, predicted=()):
    return CompletedObject(ObjectCloud('target', 'fixture', 'target', 'world', observed),
                           np.asarray(predicted, dtype=float).reshape(-1, 3))


def spatial_cloud():
    return shape([[x*.01, y*.01, z*.01] for x in (0, 1) for y in (0, 1) for z in (0, 1)])


def test_generated_support_does_not_drop_observations_or_mutate_original_completion():
    observed = np.array([[0., 0., 0.], [.1, 0., 0.]])
    dense = np.array([[.05+x*.001, y*.001, .002] for x in range(3) for y in range(3)])
    generated = np.vstack((dense, [4., 4., 4.]))
    cloud = shape(observed, generated)
    points, diagnostics = supported_points(cloud, 8, .01)
    np.testing.assert_array_equal(points[:2], observed)
    np.testing.assert_array_equal(points[2:], dense)
    np.testing.assert_array_equal(cloud.predicted, generated)
    assert diagnostics['meshed_generated_points'] == 9
    assert diagnostics['rejected_generated_points'] == 1


@pytest.mark.parametrize('options', [
    {'depth': 10}, {'depth': True}, {'normal_radius_m': float('nan')},
    {'normal_neighbors': 3}, {'threads': 9}, {'timeout_s': 0.},
    {'density_trim_quantile': .5}, {'min_generated_neighbors': -1},
    {'generated_neighbor_radius_m': 0.}, {'unknown': 1},
    {'remesh_resolution_m': .0001}, {'max_grid_points': 4_000_001},
    {'density_trim_quantile': .01}, {'linear_fit': 1},
])
def test_invalid_surface_parameters_fail_before_native_import(options):
    with pytest.raises(ValueError):
        PoissonSurface(options=options, assets={})


def test_missing_library_is_reported_before_scan_motion(monkeypatch):
    monkeypatch.setattr(meshing.importlib.util, 'find_spec', lambda _: None)
    with pytest.raises(RuntimeError, match='Pilot meshing extra'):
        PoissonSurface(options={}, assets={}).prepare()


def test_planar_input_requires_more_views_instead_of_inventing_a_solid(monkeypatch):
    model = PoissonSurface(options={}, assets={})
    monkeypatch.setattr(model, 'prepare', lambda: None)
    with pytest.raises(ValueError, match='more distinct views'):
        model.mesh(shape([[x, y, 0.] for x in range(3) for y in range(3)]), Task('plane', 'target'))


def test_cancellation_before_native_import():
    with pytest.raises(PipelineCancelled):
        PoissonSurface(options={}, assets={}).mesh(spatial_cloud(), Task('cancelled', 'target'),
                                                  cancelled=lambda: True)


def exit_139(*args):
    os._exit(139)


def hang(*args):
    time.sleep(20)


def worker_context(monkeypatch, target):
    real = multiprocessing.get_context('spawn')
    class Context:
        Pipe = staticmethod(real.Pipe)
        @staticmethod
        def Process(**kwargs):
            return real.Process(target=target, args=kwargs['args'], daemon=True)
    monkeypatch.setattr(meshing.multiprocessing, 'get_context', lambda _: Context())
    return {process.pid for process in multiprocessing.active_children()}


def test_native_exit_139_does_not_kill_pilot_or_leave_a_child(monkeypatch):
    before = worker_context(monkeypatch, exit_139)
    model = PoissonSurface(options={}, assets={})
    monkeypatch.setattr(model, 'prepare', lambda: None)
    with pytest.raises(RuntimeError, match=r'without a result \(exit 139\)'):
        model.mesh(spatial_cloud(), Task('failure', 'target'))
    assert {process.pid for process in multiprocessing.active_children()} == before


def test_deadline_terminates_only_the_owned_worker(monkeypatch):
    before = worker_context(monkeypatch, hang)
    model = PoissonSurface(options={'timeout_s': 1.}, assets={})
    monkeypatch.setattr(model, 'prepare', lambda: None)
    with pytest.raises(TimeoutError, match='deadline'):
        model.mesh(spatial_cloud(), Task('timeout', 'target'))
    assert {process.pid for process in multiprocessing.active_children()} == before


def test_cancellation_during_native_work_terminates_the_child(monkeypatch):
    before = worker_context(monkeypatch, hang)
    model = PoissonSurface(options={}, assets={})
    monkeypatch.setattr(model, 'prepare', lambda: None)
    calls = []
    def cancelled():
        calls.append(True)
        return len(calls) >= 4
    with pytest.raises(PipelineCancelled):
        model.mesh(spatial_cloud(), Task('cancelled', 'target'), cancelled=cancelled)
    assert {process.pid for process in multiprocessing.active_children()} == before


def test_same_factory_contract_loads_surface_mesher_without_native_import(tmp_path):
    model = load_model('mesher', {'factory': 'elesim_pilot.reconstruction.meshing:PoissonSurface'}, tmp_path)
    assert model.implementation.version == 'screened-poisson-surface-v2'


def test_pipeline_prepare_uses_the_same_cooperative_cancellation_contract(tmp_path, monkeypatch):
    monkeypatch.setattr(meshing.importlib.util, 'find_spec', lambda _: object())
    model = load_model('mesher', {'factory': 'elesim_pilot.reconstruction.meshing:PoissonSurface'}, tmp_path)
    call_model(model, 'prepare')


def test_topology_distinguishes_vertex_contact_from_shared_triangle_edges():
    cloud = shape([[0., 0., 0.], [.01, .01, .01]])
    mesh = VoxelSurface(options={'voxel_m': .01}, assets={}).mesh(cloud, Task('contact', 'target'))
    metrics = topology(mesh.vertices, mesh.triangles)
    assert metrics['mesh_vertex_components'] == 1
    assert metrics['mesh_edge_components'] == 2
    assert metrics['nonmanifold_edges'] == 0


def test_zero_surface_interpolates_a_plane_and_shares_edges_across_cells():
    grid = np.indices((3,3,3)).transpose(1,2,3,0)*.005
    origin = np.array([.03, -.02, .6])
    vertices, faces = marching_tetrahedra(grid[:,:,:,2]-.0041, origin, .005)
    np.testing.assert_allclose(vertices[:,2], .6041)
    normals = np.cross(vertices[faces[:,1]]-vertices[faces[:,0]],
                       vertices[faces[:,2]]-vertices[faces[:,0]])
    assert np.all(normals[:,2] > 0)
    metrics = topology(vertices, faces)
    assert metrics['mesh_edge_components'] == metrics['mesh_vertex_components'] == 1
    assert metrics['nonmanifold_edges'] == metrics['inconsistent_winding_edges'] == 0
    with pytest.raises(ValueError, match='triangle budget'):
        marching_tetrahedra(grid[:,:,:,2]-.0041, origin, .005, max_triangles=1)


@pytest.mark.parametrize('scale,offset', [(.001,0.), (1.,0.), (1000.,0.), (.001,100.)])
def test_separating_axes_keep_true_crossings_and_filter_disjoint_near_coplanar_faces(scale,offset):
    a = np.array([[0.,0.,0.], [.003,0.,0.], [0.,.003,0.]])
    crossing = np.array([[.001,.001,-.001], [.001,.001,.001], [.002,.001,0.]])
    coplanar_gap = np.array([[.0031,0.,0.], [0.,.0031,0.], [.0031,.0031,0.]])
    touching = np.array([[.003,0.,0.], [.004,0.,0.], [.003,.001,0.]])
    # Real sphere regression: triangles touch the same grid-face plane at two
    # different points, so their AABBs touch but the triangles are disjoint.
    near_a = np.array([[.27597241123319227,-.01202693470600509,.7999725504264594],
        [.275971413413424,-.012027932525773374,.8], [.2759367542637866,-.011999485132464585,.8]])
    near_b = np.array([[.27594191615380625,-.011999485132464585,.7999420553470734],
        [.27377103186200036,-.010228314077197043,.8], [.2738569160475809,-.01014242989161647,.797857055240848]])
    left = np.stack((a,a,a,near_a))*scale+offset
    right = np.stack((crossing,coplanar_gap,touching,near_b))*scale+offset
    np.testing.assert_array_equal(meshing._separated_triangles(left,right), [False,True,False,True])


@pytest.mark.skipif(importlib.util.find_spec('open3d') is None, reason='Open3D meshing extra unavailable')
def test_native_cleanup_cancels_reverse_duplicates_and_bounds_rounding_displacement():
    import open3d as o3d
    vertices = np.array([[0.,0.,0.], [.01,0.,0.], [0.,.01,0.], [0.,0.,.01],
                         [.005,-.002,0.], [.01,0.,.00000005]])
    faces = np.array([[0,2,1], [0,1,3], [0,3,2], [1,2,3],
                      [0,1,4], [0,4,5]])
    surface = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices),
                                      o3d.utility.Vector3iVector(faces))
    diagnostics = meshing._clean_surface(surface, .000001)
    assert diagnostics['cancelled_opposite_triangles'] == 2
    assert diagnostics['weld_max_displacement_m'] <= .000001
    assert len(surface.triangles) == 4 and len(surface.vertices) == 4
    metrics = topology(np.asarray(surface.vertices), np.asarray(surface.triangles))
    assert metrics['boundary_edges'] == metrics['nonmanifold_edges'] == 0


@pytest.mark.skipif(importlib.util.find_spec('open3d') is None, reason='Open3D meshing extra unavailable')
def test_component_filter_uses_metric_area_and_preserves_multiple_substantial_parts():
    import open3d as o3d
    def tetra(size, offset):
        item = o3d.geometry.TriangleMesh.create_tetrahedron(radius=size)
        return item.translate(offset)
    surface = tetra(.01, [0.,0.,0.])+tetra(.01, [.05,0.,0.])+tetra(.0001, [.1,0.,0.])
    diagnostics = meshing._clean_surface(surface, .000001, .000025)
    assert diagnostics['removed_small_components'] == 1
    assert 0 < diagnostics['removed_component_area_m2'] < .000025
    assert topology(np.asarray(surface.vertices), np.asarray(surface.triangles))['mesh_edge_components'] == 2


@pytest.mark.skipif(importlib.util.find_spec('open3d') is None, reason='Open3D meshing extra unavailable')
def test_distance_field_budget_fails_before_allocating_the_grid():
    import open3d as o3d
    surface = o3d.geometry.TriangleMesh.create_sphere(radius=.05)
    options = PoissonSurface(options={'max_grid_points':10}, assets={}).options
    with pytest.raises(ValueError, match='grid budget'):
        meshing._regularize_surface(surface, options)


@pytest.mark.skipif(importlib.util.find_spec('open3d') is None, reason='Open3D meshing extra unavailable')
@pytest.mark.parametrize('geometry', ['sphere', 'torus'])
def test_native_poisson_fits_points_preserves_a_real_opening_and_exports(geometry, tmp_path):
    if geometry == 'sphere':
        z = np.linspace(-.999, .999, 1000)
        angle = np.arange(1000)*np.pi*(3-np.sqrt(5))
        observed = .04*np.column_stack((np.sqrt(1-z*z)*np.cos(angle),
                                       np.sqrt(1-z*z)*np.sin(angle), z))
    else:
        u, v = np.meshgrid(np.linspace(0, 2*np.pi, 60, endpoint=False),
                           np.linspace(0, 2*np.pi, 20, endpoint=False))
        observed = np.column_stack(((.03+.01*np.cos(v.ravel()))*np.cos(u.ravel()),
                                    (.03+.01*np.cos(v.ravel()))*np.sin(u.ravel()), .01*np.sin(v.ravel())))
    center = np.array([.3, .02, .8])
    cloud = shape(observed+center)
    model = PoissonSurface(options={'depth': 7, 'density_trim_quantile': 0.}, assets={})
    task = Task(geometry, 'target')
    mesh = model.mesh(cloud, task)
    diagnostics = model.diagnostics()
    assert diagnostics['nonmanifold_edges'] == diagnostics['inconsistent_winding_edges'] == 0
    assert diagnostics['boundary_edges'] == 0
    assert diagnostics['mesh_edge_components'] == 1
    if geometry == 'sphere':
        assert np.quantile(np.abs(np.linalg.norm(mesh.vertices-center, axis=1)-.04), .95) < .003
    else:
        assert np.min(np.linalg.norm((mesh.vertices-center)[:, :2], axis=1)) > .017
        assert len(mesh.vertices)-len(mesh.triangles)//2 == 0  # torus Euler characteristic
    manifest = export_result(Result(task, (SceneObject(cloud, mesh),), None, diagnostics), tmp_path/'scene')
    assert SceneArtifact.load(manifest).objects[0].object_id == 'target'
    with np.load(manifest.parent/'target.npz', allow_pickle=False) as packet:
        np.testing.assert_array_equal(packet['observed'], cloud.cloud.observed)
        np.testing.assert_array_equal(packet['predicted'], cloud.predicted)
