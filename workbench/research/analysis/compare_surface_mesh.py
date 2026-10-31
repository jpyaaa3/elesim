"""Compare voxel boxes with screened Poisson surface reconstruction.

Run on one saved observed/completion pair. No Sim, Robot, camera capture or
completion inference is performed. Reference geometry is used only for offline
evaluation/display, never passed to a mesher.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import platform
import statistics
import subprocess
import time
import zipfile

import numpy as np
from scipy.spatial import cKDTree

from elesim_pilot.reconstruction.artifacts import export_result
from elesim_pilot.reconstruction.contracts import CompletedObject, ObjectCloud, SceneObject, Task
from elesim_pilot.reconstruction.meshing import PoissonSurface, topology
from elesim_pilot.reconstruction import meshing
from elesim_pilot.reconstruction.pipeline import Result
from elesim_pilot.reconstruction.providers import VoxelSurface
from elesim_protocol.scene_artifact import SceneArtifact
from .compare_voxel_mesh import sha, write_views
from .evaluate_scan_cycle import surface_errors


def sample_surface(vertices, triangles, count, seed=0):
    """Area-weighted samples, used only by the post-meshing evaluator."""
    vertices, triangles = np.asarray(vertices), np.asarray(triangles)
    a, b, c = vertices[triangles].transpose(1, 0, 2)
    areas = np.linalg.norm(np.cross(b-a, c-a), axis=1)
    rng = np.random.default_rng(seed)
    ids = rng.choice(len(triangles), size=count, p=areas/areas.sum())
    u = np.sqrt(rng.random(count))[:, None]
    v = rng.random(count)[:, None]
    return (1-u)*a[ids]+u*(1-v)*b[ids]+u*v*c[ids]


def probe_topology(shape, support, output, input_path):
    """Compare normal settings on input topology only, without reading truth."""
    output.mkdir(parents=True)
    candidates = [(r, k, penalty, cosine, depth)
        for r, k, penalty, cosine in ((.012,30,0.,1.), (.020,50,0.,1.),
                                      (.020,50,10.,.5), (.030,60,10.,.5))
        for depth in (7,8)]
    rows = []
    for radius, neighbors, penalty, cosine, depth in candidates:
        options = dict(support, normal_radius_m=radius, normal_neighbors=neighbors,
                       orientation_penalty=penalty, orientation_cosine=cosine, depth=depth)
        model = PoissonSurface(options=options, assets={})
        started = time.perf_counter()
        row = dict(options=model.options)
        try:
            mesh = model.mesh(shape, Task('topology-probe', 'target'))
            row.update(accepted=True, diagnostics=model.diagnostics(),
                       vertices=len(mesh.vertices), triangles=len(mesh.triangles))
        except (RuntimeError, ValueError, TimeoutError) as error:
            row.update(accepted=False, error=str(error))
        row['elapsed_ms'] = (time.perf_counter()-started)*1000
        rows.append(row)
        report = dict(mode='topology_probe', source_npz_sha256=sha(input_path),
                      implementation_sha256=sha(Path(meshing.__file__)), rows=rows,
                      ground_truth_used=False, geometry_accuracy_evaluated=False)
        (output/'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(json.dumps(row), flush=True)
    return 0 if any(row['accepted'] for row in rows) else 1


def raw_diagnostics(shape, model, output):
    """Inspect a rejected native candidate; never write a loadable SceneArtifact."""
    points, support = meshing.supported_points(shape, model.options['min_generated_neighbors'],
                                               model.options['generated_neighbor_radius_m'])
    surface, center, diagnostics = meshing._fit_poisson(np.unique(points, axis=0), model.options)
    vertices, faces = np.asarray(surface.vertices)+center, np.asarray(surface.triangles)
    pairs, candidates = meshing._self_intersections(surface)
    corners = vertices[faces]
    areas = np.linalg.norm(np.cross(corners[:, 1]-corners[:, 0], corners[:, 2]-corners[:, 0]), axis=1)/2
    report = dict(mode='raw_diagnostics', options=model.options, diagnostics=diagnostics,
        **support, **topology(vertices, faces), vertices=len(vertices), triangles=len(faces),
        nonmanifold_vertices=len(surface.get_non_manifold_vertices()),
        self_intersection_pairs=len(pairs), intersection_candidates=candidates,
        tiny_triangles=int(np.sum(areas <= 5e-13)),
        bad_pair_details=[dict(triangle_ids=pair.tolist(), areas_m2=areas[pair].tolist(),
                              vertices_m=corners[pair].tolist()) for pair in pairs[:32]],
        ground_truth_used=False, valid_scene_exported=False)
    output.mkdir(parents=True)
    np.savez_compressed(output/'diagnostic-mesh.npz', vertices=vertices, triangles=faces, intersection_pairs=pairs)
    (output/'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--reference-report', type=Path,
                        help='Existing jar demo report; target.truth is evaluator-only')
    parser.add_argument('--reference', type=Path, help='Original local-centred OBJ')
    parser.add_argument('--topology-probe', action='store_true', help='Check normal settings without truth/shape evaluation')
    parser.add_argument('--raw-diagnostics', action='store_true', help='Inspect native defects without exporting a scene')
    parser.add_argument('--poisson-options', type=Path, help='Explicit Poisson option JSON for comparison')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.topology_probe and not args.raw_diagnostics and (not args.reference_report or not args.reference):
        parser.error('comparison requires --reference-report and --reference')
    if args.topology_probe and args.raw_diagnostics:
        parser.error('select only one diagnostic mode')
    if (args.topology_probe or args.raw_diagnostics) and (args.reference_report or args.reference):
        parser.error('diagnostic modes do not accept truth/reference inputs')
    if args.topology_probe and args.poisson_options:
        parser.error('--topology-probe uses its fixed candidate set')
    output = args.output.absolute()
    if output.exists():
        raise FileExistsError('use a new comparison directory')
    with zipfile.ZipFile(args.input) as archive:
        if sum(item.file_size for item in archive.infolist()) > 16*1024*1024:
            raise ValueError('cloud exceeds comparison budget')
    with np.load(args.input, allow_pickle=False) as packet:
        shape = CompletedObject(ObjectCloud('target', 'jar', 'target', 'world', packet['observed']),
                                packet['predicted'])
    support = dict(min_generated_neighbors=8, generated_neighbor_radius_m=.01)
    if args.topology_probe:
        return probe_topology(shape, support, output, args.input)
    if args.poisson_options and args.poisson_options.stat().st_size > 32768:
        raise ValueError('Poisson options exceed their size budget')
    poisson_options = {} if args.poisson_options is None else json.loads(args.poisson_options.read_text())
    if not isinstance(poisson_options, dict):
        raise ValueError('Poisson options must be an object')
    if args.raw_diagnostics:
        return raw_diagnostics(shape, PoissonSurface(options={**poisson_options, **support}, assets={}), output)
    models = {
        'voxel-10mm': VoxelSurface(options={**support, 'voxel_m': .01}, assets={}),
        'voxel-5mm': VoxelSurface(options={**support, 'voxel_m': .005, 'max_occupied_voxels': 8000}, assets={}),
        'poisson': PoissonSurface(options={**poisson_options, **support}, assets={}),
    }
    meshes, timings = {}, {name: [] for name in models}
    # Warm imports and each implementation once; alternate five CPU-only runs.
    for model in models.values():
        model.mesh(shape, Task('warmup', 'target'))
    for _ in range(5):
        for name, model in models.items():
            started = time.perf_counter()
            meshes[name] = model.mesh(shape, Task('comparison', 'target'))
            timings[name].append((time.perf_counter()-started)*1000)
    # Truth is loaded after reconstruction and never enters the model call.
    geometry = json.loads(args.reference_report.read_text())['target']['truth']
    lines = args.reference.read_text().splitlines()
    reference_vertices = np.array([list(map(float, line.split()[1:4]))
        for line in lines if line.startswith('v ')])+geometry['center_m']
    reference_faces = np.array([[int(i)-1 for i in line.split()[1:4]]
        for line in lines if line.startswith('f ')])
    reference_samples = sample_surface(reference_vertices, reference_faces, 30_000)
    output.mkdir(parents=True)
    (output/'reference.obj').write_bytes(args.reference.read_bytes())
    displays = [dict(label='Original jar (120 x 220 mm)', note='Evaluator/display only; not mesher input',
        vertices=reference_vertices, faces=reference_faces, color=(220,70,65), file='reference.obj')]
    rows = []
    for name, model in models.items():
        mesh = meshes[name]
        diagnostics = model.diagnostics()
        manifest = export_result(Result(Task(name, 'target'), (SceneObject(shape, mesh),), None,
            {'source_npz_sha256': sha(args.input), 'mesher_version': model.version,
             'model_execution': {'models': {'mesher': diagnostics}}}), output/name)
        artifact = SceneArtifact.load(manifest)
        with np.load(manifest.parent/'target.npz', allow_pickle=False) as packet:
            np.testing.assert_array_equal(packet['observed'], shape.cloud.observed)
            np.testing.assert_array_equal(packet['predicted'], shape.predicted)
        obj = artifact.objects[0].mesh_file
        copy = output/(name+'.obj'); copy.write_bytes(obj.read_bytes())
        vertices_error = surface_errors(mesh.vertices, geometry)
        samples = sample_surface(mesh.vertices, mesh.triangles, 30_000)
        forward = surface_errors(samples, geometry)
        reverse = cKDTree(samples).query(reference_samples)[0]
        precision, recall = float(np.mean(forward <= .01)), float(np.mean(reverse <= .01))
        row = dict(name=name, mesher_version=model.version, diagnostics=diagnostics,
            vertices=len(mesh.vertices), triangles=len(mesh.triangles), **topology(mesh.vertices, mesh.triangles),
            obj_file=copy.name, obj_bytes=copy.stat().st_size, obj_sha256=sha(copy),
            scene_manifest=str(manifest.relative_to(output)),
            mesh_vertex_surface_error_p95_m=float(np.quantile(vertices_error, .95)),
            sampled_mesh_to_true_surface_p95_m=float(np.quantile(forward, .95)),
            sampled_truth_to_sampled_mesh_p95_m=float(np.quantile(reverse, .95)),
            sampled_precision_at_10mm=precision, sampled_recall_at_10mm=recall,
            sampled_fscore_at_10mm=2*precision*recall/(precision+recall) if precision+recall else 0.,
            meshing_cpu_ms=timings[name], meshing_cpu_median_ms=statistics.median(timings[name]))
        rows.append(row)
        note = (f'{len(mesh.triangles):,} triangles / {row["mesh_edge_components"]} edge components / '
                f'{row["nonmanifold_edges"]} nonmanifold edges')
        label = name
        if name == 'poisson':
            resolution = model.options['remesh_resolution_m']
            label = f'Poisson + distance field ({resolution*1000:g} mm)' if resolution else 'Poisson (native)'
        displays.append(dict(label=label, note=note, vertices=mesh.vertices, faces=mesh.triangles,
                             color=(215,220,225), file=copy.name))
    report = dict(schema_version=1, source_npz=str(args.input), source_npz_sha256=sha(args.input),
        reference_report_sha256=sha(args.reference_report), reference_sha256=sha(args.reference),
        reference_local_to_world_translation_m=geometry['center_m'],
        observed_points=len(shape.cloud.observed), raw_predicted_points=len(shape.predicted),
        support_policy=support, rows=rows, sample_count_per_surface=30_000,
        timing_scope='Host CPU only; warmup plus five alternating runs. Poisson includes owned worker spawn/import/validation. No file I/O, GPU, Sim or scan timing.',
        python=platform.python_version(), platform=platform.platform(),
        implementation_sha256=sha(Path(meshing.__file__)),
        isosurface_sha256=sha(Path(meshing.__file__).with_name('isosurface.py')),
        limitations=['Poisson estimates normals/extrapolates. The distance field is an inferred finite solid proxy; it is not measured free space or proof of the original opening/interior.',
            'Sampled reverse distances approximate coverage and are not exact point-to-triangle distances.',
            'No claim of complete geometry, preserved original inner walls, or grasp/contact accuracy.',
            'No new scan, completion inference, live Sim import or Robot motion.'])
    (output/'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    write_views(output, displays, title='Jar: voxel boxes and Poisson surface reconstruction',
                subtitle=f'Same {len(shape.cloud.observed):,} observed + {rows[-1]["diagnostics"]["meshed_generated_points"]:,} supported predicted points.')
    subprocess.run(['ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error', '-n',
                    '-i', str(output/'comparison.svg'), '-frames:v', '1', str(output/'comparison.png')],
                   check=True, timeout=120)
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
