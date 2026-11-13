"""Pinned cuRobo V2 MorphIt fitting and measured shell-policy export."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import asdict
import hashlib
import importlib.metadata
import json
from pathlib import Path
import time
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from .v2_geometry import restore_mesh_urdf, save, sha

PIN = '42498cf4e7d0d6fbd4ef9fbcec1243c00c888a83'


def require_v2():
    version = importlib.metadata.version('nvidia-curobo')
    if version != '0.9.0':
        raise ValueError(f'Pinned cuRobo 0.9.0 required, found {version}')
    import torch
    if not torch.cuda.is_available() or torch.cuda.device_count() != 1:
        raise ValueError('Expose exactly one CUDA GPU by UUID')
    return version


def load_link_mesh(link):
    import trimesh
    pieces = []
    for collision in link.findall('collision'):
        node = collision.find('geometry/mesh')
        if node is None:
            raise ValueError('Expected original collision mesh')
        mesh = trimesh.load_mesh(node.get('filename'), process=True)
        mesh.apply_scale(np.fromstring(node.get('scale', '1 1 1'), sep=' '))
        # OBJ stores separate vertices at normal/UV seams. Welding position-
        # identical vertices restores closed topology without changing shape.
        mesh.merge_vertices(merge_norm=True, merge_tex=True)
        origin = collision.find('origin')
        t = np.eye(4)
        if origin is not None:
            t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
            t[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
        mesh.apply_transform(t)
        # Same preprocessing as the official RobotBuilder; report topology.
        mesh.fill_holes()
        trimesh.repair.fix_normals(mesh)
        trimesh.repair.fix_inversion(mesh)
        trimesh.repair.fix_winding(mesh)
        pieces.append(mesh)
    return trimesh.util.concatenate(pieces)


@contextmanager
def surface_inset_sampling():
    """Feed official MorphIt only its surface-inset samples, without volume fill.

    The pinned optimizer already generates inward surface samples. Suppressing
    the independent volume samples makes that set its coverage target. This
    scoped change affects only the isolated fitting worker, not vendor files.
    """
    with patch('trimesh.sample.volume_mesh', return_value=np.empty((0, 3))):
        yield


def fit(args):
    version = require_v2()
    import torch
    import trimesh
    from curobo._src.geom.sphere_fit.fit_morphit import (
        MorphItConfig, MorphItLossWeights, _run_training,
    )
    from curobo._src.geom.sphere_fit.metrics import compute_sphere_fit_metrics
    from curobo._src.geom.sphere_fit.wp_mesh_query import WarpMeshQuery

    torch.manual_seed(42)
    np.random.seed(42)
    args.output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    meshes_hash = restore_mesh_urdf(args.source_model, args.assembly, args.output / 'arm.urdf')
    root = ET.parse(args.output / 'arm.urdf').getroot()
    old = yaml.safe_load((args.source_model / 'robot.yml').read_text())['robot_cfg']['kinematics']
    meta = json.loads((args.source_model / 'metadata.json').read_text())
    reference = json.loads(args.reference_plan.read_text())
    links = {link.get('name'): load_link_mesh(link) for link in root.findall('link')
             if link.findall('collision')}
    # Allocate against actual surface area, independently of the old voxel count.
    names = list(links)
    weights = np.array([np.sqrt(links[n].area) * (1.5 if n.startswith(('node', 'gripper')) else 1.) for n in names])
    minimum = 8
    if args.sphere_budget < minimum * len(names):
        raise ValueError('Sphere budget is too small for the retained links')
    shares = (args.sphere_budget - minimum * len(names)) * weights / weights.sum()
    budgets = np.floor(shares).astype(int) + minimum
    for i in np.argsort(-(shares - np.floor(shares)))[:args.sphere_budget - int(budgets.sum())]:
        budgets[i] += 1
    report = dict(schema='hug-curobo-v2-model', version=1, curobo_version=version,
                  curobo_commit=PIN, source_artifacts=meta['artifacts'], source_meshes=meshes_hash,
                  builder_sha256=sha(__file__), reference_plan_sha256=sha(args.reference_plan),
                  source_model=str(args.source_model.resolve()), excluded_collision_links=['plate'],
                  added_task_collision_links=['camera'],
                  sphere_budget=args.sphere_budget, sampling='surface-inset only, volume samples suppressed',
                  fit='official MorphIt optimizer; inward-tangent post-filter',
                  iterations=args.iterations, seed=42, links={},
                  optimizer_config=dict(inside_samples=6000, radius_threshold_ratio=.0002,
                                        coverage_threshold_ratio=.005, density_control_interval=40,
                                        center_lr_extent_scale=.005, radius_lr_extent_scale=.001),
                  self_collision_policy='Existing mechanical and analytic separation ignores; no random pruning',
                  containment_certified=False, research_only=True)
    spheres = {}
    cache = {}
    try:
        for name, budget in zip(names, budgets):
            mesh = links[name]
            key = hashlib.sha256(mesh.vertices.tobytes() + mesh.faces.tobytes() + str(budget).encode()).hexdigest()
            if key in cache:
                fitted, audit = cache[key]
            else:
                t = time.monotonic()
                query = WarpMeshQuery(mesh, torch.device('cuda', 0))
                # Seed with inward surface points and locally inscribed radii.
                pts, face_ids = trimesh.sample.sample_surface(mesh, max(3000, int(budget) * 30))
                nominal = float(np.clip(np.min(mesh.extents) * .2, .0005, .015))
                candidates = pts - mesh.face_normals[face_ids] * nominal
                tensor = torch.as_tensor(candidates, dtype=torch.float32, device='cuda').contiguous()
                distance = query.query_sdf(tensor)[0].cpu().numpy()
                inside = distance < -.00025
                if np.count_nonzero(inside) < 2:
                    raise ValueError(f'No inward seeds for {name}; topology needs repair')
                seeds = candidates[inside]
                radii = np.minimum(-distance[inside], nominal)
                chosen = np.linspace(0, len(seeds) - 1, min(int(budget), len(seeds)), dtype=int)
                extent = float(np.linalg.norm(mesh.extents))
                optimizer = MorphItConfig(
                    num_spheres=int(budget), max_spheres=int(budget), iterations=args.iterations,
                    num_inside_samples=6000, radius_threshold_ratio=.0002,
                    coverage_threshold_ratio=.005, density_control_interval=40,
                    center_lr=extent * .005, radius_lr=extent * .001,
                    loss_weights=MorphItLossWeights(coverage=1000., protrusion=1000., tangency=10., overlap=.01),
                    device=torch.device('cuda', 0), verbose_frequency=0)
                with surface_inset_sampling():
                    # The official public wrapper hides these density and scale
                    # controls; this pinned backend exposes them without edits
                    # to the vendor implementation.
                    centers, radii, _ = _run_training(
                        mesh, optimizer,
                        torch.as_tensor(seeds[chosen], dtype=torch.float32, device='cuda'),
                        torch.as_tensor(radii[chosen], dtype=torch.float32, device='cuda'))
                centers, radii = np.asarray(centers), np.asarray(radii)
                if not len(centers):
                    raise ValueError(f'MorphIt returned no spheres for {name}')
                d = query.query_sdf(torch.as_tensor(centers, dtype=torch.float32, device='cuda').contiguous())[0].cpu().numpy()
                # Keep only surface-adjacent spheres. Shrink protrusion; never
                # grow a sphere to fill a deep interior gap.
                inside_radius = np.maximum(-d - .00001, 0.)
                keep = (inside_radius > .00025) & (inside_radius - radii <= .001)
                original_count = len(radii)
                centers, radii = centers[keep], np.minimum(radii[keep], inside_radius[keep])
                if not len(centers):
                    raise ValueError(f'Shell policy removed every sphere for {name}')
                metrics = compute_sphere_fit_metrics(mesh, centers, radii, n_interior=1000,
                                                     n_surface=3000, n_sphere_surface=128)
                fitted = [dict(center=c.tolist(), radius=float(r)) for c, r in zip(centers, radii)]
                audit = dict(metrics=asdict(metrics), budget=int(budget), before_shell_filter=original_count,
                             final_spheres=len(fitted), source_watertight=bool(mesh.is_watertight),
                             min_radius_mm=float(radii.min() * 1000), max_radius_mm=float(radii.max() * 1000),
                             fit_elapsed_s=time.monotonic() - t)
                cache[key] = fitted, audit
            spheres[name] = fitted
            report['links'][name] = audit
            print(json.dumps(dict(link=name, **audit)), flush=True)
            save(args.output / 'fit-progress.json', report)
        ignored = {n: [x for x in v if x in spheres]
                   for n, v in old['self_collision_ignore'].items() if n in spheres}
        if 'camera' in spheres:
            # The task treats these links as a single rigid assembly. Keep
            # camera-vs-other-arm and camera-vs-world collision checks enabled.
            rigid = ('node9', 'gripper_base', 'gripper_claw_left', 'gripper_claw_right', 'camera')
            for name in rigid:
                if name in spheres:
                    ignored[name] = sorted((set(ignored.get(name, [])) | (set(rigid) & set(spheres))) - {name})
        kin = dict(
            format_version=2., urdf_path=str((args.output / 'arm.urdf').resolve()),
            asset_root_path=str(args.assembly.resolve()), base_link='plate',
            tool_frames=[link.get('name') for link in root.findall('link')],
            mesh_link_names=list(spheres), collision_link_names=list(spheres),
            collision_spheres=spheres, collision_sphere_buffer=0.,
            self_collision_buffer={n: 0. for n in spheres},
            self_collision_ignore=ignored,
            cspace=dict(joint_names=meta['joint_names'], default_joint_position=reference['home'],
                        cspace_distance_weight=[1.] * 4, null_space_weight=[1.] * 4,
                        max_acceleration=1., max_jerk=10.))
        (args.output / 'robot.yml').write_text(yaml.safe_dump(dict(kinematics=kin), sort_keys=False))
        report.update(status='generated', sphere_count=sum(map(len, spheres.values())),
                      joint_names=meta['joint_names'], q_bounds=meta['q_bounds'],
                      elapsed_s=time.monotonic() - started,
                      artifacts={n: sha(args.output / n) for n in ('arm.urdf', 'robot.yml')})
        save(args.output / 'metadata.json', report)
    except BaseException as exc:
        report.update(status='fit_failed', error=repr(exc), elapsed_s=time.monotonic() - started)
        save(args.output / 'completion.json', report)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source-model', type=Path, default=Path('payload/data/models/planning/zed-mini'))
    parser.add_argument('--reference-plan', type=Path, required=True,
                        help='Task plan supplying the canonical Home preset')
    parser.add_argument('--assembly', type=Path, default=Path('payload/data/models/assemblies/zed-mini'))
    parser.add_argument('--sphere-budget', type=int, default=2048)
    parser.add_argument('--iterations', type=int, default=250)
    args = parser.parse_args()
    fit(args)


if __name__ == '__main__':
    main()
