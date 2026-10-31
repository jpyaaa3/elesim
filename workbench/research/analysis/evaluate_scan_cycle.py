"""Evaluate an exported scan against Sim-private truth after acquisition.

Run in the Sim image via stdin, with its read-only exchange mount. Ground truth
is used only here, never for segmentation, pose estimation or motion planning.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def surface_errors(points,geometry):
    """Absolute distance to a sphere or a closed, world-Z cylinder surface."""
    import numpy as np
    offset = np.asarray(points,dtype=float)-np.asarray(geometry['center_m'],dtype=float)
    radius = float(geometry['radius_m'])
    shape = geometry.get('shape','sphere')
    if shape == 'sphere':
        return np.abs(np.linalg.norm(offset,axis=1)-radius)
    if shape == 'cylinder':
        distances = np.column_stack((np.linalg.norm(offset[:,:2],axis=1)-radius,
            np.abs(offset[:,2])-float(geometry['height_m'])/2))
        return np.abs(np.linalg.norm(np.maximum(distances,0.),axis=1)
            +np.minimum(np.max(distances,axis=1),0.))
    raise ValueError('unsupported evaluation geometry: '+str(shape))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exchange',type=Path,default=Path('/exchange'))
    parser.add_argument('--evidence',type=Path,default=Path('/cycle-output'))
    parser.add_argument('--pipeline-mode',choices=('baseline','integrated'),default='baseline')
    args = parser.parse_args()
    import numpy as np
    from elesim_protocol.scene_artifact import SceneArtifact

    truth = json.loads((args.evidence/'sim.json').read_text())
    candidates = list(args.exchange.glob('*/scene/scene.json'))
    assert len(candidates) == 1, candidates
    manifest = candidates[0]
    artifact = SceneArtifact.load(manifest)
    scan = json.loads((manifest.parent.parent/'scan.json').read_text())
    result = dict(complete=False, scope='offline independent Sim truth evaluation after scan',
        scene_revision=artifact.revision, manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),
        scan_phase=scan['phase'], accepted_views=scan.get('accepted_views'),
        geometry_complete_claimed=scan['geometry_complete_claimed'],
        target_geometry=truth['truth'],objects=[])
    result['pipeline_mode'] = args.pipeline_mode
    result['model_execution'] = scan.get('model_execution', {})
    result['calibration_captures'] = len(scan.get('calibration_observations', []))
    result['floor_corrections_by_view'] = [view.get('pose_corrections', []) for view in scan['views']]
    for item in artifact.objects:
        with np.load(manifest.parent/(item.object_id+'.npz'),allow_pickle=False) as cloud:
            observed, predicted = cloud['observed'], cloud['predicted']
        vertices = np.array([list(map(float,line.split()[1:])) for line in item.mesh_file.read_text().splitlines()
            if line.startswith('v ')])
        observed_error = surface_errors(observed,truth['truth'])
        vertex_error = surface_errors(vertices,truth['truth'])
        metrics = dict(object_id=item.object_id,observed_points=len(observed),
            predicted_points=len(predicted),vertices=len(vertices),
            faces=sum(line.startswith('f ') for line in item.mesh_file.read_text().splitlines()),
            observed_surface_error_p50_m=float(np.median(observed_error)),
            observed_surface_error_p95_m=float(np.quantile(observed_error,.95)),
            observed_surface_error_max_m=float(observed_error.max()),
            mesh_vertex_surface_error_p95_m=float(np.quantile(vertex_error,.95)),
            observed_aabb_m=[observed.min(axis=0).tolist(),observed.max(axis=0).tolist()],
            mesh_aabb_m=[vertices.min(axis=0).tolist(),vertices.max(axis=0).tolist()])
        if len(predicted):
            predicted_error = surface_errors(predicted,truth['truth'])
            metrics.update(predicted_surface_error_p95_m=float(np.quantile(predicted_error,.95)),
                           predicted_surface_error_max_m=float(predicted_error.max()))
        if truth['truth'].get('shape','sphere') == 'sphere':
            for old,new in (('observed_radial_error_p50_m','observed_surface_error_p50_m'),
                    ('observed_radial_error_p95_m','observed_surface_error_p95_m'),
                    ('observed_radial_error_max_m','observed_surface_error_max_m'),
                    ('mesh_vertex_radial_error_p95_m','mesh_vertex_surface_error_p95_m')):
                metrics[old] = metrics[new]
        result['objects'].append(metrics)
    result['view_encoder_q'] = [view.get('encoder_q',view.get('q')) for view in scan['views']]
    import cv2
    packet_path = next((manifest.parent.parent/'frames').glob('*.npz'))
    mask_path = manifest.parent.parent/'silhouettes'/packet_path.name
    with np.load(packet_path,allow_pickle=False) as packet, np.load(mask_path,allow_pickle=False) as silhouette:
        depth = packet['depth_raw']*float(packet['depth_scale'])
        mask = silhouette['mask'].astype(np.uint8)
        result['acquisition_base_world_transform'] = silhouette['base_world_transform'].tolist()
        result['mask_depth_diagnostics'] = []
        for radius in (0,1,2,3,4):
            selected = mask if radius == 0 else cv2.erode(mask,np.ones((2*radius+1,2*radius+1),np.uint8))
            samples = depth[(selected>0)&np.isfinite(depth)&(depth>0)]
            result['mask_depth_diagnostics'].append(dict(erode_px=radius,pixels=len(samples),
                depth_percentiles_m=np.quantile(samples,[.01,.5,.9,.95,.99,1.]).tolist()))
    operational = (scan['phase']=='completed' and scan['accepted_views'] >= 2
        and not scan['geometry_complete_claimed'] and bool(truth['edits'])
        and all(edit['ok'] and edit['entity_state_preserved'] and edit['camera_processes_preserved']
            and edit.get('original_target_hidden_before_load',False)
            for edit in truth['edits'])
        and bool(result['objects']) and all(row['observed_points'] > 0 for row in result['objects']))
    result['geometry_accepted'] = all(row['observed_surface_error_p95_m'] <= .015
        and row['mesh_vertex_surface_error_p95_m'] <= .045
        and row.get('predicted_surface_error_p95_m',0.) <= .045 for row in result['objects'])
    if args.pipeline_mode == 'integrated':
        completion = result['model_execution'].get('models',{}).get('completion',{})
        stages = (completion.get('applied') and completion.get('loaded')
            and completion.get('predicted_points') == 8192
            and completion.get('profile') == 'projected55'
            and completion.get('missing_keys') == [] and completion.get('unexpected_keys') == []
            and all(row['predicted_points'] == 8192 for row in result['objects'])
            and all(view and all(c.get('applied') for c in view)
                    for view in result['floor_corrections_by_view']))
    else:
        stages = all(row['predicted_points'] == 0 for row in result['objects'])
    result['stages_applied'] = bool(stages)
    result['complete'] = bool(operational and stages)
    result['accepted'] = result['complete'] and result['geometry_accepted']
    if args.pipeline_mode == 'baseline':
        result['complete'] = result['accepted']
    (args.evidence/'evaluation.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result),flush=True)
    return 0 if result['complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
