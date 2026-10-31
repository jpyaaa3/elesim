"""Evaluate an exported scan against Sim-private truth after acquisition.

Run in the Sim image via stdin, with its read-only exchange mount. Ground truth
is used only here, never for segmentation, pose estimation or motion planning.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exchange',type=Path,default=Path('/exchange'))
    parser.add_argument('--evidence',type=Path,default=Path('/cycle-output'))
    args = parser.parse_args()
    import numpy as np
    from elesim_protocol.scene_artifact import SceneArtifact

    truth = json.loads((args.evidence/'sim.json').read_text())
    center = np.array(truth['truth']['center_m'])
    radius = truth['truth']['radius_m']
    candidates = list(args.exchange.glob('*/scene/scene.json'))
    assert len(candidates) == 1, candidates
    manifest = candidates[0]
    artifact = SceneArtifact.load(manifest)
    scan = json.loads((manifest.parent.parent/'scan.json').read_text())
    result = dict(complete=False, scope='offline independent Sim truth evaluation after scan',
        scene_revision=artifact.revision, manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),
        scan_phase=scan['phase'], accepted_views=scan.get('accepted_views'),
        geometry_complete_claimed=scan['geometry_complete_claimed'], objects=[])
    for item in artifact.objects:
        with np.load(manifest.parent/(item.object_id+'.npz'),allow_pickle=False) as cloud:
            observed, predicted = cloud['observed'], cloud['predicted']
        vertices = np.array([list(map(float,line.split()[1:])) for line in item.mesh_file.read_text().splitlines()
            if line.startswith('v ')])
        radial = np.abs(np.linalg.norm(observed-center,axis=1)-radius)
        vertex_radial = np.abs(np.linalg.norm(vertices-center,axis=1)-radius)
        result['objects'].append(dict(object_id=item.object_id,observed_points=len(observed),
            predicted_points=len(predicted),vertices=len(vertices),
            faces=sum(line.startswith('f ') for line in item.mesh_file.read_text().splitlines()),
            observed_radial_error_p50_m=float(np.median(radial)),
            observed_radial_error_p95_m=float(np.quantile(radial,.95)),
            observed_radial_error_max_m=float(radial.max()),
            mesh_vertex_radial_error_p95_m=float(np.quantile(vertex_radial,.95)),
            observed_aabb_m=[observed.min(axis=0).tolist(),observed.max(axis=0).tolist()],
            mesh_aabb_m=[vertices.min(axis=0).tolist(),vertices.max(axis=0).tolist()]))
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
    result['complete'] = (scan['phase']=='completed' and scan['accepted_views'] >= 2
        and not scan['geometry_complete_claimed'] and bool(truth['edits'])
        and all(edit['ok'] and edit['entity_state_preserved'] and edit['camera_processes_preserved']
            and edit.get('original_target_hidden_before_load',False)
            for edit in truth['edits'])
        and bool(result['objects'])
        and all(row['observed_points'] > 0 and row['predicted_points'] == 0
            and row['observed_radial_error_p95_m'] <= .015
            and row['mesh_vertex_radial_error_p95_m'] <= .045 for row in result['objects']))
    (args.evidence/'evaluation.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result),flush=True)
    return 0 if result['complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
