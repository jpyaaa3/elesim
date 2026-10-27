"""Explicit checkpoint/config and input-frame declarations for research trials.

An official affine of ``none`` assumes dataset coordinates are already prepared;
it does not mean arbitrary robot-world meters can be used unchanged. Matching a
config and loading tensors strictly also does not authenticate training lineage.
"""
from pathlib import PurePosixPath

import numpy as np

from workbench.research.experiments.adapointr import CloudFrame, normalize_partial, pcn_axes, restore_completion


PROFILES = {
    'pcn': ('PCN.yaml', 16384, [512, 256], 'none'),
    'shapenet55': ('ShapeNet-55.yaml', 8192, [512, 256], 'observed-unit-radius'),
    'projected55': ('Projected_ShapeNet-55_noise.yaml', 8192, [256, 128], 'none'),
}


def validate_profile(config, profile):
    if profile not in PROFILES:
        raise ValueError('unknown model profile')
    filename, output_points, centers, affine = PROFILES[profile]
    model = config['model']
    dataset = config['dataset']['train']['_base_']
    if (not isinstance(dataset, str) or PurePosixPath(dataset).name != filename
            or model.get('NAME') != 'AdaPoinTr' or model.get('encoder_type') != 'graph'
            or model.get('num_points') != output_points or model.get('center_num') != centers):
        raise ValueError('config does not match declared model profile')
    return {'name': profile, 'dataset_config': dataset, 'output_points': output_points,
            'center_num': centers.copy(), 'official_affine': affine,
            'frame_warning': 'none assumes prepared dataset coordinates, not raw world coordinates'}


def prepare_cloud(points, normalization, *, radius=.5):
    if normalization == 'observed-radius':
        return normalize_partial(points, radius=radius)
    points = np.asarray(points, dtype=np.float64)
    if (points.ndim != 2 or points.shape[1] != 3 or not 32 <= len(points) <= 100_000
            or not np.all(np.isfinite(points))):
        raise ValueError('unsupported partial point cloud')
    if normalization == 'center-only':
        center = points.mean(axis=0)
    elif normalization == 'native':
        center = np.zeros(3)
    else:
        raise ValueError('unsupported normalization')
    return (points - center).astype(np.float32), CloudFrame(center, 1.)


def restore_prediction(generated, frame, axis_order, *, expected_points):
    generated = np.asarray(generated)
    if generated.shape != (expected_points, 3) or not np.all(np.isfinite(generated)):
        raise ValueError('invalid model output')
    if axis_order not in ('world-z-up', 'model-native'):
        raise ValueError('unsupported output axes')
    model_points = pcn_axes(generated) if axis_order == 'world-z-up' else generated
    return restore_completion(model_points, frame)
