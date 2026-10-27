import copy

import numpy as np
import pytest

from workbench.research.experiments.adapointr import restore_completion
from workbench.research.support.adapointr_profiles import prepare_cloud, validate_profile, restore_prediction


def config(profile):
    base, count, centers = {
        'pcn': ('PCN.yaml', 16384, [512, 256]),
        'shapenet55': ('ShapeNet-55.yaml', 8192, [512, 256]),
        'projected55': ('Projected_ShapeNet-55_noise.yaml', 8192, [256, 128]),
    }[profile]
    return {'dataset': {'train': {'_base_': 'cfgs/dataset_configs/' + base}},
            'model': {'NAME': 'AdaPoinTr', 'encoder_type': 'graph',
                      'num_points': count, 'center_num': centers}}


@pytest.mark.parametrize('profile,count', [('pcn', 16384), ('shapenet55', 8192), ('projected55', 8192)])
def test_profile_declares_the_dataset_frame_and_output_budget(profile, count):
    info = validate_profile(config(profile), profile)
    assert info['output_points'] == count
    assert info['official_affine'] == ('observed-unit-radius' if profile == 'shapenet55' else 'none')


def test_same_output_size_does_not_make_projected_and_cropped_shapenet_interchangeable():
    with pytest.raises(ValueError, match='profile'):
        validate_profile(config('projected55'), 'shapenet55')
    wrong = copy.deepcopy(config('projected55'))
    wrong['model']['center_num'] = [512, 256]
    with pytest.raises(ValueError, match='profile'):
        validate_profile(wrong, 'projected55')


def test_unit_radius_matches_the_official_shapenet_affine_and_inverts_to_meters():
    points = np.random.default_rng(3).normal(size=(300, 3)) * .09 + [.5, -.3, .2]
    before = points.copy()
    normalized, frame = prepare_cloud(points, 'observed-radius', radius=1.)
    center = points.mean(0)
    extent = np.linalg.norm(points - center, axis=1).max()
    np.testing.assert_allclose(normalized, (points - center) / extent, atol=1e-7)
    np.testing.assert_allclose(restore_completion(normalized, frame), points, atol=1e-7)
    np.testing.assert_array_equal(points, before)


def test_center_only_retains_metric_size_and_native_keeps_the_prepared_frame():
    points = np.random.default_rng(2).normal(size=(137, 3)) * .02 + [.7, -.2, .4]
    centered, frame = prepare_cloud(points, 'center-only')
    assert frame.meters_per_unit == 1.
    np.testing.assert_allclose(np.ptp(centered, axis=0), np.ptp(points, axis=0), atol=1e-7)
    np.testing.assert_allclose(restore_completion(centered, frame), points, atol=1e-7)
    native, native_frame = prepare_cloud(points, 'native')
    np.testing.assert_array_equal(native, points.astype(np.float32))
    np.testing.assert_array_equal(native_frame.center, [0, 0, 0])
    assert native_frame.meters_per_unit == 1.


def test_projected_output_restores_axes_and_accepts_8192_without_inventing_points():
    observed = np.random.default_rng(9).normal(size=(8192, 3)) * .05 + [1., 2., 3.]
    normalized, frame = prepare_cloud(observed, 'observed-radius', radius=1.)
    model_output = normalized[:, [0, 2, 1]]
    restored = restore_prediction(model_output, frame, 'world-z-up', expected_points=8192)
    np.testing.assert_allclose(restored, observed, atol=1e-7)
    with pytest.raises(ValueError, match='output'):
        restore_prediction(model_output, frame, 'world-z-up', expected_points=16384)
    invalid = model_output.copy()
    invalid[0, 0] = np.nan
    with pytest.raises(ValueError, match='output'):
        restore_prediction(invalid, frame, 'world-z-up', expected_points=8192)
