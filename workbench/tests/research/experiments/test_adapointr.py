import numpy as np
import pytest

from workbench.research.experiments.adapointr import normalize_partial, restore_completion, sample_input


def test_sensor_only_normalization_is_translation_and_scale_equivariant():
    points=np.random.default_rng(7).normal(size=(300,3))*.03+[.8,-.2,.4]
    normalized,frame=normalize_partial(points)
    moved,other=normalize_partial(points*2.5+[2,-1,.6])
    np.testing.assert_allclose(moved,normalized,atol=2e-6)
    predicted=np.array([[.1,.2,-.3],[-.4,.1,.2]],dtype=np.float32)
    original=restore_completion(predicted,frame)
    translated=restore_completion(predicted,other)
    np.testing.assert_allclose(translated,original*2.5+[2,-1,.6],atol=2e-6)
    np.testing.assert_allclose(restore_completion(normalized,frame),points,atol=2e-7)
    assert np.isclose(np.linalg.norm(normalized,axis=1).max(),.5)


def test_padding_uses_observed_points_and_never_invents_zero_geometry():
    points=np.random.default_rng(4).normal(size=(131,3)).astype(np.float32)
    a=sample_input(points,seed=8)
    b=sample_input(points,seed=8)
    assert a.shape==(2048,3)
    np.testing.assert_array_equal(a,b)
    assert {tuple(p) for p in a}=={tuple(p) for p in points}
    np.testing.assert_array_equal(points,np.random.default_rng(4).normal(size=(131,3)).astype(np.float32))


@pytest.mark.parametrize('points',[np.zeros((100,3)),np.zeros((20,3)),np.ones((200,4)),np.full((200,3),np.nan)])
def test_unsupported_observations_are_refused(points):
    with pytest.raises(ValueError):normalize_partial(points)
