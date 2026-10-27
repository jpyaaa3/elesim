import numpy as np
import pytest

from workbench.research.experiments.adapointr import restore_completion,sample_input
from workbench.tests.research.support.completion_controls import prepare_control,comparison_metrics


@pytest.mark.parametrize('variant',['native','center_only','scale_only_r05','center_scale_r05','center_scale_r10'])
def test_control_roundtrip_keeps_the_same_sampled_physical_points(variant):
    points=np.random.default_rng(42).normal(size=(137,3)).astype(np.float32)*.07+[.3,-.1,.2]
    original=points.copy()
    prepared,frame=prepare_control(points,variant,seed=9)
    restored=restore_completion(prepared,frame)
    np.testing.assert_allclose(restored,sample_input(points,seed=9),atol=1e-7)
    np.testing.assert_array_equal(points,original)


def test_native_public_demo_coordinates_are_not_recentered_or_axis_swapped():
    points=np.random.default_rng(7).normal(size=(333,3)).astype(np.float32)*[.2,.04,.1]+[.1,-.03,.4]
    prepared,frame=prepare_control(points,'native',seed=2)
    np.testing.assert_array_equal(prepared,sample_input(points,seed=2))
    np.testing.assert_array_equal(frame.center,[0,0,0])
    assert frame.meters_per_unit==1.


def test_fidelity_distance_uses_original_units_and_not_a_ground_truth_alignment():
    y,z=np.meshgrid(np.linspace(-.5,.5,20),np.linspace(-.25,.25,10))
    observed=np.column_stack((np.zeros(y.size),y.ravel(),z.ravel()))
    exact=comparison_metrics(observed,observed,observed)
    assert exact['observed_to_prediction_p95']==0 and exact['reference_chamfer_mean']==0
    displaced=observed+[.02,0,0]
    result=comparison_metrics(observed,displaced,observed)
    assert np.isclose(result['observed_to_prediction_p95'],.02)
    assert np.isclose(result['reference_chamfer_mean'],.02)
    assert np.isclose(result['observed_to_prediction_p95_pct_diag'],100*.02/np.sqrt(1.25))
