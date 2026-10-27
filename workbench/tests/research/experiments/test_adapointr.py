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


def test_input_preparation_splits_objects_without_reading_truth(tmp_path):
    import json
    from workbench.research.experiments.adapointr_inputs import prepare
    sensors=tmp_path/'sensors';predictions=tmp_path/'predictions'
    sensors.mkdir();predictions.mkdir()
    color=np.zeros((64,64,3),np.uint8)
    color[3:19,3:19,2]=255;color[35:51,35:51,2]=255
    v,u=np.nonzero(color[:,:,2]);pixels=np.column_stack((u,v))
    points=np.column_stack((u*.001,v*.001,np.ones(len(u))*.4))
    name='view-0000-2.npz'
    np.savez(sensors/name,color_bgr=color)
    np.savez(predictions/name,points_after=points,pixels_uv=pixels)
    (predictions/'result.json').write_text(json.dumps({'frames':[
        {'name':name,'floor_valid':True},{'name':'view-0001-2.npz','floor_valid':False}]}))
    result=prepare(sensors,predictions,tmp_path/'inputs',max_captures=2)
    assert len(result['captures'])==2 and len(result['inputs'])==2
    assert not result['truth_access'] and not result['object_catalog_access']
    for item in result['inputs']:
        cloud=np.load(tmp_path/'inputs'/item['file'],allow_pickle=False)
        assert len(cloud)==256 and np.ptp(cloud[:,0])<.02
    assert not list(tmp_path.glob('**/truth*'))


def test_independent_box_metrics_distinguish_partial_from_full_surface():
    from workbench.research.experiments.adapointr_evaluate import box_distance,sample_box
    center=np.array([.3,-.5,.4]);size=np.array([.2,.3,.4]);rotation=np.eye(3)
    surface=sample_box(center,size,rotation,count=12000)
    np.testing.assert_allclose(box_distance(surface,center,size,rotation),0,atol=1e-14)
    np.testing.assert_allclose(box_distance(np.array([center,center+[.3,0,0]]),center,size,rotation),[.1,.2])
