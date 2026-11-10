import json
import numpy as np
import pytest
from workbench.research.experiments.hug_ga.handoff import load_demo


def write(path,**changes):
    values=dict(schema=np.asarray('hug-ga-verified-demo-v1'),observations=np.zeros((5,20)),actions=np.zeros((5,4)),metadata_json=np.asarray(json.dumps(dict(verified=True,repetitions=3))))
    values.update(changes);np.savez_compressed(path,**values)


def test_search_data_and_failed_verification_are_rejected(tmp_path):
    path=tmp_path/'demo.npz'
    write(path,schema=np.asarray('hug-ga-rollouts-v1'))
    with pytest.raises(ValueError):load_demo(path)
    write(path,metadata_json=np.asarray(json.dumps(dict(verified=False,repetitions=3))))
    with pytest.raises(ValueError):load_demo(path)
    write(path,metadata_json=np.asarray(json.dumps(dict(verified=True,repetitions=1))))
    with pytest.raises(ValueError):load_demo(path)


def test_demo_shape_finiteness_and_action_bounds(tmp_path):
    path=tmp_path/'demo.npz';write(path)
    assert load_demo(path)[0].shape==(5,20)
    for bad in (np.zeros((4,4)),np.full((5,4),np.nan),np.full((5,4),2.)):
        write(path,actions=bad)
        with pytest.raises(ValueError):load_demo(path)
