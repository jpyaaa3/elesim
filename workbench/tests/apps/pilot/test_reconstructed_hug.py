"""Reconstruction -> solver -> three-anchor command interpolation contracts."""
import json
from pathlib import Path
import shutil
import numpy as np
import pytest
from elesim_pilot.reconstruction.hug import HugModel, fit_cylinder

ROOT=Path(__file__).resolve().parents[4]
MODEL=ROOT/'payload/data/policies/hug-d100'


def cloud(center=(.42,.11,.55),diameter=.1):
    a=np.linspace(0,2*np.pi,96,endpoint=False)
    return np.array([[center[0]+diameter/2*np.cos(t),center[1]+diameter/2*np.sin(t),z]
                     for z in np.linspace(0,1.1,20) for t in a])


def test_fits_geometry_not_mock_pose():
    fit=fit_cylinder(cloud())
    assert np.allclose(fit['center_world_m'],[.42,.11,.55])
    assert fit['diameter_mm']==pytest.approx(100)
    assert fit['residual_m']<1e-10


@pytest.mark.parametrize('diameter',[.08,.12])
def test_wrong_diameter_is_not_rounded_to_supported_model(diameter):
    with pytest.raises(ValueError):fit_cylinder(cloud(diameter=diameter))


def test_rejects_box_and_nonfinite():
    p=cloud();p[:,0]=np.where(p[:,0]>.42,.47,.37)
    with pytest.raises(ValueError):fit_cylinder(p)
    p[0]=np.nan
    with pytest.raises(ValueError):fit_cylinder(p)


@pytest.mark.skipif(not shutil.which('node'),reason='Node solver unavailable')
def test_exact_anchor_inverse_and_full_hold_budget():
    m=HugModel(MODEL)
    for i,c in enumerate(m.meta['anchors']):
        path,result=m.resolve(c['xy_mm'])
        assert path.shape==(2001,4)
        assert np.allclose(result['weights'],np.eye(3)[i],atol=1e-9)
        assert np.allclose(path[:m.paths.shape[1]],m.paths[i],atol=1e-9)
        assert np.allclose(path[m.paths.shape[1]:],m.paths[i,-1])
        assert result['physical_success_verified'] is False
    with pytest.raises(ValueError):m.resolve([300,300])


def test_changed_model_is_rejected(tmp_path):
    shutil.copytree(MODEL,tmp_path/'m')
    with (tmp_path/'m/kernel.js').open('a') as f:f.write('// changed')
    with pytest.raises(ValueError,match='checksum'):HugModel(tmp_path/'m')
