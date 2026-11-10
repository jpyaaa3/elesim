"""Workspace is a soft footprint preference; it cannot certify an approach."""
import numpy as np
import pytest
from workbench.research.experiments.hug_c import workspace as w
from workbench.research.experiments.hug_c.configure import DEFAULTS


def case():
    return dict(start_command=[-.12,0,np.deg2rad(-34),np.deg2rad(34)],
                goal_command=[0,-np.pi/2,np.deg2rad(-89.5/5),np.deg2rad(-162.5/5)],
                object_xyz_m=[.37608581371059857,.1133127080913172,.55],diameter_mm=100.)


def test_square_distance_uses_vertex_regions_and_near_duplicate_hull_vertices():
    square=w.hull_2d([[0,0],[1,0],[1,1],[0,1],[1,1+1e-15],[.5,.5]])
    assert len(square)==4
    points=np.array([[.5,.5,2],[1.2,.5,0],[1.3,1.4,0],[-.1,0,0]])
    np.testing.assert_allclose(w.outside_distance(points,square),[0,.2,.5,.1])


def test_actual_home_goal_and_lift_height_have_no_workspace_cost():
    c=case();owners,markers=w.marker_geometry();polygon=w.envelope(c,DEFAULTS,owners,markers)
    assert len(markers)>=100 and len(polygon)>3
    for command in (c['start_command'],c['goal_command']):
        p=w.command_points(command,owners,markers,DEFAULTS['spawn_height_m'])
        assert w.workspace_excess(p,polygon,DEFAULTS)==0
        p[:,2]+=1.
        assert w.workspace_excess(p,polygon,DEFAULTS)==0
    open_arm=w.command_points([0,0,0,0],owners,markers,DEFAULTS['spawn_height_m'])
    assert w.workspace_excess(open_arm,polygon,DEFAULTS)>.1
    assert w.workspace_fraction(w.workspace_excess(open_arm,polygon,DEFAULTS),DEFAULTS)==1.


def test_space_margin_cost_is_bounded_and_independent_of_point_count():
    s=dict(DEFAULTS,workspace_margin_m=.05,workspace_scale_m=.1)
    polygon=np.array([[0,0],[1,0],[1,1],[0,1]])
    points=np.array([[[1.04,.5,0]],[[1.10,.5,0]],[[10,.5,0]]])
    exposure=w.workspace_fraction(w.workspace_excess(points,polygon,s),s)
    np.testing.assert_allclose(exposure,[0,.25,1.])
    np.testing.assert_allclose(exposure,w.workspace_fraction(w.workspace_excess(np.repeat(points,8,axis=1),polygon,s),s))


def test_marker_rotation_and_torch_geometry_agree_with_numpy():
    torch=pytest.importorskip('torch')
    p=np.array([[[1.,2,3],[4,5,6]]],np.float32)
    q=np.array([[[1.,0,0,0],[np.sqrt(.5),0,0,np.sqrt(.5)]]],np.float32)
    markers=np.array([[.1,0,0],[.1,0,0]],np.float32)
    expected=w.transform_markers(p,q,markers)
    actual=w.transform_markers(torch.tensor(p),torch.tensor(q),torch.tensor(markers),torch).numpy()
    np.testing.assert_allclose(expected,actual,atol=1e-6)
    polygon=w.hull_2d([[0,0],[1,0],[1,1],[0,1],[1,1+1e-15]]).astype(np.float32)
    points=np.array([[[.2,.2,0],[1.15,1.2,0]],[[3,3,0],[.1,.1,0]]],np.float32)
    expected=w.workspace_excess(points,polygon,DEFAULTS)
    actual=w.workspace_excess(torch.tensor(points),torch.tensor(polygon),DEFAULTS,torch).numpy()
    assert np.isfinite(actual).all()
    np.testing.assert_allclose(actual,expected,atol=1e-6)
