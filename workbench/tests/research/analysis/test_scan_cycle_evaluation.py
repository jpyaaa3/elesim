import numpy as np
import pytest

from workbench.research.analysis import evaluate_scan_cycle as evaluation


def test_closed_cylinder_distance_includes_caps_and_outside_edges():
    center = np.array([2.,3.,4.])
    points = np.array([[1.,0.,0.],[0.,0.,2.],[0.,0.,2.5],
        [1.3,0.,2.4],[0.,0.,0.],[.9,0.,1.99]])+center
    distances = evaluation.surface_errors(points,dict(shape='cylinder',center_m=center,
        radius_m=1.,height_m=4.))
    assert distances == pytest.approx([0.,0.,.5,.5,1.,.01])


def test_legacy_sphere_truth_keeps_its_surface_distance():
    distances = evaluation.surface_errors(np.array([[1.,0.,0.],[0.,0.,0.],[0.,0.,1.5]]),
        dict(center_m=[0.,0.,0.],radius_m=1.))
    assert distances == pytest.approx([0.,1.,.5])


def test_revolved_profile_measures_caps_inner_wall_and_corner_distance():
    # A hollow cylinder profile in r,z. The segment on the rotation axis has
    # zero area and must not become an imaginary surface inside the base.
    geometry = dict(shape='jar', center_m=[2., 3., 4.], radius_m=1., height_m=2.,
                    profile_rz_m=[[0.,-1.],[1.,-1.],[1.,1.],[.8,1.],[.8,-.8],[0.,-.8]])
    points = np.array([[1.,0.,0.],[.8,0.,0.],[0.,0.,-.9],
                       [0.,0.,0.],[1.3,0.,1.4],[.9,0.,1.]])+[2.,3.,4.]
    assert evaluation.surface_errors(points,geometry) == pytest.approx([0.,0.,.1,.8,.5,0.])
