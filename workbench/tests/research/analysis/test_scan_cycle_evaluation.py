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
