"""Placement and exact-circle invariants for the standalone Wrap comparison."""
import math

import numpy as np
import pytest

from workbench.research.analysis.newhug_wrap_comparison import load_task, tangent_centres


def test_task_uses_current_mount_and_physical_rail_instead_of_old_training_range():
    task = load_task()
    np.testing.assert_allclose(task.neutral_hinge_xyz_m, [.343, 0., .562], atol=1e-12)
    assert task.radius_m == pytest.approx(.05)
    np.testing.assert_allclose(task.object_xyz_m, [.368, .16, .55])
    np.testing.assert_allclose(task.linear_limits_m, [-260/1500, 0.])
    np.testing.assert_allclose(task.centre(0), [-.16, .025])
    np.testing.assert_allclose(task.centre(task.linear_limits_m[0]), [-.16, .1983333333333333])
    assert task.coupled_cap_rad == pytest.approx(1.0647)


def test_linear_motion_cannot_supply_lateral_base_motion_or_yaw():
    task = load_task()
    for linear in np.linspace(*task.linear_limits_m, 31):
        centre = task.centre(linear)
        assert centre[0] == pytest.approx(-.16)
        assert task.linear_at(centre) == pytest.approx(linear, abs=1e-14)


def test_tangent_circle_matches_support_distance_and_contact_feet():
    starts = np.array([[0., 0.], [0., 0.]])
    ends = np.array([[0., 2.], [2., 0.]])
    normals = np.array([[1., 0.], [0., 1.]])
    centres, fa, fb, det = tangent_centres(starts, ends, normals, np.array([0]), np.array([1]), .5)
    np.testing.assert_allclose(centres, [[.5, .5]])
    np.testing.assert_allclose([fa[0], fb[0], det[0]], [.25, .25, 1.])
    feet = centres[0]-.5*normals
    np.testing.assert_allclose(np.linalg.norm(centres[0]-feet, axis=1), [.5, .5])
    np.testing.assert_allclose(feet, [[0., .5], [.5, 0.]])


@pytest.mark.parametrize('angle', [-2.7, -.3, 1.7])
def test_circle_tangencies_are_equivariant_under_rigid_transforms(angle):
    rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
    offset = np.array([-.37, 1.23])
    starts = np.zeros((2, 2))@rotation.T+offset
    ends = np.array([[0., 2.], [2., 0.]])@rotation.T+offset
    normals = np.eye(2)@rotation.T
    centres, fa, fb, _ = tangent_centres(starts, ends, normals, np.array([0]), np.array([1]), .5)
    np.testing.assert_allclose(centres[0], np.array([.5, .5])@rotation.T+offset, atol=1e-14)
    np.testing.assert_allclose([fa[0], fb[0]], [.25, .25], atol=1e-14)


def test_parallel_contact_faces_do_not_manufacture_a_finite_circle_centre():
    centres, _, _, det = tangent_centres(np.array([[0., 0.], [0., 1.]]),
                                        np.array([[2., 0.], [2., 1.]]),
                                        np.array([[0., 1.], [0., -1.]]),
                                        np.array([0]), np.array([1]), .5)
    assert det[0] == 0
    assert not np.isfinite(centres).all()
