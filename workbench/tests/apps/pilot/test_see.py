"""SEE boundaries: measured frames, camera frustum, occlusion and worker errors."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial import cKDTree

from elesim_pilot.vision.see import SeeBackend, LIBRARY
from elesim_pilot.vision.see_planner import SeeViewPlanner


def camera():
    return SimpleNamespace(fx=100., fy=100., cx=60., cy=30., width=120, height=60)


def test_observation_snapshot_replaces_old_frames_and_uses_measured_pose():
    planner = object.__new__(SeeViewPlanner)
    points = np.array([[0., 0., .3], [.01, 0., .3], [0., .01, .3]])
    pose = np.eye(4); pose[0, 3] = .2
    planner.set_observations([(points, pose)], camera())
    assert np.allclose(planner.frames[0][0], points+[.2, 0., 0.])
    pose[0, 3] = .1
    planner.set_observations([(points, pose)], camera())
    assert len(planner.frames) == 1
    assert np.allclose(planner.frames[0][0], points+[.1, 0., 0.])


def test_achieved_camera_frustum_is_rectangular_and_not_a_viewing_cone():
    planner = object.__new__(SeeViewPlanner)
    planner.intrinsics = np.array([100., 100., 60., 30., 120., 60.])
    targets = np.array([[.15, 0, .3], [0, .15, .3], [0, 0, -.3]])
    visible = planner._visible(targets, np.eye(4), cKDTree([[10, 10, 10]]))
    assert visible.tolist() == [True, False, False]


def test_achieved_camera_view_rejects_measured_occlusion():
    planner = object.__new__(SeeViewPlanner)
    planner.intrinsics = np.array([100., 100., 60., 30., 120., 60.])
    target = np.array([[0, 0, .3]])
    assert not planner._visible(target, np.eye(4), cKDTree([[0, 0, .15]])).any()
    assert planner._visible(target, np.eye(4), cKDTree(target)).all()


def test_missing_native_library_is_explicit(tmp_path):
    with pytest.raises(RuntimeError, match='absent'):
        SeeBackend(library=tmp_path/'missing.so')


def test_native_failure_is_contained_in_child_process(tmp_path):
    invalid = tmp_path/'invalid.so'; invalid.write_text('not an ELF library')
    backend = SeeBackend(library=invalid)
    with pytest.raises(RuntimeError, match='OSError'):
        backend.propose([(np.array([[0, 0, .3], [.01, 0, .3], [0, .01, .3]]), np.eye(4))],
                        [100, 100, 60, 30, 120, 60])


def test_invalid_and_cancelled_inputs_do_not_launch_worker(tmp_path):
    library = tmp_path/'placeholder.so'; library.touch()
    backend = SeeBackend(library=library)
    intr = [100, 100, 60, 30, 120, 60]
    with pytest.raises(ValueError, match='accepted observations'):
        backend.propose([], intr)
    with pytest.raises(ValueError, match='sensor origin'):
        backend.propose([(np.zeros((3, 3)), np.eye(4))], intr)
    with pytest.raises(InterruptedError):
        backend.propose([(np.ones((3, 3)), np.eye(4))], intr, cancelled=lambda: True)


def test_exhaustion_is_not_a_completeness_claim():
    planner = object.__new__(SeeViewPlanner)
    planner.frames = [(np.ones((3, 3)), np.eye(4))]
    planner.intrinsics = np.array([100, 100, 60, 30, 120, 60])
    planner.return_q = None
    planner.first_q = None
    planner.fallback = False
    planner.backend = SimpleNamespace(propose=lambda *args, **kwargs: np.empty((0, 9)))
    assert planner.choose(np.ones((3, 3)), np.zeros(4), [np.eye(4)], []) is None
    assert planner.last_search['reason'] == 'frontier_exhausted_not_completeness'


def test_exhaustion_uses_information_with_the_same_return_guard():
    planner = object.__new__(SeeViewPlanner)
    points = np.ones((3, 3))
    planner.frames = [(points, np.eye(4))]
    planner.intrinsics = np.array([100, 100, 60, 30, 120, 60])
    planner.first_q = None
    planner.return_q = np.array([-.12, 0., -.4, .4])
    planner.fallback = True
    planner.backend = SimpleNamespace(propose=lambda *args, **kwargs: np.empty((0, 9)))
    start = np.array([-.05, 1., -.35, .55])
    good = np.array([-.03, 1., -.28, .50])
    blocked = good.copy()
    blocked[1] = 0.

    def certify(q, anchor):
        return {'accepted': np.array_equal(q, good) and np.array_equal(anchor, start)}

    def fallback(observed, current, poses, attempted, *, cancelled, can_return):
        assert observed is points
        assert attempted[0] is start
        assert not can_return(blocked)
        assert can_return(good)
        return {'q': good}

    planner.collision_checker = SimpleNamespace(certify_segment=certify)
    planner.information = SimpleNamespace(choose=fallback, last_search={'geometric_candidates': 2})
    choice = planner.choose(points, start, [np.eye(4)], [start])
    np.testing.assert_array_equal(choice['q'], good)
    assert choice['policy'] == 'information'
    assert planner._return(good)[2] == 'via_first'
    assert planner.last_search['fallback']['geometric_candidates'] == 2


@pytest.mark.skipif(not LIBRARY.is_file(), reason='official SEE native library not built in this environment')
def test_official_core_reports_density_frontier_exhaustion_on_dense_plane():
    x, y = np.meshgrid(np.linspace(-.07, .07, 65), np.linspace(-.07, .07, 65))
    points = np.column_stack([x.ravel(), y.ravel(), np.full(x.size, .35)])
    proposals = SeeBackend().propose([(points, np.eye(4))], [500, 500, 320, 240, 640, 480])
    # A uniformly dense cropped plane has no sparse outlier neighbours. This
    # result must remain exhaustion, never a claim that hidden surfaces are seen.
    assert proposals.shape == (0, 9)


def test_view_must_keep_observed_object_extent_in_actual_frame():
    planner = object.__new__(SeeViewPlanner)
    planner.intrinsics = np.array([100., 100., 60., 30., 120., 60.])
    planner.roi_height_m = None
    assert planner._contains_extent(np.array([[-.03, -.03, .3], [.03, .03, .3]]), np.eye(4))
    assert not planner._contains_extent(np.array([[-.03, -.15, .3], [.03, .15, .3]]), np.eye(4))


def test_visible_ray_checks_can_be_cancelled():
    planner = object.__new__(SeeViewPlanner)
    planner.intrinsics = np.array([100., 100., 60., 30., 120., 60.])
    with pytest.raises(InterruptedError):
        planner._visible(np.array([[0, 0, .3]]), np.eye(4), cKDTree([[10, 10, 10]]),
                         cancelled=lambda: True)


@pytest.mark.skipif(not LIBRARY.is_file(), reason='official SEE native library not built in this environment')
def test_native_deadline_reaps_the_worker():
    import multiprocessing
    existing = {child.pid for child in multiprocessing.active_children()}
    backend = SeeBackend(timeout_s=.001)
    with pytest.raises(TimeoutError, match='deadline'):
        backend.propose([(np.array([[0, 0, .3], [.01, 0, .3], [0, .01, .3]]), np.eye(4))],
                        [100, 100, 60, 30, 120, 60])
    assert {child.pid for child in multiprocessing.active_children()} <= existing


def test_see_density_parameters_reject_unusable_neighbourhood():
    from elesim_pilot.vision.see import SeeOptions
    with pytest.raises(ValueError):
        SeeOptions(density_per_m3=1)
    with pytest.raises(ValueError):
        SeeOptions(view_distance_m=float('nan'))
