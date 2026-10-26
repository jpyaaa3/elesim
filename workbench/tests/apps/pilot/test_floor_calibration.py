"""Independent geometry and failure cases for depth-aided camera calibration."""
from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from elesim_pilot.vision.perception.depth_pose import CameraIntrinsics
from elesim_pilot.vision.floor_calibration import (
    FloorEstimator, FloorSettings, Plane, PoseHistory, correct_camera_pose,
    depth_points, detect_planes, plane_in_camera, plane_in_world,
)


def camera_pose(height=0.8, pitch=0.0):
    result = np.eye(4)
    # Optical z looks down, optical y points towards world -y.
    result[:3, :3] = Rotation.from_euler("x", np.pi + pitch).as_matrix()
    result[:3, 3] = [0.3, -0.2, height]
    return result


def cloud(normal=(0, 0, 1), offset=-1.0, *, seed=2, noise=0.001):
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-0.6, 0.6, (1800, 2))
    a, b, c = normal
    z = -(xy[:, 0]*a + xy[:, 1]*b + offset)/c
    return np.column_stack((xy, z)) + rng.normal(0, noise, (len(xy), 3))


def measured_floor(pose, height=0.0):
    return plane_in_camera(Plane((0, 0, 1), -height, area_m2=1.5,
                                 span_m=(1.0, 1.5), inliers=1000, samples=1200), pose)


def test_plane_detection_recovers_tilt_with_outliers_and_multiple_surfaces():
    rng = np.random.default_rng(4)
    normal = np.array([0.12, -0.08, 1.0])
    normal /= np.linalg.norm(normal)
    points = np.vstack((cloud(normal, -0.9), cloud(offset=-1.6, seed=8),
                        rng.uniform(-1.5, 1.5, (400, 3))))
    planes = detect_planes(points, FloorSettings())
    assert len(planes) >= 2
    closest = min(planes, key=lambda p: abs(abs(p.offset_m)-0.9))
    assert abs(np.dot(closest.normal, normal)) > 0.9999
    assert abs(abs(closest.offset_m)-0.9) < 0.003
    assert closest.rms_m < 0.003
    assert closest.area_m2 > 1.0


def test_line_of_points_is_not_a_calibration_plane():
    x = np.linspace(-1, 1, 1000)
    assert detect_planes(np.column_stack((x, np.zeros_like(x), np.ones_like(x))), FloorSettings()) == []


def test_depth_deprojection_preserves_units_pixels_and_exclusion_mask():
    intr = CameraIntrinsics(100, 100, 1, 1, 3, 3)
    depth = np.full((3, 3), 1000, dtype=np.uint16)
    depth[0, 0] = 0
    excluded = np.zeros_like(depth, dtype=bool)
    excluded[2, 2] = True
    points = depth_points(depth, intr, 0.001, max_points=100, excluded=excluded)
    assert len(points) == 7
    np.testing.assert_allclose(points[:, 2], 1)
    assert not np.any(np.all(np.isclose(points, [0.01, 0.01, 1]), axis=1))


@pytest.mark.parametrize("scale", [0, -1, np.nan, np.inf])
def test_bad_depth_scale_is_rejected(scale):
    with pytest.raises(ValueError, match="scale"):
        depth_points(np.ones((3, 3)), CameraIntrinsics(1, 1, 1, 1, 3, 3), scale)


def test_plane_transform_roundtrip_with_rotation_and_translation():
    pose = camera_pose(pitch=0.17)
    world = Plane((0, 0, 1), -0.2)
    observed = plane_in_camera(world, pose)
    recovered = plane_in_world(observed, pose)
    np.testing.assert_allclose(recovered.normal, world.normal, atol=1e-12)
    assert recovered.offset_m == pytest.approx(world.offset_m)


def test_correction_recovers_height_and_normal_without_inventing_planar_translation():
    actual = camera_pose(0.8, pitch=0.12)
    predicted = camera_pose(0.86, pitch=0.19)
    reference = Plane((0, 0, 1), 0)
    corrected = correct_camera_pose(predicted, measured_floor(actual), reference, correct_distance=True)
    plane = plane_in_world(measured_floor(actual), corrected)
    np.testing.assert_allclose(plane.normal, reference.normal, atol=1e-12)
    assert plane.offset_m == pytest.approx(0, abs=1e-12)
    np.testing.assert_allclose(corrected[:2, 3], predicted[:2, 3])
    assert corrected[2, 3] == pytest.approx(actual[2, 3])


def test_unobservable_yaw_is_equivariant_not_estimated_from_floor():
    reference = Plane((0, 0, 1), 0)
    predicted = camera_pose(pitch=0.2)
    measured = measured_floor(camera_pose(pitch=0.1))
    first = correct_camera_pose(predicted, measured, reference)
    yaw = np.eye(4)
    yaw[:3, :3] = Rotation.from_euler("z", 1.1).as_matrix()
    second = correct_camera_pose(yaw @ predicted, measured, reference)
    np.testing.assert_allclose(second, yaw @ first, atol=1e-12)


def test_table_does_not_become_floor_without_confirmation():
    estimator = FloorEstimator()
    pose = camera_pose()
    # Even a large, plausible horizontal plane is only a candidate.
    result = estimator.update([measured_floor(pose, 0.35)], pose, stamp=10,
                              identity=("sim", "boot", "calib"), now=10)
    assert result["state"] == "candidate"
    assert not result["valid"]
    assert result["reference"] is None


def test_floor_and_table_remain_distinct_and_only_selected_plane_is_anchored():
    estimator = FloorEstimator(FloorSettings(stable_frames=2), correct_distance=True)
    pose = camera_pose()
    identity = ("sim", "boot", "calib")
    for stamp in (10, 10.1):
        result = estimator.update([measured_floor(pose, 0.35), measured_floor(pose)],
                                  pose, stamp=stamp, identity=identity, now=stamp)
    floor = min(result["candidates"], key=lambda c: abs(c["world_offset_m"]))
    estimator.confirm(floor["id"], now=10.1)
    actual = camera_pose(0.8, pitch=0.06)
    predicted = camera_pose(0.85, pitch=0.1)
    result = estimator.update([measured_floor(actual)], predicted, stamp=10.2,
                              identity=identity, now=10.2)
    assert result["valid"]
    assert result["distance_error_m"] == pytest.approx(-0.05)
    assert result["normal_error_deg"] == pytest.approx(np.degrees(0.04))
    assert result["reference"]["kind"] == "initial_observation"
    assert result["reference"]["absolute"] is False
    # The reference has not followed the erroneous FK pose.
    assert result["reference"]["offset_m"] == pytest.approx(0)


@pytest.mark.parametrize("new_identity", [("other", "boot", "calib"),
                                          ("sim", "new", "calib"),
                                          ("sim", "boot", "new")])
def test_source_restart_or_calibration_change_invalidates_selected_reference(new_identity):
    estimator = FloorEstimator(FloorSettings(stable_frames=1))
    pose = camera_pose()
    result = estimator.update([measured_floor(pose)], pose, stamp=10,
                              identity=("sim", "boot", "calib"), now=10)
    estimator.confirm(result["candidates"][0]["id"], now=10)
    result = estimator.update([measured_floor(pose)], pose, stamp=10.1,
                              identity=new_identity, now=10.1)
    assert result["reference"] is None
    assert not result["valid"]


def test_lost_stale_duplicate_and_future_frames_do_not_supply_corrections():
    estimator = FloorEstimator(FloorSettings(stable_frames=1), reference_height_m=0.0)
    pose = camera_pose()
    identity = ("sim", "boot", "calib")
    assert estimator.update([measured_floor(pose)], pose, stamp=10,
                            identity=identity, now=10)["valid"]
    assert not estimator.update([], pose, stamp=10.1, identity=identity, now=10.1)["valid"]
    assert estimator.snapshot(now=11)["state"] == "lost"
    assert not estimator.update([measured_floor(pose)], pose, stamp=10,
                                identity=identity, now=11)["valid"]
    assert not estimator.update([measured_floor(pose)], pose, stamp=13,
                                identity=identity, now=11)["valid"]


def test_pose_history_interpolates_at_capture_time_and_rejects_gaps():
    history = PoseHistory(max_gap_s=0.2)
    first, second = camera_pose(), camera_pose(1.0, pitch=0.2)
    history.add(10, first, identity="boot")
    history.add(10.1, second, identity="boot")
    pose = history.at(10.05, identity="boot")
    assert pose is not None
    assert pose[2, 3] == pytest.approx(0.9)
    np.testing.assert_allclose(pose[:3, :3], camera_pose(pitch=0.1)[:3, :3], atol=1e-12)
    assert history.at(10.05, identity="other") is None
    assert history.at(9, identity="boot") is None
    history.add(11, first, identity="boot")
    assert history.at(10.5, identity="boot") is None


def test_rank_deficient_stationary_data_cannot_calibrate_all_hand_eye_parameters():
    from elesim_pilot.vision.hand_eye_calibration import fit_hand_eye
    pose = camera_pose()
    with pytest.raises(ValueError, match="observable|motion|rank"):
        fit_hand_eye([pose]*10, [measured_floor(pose)]*10, np.eye(4))


def test_hand_eye_fit_recovers_mount_from_independent_plane_measurements():
    from elesim_pilot.vision.hand_eye_calibration import fit_hand_eye
    rng = np.random.default_rng(44)
    true_mount = np.eye(4)
    true_mount[:3, :3] = Rotation.from_rotvec([0.08, -0.04, 0.06]).as_matrix()
    true_mount[:3, 3] = [0.05, -0.02, 0.03]
    poses, planes = [], []
    floor = Plane((0, 0, 1), 0)
    for _ in range(30):
        pose = camera_pose()
        pose[:3, :3] = Rotation.from_euler("xyz", rng.uniform(-0.7, 0.7, 3)).as_matrix()
        pose[:3, 3] += rng.uniform(-0.2, 0.2, 3)
        poses.append(pose)
        planes.append(plane_in_camera(floor, pose @ true_mount))
    fit = fit_hand_eye(poses[:22], planes[:22], np.eye(4), reference=floor)
    np.testing.assert_allclose(fit.transform, true_mount, atol=1e-6)
    # Held-out poses were not used by the optimizer.
    errors = [abs(plane_in_world(p, t @ fit.transform).offset_m)
              for p, t in zip(planes[22:], poses[22:])]
    assert max(errors) < 1e-6


def test_unknown_reference_plane_is_fitted_without_forcing_it_horizontal():
    from elesim_pilot.vision.hand_eye_calibration import fit_hand_eye
    rng = np.random.default_rng(13)
    mount = np.eye(4)
    mount[:3, :3] = Rotation.from_rotvec([0.04, -0.03, 0.02]).as_matrix()
    mount[:3, 3] = [0.02, -0.01, 0.04]
    reference = Plane((0.04, -0.07, 1), -0.1)
    poses, measurements = [], []
    for _ in range(25):
        pose = camera_pose()
        pose[:3, :3] = Rotation.from_rotvec(rng.uniform(-0.8, 0.8, 3)).as_matrix()
        pose[:3, 3] += rng.uniform(-0.3, 0.3, 3)
        poses.append(pose)
        measurements.append(plane_in_camera(reference, pose @ mount))
    fitted = fit_hand_eye(poses, measurements, np.eye(4))
    np.testing.assert_allclose(fitted.transform, mount, atol=1e-6)
    np.testing.assert_allclose(fitted.reference.normal, reference.normal, atol=1e-6)
    assert fitted.reference.offset_m == pytest.approx(reference.offset_m, abs=1e-6)


def test_noisy_plane_is_not_promoted_just_because_it_is_large():
    from dataclasses import replace
    estimator = FloorEstimator(FloorSettings(stable_frames=1), reference_height_m=0)
    pose = camera_pose()
    bad = replace(measured_floor(pose), rms_m=0.08)
    result = estimator.update([bad], pose, stamp=10, identity=("a", "b", "c"), now=10)
    assert not result["valid"]
    assert not result["candidates"]


def test_offline_recording_uses_held_out_poses_and_does_not_overwrite_output(tmp_path):
    import json
    from elesim_pilot.vision.hand_eye_calibration import fit_recording, main
    rng = np.random.default_rng(54)
    mount = np.eye(4)
    mount[:3, :3] = Rotation.from_rotvec([0.03, -0.02, 0.05]).as_matrix()
    mount[:3, 3] = [0.02, 0.01, -0.03]
    floor = Plane((0, 0, 1), 0)
    rows = []
    for _ in range(25):
        parent = camera_pose()
        parent[:3, :3] = Rotation.from_rotvec(rng.uniform(-0.7, 0.7, 3)).as_matrix()
        parent[:3, 3] += rng.uniform(-0.2, 0.2, 3)
        observed = plane_in_camera(floor, parent @ mount)
        rows.append({"identity": ["robot", "boot", "calib"], "synchronization_verified": True,
            "pose_evidence": {"initial_mount": np.eye(4).tolist()},
            "predicted_camera_pose": parent.tolist(), "result": {"valid": True,
                "reference": {**floor.payload(), "kind": "configured"}, "observed": observed.payload()}})
    source = tmp_path / "observations.jsonl"
    source.write_text("\n".join(json.dumps(row) for row in rows))
    result = fit_recording(source)
    assert result["training_observations"] == 20
    assert result["validation_observations"] == 5
    assert result["validation_after"]["distance_p95_m"] < 1e-6
    assert result["validation_before"]["distance_p95_m"] > 0.01
    output = tmp_path / "candidate.json"
    main([str(source), "--output", str(output)])
    assert output.is_file()
    with pytest.raises(FileExistsError):
        main([str(source), "--output", str(output)])
    rows[0]["synchronization_verified"] = False
    source.write_text("\n".join(json.dumps(row) for row in rows))
    with pytest.raises(ValueError, match="timing"):
        fit_recording(source)
