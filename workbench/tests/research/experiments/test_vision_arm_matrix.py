"""Pose/error sampling must be independent and use the actual arm geometry."""
from pathlib import Path
import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from elesim_pilot.vision.floor_calibration import (
    FloorEstimator, FloorSettings, Plane, plane_in_camera, plane_in_world,
)
from elesim_pilot.vision.partial_cloud import ArmCameraModel
from workbench.research.experiments.vision_arm_scene import ArmUrdf, sample_cases, produce_matrix


ROOT = Path(__file__).resolve().parents[4]
DATA = ROOT / "payload/data"


def models():
    base = np.eye(4)
    base[2, 3] = .8
    pilot = ArmCameraModel(DATA / "models/arm/default.json",
                           DATA / "calibration/cameras/zed_mini.hand_eye.json", base_world_transform=base)
    producer = ArmUrdf(DATA / "models/assemblies/zed-mini/arm.urdf",
                       DATA / "calibration/cameras/zed_mini.hand_eye.json", base_height_m=.8)
    return pilot, producer


def measured_plane(pose, height=0):
    return plane_in_camera(Plane((0, 0, 1), -height, area_m2=1.0,
                                 span_m=(1.0, 1.0), inliers=1800, samples=2000), pose)


def test_real_four_dof_fk_matches_independent_urdf_across_joint_limits():
    pilot, producer = models()
    rng = np.random.default_rng(140)
    for q in rng.uniform(pilot.lower, pilot.upper, (80, 4)):
        np.testing.assert_allclose(pilot.camera_pose(q), producer.camera_pose(q), atol=1e-10)
    with pytest.raises(ValueError, match="joint"):
        pilot.camera_pose(pilot.upper + .001)


def test_pose_and_error_seeds_form_a_full_cross_product_without_visibility_filtering():
    pilot, _ = models()
    cases = sample_cases(pilot.lower, pilot.upper, poses=24, error_seeds=2, seed=217)
    pose_ids = {row["pose_id"] for row in cases}
    errors = {(row["condition"], row["error_seed"]) for row in cases}
    assert len(pose_ids) == 24
    assert len(cases) == len(pose_ids) * len(errors)
    for pose_id in pose_ids:
        rows = [row for row in cases if row["pose_id"] == pose_id]
        assert {(row["condition"], row["error_seed"]) for row in rows} == errors
        assert len({tuple(row["q"]) for row in rows}) == 1
    qs = np.array([next(row["q"] for row in cases if row["pose_id"] == i) for i in sorted(pose_ids)])
    assert np.all(np.ptp(qs, axis=0) > .8 * (pilot.upper-pilot.lower))
    assert qs[:, 1].min() < np.radians(-70)
    assert qs[:, 1].max() > np.radians(70)
    for condition, error_seed in errors:
        rows = [row for row in cases if (row["condition"], row["error_seed"]) == (condition, error_seed)]
        assert len({tuple(row["rotation_vector"]) + tuple(row["translation"]) for row in rows}) == 1


def test_observable_height_tilt_correction_is_not_limited_to_zero_roll():
    pilot, producer = models()
    rng = np.random.default_rng(809)
    floor = Plane((0, 0, 1), 0)
    for q in rng.uniform(pilot.lower, pilot.upper, (40, 4)):
        predicted = pilot.camera_pose(q)
        actual = producer.camera_pose(q)
        actual[:3, :3] = Rotation.from_rotvec([.04, -.03, 0]).as_matrix() @ actual[:3, :3]
        actual[2, 3] += .035
        estimator = FloorEstimator(FloorSettings(stable_frames=1), reference_height_m=0)
        result = estimator.update([measured_plane(actual)], predicted, stamp=10, now=10,
                                  identity=("sensor", "boot", "calibration"))
        assert result["valid"], result["reason"]
        np.testing.assert_allclose(estimator.corrected_pose(now=10), actual, atol=1e-10)
        corrected_plane = plane_in_world(measured_plane(actual), estimator.corrected_pose(now=10))
        np.testing.assert_allclose(corrected_plane.normal, floor.normal, atol=1e-10)


def test_far_table_and_missing_depth_are_rejected_across_noncentral_roll():
    pilot, producer = models()
    rng = np.random.default_rng(991)
    for q in rng.uniform(pilot.lower, pilot.upper, (30, 4)):
        pose = producer.camera_pose(q)
        for planes in ([measured_plane(pose, .4)], []):
            estimator = FloorEstimator(FloorSettings(stable_frames=1), reference_height_m=0)
            result = estimator.update(planes, pose, stamp=10, now=10, identity=("a", "b", "c"))
            assert not result["valid"]
            assert estimator.corrected_pose(now=10) is None


def test_low_table_and_camera_height_error_can_supply_identical_measurements():
    """A plane-only observer cannot promise to distinguish this negative case."""
    _, producer = models()
    pose = producer.camera_pose([-.08, .6, .18, -.1])
    table_view = measured_plane(pose, .1)
    other_camera = pose.copy(); other_camera[2,3] -= .1
    floor_view = measured_plane(other_camera)
    np.testing.assert_allclose(table_view.normal, floor_view.normal, atol=1e-12)
    assert table_view.offset_m == pytest.approx(floor_view.offset_m)


def test_arm_rgbd_matrix_keeps_absent_targets_and_reports_failures(tmp_path):
    from workbench.research.experiments.vision_reconstruct import reconstruct
    from workbench.research.experiments.vision_arm_evaluate import evaluate_matrix

    produced = tmp_path / "produced"
    generated = produce_matrix(produced, data_root=DATA,
        config_path=ROOT/"payload/config/sim/config.yaml", poses=4, error_seeds=1, width=160, height=120)
    replay = reconstruct(produced/"sensors", tmp_path/"predictions",
        arm_model=DATA/"models/arm/default.json",
        hand_eye=DATA/"calibration/cameras/zed_mini.hand_eye.json")
    assert generated["frames"] == replay["frames"] == 96
    result = evaluate_matrix(produced/"truth", tmp_path/"predictions")
    assert result["complete"]
    assert result["cases"] == 32
    assert result["metrics"]["missing_depth"]["corrected_cases"] == 0
    assert result["metrics"]["wrong_plane"]["corrected_cases"] == 0
    assert any(row["points"] == 0 for row in result["rows"])
    # A measurement-only experiment must not silently drop poses with no
    # target or unsupported floor, or call the whole matrix a 1 mm success.
    assert not result["accepted"]
    summary = json.loads((tmp_path/"predictions/result.json").read_text())
    assert len(summary["frames"]) == generated["frames"]
    # Scene condition labels cannot override the selected plane's independent
    # geometry. A genuine floor can remain visible below a high tabletop.
    clean = next(row for row in result["rows"] if row["condition"] == "clean" and row["floor_valid"])
    manifest_path = produced/"truth/manifest.json"
    manifest = json.loads(manifest_path.read_text())
    next(row for row in manifest["cases"] if row["case_id"] == clean["case_id"])["condition"] = "wrong_plane"
    manifest_path.write_text(json.dumps(manifest))
    relabelled = evaluate_matrix(produced/"truth", tmp_path/"predictions")
    assert relabelled["metrics"]["wrong_plane"]["corrected_cases"] == 1
    assert relabelled["metrics"]["wrong_plane"]["false_floor_cases"] == 0
    assert relabelled["far_plane_and_missing_depth_rejected"]
