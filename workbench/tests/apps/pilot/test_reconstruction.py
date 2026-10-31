from dataclasses import replace
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from elesim_pilot.reconstruction.artifacts import export_result
from elesim_pilot.reconstruction.contracts import (
    CompletedObject, ImuSample, InstanceMask, Measurement, Mesh, ObjectCloud, Task,
)
from elesim_pilot.reconstruction.models import load_model
from elesim_pilot.reconstruction.pipeline import Pipeline, PipelineCancelled
from elesim_pilot.reconstruction.providers import DetectorSegmenter, VoxelSurface
from elesim_pilot.vision.partial_cloud import SensorObservation
from elesim_pilot.vision.perception.depth_pose import CameraIntrinsics
from elesim_protocol.scene_artifact import SceneArtifact


def measurement(sequence=1, **changes):
    image = np.zeros((4, 6, 3), np.uint8)
    image[:, :2, 1] = 255
    image[:, 4:, 2] = 255
    sensor = SensorObservation(image, np.full((4, 6), .8, np.float32), 1.,
        CameraIntrinsics(20., 20., 2.5, 1.5, 6, 4), sequence, 100. + sequence,
        "camera", "boot", "calibration", (0., 0., 0., 0.))
    return Measurement(replace(sensor, **changes))


def config_file(tmp_path, *, planner=None, corrections=()):
    weights = tmp_path / "weights.json"
    weights.write_text('{"offset_m": 0.2}')
    model = lambda name, **kw: {"factory": "pilot.reconstruction_fixtures:" + name, **kw}
    models = {"segmenter": model("Segmenter"), "pose": model("CameraPose"),
        "completion": model("Completion", assets={"weights": {"path": "weights.json",
            "sha256": hashlib.sha256(weights.read_bytes()).hexdigest()}}),
        "mesher": {"factory": "elesim_pilot.reconstruction.providers:VoxelSurface",
            "options": {"voxel_m": .02}},
        "pose_corrections": [model("Correction", options=options) for options in corrections]}
    if planner is not None:
        models["planner"] = model("Planner", options=planner)
    path = tmp_path / "pipeline.yaml"
    path.write_text(yaml.safe_dump({"schema_version": 1, "models": models}))
    return path


def test_external_models_keep_apple_tree_identity_and_prediction_separate(tmp_path):
    pipeline = Pipeline.from_config(config_file(tmp_path, planner={}))
    last = measurement(2, encoder_q=(-.03, 0., 0., 0.))
    result = pipeline.run([measurement(), last], Task("orchard", "apple-1", operation="pick"))
    apple, tree = result.objects
    assert (apple.shape.cloud.object_id, tree.shape.cloud.object_id) == ("apple-1", "tree-1")
    assert (apple.shape.cloud.role, tree.shape.cloud.role) == ("target", "obstacle")
    assert apple.shape.cloud.observed[:, 0].max() < tree.shape.cloud.observed[:, 0].min()
    assert len(apple.shape.predicted) == 1
    assert not apple.shape.cloud.observed.flags.writeable
    assert not apple.shape.predicted.flags.writeable
    assert apple.mesh.vertices[:, 2].max() > apple.shape.cloud.observed[:, 2].max() + .15
    assert result.plan is not None
    np.testing.assert_array_equal(result.plan.q[0], last.sensor.encoder_q)
    manifest = export_result(result, tmp_path / "orchard")
    scene = SceneArtifact.load(manifest)
    assert scene.scene_id == "orchard"
    assert [o.object_id for o in scene.objects] == ["apple-1", "tree-1"]
    with np.load(manifest.parent / "apple-1.npz", allow_pickle=False) as data:
        np.testing.assert_array_equal(data["observed"], apple.shape.cloud.observed)
        np.testing.assert_array_equal(data["predicted"], apple.shape.predicted)
    plan = json.loads((manifest.parent / "plan.json").read_text())
    assert plan["scene_revision"] == scene.revision and plan["executed"] is False
    provenance = json.loads(manifest.read_text())["provenance"]
    assert provenance["models"]["completion"]["assets"]["weights"]
    with pytest.raises(FileExistsError, match="immutable"):
        export_result(result, manifest.parent)


def test_model_preparation_does_not_require_a_measurement_sample(tmp_path):
    pipeline = Pipeline.from_config(config_file(tmp_path, corrections=[{}]))
    prepared = []
    pipeline.corrections[0].implementation.prepare = lambda: prepared.append(True)
    pipeline.prepare()
    assert prepared == [True]
    with pytest.raises(ValueError, match='IMU'):
        pipeline.observe(measurement(), Task('s', 'apple-1'))


def test_ordered_corrections_require_calibrated_fresh_imu(tmp_path):
    pipeline = Pipeline.from_config(config_file(tmp_path, corrections=[{"offset_m": .1}, {"scale": 2.}]))
    sample = measurement()
    with pytest.raises(ValueError, match="requires a calibrated"):
        pipeline.run([sample], Task("s", "apple-1"))
    imu = ImuSample([0., 0., 0.], "boot", "mount-v1", sample.sensor.ts - .1)
    baseline = Pipeline.from_config(config_file(tmp_path)).run([sample], Task("s", "apple-1"))
    result = pipeline.run([replace(sample, imu=imu)], Task("s", "apple-1"))
    np.testing.assert_allclose(result.objects[0].shape.cloud.observed,
        baseline.objects[0].shape.cloud.observed + [.2, 0., 0.])
    with pytest.raises(ValueError, match="IMU source boot or freshness"):
        replace(sample, imu=replace(imu, timestamp_s=sample.sensor.ts - 1.)).validated()
    second = measurement(2)
    changed_imu = ImuSample((0., 0., 0.), "boot", "different-mount", second.sensor.ts)
    with pytest.raises(ValueError, match="IMU calibration changed"):
        pipeline.run([replace(sample, imu=imu), replace(second, imu=changed_imu)], Task("s", "apple-1"))


@pytest.mark.parametrize("change, message", [
    ({"source_boot_id": "new"}, "source boot/calibration"),
    ({"calibration_id": "new"}, "source boot/calibration"),
    ({"seq": 1}, "replayed or reordered"),
    ({"ts": 50.}, "replayed or reordered"),
])
def test_replay_or_mixed_calibration_never_fuses_clouds(tmp_path, change, message):
    pipeline = Pipeline.from_config(config_file(tmp_path))
    with pytest.raises(ValueError, match=message):
        pipeline.run([measurement(), measurement(2, **change)], Task("s", "apple-1"))


def test_cancellation_stops_between_model_calls(tmp_path):
    pipeline = Pipeline.from_config(config_file(tmp_path))
    events = []
    with pytest.raises(PipelineCancelled):
        pipeline.run([measurement()], Task("s", "apple-1"),
            progress=events.append, cancelled=lambda: bool(events))
    assert [e["stage"] for e in events] == ["pose"]


@pytest.mark.parametrize("name, message", [
    ("WrongApi", "API version"), ("WrongStage", "does not implement"),
    ("Broken", "external model dependency missing"),
])
def test_model_factory_reports_incompatible_or_unavailable_model(tmp_path, name, message):
    with pytest.raises((ValueError, RuntimeError), match=message):
        load_model("pose", {"factory": "pilot.reconstruction_fixtures:" + name}, tmp_path)


def test_changed_weights_fail_before_factory_runs(tmp_path):
    path = config_file(tmp_path)
    (tmp_path / "weights.json").write_text("changed")
    with pytest.raises(ValueError, match="asset hash mismatch"):
        Pipeline.from_config(path)


@pytest.mark.parametrize("options, message", [
    ({"linear_m": -.18}, "joint bounds"),
    ({"contact_ids": ["tree-1"]}, "obstacle or unknown"),
    ({"start_offset_m": -.02}, "measured joint state"),
])
def test_plan_is_checked_against_limits_and_object_roles(tmp_path, options, message):
    pipeline = Pipeline.from_config(config_file(tmp_path, planner=options))
    with pytest.raises(ValueError, match=message):
        pipeline.run([measurement()], Task("s", "apple-1", operation="wrap"))


def test_missing_target_and_overlapping_masks_fail(tmp_path):
    pipeline = Pipeline.from_config(config_file(tmp_path))
    with pytest.raises(ValueError, match="not observed"):
        pipeline.run([measurement()], Task("s", "not-visible"))
    image = measurement().sensor.color_bgr.copy()
    image[:, :2, 2] = 255
    with pytest.raises(ValueError, match="overlapping"):
        pipeline.run([measurement(color_bgr=image)], Task("s", "apple-1"))


def test_invalid_model_geometry_is_rejected():
    with pytest.raises(ValueError, match="bbox fallback"):
        InstanceMask("apple-1", "apple", np.ones((2, 2), bool), kind="bbox")
    cloud = ObjectCloud("apple-1", "apple", "target", "world", [[0., 0., .8]])
    with pytest.raises(ValueError, match="non-finite"):
        CompletedObject(cloud, [[np.nan, 0., 0.]])
    with pytest.raises(ValueError, match="integer indices"):
        Mesh("apple-1", "world", [[0, 0, 0], [1, 0, 0], [0, 1, 0]], [[0., 1., 2.]])


def test_voxel_mesher_consumes_prediction_without_changing_observations():
    cloud = ObjectCloud("apple-1", "apple", "target", "world", [[0., 0., .8]])
    shape = CompletedObject(cloud, [[0., 0., 1.]])
    mesh = VoxelSurface(options={"voxel_m": .02}, assets={}).mesh(shape, Task("s", "apple-1"))
    assert len(mesh.triangles) == 24  # Two separated voxel surfaces.
    assert len(shape.cloud.observed) == 1
    assert mesh.vertices[:, 2].max() == pytest.approx(1.02)


@pytest.mark.parametrize("kind", ["bbox", "unknown"])
def test_reference_detector_refuses_bbox_only_or_unknown_segmentation(kind):
    from elesim_pilot.vision.perception.detector import DetectionResult
    class Bbox:
        def detect(self, image):
            return DetectionResult(label="apple", confidence=1., bbox_xyxy=(0, 0, 2, 2),
                mask=np.ones(image.shape[:2], np.uint8), mask_kind=kind)
    segmenter = DetectorSegmenter.__new__(DetectorSegmenter)
    segmenter.label, segmenter.detector = "apple", Bbox()
    with pytest.raises(ValueError, match="bbox/unknown"):
        segmenter.segment(measurement(), Task("s", "apple-1"))


def test_shipped_config_loads_without_learned_dependencies():
    root = next(p for p in Path(__file__).resolve().parents if (p / "payload").is_dir())
    pipeline = Pipeline.from_config(root / "payload/config/pilot/reconstruction.example.yaml")
    assert set(pipeline.models) == {"segmenter", "pose", "completion", "mesher"}
    assert not pipeline.corrections


def test_integrated_profile_loads_the_floor_and_real_completion_factories(tmp_path):
    import hashlib
    import yaml
    from elesim_pilot.reconstruction.floor import FloorTilt
    from elesim_pilot.reconstruction.adapointr import AdaPoinTrCompletion
    root = next(p for p in Path(__file__).resolve().parents if (p/'payload').is_dir())/'payload/config/pilot'
    config = yaml.safe_load((root/'scan.integrated.yaml').read_text())
    for name in ('source', 'weights'):
        path = tmp_path/name
        path.write_bytes(b'explicit loader fixture; no GPU inference')
        config['models']['completion']['assets'][name] = dict(path=str(path),
            sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    for spec in config['models'].values():
        for model in (spec if isinstance(spec, list) else [spec]):
            for asset in model.get('assets', {}).values():
                if not Path(asset['path']).is_absolute():
                    asset['path'] = str((root/asset['path']).resolve())
    path = tmp_path/'integrated.yaml'
    path.write_text(yaml.safe_dump(config))
    with Pipeline.from_config(path) as pipeline:
        assert isinstance(pipeline.corrections[0].implementation, FloorTilt)
        completion = pipeline.models['completion'].implementation
        assert isinstance(completion, AdaPoinTrCompletion) and completion.output_points == 8192
        assert not completion.diagnostics()['loaded']


def test_reference_segmenter_can_exclude_rgb_depth_boundary_pixels():
    image = np.zeros((32,32,3),np.uint8)
    image[6:26,6:26,2] = 255
    depth = np.full((32,32),2.,np.float32)
    depth[8:24,8:24] = .4
    sensor = SensorObservation(image,depth,1.,CameraIntrinsics(40.,40.,15.5,15.5,32,32),
        1,1000.,'sim','boot','calibration',(0.,0.,0.,0.))
    root = next(p for p in Path(__file__).resolve().parents if (p/'payload').is_dir())
    config = root/'payload/config/pilot/perception/detector.sim_hsv.json'
    segmenter = DetectorSegmenter(options={'mask_erode_px':2},assets={'detector':config})
    instances = segmenter.segment(Measurement(sensor),Task('boundary','target'))
    assert len(instances) == 1
    assert int(instances[0].mask.sum()) == 16*16
    assert np.all(sensor.depth_raw[instances[0].mask] == .4)


def test_depth_support_removes_a_single_background_pixel_but_keeps_inner_surfaces():
    image = np.zeros((32,32,3),np.uint8)
    image[4:28,4:28,2] = 255
    depth = np.full((32,32),2.,np.float32)
    depth[4:28,4:28] = .4
    depth[10:18,10:18] = .55  # A separate, legitimate inner-wall surface.
    depth[23,23] = 1.65       # One RGB-mask pixel sees the distant floor.
    sensor = SensorObservation(image,depth,1.,CameraIntrinsics(40.,40.,15.5,15.5,32,32),
        1,1000.,'sim','boot','calibration',(0.,0.,0.,0.))
    root = next(p for p in Path(__file__).resolve().parents if (p/'payload').is_dir())
    config = root/'payload/config/pilot/perception/detector.sim_hsv.json'
    segmenter = DetectorSegmenter(options={'depth_component_gap_m':.025},assets={'detector':config})
    mask = segmenter.segment(Measurement(sensor),Task('depth','target'))[0].mask
    assert not mask[23,23]
    assert np.all(mask[10:18,10:18])
    assert int(mask.sum()) == 24*24-1
    diagnostics = segmenter.diagnostics()
    assert diagnostics['rejected_pixels'] == 1 and diagnostics['depth_components'] == 3
    legacy = DetectorSegmenter(options={},assets={'detector':config})
    assert legacy.segment(Measurement(sensor),Task('depth','target'))[0].mask[23,23]


def test_voxel_mesh_keeps_observations_and_dense_unseen_patch_but_rejects_lone_prediction():
    cloud = ObjectCloud('target','jar','target','world',[[0.,0.,0.],[.5,0.,.5]])
    cluster = np.array([[.101+i*.0001,.101,.101] for i in range(12)])
    predicted = np.vstack((cluster,[[.8,.8,.8]]))
    shape = CompletedObject(cloud,predicted)
    mesher = VoxelSurface(options={'voxel_m':.02,'min_generated_neighbors':8},assets={})
    mesh = mesher.mesh(shape,Task('supported','target'))
    assert mesh.vertices[:,2].max() == pytest.approx(.52)
    assert np.any(np.all(np.isclose(mesh.vertices,[.1,.1,.1]),axis=1))
    np.testing.assert_array_equal(shape.predicted,predicted)
    np.testing.assert_array_equal(shape.cloud.observed,[[0.,0.,0.],[.5,0.,.5]])
    assert mesher.diagnostics()['rejected_generated_points'] == 1


def test_explicit_generated_support_radius_keeps_filtering_fixed_across_voxel_sizes():
    cloud = ObjectCloud('target','jar','target','world',[[.5,0.,.5]])
    # Nine neighbours span 8 mm: supported at 10 mm, not at half a 5 mm voxel.
    predicted = np.array([[.101+i*.001,.101,.101] for i in range(10)]+[[.8,.8,.8]])
    shape = CompletedObject(cloud,predicted)
    for voxel in (.02,.01,.005):
        mesher = VoxelSurface(options={'voxel_m':voxel,'min_generated_neighbors':8,
            'generated_neighbor_radius_m':.01},assets={})
        mesher.mesh(shape,Task('resolution','target'))
        assert mesher.diagnostics()['meshed_generated_points'] == 10
        assert mesher.diagnostics()['rejected_generated_points'] == 1
        assert mesher.diagnostics()['generated_neighbor_radius_m'] == .01
    np.testing.assert_array_equal(shape.predicted,predicted)


def test_voxel_budget_can_be_explicitly_raised_without_weakening_the_default():
    points = np.array([[x*.04,y*.04,z*.04] for x in range(16) for y in range(16) for z in range(16)])
    shape = CompletedObject(ObjectCloud('target','jar','target','world',points),np.empty((0,3)))
    with pytest.raises(ValueError,match='budget exceeded'):
        VoxelSurface(options={},assets={}).mesh(shape,Task('budget','target'))
    mesher = VoxelSurface(options={'max_occupied_voxels':8000},assets={})
    mesh = mesher.mesh(shape,Task('budget','target'))
    assert mesher.diagnostics()['occupied_voxels'] == 4096
    assert len(mesh.triangles) <= 100_000
    with pytest.raises(ValueError,match='max_occupied_voxels'):
        VoxelSurface(options={'max_occupied_voxels':8001},assets={})
