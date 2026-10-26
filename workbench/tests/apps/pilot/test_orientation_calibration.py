"""Arm-induced visual tilt uses plane direction, without relocating the camera."""
from types import SimpleNamespace
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from elesim_pilot.vision.floor_calibration import (
    FloorEstimator, FloorSettings, Plane, correct_camera_pose, plane_in_camera,
)
from elesim_pilot.vision.floor_service import FloorCalibrationService
from elesim_pilot.vision.perception.depth_pose import CameraIntrinsics


def pose(*, tilt=0., height=.8):
    result=np.eye(4)
    result[:3,:3]=Rotation.from_euler("x",tilt).as_matrix() @ np.diag([1.,-1.,-1.])
    result[:3,3]=[.2,-.1,height]
    return result


def observed(transform, *, height=0., normal=(0,0,1)):
    return plane_in_camera(Plane(normal,-height,area_m2=1.,span_m=(1.,1.),
                                 inliers=1200,samples=1600),transform)


@pytest.mark.parametrize("height",[-2.,0.,.12,.45])
def test_default_correction_uses_direction_without_changing_camera_position(height):
    actual,predicted=pose(tilt=.03),pose(tilt=.12,height=.86)
    result=correct_camera_pose(predicted,observed(actual,height=height),Plane((0,0,1),0))
    np.testing.assert_array_equal(result[:3,3],predicted[:3,3])
    np.testing.assert_allclose(result[:3,:3],actual[:3,:3],atol=1e-12)


def test_parallel_surfaces_at_different_heights_supply_one_direction():
    actual,predicted=pose(),pose(tilt=.08,height=.85)
    estimator=FloorEstimator(FloorSettings(stable_frames=1),reference_normal=(0,0,1))
    result=estimator.update([observed(actual,height=h) for h in (0,.12,.45)],predicted,
        stamp=10,now=10,identity=("camera","boot","calibration"))
    assert result['valid'],result['reason']
    assert result['observable']==['tilt_2d']
    assert 'translation_3d' in result['unobservable']
    assert result['correction_mode']=='orientation'
    np.testing.assert_array_equal(estimator.corrected_pose(now=10)[:3,3],predicted[:3,3])
    np.testing.assert_allclose(estimator.corrected_pose(now=10)[:3,:3],actual[:3,:3],atol=1e-12)


def test_plane_height_changes_do_not_reset_direction_stability():
    actual,predicted=pose(),pose(tilt=.08)
    estimator=FloorEstimator(reference_normal=(0,0,1))
    for index,height in enumerate((0.,.45,-.7)):
        result=estimator.update([observed(actual,height=height)],predicted,
            stamp=10+index*.1,now=10+index*.1,identity=('camera','boot','calibration'))
    assert result['valid'],result['reason']
    assert result['candidates'][0]['stable_frames']==3


def test_conflicting_supported_directions_are_rejected():
    predicted=pose()
    second=Rotation.from_euler('x',8,degrees=True).apply([0,0,1])
    estimator=FloorEstimator(FloorSettings(stable_frames=1),reference_normal=(0,0,1))
    result=estimator.update([observed(predicted),observed(predicted,height=.2,normal=second)],predicted,
        stamp=10,now=10,identity=('camera','boot','calibration'))
    assert not result['valid'] and result['reason']=='ambiguous_reference'
    assert estimator.corrected_pose(now=10) is None


def test_confirmation_asserts_horizontal_instead_of_anchoring_the_erroneous_fk_tilt():
    actual,predicted=pose(),pose(tilt=.1)
    estimator=FloorEstimator(FloorSettings(stable_frames=1))
    result=estimator.update([observed(actual,height=.3)],predicted,stamp=10,now=10,
                            identity=('camera','boot','calibration'))
    assert not result['valid']
    estimator.confirm(result['candidates'][0]['id'],now=10)
    result=estimator.update([observed(actual,height=.3)],predicted,stamp=10.1,now=10.1,
                            identity=('camera','boot','calibration'))
    assert result['valid']
    assert result['reference']['kind']=='confirmed_horizontal'
    assert not result['reference']['absolute']
    np.testing.assert_allclose(estimator.corrected_pose(now=10.1)[:3,:3],actual[:3,:3],atol=1e-12)


def test_known_inclined_reference_keeps_its_declared_direction():
    actual,predicted=pose(),pose(tilt=.08)
    normal=Rotation.from_euler('x',6,degrees=True).apply([0,0,1])
    estimator=FloorEstimator(FloorSettings(stable_frames=1),reference_normal=normal)
    result=estimator.update([observed(actual,normal=normal)],predicted,stamp=10,now=10,
                            identity=('camera','boot','calibration'))
    assert result['valid']
    np.testing.assert_allclose(estimator.corrected_pose(now=10)[:3,:3],actual[:3,:3],atol=1e-12)


def test_cached_orientation_correction_keeps_the_current_camera_origin():
    current=[pose(tilt=.1)]
    def frame(stamp):
        return SimpleNamespace(color_bgr=np.zeros((120,160,3),np.uint8),
            depth_raw=np.full((120,160),.8,np.float32),depth_scale=1.,
            intrinsics=CameraIntrinsics(100,100,80,60,160,120),ts=stamp,seq=1,
            source_id='camera',source_boot_id='boot',calibration_id='calibration')
    service=FloorCalibrationService(reference_height_m=0,apply=True,clock_verified=True,
        pose_provider=lambda _:current[0],wall_clock=lambda:100,settings=FloorSettings(stable_frames=1))
    try:
        assert service.submit(frame(100))
        deadline=time.monotonic()+3
        while service.snapshot()['processed']<1 and time.monotonic()<deadline:time.sleep(.005)
        assert service.snapshot()['correction_available']
        current[0]=current[0].copy();current[0][:3,3]+=[.01,0,.01]
        corrected=service.corrected_camera_pose(frame(100.1))
        assert corrected is not None
        np.testing.assert_array_equal(corrected[:3,3],current[0][:3,3])
        np.testing.assert_allclose(corrected[:3,:3],pose()[:3,:3],atol=1e-6)
    finally:service.close()


def test_metric_mount_fitter_refuses_height_free_direction_records(tmp_path):
    import json
    from elesim_pilot.vision.hand_eye_calibration import fit_recording
    source=tmp_path/'directions.jsonl'
    source.write_text(json.dumps({'result':{'valid':True,'correction_mode':'orientation'}})+'\n')
    with pytest.raises(ValueError,match='orientation-only'):
        fit_recording(source)
