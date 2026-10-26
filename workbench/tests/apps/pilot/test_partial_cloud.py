import numpy as np
import pytest

from elesim_pilot.vision.partial_cloud import SensorObservation, reconstruct_view, VoxelCloud


def packet(**extra):
    value = {"schema_version": np.array(1), "color_bgr": np.zeros((120,160,3),np.uint8),
        "depth_raw": np.full((120,160),0.8,np.float32), "depth_scale": np.array(1.0),
        "intrinsics": np.array([100,100,79.5,59.5,160,120]), "seq": np.array(1),
        "ts": np.array(100.0), "source_id": np.array("camera"),
        "source_boot_id": np.array("boot"), "calibration_id": np.array("calib"),
        "encoder_q": np.array([0,0,0.8,0,0,0.0])}
    value.update(extra)
    return value


@pytest.mark.parametrize("key", ["camera_world_origin", "camera_world_pose", "object_world",
    "asset_id", "silhouette_xz", "mesh_path", "true_camera_pose"])
def test_sensor_packet_rejects_truth_and_catalog_shortcuts(tmp_path,key):
    path=tmp_path/'frame.npz'
    np.savez_compressed(path,**packet(**{key:np.array([1,2,3])}))
    with pytest.raises(ValueError,match="unexpected"):
        SensorObservation.load(path)


def test_point_cloud_keeps_visible_surface_points_and_pixel_correspondence(tmp_path):
    path=tmp_path/'frame.npz'
    np.savez_compressed(path,**packet())
    observed=SensorObservation.load(path)
    mask=np.zeros((120,160),bool);mask[35:85,55:105]=True
    pose=np.diag([1.0,-1,-1,1]);pose[2,3]=0.8
    view=reconstruct_view(observed,mask,pose,pose)
    assert len(view.points_camera)==2500
    assert view.pixels_uv.shape==(2500,2)
    np.testing.assert_allclose(view.points_after[:,2],0,atol=1e-7)
    assert np.ptp(view.points_after[:,0])>0.35


def test_voxel_fusion_is_bounded_and_does_not_align_away_pose_errors():
    cloud=VoxelCloud(voxel_m=0.01,max_voxels=100)
    first=np.array([[0.001,0.002,0.003],[0.003,0.004,0.005]])
    cloud.add(first)
    cloud.add(first+[0.05,0,0])
    assert len(cloud.points())==2
    assert np.ptp(cloud.points()[:,0])==pytest.approx(0.05)
    before=cloud.points().copy()
    with pytest.raises(ValueError,match="budget"):
        cloud.add(np.column_stack((np.arange(200)*0.02,np.zeros(200),np.zeros(200))))
    np.testing.assert_allclose(cloud.points(),before)
