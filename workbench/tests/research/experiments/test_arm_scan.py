import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from workbench.research.experiments.arm_scan_evaluate import sample_shape, surface_distance, cloud_metrics
from workbench.research.experiments.arm_scan_scene import fixtures, analytic_rgbd


def test_box_and_cylinder_truth_metrics_measure_surfaces_without_fitting():
    for shape in fixtures()[:3]:
        p=sample_shape(shape,count=6000)
        assert np.max(surface_distance(p,shape))<1e-12
        moved=p+[.035,.025,0]
        assert np.percentile(surface_distance(moved,shape),95)>.02
        stats=cloud_metrics(p,shape)
        assert stats['recall_5mm']>.98


def test_fixture_renderer_provides_depth_for_a_real_urdf_camera_pose():
    from workbench.research.experiments.vision_arm_scene import ArmUrdf
    root=Path(__file__).resolve().parents[4]/'payload/data'
    arm=ArmUrdf(root/'models/assemblies/zed-mini/arm.urdf',root/'calibration/cameras/zed_mini.hand_eye.json')
    pose=arm.camera_pose([-.12,0,-.1,.38])
    intr=np.array([120/np.tan(np.pi/6)]*2+[159.5,119.5,320,240])
    for shape in fixtures()[:3]:
        rgb,depth=analytic_rgbd(pose,intr,shape)
        mask=rgb[:,:,2]>200
        assert mask.sum()>500
        v,u=np.nonzero(mask)
        z=depth[v,u]
        p=np.column_stack([(u-intr[2])*z/intr[0],(v-intr[3])*z/intr[1],z])
        world=p@pose[:3,:3].T+pose[:3,3]
        assert np.percentile(surface_distance(world,shape),95)<1e-6


def test_observation_request_capture_and_coherent_fusion_end_to_end(tmp_path):
    from elesim_pilot.vision.partial_cloud import ArmCameraModel, SensorObservation
    from elesim_pilot.vision.perception.depth_pose import CameraIntrinsics
    from elesim_pilot.vision.multiview import transform
    from workbench.research.experiments.arm_scan import scan_episode
    from workbench.research.experiments.arm_scan_scene import ArmUrdf

    root=Path(__file__).resolve().parents[4]/'payload/data'
    base=np.eye(4);base[2,3]=.8
    model=ArmCameraModel(root/'models/arm/default.json',root/'calibration/cameras/zed_mini.hand_eye.json',base_world_transform=base)
    arm=ArmUrdf(root/'models/assemblies/zed-mini/arm.urdf',root/'calibration/cameras/zed_mini.hand_eye.json')
    # Fixed legal poses exercise the acquisition/fusion path; view ranking has
    # its own geometry test. Truth is confined to the capture and evaluator.
    proposed=[[-.111196,-.109599,-.604340,-.540127],[-.144751,.275379,.587957,-.570483],
              [-.154131,-.285329,.610798,-.606222],[-.016474,.077623,-.102489,.427529]]
    class Waypoints:
        def choose(self,observed,current,accepted,tried):
            assert len(observed)>60
            return {'q':np.array(proposed[min(len(tried)-1,3)])}
    shape=fixtures()[1]
    width,height=240,180;f=height/2/np.tan(np.pi/6)
    intr=np.array([f,f,(width-1)/2,(height-1)/2,width,height]);records=[]
    bias=np.array([.003,np.radians(3),np.radians(.8),np.radians(-.6)])
    def capture(episode,view,q):
        pose=arm.camera_pose(np.clip(q+bias,model.lower,model.upper))
        records.append(pose)
        color,depth=analytic_rgbd(pose,intr,shape)
        return SensorObservation(color,depth,1.,CameraIntrinsics(*intr[:4],width,height),view,
                                 1000.+view,'cam','boot','cal',tuple(q))
    result=scan_episode(capture,model,Waypoints(),tmp_path/'scan',max_views=6)
    assert result['joint_fit']['accepted'],result
    assert result['views_accepted']==6
    assert not result['truth_access'] and not result['geometry_complete_claimed']
    with np.load(tmp_path/'scan/cloud.npz') as cloud:
        assert len(cloud['fused'])>len(cloud['first'])*1.4
        stats=cloud_metrics(transform(cloud['fused'],records[0]),shape)
        baseline=cloud_metrics(transform(cloud['first'],records[0]),shape)
        assert stats['recall_5mm']>baseline['recall_5mm']+.12
        assert stats['surface_p95_mm']<3.
