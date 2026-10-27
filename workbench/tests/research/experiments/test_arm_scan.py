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
    from elesim_pilot.robot.arm.self_collision import ArmSelfCollision
    from elesim_pilot.vision.multiview import ViewPlanner
    safety=ArmSelfCollision(root/'models/planning/zed-mini',joint_uncertainty=[.004,.06,.018,.018])
    planner=ViewPlanner(model,collision_checker=safety,candidates=3000)
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
    result=scan_episode(capture,model,planner,tmp_path/'scan',max_views=8)
    assert result['joint_fit']['accepted'],result
    assert 4<=result['views_accepted']<=8
    assert result['views_attempted']<8
    assert result['stop_reason'] in ('marginal_information_saturated','no_safe_distinct_view')
    assert result['self_collision']['all_motion_certified']
    assert not result['truth_access'] and not result['geometry_complete_claimed']
    with np.load(tmp_path/'scan/cloud.npz') as cloud:
        assert len(cloud['fused'])>len(cloud['first'])*1.4
        stats=cloud_metrics(transform(cloud['fused'],records[0]),shape)
        baseline=cloud_metrics(transform(cloud['first'],records[0]),shape)
        assert stats['recall_5mm']>baseline['recall_5mm']+.12
        assert stats['surface_p95_mm']<3.
