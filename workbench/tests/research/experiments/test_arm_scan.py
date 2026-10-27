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
