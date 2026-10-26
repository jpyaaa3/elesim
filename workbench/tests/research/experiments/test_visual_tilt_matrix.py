"""Imperfect arm joints and arbitrary horizontal-plane levels, with exact mount."""
from pathlib import Path
import json

import numpy as np

from elesim_pilot.vision.floor_calibration import FloorEstimator, FloorSettings, Plane, plane_in_camera
from elesim_pilot.vision.partial_cloud import ArmCameraModel
from workbench.research.experiments.vision_arm_scene import ArmUrdf, sample_joint_cases, produce_matrix

ROOT=Path(__file__).resolve().parents[4]
DATA=ROOT/'payload/data'


def models():
    base=np.eye(4);base[2,3]=.8
    pilot=ArmCameraModel(DATA/'models/arm/default.json',DATA/'calibration/cameras/zed_mini.hand_eye.json',base_world_transform=base)
    actual=ArmUrdf(DATA/'models/assemblies/zed-mini/arm.urdf',DATA/'calibration/cameras/zed_mini.hand_eye.json')
    return pilot,actual


def test_joint_errors_are_private_physical_states_with_one_fixed_camera_mount():
    pilot,actual=models()
    cases=sample_joint_cases(pilot.lower,pilot.upper,poses=24,error_seeds=2)
    assert len(cases)==288
    for case in cases:
        np.testing.assert_array_equal(case['rotation_vector'],[0,0,0])
        np.testing.assert_array_equal(case['translation'],[0,0,0])
        q=np.asarray(case['actual_q'])
        assert np.all(q>=pilot.lower-1e-9) and np.all(q<=pilot.upper+1e-9)
        np.testing.assert_allclose(actual.camera_pose(q),actual.link_poses(q)['node9'] @ pilot.mount,atol=1e-10)
    for pose_id in range(24):
        rows=[row for row in cases if row['pose_id']==pose_id]
        assert len({tuple(row['q']) for row in rows})==1
        assert any(not np.array_equal(row['q'],row['actual_q']) for row in rows)


def test_joint_bias_visual_tilt_is_corrected_without_claiming_fk_position_recovery():
    pilot,actual=models()
    cases=sample_joint_cases(pilot.lower,pilot.upper,poses=24,error_seeds=2)
    for case in cases:
        predicted,truth=pilot.camera_pose(case['q']),actual.camera_pose(case['actual_q'])
        estimator=FloorEstimator(FloorSettings(stable_frames=1),reference_normal=(0,0,1))
        for index,height in enumerate((0.,.3,-.6)):
            plane=plane_in_camera(Plane((0,0,1),-height,area_m2=1,span_m=(1,1),inliers=1000,samples=1200),truth)
            result=estimator.update([plane],predicted,stamp=10+index*.1,now=10+index*.1,
                                    identity=('camera','boot','calibration'))
            assert result['valid'],result['reason']
            corrected=estimator.corrected_pose(now=10+index*.1)
            difference=corrected[:3,:3] @ truth[:3,:3].T
            np.testing.assert_allclose(difference @ [0,0,1],[0,0,1],atol=1e-10)
            np.testing.assert_array_equal(corrected[:3,3],predicted[:3,3])


def test_sensor_only_replay_reports_angles_and_rejects_camera_origin_relocation(tmp_path):
    from workbench.research.experiments.vision_reconstruct import reconstruct
    from workbench.research.experiments.vision_tilt_evaluate import evaluate_tilt
    from elesim_pilot.vision.partial_cloud import SensorObservation

    data=tmp_path/'produced';predictions=tmp_path/'predictions'
    produce_matrix(data,data_root=DATA,config_path=ROOT/'payload/config/sim/config.yaml',
                   poses=4,error_seeds=1,width=160,height=120,joint_errors=True)
    result=reconstruct(data/'sensors',predictions,arm_model=DATA/'models/arm/default.json',
                       hand_eye=DATA/'calibration/cameras/zed_mini.hand_eye.json')
    assert result['frames']==72
    for path in (data/'sensors').glob('*.npz'):
        observation=SensorObservation.load(path)
        assert len(observation.encoder_q)==4
        with np.load(path,allow_pickle=False) as packet:
            assert 'actual_q' not in packet.files and 'joint_offset' not in packet.files
    evaluation=evaluate_tilt(data/'truth',predictions)
    assert evaluation['complete'] and evaluation['cases']==24
    assert evaluation['all_camera_origins_unchanged']
    assert not evaluation['camera_mount_perturbed']
    assert evaluation['metrics']['raised_horizontal']['corrected_cases']>0
    assert evaluation['metrics']['raised_horizontal']['tilt_after_p95_deg']<.3
    summary_path=predictions/'result.json'
    summary=json.loads(summary_path.read_text())
    row=next(row for row in summary['frames'] if row['floor_valid'] and row['name'].endswith('-2.npz'))
    row['corrected_pose'][2][3]+=.12
    summary_path.write_text(json.dumps(summary))
    invalid=evaluate_tilt(data/'truth',predictions)
    assert not invalid['all_camera_origins_unchanged']
    assert not invalid['accepted']
