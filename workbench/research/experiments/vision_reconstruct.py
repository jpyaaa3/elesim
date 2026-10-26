"""Blind Pilot reconstruction: mount only this script, sensors and output.

No truth directory, scene generator, object catalog or camera oracle is read.
The virtual rig below is the declared encoder model, with a nominal camera mount.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from elesim_pilot.vision.floor_calibration import FloorEstimator, depth_points, detect_planes
from elesim_pilot.vision.partial_cloud import SensorObservation, reconstruct_view, VoxelCloud
from elesim_pilot.vision.perception.detector import HsvDetector


def reconstruct(sensor_directory, output):
    output=Path(output); output.mkdir(parents=True,exist_ok=False)
    detector=HsvDetector({"target_label":"red object", "min_area_px":40,
        "keep_largest_component":True,"morph_kernel_px":3,
        "hsv":{"ranges":[{"lower":[0,100,60],"upper":[12,255,255]},
                           {"lower":[168,100,60],"upper":[179,255,255]}]}})
    estimator=FloorEstimator(reference_height_m=0.0)
    clouds={}; rows=[]
    for path in sorted(Path(sensor_directory).glob("view-*.npz")):
        observed=SensorObservation.load(path)
        if len(observed.encoder_q)!=6: raise ValueError("this experiment uses a six-axis virtual encoder rig")
        q=observed.encoder_q
        predicted=np.eye(4)
        predicted[:3,3]=q[:3]
        predicted[:3,:3]=Rotation.from_euler("xyz",q[3:]).as_matrix() @ np.diag([1,-1,-1])
        detection=detector.detect(observed.color_bgr)
        if detection is None: raise RuntimeError(f"object not detected in {path.name}")
        mask=cv2.erode(detection.mask,np.ones((3,3),np.uint8),iterations=1)>0
        floor_points=depth_points(observed.depth_raw,observed.intrinsics,observed.depth_scale,excluded=mask)
        planes=detect_planes(floor_points)
        status=estimator.update(planes,predicted,stamp=observed.ts,now=observed.ts,
            identity=(observed.source_id,observed.source_boot_id,observed.calibration_id))
        corrected=estimator.corrected_pose(now=observed.ts)
        view=reconstruct_view(observed,mask,predicted,corrected)
        np.savez_compressed(output/path.name,points_camera=view.points_camera,pixels_uv=view.pixels_uv,
            points_before=view.points_before,points_after=view.points_after)
        rows.append({"name":path.name,"boot":observed.source_boot_id,"points":len(view.points_camera),
            "floor_valid":status["valid"],"floor_reason":status["reason"],
            "normal_error_deg":status["normal_error_deg"],"height_error_m":status["distance_error_m"]})
        if corrected is not None:
            cloud=clouds.setdefault(observed.source_boot_id,VoxelCloud())
            cloud.add(view.points_after)
    for boot,cloud in clouds.items():
        np.save(output/f"cloud-{boot}.npy",cloud.points(),allow_pickle=False)
    result={"schema_version":1,"frames":rows,"fused_clouds":len(clouds),
        "inputs":"RGB-D, intrinsics and ideal virtual-rig encoder readings only",
        "alignment":"fixed encoder/world coordinates; no ICP"}
    (output/"result.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    return {"frames":len(rows),"valid_frames":sum(r["floor_valid"] for r in rows),"clouds":len(clouds)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sensors",type=Path)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    print(json.dumps(reconstruct(args.sensors,args.output)))


if __name__=="__main__": main()
