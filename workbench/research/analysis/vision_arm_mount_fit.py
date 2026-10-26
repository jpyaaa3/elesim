"""Explore fixed-mount fitting using only the blind consumer's observations.

Every pose with object points is held out of fitting. No condition labels or
Sim truth are read; outputs are offline candidates and never installed.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np

from elesim_pilot.vision.floor_calibration import Plane, plane_in_world
from elesim_pilot.vision.hand_eye_calibration import fit_hand_eye
from elesim_pilot.vision.perception_bridge.hand_eye import load_hand_eye_transform


def fit_candidates(recording, nominal_hand_eye):
    recording, nominal_hand_eye = Path(recording), Path(nominal_hand_eye)
    replay = json.loads(recording.read_text())
    mount, _ = load_hand_eye_transform(nominal_hand_eye)
    inverse = np.linalg.inv(mount)
    groups = defaultdict(dict)
    for row in replay["frames"]:
        if row["floor_valid"]:
            groups[row["boot"]][tuple(row["encoder_q"])] = row
    results = []
    for boot, poses in groups.items():
        values = list(poses.values())
        training = [row for row in values if not row["points"]]
        validation = [row for row in values if row["points"]]
        result = {"boot":boot,"training_poses":len(training),"validation_poses":len(validation),
                  "training_frames":[row["name"] for row in training],
                  "validation_frames":[row["name"] for row in validation]}
        try:
            if not validation: raise ValueError("no held-out object observations")
            fit = fit_hand_eye(
                [np.asarray(row["predicted_pose"]) @ inverse for row in training],
                [Plane(row["observed_plane"]["normal"],row["observed_plane"]["offset_m"]) for row in training],
                mount,reference=Plane((0,0,1),0))
            distances, angles, camera_poses = [], [], {}
            for row in validation:
                camera_pose = np.asarray(row["predicted_pose"]) @ inverse @ fit.transform
                plane = plane_in_world(Plane(row["observed_plane"]["normal"],
                                             row["observed_plane"]["offset_m"]),camera_pose)
                distances.append(abs(plane.offset_m))
                angles.append(float(np.degrees(np.arccos(np.clip(abs(plane.normal[2]),0,1)))))
                camera_poses[row["name"]] = camera_pose.tolist()
            result.update(candidate_mount=fit.transform.tolist(),training_distance_rms_m=fit.distance_rms_m,
                training_normal_rms_deg=fit.normal_rms_deg,condition_number=fit.condition,
                heldout_plane_distance_p95_m=float(np.percentile(distances,95)),
                heldout_plane_normal_p95_deg=float(np.percentile(angles,95)),
                heldout_camera_poses=camera_poses)
        except ValueError as exc:
            result["unavailable"] = str(exc)
        results.append(result)
    return {"schema_version":1,"groups":results,
            "recording_sha256":hashlib.sha256(recording.read_bytes()).hexdigest(),
            "nominal_hand_eye_sha256":hashlib.sha256(nominal_hand_eye.read_bytes()).hexdigest(),
            "training_policy":"all poses with object points are held out; no condition labels or truth",
            "scope":"offline fixed-mount candidate with trusted FK, bench base and known Z=0 floor; not applied"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording",type=Path)
    parser.add_argument("nominal_hand_eye",type=Path)
    parser.add_argument("--output",type=Path,required=True)
    args = parser.parse_args()
    result = fit_candidates(args.recording,args.nominal_hand_eye)
    with args.output.open("x") as stream:stream.write(json.dumps(result,indent=2,allow_nan=False)+"\n")
    print(json.dumps({"groups":len(result["groups"]),"available":sum("candidate_mount" in row for row in result["groups"])}))


if __name__=="__main__":main()
