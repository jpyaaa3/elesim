"""Evaluate every arm pose/error case, including unavailable and false floors.

This process alone reads hidden geometry. Point metrics use the fixed bench
world coordinates, without registration or alignment. A completed test matrix
can report failed safety/accuracy criteria without losing its diagnostic data.
"""
from __future__ import annotations

import argparse
import collections
import json
from pathlib import Path

import numpy as np


def point_metrics(truth, prediction):
    uv = prediction["pixels_uv"]
    if not len(uv): return None, None
    u, v = uv[:,0], uv[:,1]
    z = truth["ideal_depth"][v,u]
    fx, fy, cx, cy = truth["true_intrinsics"][:4]
    camera = np.column_stack(((u-cx)*z/fx, (v-cy)*z/fy, z))
    pose = truth["true_camera_pose"]
    actual = camera @ pose[:3,:3].T + pose[:3,3]
    before = np.linalg.norm(prediction["points_before"]-actual, axis=1)
    after_points = prediction["points_after"]
    if not len(after_points):
        return {"points":len(z),"before_p95_m":float(np.percentile(before,95))}, None
    after = np.linalg.norm(after_points-actual, axis=1)
    surface = np.full(len(z), np.inf)
    for center, size, rotation in zip(truth["object_centers"],truth["object_sizes"],truth["object_rotations"]):
        q = np.abs((after_points-center) @ rotation)-size/2
        distance = np.abs(np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(np.max(q,axis=1),0))
        surface = np.minimum(surface,distance)
    return {"points":len(z),"before_p95_m":float(np.percentile(before,95)),
            "after_p95_m":float(np.percentile(after,95)),"surface_p95_m":float(np.percentile(surface,95))}, (before,after,surface)


def evaluate_matrix(truth_directory, predictions):
    truth_directory, predictions = Path(truth_directory), Path(predictions)
    manifest = json.loads((truth_directory/"manifest.json").read_text())
    assert manifest["schema_version"]==2
    replay = json.loads((predictions/"result.json").read_text())
    by_name = {row["name"]:row for row in replay["frames"]}
    expected = {row["name"] for row in manifest["frames"]}
    if expected != set(by_name): raise ValueError("missing or unexpected replay frames")
    last = {row["case_id"]:row for row in manifest["frames"] if row["repetition"]==2}
    groups = {}; rows = []
    for case in manifest["cases"]:
        frame = last[case["case_id"]]
        result = by_name[frame["name"]]
        row = {"case_id":case["case_id"],"pose_id":case["pose_id"],"condition":case["condition"],
               "error_seed":case["error_seed"],"q":case["q"],"frame":frame["name"],
               "floor_visible_pixels":case["floor_visible_pixels"],"floor_valid":result["floor_valid"],
               "floor_reason":result["floor_reason"],"object_detected":result["object_detected"],
               "points":result["points"],"false_floor_accepted":False}
        group = groups.setdefault(case["condition"],{"cases":0,"corrected_cases":0,
            "object_cases":0,"corrected_object_cases":0,"false_floor_cases":0,"reasons":collections.Counter(),
            "before":[],"after":[],"surface":[],"per_case":[]})
        group["cases"]+=1;group["corrected_cases"]+=int(result["floor_valid"])
        group["object_cases"]+=int(result["points"]>0)
        group["reasons"][result["floor_reason"] or "tracking"]+=1
        with np.load(truth_directory/case["truth_file"],allow_pickle=False) as truth, np.load(predictions/frame["name"],allow_pickle=False) as output:
            metrics, point_errors = point_metrics(truth,output)
            if metrics: row.update(metrics)
            if result["floor_valid"]:
                observed = result["observed_plane"]
                pose = truth["true_camera_pose"]
                normal = pose[:3,:3] @ observed["normal"]
                offset = observed["offset_m"]-float(normal @ pose[:3,3])
                tilt = float(np.degrees(np.arccos(np.clip(abs(normal[2]),0,1))))
                row.update(selected_plane_true_offset_m=float(offset),selected_plane_true_tilt_deg=tilt)
                # The test label alone never decides whether a selection was
                # wrong; the independently rendered measured plane does.
                false = abs(offset)>.025 or tilt>5.
                row["false_floor_accepted"] = bool(false)
                group["false_floor_cases"]+=int(false)
                if point_errors is not None:
                    group["corrected_object_cases"]+=1
                    for name, values in zip(("before","after","surface"),point_errors): group[name].extend(values.tolist())
                    group["per_case"].append(metrics["after_p95_m"])
        rows.append(row)
    summary = {}
    for name,group in groups.items():
        summary[name]={key:value for key,value in group.items() if key not in ("before","after","surface","per_case")}
        summary[name]["reasons"]=dict(group["reasons"])
        summary[name]["correction_availability"]=group["corrected_cases"]/group["cases"]
        for metric in ("before","after","surface"):
            summary[name][metric+"_p95_m"]=float(np.percentile(group[metric],95)) if group[metric] else None
        summary[name]["case_p95_error_p95_m"]=float(np.percentile(group["per_case"],95)) if group["per_case"] else None
    usable = [summary[name]["after_p95_m"] for name in ("clean","tilt_height","noisy")]
    accuracy = all(value is not None and value<.012 for value in usable)
    false_floors = sum(row["false_floor_accepted"] for row in rows)
    # A camera can be below the high tabletop and still see the genuine floor.
    # Its valid floor selection is not a rejection failure merely because the
    # producer labelled that scene 'wrong_plane'. Assess physical truth instead.
    guarded = (summary["missing_depth"]["corrected_cases"]==0
               and summary["wrong_plane"]["false_floor_cases"]==0)
    qs=np.array([row["q"] for row in manifest["cases"] if row["condition"]=="clean" and row["error_seed"]==0])
    return {"complete":True,"accepted":bool(accuracy and false_floors==0 and guarded),
        "backend":manifest["backend"],"cases":len(rows),"frames":len(by_name),"unique_poses":len(qs),
        "observed_q_min":qs.min(axis=0).tolist(),"observed_q_max":qs.max(axis=0).tolist(),
        "nominal_joint_lower":manifest["joint_lower"],"nominal_joint_upper":manifest["joint_upper"],
        "metrics":summary,"false_floor_accepted_cases":int(false_floors),
        "observable_accuracy_passed":bool(accuracy),"far_plane_and_missing_depth_rejected":bool(guarded),
        "criteria":{"observable_point_p95_below_m":.012,"false_floor_acceptance_cases":0,
                    "false_floor_offset_above_m":.025,"false_floor_tilt_above_deg":5},
        "genesis_reference_fk_max_abs":manifest["genesis_reference_fk_max_abs"],
        "scope":manifest["scope"],"sampling":manifest["sampling"],"rows":rows,
        "evaluation_alignment":"none; fixed declared bench coordinates"}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("truth",type=Path);parser.add_argument("predictions",type=Path)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--report-only",action="store_true")
    args=parser.parse_args()
    result=evaluate_matrix(args.truth,args.predictions)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    print(json.dumps({k:v for k,v in result.items() if k!="rows"},allow_nan=False))
    if not args.report_only and not result["accepted"]:raise SystemExit(1)


if __name__=="__main__":main()
