"""Independent object-point validation of offline multi-pose mount candidates."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from workbench.research.experiments.vision_arm_evaluate import point_metrics


def evaluate_candidates(candidates, observations):
    candidates, observations = Path(candidates), Path(observations)
    fits = json.loads(candidates.read_text())
    inputs = json.loads((observations/"observations.json").read_text())
    by_name = {row["frame"]:row for row in inputs}
    groups = {}; rows = []
    for fit in fits["groups"]:
        if "candidate_mount" not in fit:continue
        for name, pose in fit["heldout_camera_poses"].items():
            if name not in by_name:raise ValueError("missing held-out object evidence")
            if name in fit["training_frames"]:raise ValueError("held-out pose was used for fitting")
            source = by_name[name]
            assert fit["boot"]==source["boot"]
            with np.load(observations/"consumer/predictions"/name,allow_pickle=False) as predicted, np.load(observations/"produced/data/truth"/source["truth"],allow_pickle=False) as truth:
                pose = np.asarray(pose)
                after = predicted["points_camera"] @ pose[:3,:3].T + pose[:3,3]
                metrics, errors = point_metrics(truth,{"pixels_uv":predicted["pixels_uv"],
                    "points_before":predicted["points_after"],"points_after":after})
                group = groups.setdefault(source["condition"],{"cases":0,"per_frame":[],"fixed_mount":[]})
                group["cases"]+=1
                group["per_frame"].extend(errors[0].tolist());group["fixed_mount"].extend(errors[1].tolist())
                rows.append({"frame":name,"boot":fit["boot"],"condition":source["condition"],
                             "per_frame_p95_m":metrics["before_p95_m"],"fixed_mount_p95_m":metrics["after_p95_m"]})
    metrics = {name:{"heldout_object_cases":group["cases"],
        "per_frame_p95_m":float(np.percentile(group["per_frame"],95)),
        "fixed_mount_p95_m":float(np.percentile(group["fixed_mount"],95))} for name,group in groups.items()}
    return {"schema_version":1,"metrics":metrics,"rows":rows,
            "scope":"exploratory offline candidates; all object poses held out; trusted FK, bench base and known floor",
            "evaluation_alignment":"none","applied_to_runtime":False}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("candidates",type=Path);parser.add_argument("observations",type=Path)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args();result=evaluate_candidates(args.candidates,args.observations)
    with args.output.open("x") as stream:stream.write(json.dumps(result,indent=2,allow_nan=False)+"\n")
    print(json.dumps(result["metrics"]))


if __name__=="__main__":main()
