"""Regression fixture extracted from a retired experiment; no standalone CLI."""


from __future__ import annotations


import json


from pathlib import Path


import numpy as np


def evaluate(truth_directory,prediction_directory):
    truth_directory,prediction_directory=Path(truth_directory),Path(prediction_directory)
    truth=json.loads((truth_directory/"manifest.json").read_text())
    predicted=json.loads((prediction_directory/"result.json").read_text())
    conditions={"observable":{"before":[],"after":[],"surface":[]},
                "unobservable":{"before":[],"after":[],"surface":[]}}
    frames=[]
    by_name={row["name"]:row for row in predicted["frames"]}
    for row in truth["frames"]:
        summary=by_name[row["name"]]
        if not summary["floor_valid"]:
            continue
        with np.load(truth_directory/row["name"],allow_pickle=False) as gt, np.load(prediction_directory/row["name"],allow_pickle=False) as output:
            uv=output["pixels_uv"]; u,v=uv[:,0],uv[:,1]
            z=gt["ideal_depth"][v,u]; fx,fy,cx,cy=gt["true_intrinsics"][:4]
            camera=np.column_stack(((u-cx)*z/fx,(v-cy)*z/fy,z))
            pose=gt["true_camera_pose"]
            actual=camera @ pose[:3,:3].T+pose[:3,3]
            before=np.linalg.norm(output["points_before"]-actual,axis=1)
            after=np.linalg.norm(output["points_after"]-actual,axis=1)
            local=(output["points_after"]-gt["object_center"]) @ gt["object_rotation"]
            q=np.abs(local)-gt["object_size"]/2
            surface=np.abs(np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(np.max(q,axis=1),0))
            group=conditions[row["condition"]]
            group["before"].extend(before.tolist());group["after"].extend(after.tolist())
            group["surface"].extend(surface.tolist())
            frames.append({"name":row["name"],"condition":row["condition"],"points":len(z),
                "before_p95_m":float(np.percentile(before,95)),"after_p95_m":float(np.percentile(after,95))})
    metrics={}
    for name,group in conditions.items():
        if not group["after"]: raise RuntimeError(f"no valid {name} observations")
        metrics[name]={key+"_p95_m":float(np.percentile(values,95)) for key,values in group.items()}
    corrected=metrics["observable"]
    negative=metrics["unobservable"]
    accepted=(corrected["after_p95_m"]<0.012 and corrected["after_p95_m"]<corrected["before_p95_m"]*0.3
              and corrected["surface_p95_m"]<0.012 and negative["after_p95_m"]>0.025)
    return {"accepted":bool(accepted),"backend":truth["backend"],"valid_frames":len(frames),
        "metrics":metrics,"negative_control_detected":negative["after_p95_m"]>0.025,
        "scope":truth["scope"],"rig":truth["rig"],"frames":frames,
        "evaluation_alignment":"none; fixed shared encoder/world coordinates"}
