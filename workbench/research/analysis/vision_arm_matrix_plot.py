"""Availability and accuracy for every pose/error condition, including failures."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def plot_matrix(evaluation, output):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    from matplotlib.patches import Patch

    result=json.loads(Path(evaluation).read_text())
    names=list(result["metrics"])
    labels=[name.replace("_"," ") for name in names]
    poses=result["unique_poses"]
    categories=np.zeros((poses,len(names)),dtype=int)
    error=np.full_like(categories,np.nan,dtype=float)
    for pose_id in range(poses):
        for condition_id,name in enumerate(names):
            rows=[row for row in result["rows"] if row["pose_id"]==pose_id and row["condition"]==name]
            if any(row["false_floor_accepted"] for row in rows):category=3
            elif any(row["floor_valid"] for row in rows):category=2
            else:category=1
            categories[pose_id,condition_id]=category
            values=[row["after_p95_m"]*1000 for row in rows if "after_p95_m" in row]
            if values:error[pose_id,condition_id]=max(values)
    from matplotlib.colors import ListedColormap, BoundaryNorm
    cmap=ListedColormap(["#c7cdd1","#43b29b","#df6b57"])
    norm=BoundaryNorm([.5,1.5,2.5,3.5],3)
    fig,axes=plt.subplots(1,2,figsize=(15,8),layout="constrained")
    axes[0].imshow(categories,cmap=cmap,norm=norm,aspect="auto")
    axes[0].set(xticks=np.arange(len(names)),xticklabels=labels,yticks=np.arange(poses),
                yticklabels=np.arange(poses),ylabel="Joint pose (same poses in every column)",
                title="Correction outcome, worst of two error seeds")
    axes[0].tick_params(axis="x",rotation=45)
    axes[0].legend(handles=[Patch(color="#c7cdd1",label="Unavailable / rejected"),
        Patch(color="#43b29b",label="Correct floor selected"),
        Patch(color="#df6b57",label="Wrong floor accepted")],loc="upper center",bbox_to_anchor=(.5,-.2),ncol=1)
    valid_names=[name for name in names if result["metrics"][name]["after_p95_m"] is not None]
    x=np.arange(len(valid_names))
    before=[result["metrics"][name]["before_p95_m"]*1000 for name in valid_names]
    after=[result["metrics"][name]["after_p95_m"]*1000 for name in valid_names]
    for offset,values,color,label in ((-.18,before,"#c7cdd1","Before"),(.18,after,"#159a86","After")):
        bars=axes[1].bar(x+offset,values,width=.36,color=color,label=label)
        axes[1].bar_label(bars,labels=[f"{v:.1f}" for v in values],padding=4,fontsize=9)
    axes[1].set(xticks=x,xticklabels=[n.replace("_"," ") for n in valid_names],
                ylabel="Object-point world error, p95 (mm)",title="Point accuracy where both object and correction exist")
    axes[1].tick_params(axis="x",rotation=45)
    axes[1].legend()
    axes[1].set_ylim(0,max(before+after)*1.2)
    fig.suptitle(f"Four-DOF arm matrix | {result['cases']} cases, {result['frames']} frames | "
                 f"Overall acceptance: {'PASS' if result['accepted'] else 'FAIL'}",fontsize=15)
    fig.supxlabel("Known fixed bench base, ideal encoders, static Genesis rendering. No ICP. "
                  "Unavailable poses remain counted; bars do not establish accuracy at those poses.",fontsize=10)
    fig.savefig(output,dpi=160)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("evaluation",type=Path);parser.add_argument("output",type=Path)
    args=parser.parse_args();plot_matrix(args.evaluation,args.output)


if __name__=="__main__":main()
