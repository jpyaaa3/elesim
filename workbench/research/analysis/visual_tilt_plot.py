"""Plot visual tilt accuracy and availability without an absolute-height target."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def plot_tilt(evaluation,output):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt

    result=json.loads(Path(evaluation).read_text())
    names=list(result['metrics'])
    labels={'clean':'Clean','joint_offsets':'Joint bias','deflection':'Deflection',
            'raised_horizontal':'Raised plane','two_horizontal':'Parallel planes','noisy':'Noisy depth'}
    x=np.arange(len(names));fig,axes=plt.subplots(1,3,figsize=(17,5),layout='constrained')
    colors=('#d28a76','#159b89')
    panels=[('tilt_before_p95_deg','tilt_after_p95_deg','True camera tilt error'),
            ('object_normal_before_p95_deg','object_normal_after_p95_deg','Independent object top-face error')]
    for axis,(before,after,title) in zip(axes,panels):
        for offset,key,color,label in ((-.18,before,colors[0],'Before'),(.18,after,colors[1],'After')):
            values=[result['metrics'][name][key] for name in names]
            bars=axis.bar(x+offset,[v or 0 for v in values],width=.36,color=color,label=label)
            axis.bar_label(bars,labels=[f'{v:.2f}' if v is not None else 'N/A' for v in values],padding=3,fontsize=8)
        axis.set(xticks=x,xticklabels=[labels[name] for name in names],ylabel='Angle p95 (degrees)',title=title)
        axis.tick_params(axis='x',rotation=35)
        axis.set_ylim(0,max([result['metrics'][name][before] or 0 for name in names]+[1.])*1.24)
        axis.legend(fontsize=8)
    valid=np.array([result['metrics'][name]['corrected_cases'] for name in names])
    total=np.array([result['metrics'][name]['cases'] for name in names])
    axes[2].bar(x,valid,color=colors[1],label='Supported')
    axes[2].bar(x,total-valid,bottom=valid,color='#ccd1d4',label='Unavailable')
    for i,(v,n) in enumerate(zip(valid,total)):axes[2].text(i,n+.6,f'{v}/{n}',ha='center',fontsize=9)
    axes[2].set(xticks=x,xticklabels=[labels[name] for name in names],ylabel='Pose/error cases',
                ylim=(0,max(total)*1.15),title='Every sampled pose remains counted')
    axes[2].tick_params(axis='x',rotation=35);axes[2].legend(fontsize=8)
    fig.suptitle(f"Imperfect 4DOF arm | {result['cases']} cases, {result['frames']} frames | "
                 f"{'PASS' if result['accepted'] else 'FAIL'}",fontsize=15)
    fig.supxlabel('Static Genesis; fixed camera mount; trusted horizontal directions at varying heights. '
                  'Camera origin unchanged. Position, yaw and individual joint biases remain uncalibrated.',fontsize=10)
    fig.savefig(output,dpi=160);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('evaluation',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();plot_tilt(args.evaluation,args.output)


if __name__=='__main__':main()
