"""Render completion evidence with separate observations, estimates and truth."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def _cloud(axis,points,color):
    step=max(1,len(points)//2000)
    axis.scatter(*points[::step].T,s=1.3,c=color,alpha=.7,rasterized=True)


def _bounds(axes,clouds):
    all_points=np.concatenate(clouds)
    center=(all_points.min(0)+all_points.max(0))/2
    radius=max(np.ptp(all_points,axis=0).max()/2,.01)*1.08
    for axis in axes:
        axis.set(xlim=(center[0]-radius,center[0]+radius),ylim=(center[1]-radius,center[1]+radius),zlim=(center[2]-radius,center[2]+radius))
        axis.set_box_aspect((1,1,1));axis.view_init(elev=22,azim=-55)
        axis.set_xticks([]);axis.set_yticks([]);axis.set_zticks([])


def plot(root,truth,smoke,output):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    from workbench.research.experiments.adapointr_evaluate import sample_box
    from workbench.research.experiments.adapointr import pcn_axes

    root,truth,smoke,output=map(Path,(root,truth,smoke,output));output.mkdir(exist_ok=True,parents=True)
    evaluation=json.loads((root/'evaluation/evaluation-r05.json').read_text())
    selected=evaluation['objects'][:4]
    figure=plt.figure(figsize=(11,3.2*len(selected)),layout='constrained')
    for i,row in enumerate(selected):
        with np.load(root/'predictions-r05/predictions'/row['output_file'],allow_pickle=False) as raw:
            observed,predicted=raw['observed'],raw['predicted']
        case=int(row['capture'].split('-')[1])
        with np.load(truth/f'case-{case:04d}.npz',allow_pickle=False) as raw:
            j=row['assigned_box'];gt=sample_box(raw['object_centers'][j],raw['object_sizes'][j],raw['object_rotations'][j],count=3000)
        axes=[figure.add_subplot(len(selected),3,i*3+k+1,projection='3d') for k in range(3)]
        _cloud(axes[0],observed,'#176fba');_cloud(axes[1],predicted,'#d97529');_cloud(axes[2],gt,'#159b89')
        _bounds(axes,[observed,predicted,gt])
        if i==0:
            for axis,title in zip(axes,['Observed RGB-D surface','AdaPoinTr estimate','Private truth: evaluator only']):axis.set_title(title,fontsize=11)
        axes[0].text2D(.02,.02,f"Capture {case}; component {row['component']}",transform=axes[0].transAxes,fontsize=9)
        axes[1].text2D(.02,.02,f"Surface p95: {row['surface_p95_mm']:.1f} mm\nF1@10mm: {row['f1_10mm']:.2f}",transform=axes[1].transAxes,fontsize=9)
    figure.suptitle('Original pretrained AdaPoinTr on EleSim observations\nPCN Y-up convention; observed-centroid frame; no truth alignment',fontsize=13)
    figure.supxlabel('First four objects in capture order, not selected by quality. Each row shares bounds. Blue observations remain stored separately.',fontsize=9)
    figure.savefig(output/'adapointr_objects.png',dpi=150);plt.close(figure)

    figure=plt.figure(figsize=(9,7),layout='constrained')
    for i,name in enumerate(['airplane','sofa']):
        with np.load(smoke/(name+'.npz'),allow_pickle=False) as raw:
            observed,predicted=pcn_axes(raw['observed']),pcn_axes(raw['predicted'])
        axes=[figure.add_subplot(2,2,i*2+k+1,projection='3d') for k in range(2)]
        _cloud(axes[0],observed,'#176fba');_cloud(axes[1],predicted,'#d97529');_bounds(axes,[observed,predicted])
        axes[0].set_title(name+': partial demo');axes[1].set_title('16,384 predicted points')
    figure.suptitle('Official demo inference sanity check\nNo complete ground truth supplied; this figure is qualitative',fontsize=13)
    figure.savefig(output/'adapointr_demos.png',dpi=150);plt.close(figure)

    native_path=root/'evaluation/evaluation-native05.json'
    if not native_path.exists():native_path=root.parent/'evaluation/evaluation-r05.json'
    native=json.loads(native_path.read_text())
    labels=['Native axes, radius .5','PCN axes, radius .5','PCN axes, radius 1.0']
    variants=[native,evaluation,json.loads((root/'evaluation/evaluation-r10.json').read_text())]
    figure,axes=plt.subplots(1,3,figsize=(13,4),layout='constrained')
    x=np.arange(3)
    for axis,key,title in zip(axes,['surface_p95_mm','f1_10mm','hidden_recall_10mm'],['Median per-object surface p95 (mm)','Median F1 at 10 mm','Median hidden-surface recall at 10 mm']):
        values=[v['metrics'][key] for v in variants]
        bars=axis.bar(x,values,color=['#aaaaaa','#159b89','#d97529'])
        axis.bar_label(bars,fmt='%.3f',padding=3,fontsize=9)
        axis.set(xticks=x,xticklabels=labels,title=title)
        axis.tick_params(axis='x',rotation=20)
        axis.set_ylim(0,max(max(values)*1.25,.1))
        if key=='f1_10mm':
            axis.set_ylim(0,1)
            axis.axhline(evaluation['metrics']['partial_f1_10mm'],color='#176fba',linestyle='--',label='Observed-only baseline')
            axis.legend(fontsize=8)
        if key=='hidden_recall_10mm':
            axis.set_ylim(0,1)
            axis.axhline(.5,color='#176fba',linestyle='--',label='Initial acceptance threshold')
            axis.legend(fontsize=8)
    figure.suptitle('Independent metric-scale evaluation; unchanged criteria; no ICP or ground-truth normalization')
    figure.savefig(output/'adapointr_metrics.png',dpi=150);plt.close(figure)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['root','truth','smoke','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();plot(args.root,args.truth,args.smoke,args.output)


if __name__=='__main__':main()
