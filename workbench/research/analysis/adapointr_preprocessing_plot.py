"""Plot public-demo preprocessing sensitivity, without absolute accuracy claims."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def plot(root,output):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    from workbench.research.experiments.adapointr import pcn_axes
    root,output=map(Path,(root,output));output.mkdir(parents=True,exist_ok=True)
    result=json.loads((root/'result.json').read_text())
    modes=['native','center_only','scale_only_r05','center_scale_r05','center_scale_r10']
    labels=['Native coords','Center only','Scale only (.5)','Center + scale (.5)','Center + scale (1.0)']
    colors=['#159b89','#7871b8','#d59c42','#ce705c','#a77153']
    figure,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    for row,name in enumerate(['airplane','sofa']):
        for column,(metric,title) in enumerate([
            ('observed_to_prediction_p95_pct_diag','Observed-to-output distance p95'),
            ('reference_chamfer_mean_pct_diag','Output change vs same-seed original')]):
            axis=axes[row,column]
            values=[[r[metric] for r in result['rows'] if r['sample']==name and r['variant']==mode] for mode in modes]
            median=np.array([np.median(v) for v in values])
            error=np.array([[m-min(v) for m,v in zip(median,values)],[max(v)-m for m,v in zip(median,values)]])
            bars=axis.bar(range(5),median,color=colors,yerr=error,capsize=3)
            axis.bar_label(bars,fmt='%.2f',padding=7,fontsize=8)
            axis.set(xticks=range(5),xticklabels=labels,ylabel='% of original observed diagonal',title=name+': '+title)
            axis.tick_params(axis='x',rotation=18)
            limit=max(max(v) for v in values)
            if column==1:
                noise=result['controls'][name]['native_sampling_chamfer_pct_diag_median']
                axis.axhline(noise,color='#176fba',linestyle='--',label='Original resampling variation')
                axis.legend(fontsize=8);limit=max(limit,noise)
            axis.set_ylim(0,max(.1,limit*1.3))
    figure.suptitle('Same physical input points, unchanged PCN axes; outputs restored to original coordinates',fontsize=13)
    figure.supxlabel('Five sampling seeds; bars show medians and ranges. Public demos have no complete truth: these are fidelity/equivariance metrics.',fontsize=9)
    figure.savefig(output/'preprocessing_metrics.png',dpi=150);plt.close(figure)

    figure=plt.figure(figsize=(14,7),layout='constrained')
    for row,name in enumerate(['airplane','sofa']):
        clouds=[]
        for mode in ['native','center_only','center_scale_r05']:
            with np.load(root/(name+'-0-'+mode+'.npz'),allow_pickle=False) as raw:
                if not clouds:clouds.append(pcn_axes(raw['observed']))
                clouds.append(pcn_axes(raw['predicted']))
        combined=np.concatenate(clouds);center=(combined.min(0)+combined.max(0))/2
        extent=max(np.ptp(combined,axis=0).max()/2,.01)*1.1
        for column,cloud in enumerate(clouds):
            axis=figure.add_subplot(2,4,row*4+column+1,projection='3d')
            chosen=cloud[::max(1,len(cloud)//2000)]
            axis.scatter(*chosen.T,s=1.3,color='#176fba' if column==0 else colors[[0,0,1,3][column]],alpha=.75,rasterized=True)
            axis.set(xlim=(center[0]-extent,center[0]+extent),ylim=(center[1]-extent,center[1]+extent),zlim=(center[2]-extent,center[2]+extent))
            axis.set_box_aspect((1,1,1));axis.view_init(elev=22,azim=-55)
            axis.set_xticks([]);axis.set_yticks([]);axis.set_zticks([])
            axis.set_title(['Observed','Native output','Center only output','Our center + scale (.5)'][column],fontsize=10)
            if column==0:axis.text2D(.02,.02,name,transform=axis.transAxes,fontsize=10)
    figure.suptitle('Public-demo preprocessing control: qualitative outputs, no complete ground truth',fontsize=13)
    figure.supxlabel('Seed 0 shown; all outputs share the original frame and bounds. Axes are reordered only for presentation.',fontsize=9)
    figure.savefig(output/'preprocessing_clouds.png',dpi=150);plt.close(figure)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();plot(args.root,args.output)


if __name__=='__main__':main()
