"""Evidence figures for four-DOF multi-view acquisition and derived geometry."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from workbench.research.experiments.arm_scan_evaluate import sample_shape


COLORS={'first':'#f4a261','fk_only':'#dc5263','fused':'#159fbc','truth':'#7a8590'}


def plot_views(scans, truth, output, episode):
    result=json.loads((scans/f'episode-{episode:02d}/result.json').read_text())
    accepted=[v for v in result['rows'] if v['accepted']]
    selected=[accepted[i] for i in np.linspace(0,len(accepted)-1,6,dtype=int)]
    fig,axes=plt.subplots(2,3,figsize=(13,8),layout='constrained')
    for ax,row in zip(axes.ravel(),selected):
        with np.load(scans/f'episode-{episode:02d}'/f"view-{row['view']:03d}.npz",allow_pickle=False) as packet:
            image=packet['color_bgr'][...,:3][...,::-1].copy();mask=packet['mask'].astype(np.uint8)
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image,contours,-1,(80,255,130),1)
        ax.imshow(image);ax.set_title(f"Camera view {row['view']+1} | {row['object_points']:,} object points")
        ax.axis('off')
    fig.suptitle(f'Actual mounted-camera RGB | episode {episode} | green: observed object mask',fontsize=15)
    fig.savefig(output/f'arm_scan_views_{episode:02d}.png',dpi=160);plt.close(fig)
    records=json.loads((truth/'manifest.json').read_text())['records']
    observer=[r for r in records if r['episode']==episode and (truth/f"observer-{r['file'][5:10]}.png").exists()]
    if observer:
        observer=[observer[i] for i in np.linspace(0,len(observer)-1,min(4,len(observer)),dtype=int)]
        fig,axes=plt.subplots(1,len(observer),figsize=(4.5*len(observer),3.8),squeeze=False)
        for ax,row in zip(axes.ravel(),observer):
            ax.imshow(cv2.cvtColor(cv2.imread(str(truth/f"observer-{row['file'][5:10]}.png")),cv2.COLOR_BGR2RGB))
            ax.set_title(f"Actual URDF pose | view {row['view']+1}");ax.axis('off')
        fig.suptitle('Same fixed-base 4DOF arm; camera motion comes from its joints',fontsize=15)
        fig.tight_layout();fig.savefig(output/f'arm_scan_arm_{episode:02d}.png',dpi=160);plt.close(fig)


def plot_clouds(scans,truth,output,episodes):
    private=json.loads((truth/'manifest.json').read_text());evaluation=json.loads((output.parent/'evaluation/evaluation.json').read_text())
    fig=plt.figure(figsize=(17,5.5*len(episodes)),layout='constrained')
    for ri,episode in enumerate(episodes):
        summary=json.loads((scans/f'episode-{episode:02d}/result.json').read_text())
        first=next(r for r in summary['rows'] if r['accepted'])
        with np.load(truth/f"view-{first['seq']:05d}.npz") as packet:pose=packet['true_camera_pose']
        shape=private['shape_parameters'][episode];center=np.asarray(shape['center']);full=sample_shape(shape,count=12000)
        with np.load(scans/f'episode-{episode:02d}/cloud.npz') as cloud:
            arrays={k:cloud[k]@pose[:3,:3].T+pose[:3,3] for k in ('first','fk_only','fused')}
        arrays['truth']=full
        limit=max(np.asarray(shape['size']))*.64*1000
        metrics=next(r for r in evaluation['episodes'] if r['episode']==episode)
        for ci,(label,points) in enumerate(arrays.items()):
            ax=fig.add_subplot(len(episodes),4,ri*4+ci+1,projection='3d')
            if len(points)>4500:points=points[np.linspace(0,len(points)-1,4500,dtype=int)]
            p=(points-center)*1000
            ax.scatter(*p.T,s=.7,c=COLORS[label],alpha=.85,rasterized=True)
            ax.set(xlim=(-limit,limit),ylim=(-limit,limit),zlim=(-limit,limit),xlabel='X (mm)',ylabel='Y (mm)',zlabel='Z (mm)')
            ax.set_box_aspect((1,1,1));ax.view_init(elev=24,azim=-58)
            title={'first':'First view','fk_only':'All views: nominal FK','fused':'All views: coherent fusion','truth':'Independent truth'}[label]
            if label!='truth':
                m=metrics[label];title+=f"\nSurface p95 {m['surface_p95_mm']:.1f} mm | coverage {100*m['recall_5mm']:.0f}%"
            prefix=f"Episode {episode}: {shape['kind']} | "+('joint errors' if episode>=3 else 'clean joints')
            ax.set_title(prefix+'\n'+title,fontsize=10,pad=14)
    fig.savefig(output/'arm_scan_clouds.png',dpi=170);plt.close(fig)


def plot_metrics(evaluation,output):
    rows=evaluation['episodes'];x=np.arange(len(rows));width=.24
    fig,axes=plt.subplots(1,3,figsize=(15,4.5))
    for offset,label in zip((-width,0,width),('first','fk_only','fused')):
        axes[0].bar(x+offset,[100*r[label]['recall_5mm'] for r in rows],width,color=COLORS[label],label=label)
        axes[1].bar(x+offset,[r[label]['surface_p95_mm'] for r in rows],width,color=COLORS[label])
    axes[0].axhline(60,color='#777',ls='--',lw=1,label='60% full-surface test')
    axes[0].set(ylabel='Full truth surface within 5 mm (%)',ylim=(0,100));axes[0].legend(fontsize=8)
    axes[1].set(ylabel='Cloud-to-true-surface p95 (mm)')
    for i,axis in enumerate(('X','Y','Z')):
        axes[2].bar(x+(i-1)*width,[r['fused']['dimension_error_mm'][i] for r in rows],width,label=axis)
    axes[2].set(ylabel='Fused dimension error (mm)');axes[2].legend()
    for ax in axes:
        ax.set_xticks(x,['Box','Thin box','Cylinder','Box + error','Thin + error','Cylinder + error'],rotation=22,ha='right')
        ax.grid(axis='y',alpha=.2)
    fig.suptitle('Six fixed-base scan episodes | unseen backsides and undersides remain unknown',fontsize=15)
    fig.tight_layout();fig.savefig(output/'arm_scan_metrics.png',dpi=170);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path)
    parser.add_argument('--figures-dir',type=Path)
    args=parser.parse_args();root=args.root;output=args.figures_dir or root/'figures';output.mkdir(exist_ok=True)
    for episode in (1,5):plot_views(root/'consumer/scans',root/'produced/truth',output,episode)
    plot_clouds(root/'consumer/scans',root/'produced/truth',output,(1,5))
    plot_metrics(json.loads((root/'evaluation/evaluation.json').read_text()),output)


if __name__=='__main__':main()
