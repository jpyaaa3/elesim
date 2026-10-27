"""Compare fewer-view scans under identical collision and shape constraints."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def compare(baseline, candidate):
    """Do not count fewer captures as success if paired geometry degrades."""
    old={r['episode']:r for r in baseline['episodes']}
    new={r['episode']:r for r in candidate['episodes']}
    if len(old)!=6 or set(old)!=set(new):
        raise ValueError('expected six matching scan episodes')
    rows=[]
    for episode in sorted(old):
        a,b=old[episode],new[episode]
        if a['kind']!=b['kind']:
            raise ValueError('scan fixtures differ')
        am,bm=a['fused'],b['fused']
        recall_loss=float(am['recall_5mm']-bm['recall_5mm'])
        error_increase=float(bm['surface_p95_mm']-am['surface_p95_mm'])
        dimension_increase=float(np.max(np.asarray(bm['dimension_error_mm'])-am['dimension_error_mm']))
        preserved=bool(recall_loss<=.05 and error_increase<=1. and dimension_increase<=3.)
        rows.append({'episode':episode,'kind':a['kind'],'baseline_views':a['views_attempted'],
            'candidate_views':b['views_attempted'],'fewer_views':b['views_attempted']<a['views_attempted'],
            'baseline_coverage':am['recall_5mm'],'candidate_coverage':bm['recall_5mm'],
            'coverage_loss_pp':100*recall_loss,'surface_p95_increase_mm':error_increase,
            'max_dimension_error_increase_mm':dimension_increase,'paired_quality_preserved':preserved,
            'baseline_full_shape_pass':a['accepted'],'candidate_full_shape_pass':b['accepted']})
    safety=all((value.get('collision_audit') or {}).get('accepted',False) for value in (baseline,candidate))
    before=sum(r['baseline_views'] for r in rows);after=sum(r['candidate_views'] for r in rows)
    return {'complete':True,'accepted':bool(safety and all(r['fewer_views'] and r['paired_quality_preserved'] for r in rows)),
            'collision_pass':safety,'baseline_total_views':before,'candidate_total_views':after,
            'capture_reduction_fraction':1-after/before,'episodes':rows,
            'criteria':{'each_episode_fewer_views':True,'coverage_loss_max_pp':5.,
                        'surface_p95_increase_max_mm':1.,'each_dimension_error_increase_max_mm':3.},
            'full_shape_pass_baseline':sum(r['baseline_full_shape_pass'] for r in rows),
            'full_shape_pass_candidate':sum(r['candidate_full_shape_pass'] for r in rows),
            'full_shape_complete_claimed':False}


def plot_comparison(baseline,candidate,result,path):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    a={r['episode']:r for r in baseline['episodes']};b={r['episode']:r for r in candidate['episodes']}
    x=np.arange(6);width=.36
    fig,axes=plt.subplots(1,3,figsize=(15,4.8),layout='constrained')
    for offset,source,label,color in ((-width/2,a,'Collision-checked angular / up to 20 views','#969ca4'),
                                     (width/2,b,'Information gain / early stop','#179ba8')):
        bars=axes[0].bar(x+offset,[source[i]['views_attempted'] for i in x],width,color=color,label=label)
        axes[0].bar_label(bars,padding=2)
        axes[1].bar(x+offset,[source[i]['fused']['recall_5mm']*100 for i in x],width,color=color)
        axes[2].bar(x+offset,[source[i]['fused']['surface_p95_mm'] for i in x],width,color=color)
    axes[0].set(title='Fewer actual captures',ylabel='Captured RGB-D frames');axes[0].legend(fontsize=8)
    axes[1].set(title='Full-surface coverage within 5 mm',ylabel='Truth surface recovered (%)',ylim=(0,100))
    axes[1].axhline(60,color='#555',ls='--',lw=.8)
    axes[2].set(title='Cloud-to-true-surface p95',ylabel='Surface error (mm)')
    for ax in axes:
        ax.set_xticks(x,['Box','Thin box','Cylinder','Box + error','Thin + error','Cylinder + error'],rotation=22,ha='right')
        ax.grid(axis='y',alpha=.2)
    fig.suptitle(f"Same self-collision constraints | {result['baseline_total_views']} → {result['candidate_total_views']} captures | no hidden geometry added",fontsize=14)
    fig.savefig(path,dpi=160);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline',type=Path);parser.add_argument('candidate',type=Path)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--figure',type=Path)
    args=parser.parse_args();a=json.loads(args.baseline.read_text());b=json.loads(args.candidate.read_text())
    result=compare(a,b);args.output.write_text(json.dumps(result,indent=2)+'\n')
    if args.figure:plot_comparison(a,b,result,args.figure)
    print(json.dumps(result))


if __name__=='__main__':main()
