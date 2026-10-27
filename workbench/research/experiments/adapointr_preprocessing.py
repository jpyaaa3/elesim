"""Same-sample preprocessing ablation on public AdaPoinTr demo clouds.

Public demos have no complete truth. This measures observation fidelity and
output equivariance after restoring every result to the original input frame.
It cannot establish absolute completion accuracy or isolate the box-trial cause.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial import cKDTree

from .adapointr import CloudFrame,normalize_partial,restore_completion,sample_input


VARIANTS=('native','center_only','scale_only_r05','center_scale_r05','center_scale_r10')


def prepare_control(points,variant,*,seed=0):
    points=np.asarray(points,dtype=np.float64)
    _,normalized_frame=normalize_partial(points)
    if variant=='native':frame=CloudFrame(np.zeros(3),1.)
    elif variant=='center_only':frame=CloudFrame(normalized_frame.center,1.)
    elif variant=='scale_only_r05':frame=CloudFrame(np.zeros(3),normalized_frame.meters_per_unit)
    elif variant=='center_scale_r05':frame=normalized_frame
    elif variant=='center_scale_r10':frame=CloudFrame(normalized_frame.center,normalized_frame.meters_per_unit/2)
    else:raise ValueError('unknown preprocessing control')
    transformed=(points-frame.center)/frame.meters_per_unit
    return sample_input(transformed,seed=seed),frame


def comparison_metrics(observed,predicted,reference):
    observed,predicted,reference=[np.asarray(v,dtype=np.float64) for v in (observed,predicted,reference)]
    if any(v.ndim!=2 or v.shape[1]!=3 or not len(v) or not np.all(np.isfinite(v)) for v in (observed,predicted,reference)):
        raise ValueError('invalid comparison cloud')
    diagonal=float(np.linalg.norm(np.ptp(observed,axis=0)))
    if diagonal<1e-9:raise ValueError('degenerate observed extent')
    predicted_tree=cKDTree(predicted)
    fidelity=predicted_tree.query(observed)[0]
    to_reference=cKDTree(reference).query(predicted)[0]
    from_reference=predicted_tree.query(reference)[0]
    symmetric=float((to_reference.mean()+from_reference.mean())/2)
    p95=float(np.percentile(fidelity,95))
    return {'input_diagonal':diagonal,'observed_to_prediction_mean':float(fidelity.mean()),
            'observed_to_prediction_p95':p95,'observed_to_prediction_p95_pct_diag':100*p95/diagonal,
            'reference_chamfer_mean':symmetric,'reference_chamfer_mean_pct_diag':100*symmetric/diagonal,
            'reference_distance_p95_pct_diag':100*float(np.percentile(np.concatenate([to_reference,from_reference]),95))/diagonal,
            'reference_centroid_shift_pct_diag':100*float(np.linalg.norm(predicted.mean(0)-reference.mean(0)))/diagonal}


def main():
    import torch
    from workbench.research.support.adapointr_backend import load_model
    from .adapointr_gpu_smoke import read_demo
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vendor',type=Path,required=True);parser.add_argument('--checkpoint',type=Path,required=True)
    parser.add_argument('--checkpoint-sha256',required=True);parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--seeds',type=int,default=5)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    assert torch.cuda.is_available() and torch.cuda.device_count()==1
    if not 1<=args.seeds<=20:raise ValueError('invalid seed budget')
    torch.set_num_threads(2);torch.manual_seed(17)
    torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
    model,metadata=load_model(args.vendor,args.config,args.checkpoint,checkpoint_sha256=args.checkpoint_sha256)
    rows=[];native_outputs={};originals={}
    with torch.inference_mode():
        for name in ('airplane','sofa'):
            observed=read_demo(args.vendor/'demo'/(name+'.pcd'));originals[name]=observed.copy()
            warm,_=prepare_control(observed,'native',seed=0)
            model(torch.from_numpy(warm[None]).cuda());torch.cuda.synchronize()
            for seed in range(args.seeds):
                for variant in VARIANTS:
                    prepared,frame=prepare_control(observed,variant,seed=seed)
                    restored_samples=restore_completion(prepared,frame)
                    np.testing.assert_allclose(restored_samples,sample_input(observed,seed=seed),atol=1e-7)
                    tensor=torch.from_numpy(prepared[None]).cuda();torch.cuda.synchronize()
                    start=time.perf_counter();generated=model(tensor)[-1];torch.cuda.synchronize()
                    elapsed=(time.perf_counter()-start)*1000
                    predicted=restore_completion(generated[0].cpu().numpy(),frame)
                    assert len(predicted)==16384 and np.all(np.isfinite(predicted))
                    if variant=='native':native_outputs[(name,seed)]=predicted
                    reference=native_outputs[(name,seed)]
                    metrics=comparison_metrics(observed,predicted,reference)
                    row={'sample':name,'seed':seed,'variant':variant,'output_points':len(predicted),
                         'forward_wall_ms':elapsed,'center':frame.center.tolist(),'scale':frame.meters_per_unit,
                         'model_input_near_zero_points':int(np.sum(np.sum(prepared*prepared,axis=1)<=1e-3)),
                         'file':name+'-'+str(seed)+'-'+variant+'.npz',**metrics}
                    np.savez_compressed(args.output/row['file'],observed=observed,predicted=predicted,prepared=prepared,
                                        center=frame.center,scale=frame.meters_per_unit)
                    rows.append(row)
                print(json.dumps({'sample':name,'completed_seeds':seed+1}),flush=True)
            np.testing.assert_array_equal(observed,originals[name])
    summaries={};controls={}
    for name in ('airplane','sofa'):
        native={r['seed']:r for r in rows if r['sample']==name and r['variant']=='native'}
        summaries[name]={}
        for variant in VARIANTS:
            selected=[r for r in rows if r['sample']==name and r['variant']==variant]
            ratios=[r['observed_to_prediction_p95']/max(native[r['seed']]['observed_to_prediction_p95'],1e-12) for r in selected]
            worse=sum(ratio>1 for ratio in ratios)
            summaries[name][variant]={
                'fidelity_p95_pct_diag_median':float(np.median([r['observed_to_prediction_p95_pct_diag'] for r in selected])),
                'fidelity_p95_pct_diag_range':[min(r['observed_to_prediction_p95_pct_diag'] for r in selected),max(r['observed_to_prediction_p95_pct_diag'] for r in selected)],
                'paired_fidelity_ratio_median':float(np.median(ratios)),
                'seeds_worse_than_native':worse,'output_chamfer_pct_diag_median':float(np.median([r['reference_chamfer_mean_pct_diag'] for r in selected])),
                'near_zero_points_median':float(np.median([r['model_input_near_zero_points'] for r in selected])),
                'strong_fidelity_degradation':bool(np.median(ratios)>1.25 and worse>=(4*args.seeds+4)//5)}
        noise=[comparison_metrics(originals[name],native_outputs[(name,seed)],native_outputs[(name,0)])['reference_chamfer_mean_pct_diag'] for seed in range(1,args.seeds)]
        controls[name]={'native_sampling_chamfer_pct_diag_median':float(np.median(noise)) if noise else 0.,
                        'partial_centroid':originals[name].mean(0).tolist()}
    result={'schema_version':1,'complete':True,'model':metadata,'rows':rows,'summary':summaries,'controls':controls,
            'seeds':args.seeds,'gpu':torch.cuda.get_device_name(0),'gpu_count':torch.cuda.device_count(),
            'torch':torch.__version__,'cuda':torch.version.cuda,'tf32':False,
            'same_physical_sample_indices':True,'axis_order':'unchanged original PCN Y-up in every variant',
            'comparison_criterion':'paired observation-fidelity p95 median ratio >1.25 and worse in >=80% of seeds (4/5 by default)',
            'complete_ground_truth_available':False,'units':'original dataset coordinate units; percentages use original observed AABB diagonal',
            'scope':'preprocessing sensitivity, observation fidelity and output equivariance; no absolute completion-accuracy conclusion'}
    (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'complete':True,'summary':summaries,'controls':controls}),flush=True)


if __name__=='__main__':main()
